#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

enum class FlightState
{
  WARMUP,
  GUIDED,
  ARMING,
  TAKEOFF,
  MISSION
};

enum class AvoidanceMode
{
  CRUISING,           // Normal forward flight
  SIDE_AVOIDING,      // Strafing sideways to avoid obstacle
  EXTRA_DISTANCE,     // Moving extra 1m after clearing obstacle
  FORWARD_CLEARING,   // Moving forward past obstacle
  RETURNING           // Returning to original path
};

class SideObstacleAvoidance : public rclcpp::Node
{
public:
  SideObstacleAvoidance() : Node("side_obstacle_avoidance")
  {
    // Subscribe to front depth camera
    depth_front_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      std::bind(&SideObstacleAvoidance::depthFrontCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&SideObstacleAvoidance::stateCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&SideObstacleAvoidance::controlLoop, this));
    
    cmd_timer_ = this->create_wall_timer(
      50ms, std::bind(&SideObstacleAvoidance::publishCmd, this));

    state_ = FlightState::WARMUP;
    avoidance_mode_ = AvoidanceMode::CRUISING;
    
    last_depth_front_time_ = now();
    last_update_time_ = now();

    current_mavros_state_.connected = false;
    current_mavros_state_.armed = false;
    current_mavros_state_.mode = "";

    // Altitude tracking
    base_altitude_ = 2.0;
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Side Obstacle Avoidance Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Strategy:");
    RCLCPP_INFO(this->get_logger(), "1. Detect obstacle in FRONT camera -> STOP");
    RCLCPP_INFO(this->get_logger(), "2. STRAFE left/right (no rotation)");
    RCLCPP_INFO(this->get_logger(), "3. Wait until obstacle clears FOV");
    RCLCPP_INFO(this->get_logger(), "4. Strafe EXTRA 1m for clearance");
    RCLCPP_INFO(this->get_logger(), "5. Move FORWARD past obstacle");
    RCLCPP_INFO(this->get_logger(), "6. Return to original path");
    RCLCPP_INFO(this->get_logger(), "7. Resume cruise");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  // ROS Interface
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_front_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // State variables
  FlightState state_;
  AvoidanceMode avoidance_mode_;
  
  cv::Mat depth_front_image_;
  
  rclcpp::Time last_depth_front_time_;
  rclcpp::Time last_update_time_;
  
  mavros_msgs::msg::State current_mavros_state_;
  geometry_msgs::msg::Twist current_cmd_;

  int warmup_counter_ = 0;
  int state_counter_ = 0;
  double traveled_distance_ = 0.0;

  double base_altitude_;
  
  // Obstacle detection
  double obstacle_distance_ = 200.0;
  double lateral_offset_ = 0.0;        // Track how far we've moved sideways
  double forward_clearing_distance_ = 0.0;  // Track forward distance during clearing
  int avoidance_direction_ = 1;        // 1 = left, -1 = right
  
  /* ================= CALLBACKS ================= */

  void depthFrontCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg);
      if (msg->encoding == "32FC1")
        depth_front_image_ = cv_ptr->image.clone();
      else if (msg->encoding == "16UC1")
        cv_ptr->image.convertTo(depth_front_image_, CV_32F, 0.001);
      last_depth_front_time_ = now();
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Front depth conversion failed");
    }
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_mavros_state_ = *msg;
  }

  /* ================= CONTROL LOOP ================= */

  void controlLoop()
  {
    switch (state_)
    {
      case FlightState::WARMUP:  warmup();  break;
      case FlightState::GUIDED:  setGuided(); break;
      case FlightState::ARMING:  arm(); break;
      case FlightState::TAKEOFF: takeoff(); break;
      case FlightState::MISSION: mission(); break;
    }
  }

  void warmup()
  {
    current_cmd_ = geometry_msgs::msg::Twist();
    warmup_counter_++;

    if (warmup_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Warming up... Connected: %s", 
        current_mavros_state_.connected ? "YES" : "NO");
    }

    if (!current_mavros_state_.connected) {
      warmup_counter_ = 0;
      return;
    }

    if (warmup_counter_ > 40) {
      RCLCPP_INFO(this->get_logger(), "Warmup complete, setting GUIDED mode");
      state_counter_ = 0;
      state_ = FlightState::GUIDED;
    }
  }

  void setGuided()
  {
    state_counter_++;

    if (state_counter_ == 1)
    {
      if (!mode_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Mode service not available");
        state_counter_ = 0;
        return;
      }

      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "GUIDED";
      auto result_future = mode_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "Switching to GUIDED mode...");
    }

    if (current_mavros_state_.mode == "GUIDED")
    {
      RCLCPP_INFO(this->get_logger(), "GUIDED mode confirmed, proceeding to arming");
      state_counter_ = 0;
      state_ = FlightState::ARMING;
    }
    else if (state_counter_ > 50)
    {
      RCLCPP_ERROR(this->get_logger(), "Mode change timeout, retrying...");
      state_counter_ = 0;
    }
  }

  void arm()
  {
    state_counter_++;

    if (state_counter_ == 1)
    {
      if (!arm_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Arming service not available");
        state_counter_ = 0;
        return;
      }

      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      auto result_future = arm_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "Arming request sent...");
    }

    if (current_mavros_state_.armed)
    {
      RCLCPP_INFO(this->get_logger(), "Armed! Proceeding to takeoff");
      state_counter_ = 0;
      state_ = FlightState::TAKEOFF;
    }
    else if (state_counter_ > 50)
    {
      RCLCPP_ERROR(this->get_logger(), "Arming timeout, retrying...");
      state_counter_ = 0;
    }
  }

  void takeoff()
  {
    state_counter_++;

    if (state_counter_ == 1)
    {
      if (!takeoff_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service not available");
        state_counter_ = 0;
        return;
      }

      auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude = base_altitude_;
      req->latitude = 0.0;
      req->longitude = 0.0;
      req->min_pitch = 0.0;
      req->yaw = 0.0;

      auto result_future = takeoff_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "Takeoff Request Sent! Target altitude: %.1fm", base_altitude_);
    }

    if (state_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), "Waiting for takeoff... (%d/100)", state_counter_);
    }

    if (state_counter_ > 100)
    {
      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "Takeoff complete - MISSION START");
      RCLCPP_INFO(this->get_logger(), "========================================");
      last_update_time_ = now();
      traveled_distance_ = 0.0;
      avoidance_mode_ = AvoidanceMode::CRUISING;
      state_ = FlightState::MISSION;
    }
  }

  /* ================= MISSION LOGIC ================= */

  void mission()
  {
    if (depth_front_image_.empty()) { 
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "No Front depth data!");
      stopDrone(); 
      return; 
    }

    if ((now() - last_depth_front_time_).seconds() > 1.0)
    {
      RCLCPP_WARN(this->get_logger(), "Depth data stale!");
      stopDrone();
      return;
    }

    // Process obstacle avoidance
    processSideObstacleAvoidance();

    // Track distance
    rclcpp::Time t = now();
    double dt = (t - last_update_time_).seconds();
    last_update_time_ = t;

    traveled_distance_ += std::abs(current_cmd_.linear.y) * dt;

    // Logging
    std::string mode_str;
    switch(avoidance_mode_) {
      case AvoidanceMode::CRUISING:        mode_str = "CRUISING"; break;
      case AvoidanceMode::SIDE_AVOIDING:   mode_str = "SIDE_AVOIDING"; break;
      case AvoidanceMode::EXTRA_DISTANCE:  mode_str = "EXTRA_DISTANCE"; break;
      case AvoidanceMode::FORWARD_CLEARING: mode_str = "FORWARD_CLEAR"; break;
      case AvoidanceMode::RETURNING:       mode_str = "RETURNING"; break;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Dist: %.2f/300m | %s | Front: %.2fm | Offset: %.2fm", 
      traveled_distance_, mode_str.c_str(), obstacle_distance_, lateral_offset_);

    if (traveled_distance_ >= 300.0)
    {
      stopDrone();
      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE - 300m traveled!");
      RCLCPP_INFO(this->get_logger(), "========================================");
    }
  }

  /* ================= SIDE OBSTACLE AVOIDANCE ================= */

  void processSideObstacleAvoidance()
  {
    // Analyze front camera
    obstacle_distance_ = analyzeFrontCamera();

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;  // NEVER rotate

    // === THRESHOLDS ===
    float obstacle_detect_dist = 3.0;     // Trigger avoidance
    float obstacle_clear_dist = 5.0;      // Obstacle no longer in FOV
    float extra_clearance = 1.0;          // Extra 1m after clearing
    float strafe_speed = 0.6;             // Sideways speed
    float forward_speed = 0.8;            // Forward speed

    rclcpp::Time t = now();
    double dt = (t - last_update_time_).seconds();

    switch(avoidance_mode_)
    {
      case AvoidanceMode::CRUISING:
      {
        if (obstacle_distance_ < obstacle_detect_dist)
        {
          // === STEP 1: OBSTACLE DETECTED - STOP AND STRAFE ===
          RCLCPP_INFO(this->get_logger(), 
            "╔════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ OBSTACLE DETECTED at %.2fm - STOPPING ║", obstacle_distance_);
          RCLCPP_INFO(this->get_logger(), 
            "╚════════════════════════════════════════╝");
          
          // Determine which side to strafe (left or right)
          // Simple: alternate or check which side has more space
          avoidance_direction_ = 1;  // Default to left (positive X)
          
          lateral_offset_ = 0.0;
          avoidance_mode_ = AvoidanceMode::SIDE_AVOIDING;
        }
        else
        {
          // Normal forward flight
          cmd.linear.x = 0.0;
          cmd.linear.y = 1.0;
          cmd.linear.z = 0.0;
        }
        break;
      }

      case AvoidanceMode::SIDE_AVOIDING:
      {
        // === STEP 2: STRAFE SIDEWAYS (NO ROTATION) ===
        cmd.linear.x = strafe_speed * avoidance_direction_;  // Strafe left or right
        cmd.linear.y = 0.0;  // STOPPED forward
        cmd.linear.z = 0.0;
        
        lateral_offset_ += std::abs(cmd.linear.x) * dt;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "⬅️  STRAFING %s (STOPPED) | Front: %.2fm | Offset: %.2fm", 
          avoidance_direction_ > 0 ? "LEFT" : "RIGHT",
          obstacle_distance_, lateral_offset_);
        
        // === STEP 3: WAIT UNTIL OBSTACLE CLEARS FOV ===
        if (obstacle_distance_ > obstacle_clear_dist)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared from FOV!               ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Distance: %.2fm > %.2fm                  ║", 
            obstacle_distance_, obstacle_clear_dist);
          RCLCPP_INFO(this->get_logger(), 
            "║ Moving extra 1m for clearance           ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          
          lateral_offset_ = 0.0;  // Reset for extra distance tracking
          avoidance_mode_ = AvoidanceMode::EXTRA_DISTANCE;
        }
        break;
      }

      case AvoidanceMode::EXTRA_DISTANCE:
      {
        // === STEP 4: MOVE EXTRA 1M IN SAME DIRECTION ===
        cmd.linear.x = strafe_speed * avoidance_direction_;  // Continue strafing
        cmd.linear.y = 0.0;  // Still stopped forward
        cmd.linear.z = 0.0;
        
        lateral_offset_ += std::abs(cmd.linear.x) * dt;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
          "⬅️⬅️ EXTRA CLEARANCE | %.2fm / 1.0m", lateral_offset_);
        
        if (lateral_offset_ >= extra_clearance)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Extra 1m clearance complete!             ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Moving forward past obstacle             ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          
          forward_clearing_distance_ = 0.0;
          avoidance_mode_ = AvoidanceMode::FORWARD_CLEARING;
        }
        break;
      }

      case AvoidanceMode::FORWARD_CLEARING:
      {
        // === STEP 5: MOVE FORWARD PAST OBSTACLE ===
        cmd.linear.x = 0.0;
        cmd.linear.y = forward_speed;  // Move forward
        cmd.linear.z = 0.0;
        
        forward_clearing_distance_ += std::abs(cmd.linear.y) * dt;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "➡️  FORWARD CLEARING | Distance: %.2fm", forward_clearing_distance_);
        
        // Move forward enough to clear obstacle (roughly equal to lateral offset)
        if (forward_clearing_distance_ >= 4.0)  // Move forward 4m past obstacle
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared! Returning to path     ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          
          lateral_offset_ = 0.0;
          avoidance_mode_ = AvoidanceMode::RETURNING;
        }
        break;
      }

      case AvoidanceMode::RETURNING:
      {
        // === STEP 6: RETURN TO ORIGINAL PATH ===
        cmd.linear.x = strafe_speed * (-avoidance_direction_);  // Strafe back (opposite direction)
        cmd.linear.y = forward_speed * 0.5;  // Slow forward while returning
        cmd.linear.z = 0.0;
        
        lateral_offset_ += std::abs(cmd.linear.x) * dt;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "↩️  RETURNING TO PATH | Offset: %.2fm", lateral_offset_);
        
        // Return approximately to original path (we moved ~4m sideways total including extra)
        if (lateral_offset_ >= 4.0)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Returned to original path!               ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle maneuver complete!              ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Resuming normal cruise                   ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          
          lateral_offset_ = 0.0;
          avoidance_mode_ = AvoidanceMode::CRUISING;
        }
        break;
      }
    }

    current_cmd_ = cmd;
  }

  /* ================= CAMERA ANALYSIS ================= */

  float analyzeFrontCamera()
  {
    if (depth_front_image_.empty()) return 200.0f;

    int rows = depth_front_image_.rows;
    int cols = depth_front_image_.cols;

    // Analyze center region for obstacles ahead
    float center_distance = getMinDepth(depth_front_image_, 
                                        rows*0.3, rows*0.7, 
                                        cols*0.3, cols*0.7);
    
    return center_distance;
  }

  float getMinDepth(const cv::Mat& image, int r1, int r2, int c1, int c2)
  {
    float min_val = 200.0f;

    r1 = std::max(0, r1); 
    r2 = std::min(image.rows, r2);
    c1 = std::max(0, c1); 
    c2 = std::min(image.cols, c2);

    for (int r = r1; r < r2; r++)
    {
      for (int c = c1; c < c2; c++)
      {
        float d = image.at<float>(r, c);
        if (d > 0.2f && d < min_val) 
          min_val = d;
      }
    }
    return min_val;
  }

  void stopDrone()
  {
    current_cmd_ = geometry_msgs::msg::Twist();
  }

  void publishCmd()
  {
    if (state_ == FlightState::MISSION || state_ == FlightState::TAKEOFF)
    {
      vel_pub_->publish(current_cmd_);
    }
  }
};

/* ================= MAIN ================= */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SideObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}