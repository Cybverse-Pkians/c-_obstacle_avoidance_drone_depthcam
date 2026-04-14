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
  CRUISING,        // Normal forward flight
  CLIMBING,        // Stopped - increasing altitude with 1m clearance
  CLEARING,        // Moving forward over obstacle (front FOV clear)
  DESCENDING       // Returning to normal altitude (down FOV clear)
};

class DualCamObstacleAvoidance : public rclcpp::Node
{
public:
  DualCamObstacleAvoidance() : Node("dual_cam_obstacle_avoidance")
  {
    // Subscribe to BOTH depth cameras
    depth_front_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      std::bind(&DualCamObstacleAvoidance::depthFrontCallback, this, std::placeholders::_1));

    depth_down_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image_down", 10,
      std::bind(&DualCamObstacleAvoidance::depthDownCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&DualCamObstacleAvoidance::stateCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&DualCamObstacleAvoidance::controlLoop, this));
    
    cmd_timer_ = this->create_wall_timer(
      50ms, std::bind(&DualCamObstacleAvoidance::publishCmd, this));

    state_ = FlightState::WARMUP;
    avoidance_mode_ = AvoidanceMode::CRUISING;
    
    last_depth_front_time_ = now();
    last_depth_down_time_ = now();
    last_update_time_ = now();

    current_mavros_state_.connected = false;
    current_mavros_state_.armed = false;
    current_mavros_state_.mode = "";

    // Altitude tracking
    base_altitude_ = 2.0;           // Normal cruising altitude
    current_altitude_target_ = 2.0; // Current target altitude
    max_altitude_ = 80.0;            // Maximum climb altitude
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Dual Camera Obstacle Avoidance Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Strategy:");
    RCLCPP_INFO(this->get_logger(), "1. Detect obstacle in FRONT camera -> STOP");
    RCLCPP_INFO(this->get_logger(), "2. CLIMB maintaining 1m clearance");
    RCLCPP_INFO(this->get_logger(), "3. Wait until obstacle clears FRONT FOV");
    RCLCPP_INFO(this->get_logger(), "4. Move forward monitoring DOWN camera");
    RCLCPP_INFO(this->get_logger(), "5. When obstacle leaves DOWN FOV -> DESCEND");
    RCLCPP_INFO(this->get_logger(), "6. Return to base altitude and continue");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  // ROS Interface
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_down_sub_;
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
  cv::Mat depth_down_image_;
  
  rclcpp::Time last_depth_front_time_;
  rclcpp::Time last_depth_down_time_;
  rclcpp::Time last_update_time_;
  
  mavros_msgs::msg::State current_mavros_state_;
  geometry_msgs::msg::Twist current_cmd_;

  int warmup_counter_ = 0;
  int state_counter_ = 0;
  double traveled_distance_ = 0.0;

  // Altitude control
  double base_altitude_;
  double current_altitude_target_;
  double max_altitude_;
  
  // Obstacle detection
  bool obstacle_ahead_ = false;
  bool obstacle_cleared_ = false;
  double obstacle_distance_ = 200.0;
  double ground_clearance_ = 200.0;

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

  void depthDownCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg);
      if (msg->encoding == "32FC1")
        depth_down_image_ = cv_ptr->image.clone();
      else if (msg->encoding == "16UC1")
        cv_ptr->image.convertTo(depth_down_image_, CV_32F, 0.001);
      last_depth_down_time_ = now();
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Down depth conversion failed");
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
    // Check if we have valid depth data
    if (depth_front_image_.empty() || depth_down_image_.empty()) { 
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Missing depth data! Front: %s, Down: %s", 
        depth_front_image_.empty() ? "NO" : "YES",
        depth_down_image_.empty() ? "NO" : "YES");
      stopDrone(); 
      return; 
    }

    if ((now() - last_depth_front_time_).seconds() > 1.0 ||
        (now() - last_depth_down_time_).seconds() > 1.0)
    {
      RCLCPP_WARN(this->get_logger(), "Depth data stale!");
      stopDrone();
      return;
    }

    // Process obstacle avoidance with dual cameras
    processDualCamObstacleAvoidance();

    // Track distance
    rclcpp::Time t = now();
    double dt = (t - last_update_time_).seconds();
    last_update_time_ = t;

    traveled_distance_ += std::abs(current_cmd_.linear.y) * dt;

    // Logging
    std::string mode_str;
    switch(avoidance_mode_) {
      case AvoidanceMode::CRUISING:   mode_str = "CRUISING"; break;
      case AvoidanceMode::CLIMBING:   mode_str = "CLIMBING"; break;
      case AvoidanceMode::CLEARING:   mode_str = "CLEARING"; break;
      case AvoidanceMode::DESCENDING: mode_str = "DESCENDING"; break;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Dist: %.2f/300m | %s | Alt: %.2fm | Front: %.2fm | Down: %.2fm", 
      traveled_distance_, mode_str.c_str(), current_altitude_target_,
      obstacle_distance_, ground_clearance_);

    if (traveled_distance_ >= 300.0)
    {
      stopDrone();
      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE - 300m traveled!");
      RCLCPP_INFO(this->get_logger(), "========================================");
    }
  }

  /* ================= DUAL CAMERA OBSTACLE AVOIDANCE ================= */

  void processDualCamObstacleAvoidance()
  {
    // Analyze cameras
    obstacle_distance_ = analyzeFrontCamera();
    ground_clearance_ = analyzeDownCamera();

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;  // Never rotate

    // === THRESHOLDS ===
    float obstacle_detect_dist = 3.0;     // Trigger avoidance when obstacle < 3m in front
    float min_clearance = 1.0;            // Maintain 1m clearance from obstacle
    float front_clear_dist = 5.0;         // Obstacle cleared from front FOV
    float down_clear_dist = 5.0;          // Obstacle cleared from down FOV
    float climb_speed = 1.0;              // Climb rate
    float descend_speed = 0.1;            // Descend rate
    float forward_speed = 0.8;            // Forward speed

    switch(avoidance_mode_)
    {
      case AvoidanceMode::CRUISING:
      {
        if (obstacle_distance_ < obstacle_detect_dist)
        {
          // === STEP 1: OBSTACLE DETECTED - STOP ===
          RCLCPP_INFO(this->get_logger(), 
            "╔════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ OBSTACLE DETECTED at %.2fm - STOPPING ║", obstacle_distance_);
          RCLCPP_INFO(this->get_logger(), 
            "╚════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::CLIMBING;
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

      case AvoidanceMode::CLIMBING:
      {
        // === STEP 2: STOPPED - CLIMB WITH 1M CLEARANCE ===
        cmd.linear.x = 0.0;     // STOPPED
        cmd.linear.y = 0.0;     // STOPPED
        cmd.linear.z = climb_speed;
        
        current_altitude_target_ += climb_speed * 0.1;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "⬆️  CLIMBING (STOPPED) | Front: %.2fm | Clearance: 1.0m target", 
          obstacle_distance_);
        
        // === STEP 3: WAIT UNTIL OBSTACLE CLEARS FRONT FOV ===
        if (obstacle_distance_ > front_clear_dist)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared from FRONT camera!     ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Distance: %.2fm > %.2fm                  ║", 
            obstacle_distance_, front_clear_dist);
          RCLCPP_INFO(this->get_logger(), 
            "║ Starting forward movement over obstacle ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::CLEARING;
        }
        
        // Safety limit
        if (current_altitude_target_ > max_altitude_)
        {
          RCLCPP_WARN(this->get_logger(), "⚠️  Max altitude reached!");
          current_altitude_target_ = max_altitude_;
          cmd.linear.z = 0.0;
        }
        break;
      }

      case AvoidanceMode::CLEARING:
      {
        // === STEP 4: FRONT CLEAR - MOVE FORWARD, MONITOR DOWN ===
        cmd.linear.x = 0.0;
        cmd.linear.y = forward_speed;
        
        // Maintain at least 1m clearance above obstacle
        if (ground_clearance_ < min_clearance)
        {
          cmd.linear.z = climb_speed * 0.5;
          current_altitude_target_ += climb_speed * 0.05;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "⚠️  CLEARING | Too close! Down: %.2fm | Climbing to maintain 1m", 
            ground_clearance_);
        }
        else
        {
          cmd.linear.z = 0.0;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "➡️  CLEARING | Down: %.2fm | Waiting for obstacle to leave view", 
            ground_clearance_);
        }
        
        // === STEP 5: WAIT UNTIL OBSTACLE LEAVES DOWN FOV ===
        if (ground_clearance_ > down_clear_dist)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared from DOWN camera!      ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Distance: %.2fm > %.2fm                  ║", 
            ground_clearance_, down_clear_dist);
          RCLCPP_INFO(this->get_logger(), 
            "║ Starting descent to base altitude       ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::DESCENDING;
        }
        break;
      }

      case AvoidanceMode::DESCENDING:
      {
        // === STEP 6: BOTH CAMERAS CLEAR - DESCEND AND CONTINUE ===
        cmd.linear.x = 0.0;
        cmd.linear.y = forward_speed;
        
        if (current_altitude_target_ > base_altitude_)
        {
          cmd.linear.z = -descend_speed;
          current_altitude_target_ -= descend_speed * 0.1;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "⬇️  DESCENDING | Alt: %.2fm → %.2fm", 
            current_altitude_target_, base_altitude_);
        }
        else
        {
          cmd.linear.z = 0.0;
          current_altitude_target_ = base_altitude_;
          
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Returned to base altitude: %.2fm         ║", base_altitude_);
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle maneuver complete!              ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Resuming normal cruise                   ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
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
    if (depth_front_image_.empty()) return 100.0f;

    int rows = depth_front_image_.rows;
    int cols = depth_front_image_.cols;

    float center_distance = getMinDepth(depth_front_image_, 
                                        rows*0.3, rows*0.7, 
                                        cols*0.3, cols*0.7);
    
    return center_distance;
  }

  float analyzeDownCamera()
  {
    if (depth_down_image_.empty()) return 100.0f;

    int rows = depth_down_image_.rows;
    int cols = depth_down_image_.cols;

    float ground_distance = getMinDepth(depth_down_image_,
                                        rows*0.3, rows*0.7,
                                        cols*0.3, cols*0.7);
    
    return ground_distance;
  }

  float getMinDepth(const cv::Mat& image, int r1, int r2, int c1, int c2)
  {
    float min_val = 100.0f;

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

int main(int argc, ch#include <rclcpp/rclcpp.hpp>
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
  CRUISING,        // Normal forward flight
  CLIMBING,        // Stopped - increasing altitude with 1m clearance
  CLEARING,        // Moving forward over obstacle (front FOV clear)
  DESCENDING       // Returning to normal altitude (down FOV clear)
};

class DualCamObstacleAvoidance : public rclcpp::Node
{
public:
  DualCamObstacleAvoidance() : Node("dual_cam_obstacle_avoidance")
  {
    // Subscribe to BOTH depth cameras
    depth_front_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      std::bind(&DualCamObstacleAvoidance::depthFrontCallback, this, std::placeholders::_1));

    depth_down_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image_down", 10,
      std::bind(&DualCamObstacleAvoidance::depthDownCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&DualCamObstacleAvoidance::stateCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&DualCamObstacleAvoidance::controlLoop, this));
    
    cmd_timer_ = this->create_wall_timer(
      50ms, std::bind(&DualCamObstacleAvoidance::publishCmd, this));

    state_ = FlightState::WARMUP;
    avoidance_mode_ = AvoidanceMode::CRUISING;
    
    last_depth_front_time_ = now();
    last_depth_down_time_ = now();
    last_update_time_ = now();

    current_mavros_state_.connected = false;
    current_mavros_state_.armed = false;
    current_mavros_state_.mode = "";

    // Altitude tracking
    base_altitude_ = 2.0;           // Normal cruising altitude
    current_altitude_target_ = 2.0; // Current target altitude
    max_altitude_ = 80.0;            // Maximum climb altitude
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Dual Camera Obstacle Avoidance Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Strategy:");
    RCLCPP_INFO(this->get_logger(), "1. Detect obstacle in FRONT camera -> STOP");
    RCLCPP_INFO(this->get_logger(), "2. CLIMB maintaining 1m clearance");
    RCLCPP_INFO(this->get_logger(), "3. Wait until obstacle clears FRONT FOV");
    RCLCPP_INFO(this->get_logger(), "4. Move forward monitoring DOWN camera");
    RCLCPP_INFO(this->get_logger(), "5. When obstacle leaves DOWN FOV -> DESCEND");
    RCLCPP_INFO(this->get_logger(), "6. Return to base altitude and continue");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  // ROS Interface
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_down_sub_;
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
  cv::Mat depth_down_image_;
  
  rclcpp::Time last_depth_front_time_;
  rclcpp::Time last_depth_down_time_;
  rclcpp::Time last_update_time_;
  
  mavros_msgs::msg::State current_mavros_state_;
  geometry_msgs::msg::Twist current_cmd_;

  int warmup_counter_ = 0;
  int state_counter_ = 0;
  double traveled_distance_ = 0.0;

  // Altitude control
  double base_altitude_;
  double current_altitude_target_;
  double max_altitude_;
  
  // Obstacle detection
  bool obstacle_ahead_ = false;
  bool obstacle_cleared_ = false;
  double obstacle_distance_ = 200.0;
  double ground_clearance_ = 200.0;

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

  void depthDownCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg);
      if (msg->encoding == "32FC1")
        depth_down_image_ = cv_ptr->image.clone();
      else if (msg->encoding == "16UC1")
        cv_ptr->image.convertTo(depth_down_image_, CV_32F, 0.001);
      last_depth_down_time_ = now();
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Down depth conversion failed");
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
    // Check if we have valid depth data
    if (depth_front_image_.empty() || depth_down_image_.empty()) { 
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Missing depth data! Front: %s, Down: %s", 
        depth_front_image_.empty() ? "NO" : "YES",
        depth_down_image_.empty() ? "NO" : "YES");
      stopDrone(); 
      return; 
    }

    if ((now() - last_depth_front_time_).seconds() > 1.0 ||
        (now() - last_depth_down_time_).seconds() > 1.0)
    {
      RCLCPP_WARN(this->get_logger(), "Depth data stale!");
      stopDrone();
      return;
    }

    // Process obstacle avoidance with dual cameras
    processDualCamObstacleAvoidance();

    // Track distance
    rclcpp::Time t = now();
    double dt = (t - last_update_time_).seconds();
    last_update_time_ = t;

    traveled_distance_ += std::abs(current_cmd_.linear.y) * dt;

    // Logging
    std::string mode_str;
    switch(avoidance_mode_) {
      case AvoidanceMode::CRUISING:   mode_str = "CRUISING"; break;
      case AvoidanceMode::CLIMBING:   mode_str = "CLIMBING"; break;
      case AvoidanceMode::CLEARING:   mode_str = "CLEARING"; break;
      case AvoidanceMode::DESCENDING: mode_str = "DESCENDING"; break;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Dist: %.2f/300m | %s | Alt: %.2fm | Front: %.2fm | Down: %.2fm", 
      traveled_distance_, mode_str.c_str(), current_altitude_target_,
      obstacle_distance_, ground_clearance_);

    if (traveled_distance_ >= 300.0)
    {
      stopDrone();
      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE - 300m traveled!");
      RCLCPP_INFO(this->get_logger(), "========================================");
    }
  }

  /* ================= DUAL CAMERA OBSTACLE AVOIDANCE ================= */

  void processDualCamObstacleAvoidance()
  {
    // Analyze cameras
    obstacle_distance_ = analyzeFrontCamera();
    ground_clearance_ = analyzeDownCamera();

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;  // Never rotate

    // === THRESHOLDS ===
    float obstacle_detect_dist = 3.0;     // Trigger avoidance when obstacle < 3m in front
    float min_clearance = 1.0;            // Maintain 1m clearance from obstacle
    float front_clear_dist = 5.0;         // Obstacle cleared from front FOV
    float down_clear_dist = 5.0;          // Obstacle cleared from down FOV
    float climb_speed = 1.0;              // Climb rate
    float descend_speed = 0.1;            // Descend rate
    float forward_speed = 0.8;            // Forward speed

    switch(avoidance_mode_)
    {
      case AvoidanceMode::CRUISING:
      {
        if (obstacle_distance_ < obstacle_detect_dist)
        {
          // === STEP 1: OBSTACLE DETECTED - STOP ===
          RCLCPP_INFO(this->get_logger(), 
            "╔════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ OBSTACLE DETECTED at %.2fm - STOPPING ║", obstacle_distance_);
          RCLCPP_INFO(this->get_logger(), 
            "╚════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::CLIMBING;
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

      case AvoidanceMode::CLIMBING:
      {
        // === STEP 2: STOPPED - CLIMB WITH 1M CLEARANCE ===
        cmd.linear.x = 0.0;     // STOPPED
        cmd.linear.y = 0.0;     // STOPPED
        cmd.linear.z = climb_speed;
        
        current_altitude_target_ += climb_speed * 0.1;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "⬆️  CLIMBING (STOPPED) | Front: %.2fm | Clearance: 1.0m target", 
          obstacle_distance_);
        
        // === STEP 3: WAIT UNTIL OBSTACLE CLEARS FRONT FOV ===
        if (obstacle_distance_ > front_clear_dist)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared from FRONT camera!     ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Distance: %.2fm > %.2fm                  ║", 
            obstacle_distance_, front_clear_dist);
          RCLCPP_INFO(this->get_logger(), 
            "║ Starting forward movement over obstacle ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::CLEARING;
        }
        
        // Safety limit
        if (current_altitude_target_ > max_altitude_)
        {
          RCLCPP_WARN(this->get_logger(), "⚠️  Max altitude reached!");
          current_altitude_target_ = max_altitude_;
          cmd.linear.z = 0.0;
        }
        break;
      }

      case AvoidanceMode::CLEARING:
      {
        // === STEP 4: FRONT CLEAR - MOVE FORWARD, MONITOR DOWN ===
        cmd.linear.x = 0.0;
        cmd.linear.y = forward_speed;
        
        // Maintain at least 1m clearance above obstacle
        if (ground_clearance_ < min_clearance)
        {
          cmd.linear.z = climb_speed * 0.5;
          current_altitude_target_ += climb_speed * 0.05;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "⚠️  CLEARING | Too close! Down: %.2fm | Climbing to maintain 1m", 
            ground_clearance_);
        }
        else
        {
          cmd.linear.z = 0.0;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "➡️  CLEARING | Down: %.2fm | Waiting for obstacle to leave view", 
            ground_clearance_);
        }
        
        // === STEP 5: WAIT UNTIL OBSTACLE LEAVES DOWN FOV ===
        if (ground_clearance_ > down_clear_dist)
        {
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle cleared from DOWN camera!      ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Distance: %.2fm > %.2fm                  ║", 
            ground_clearance_, down_clear_dist);
          RCLCPP_INFO(this->get_logger(), 
            "║ Starting descent to base altitude       ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
          avoidance_mode_ = AvoidanceMode::DESCENDING;
        }
        break;
      }

      case AvoidanceMode::DESCENDING:
      {
        // === STEP 6: BOTH CAMERAS CLEAR - DESCEND AND CONTINUE ===
        cmd.linear.x = 0.0;
        cmd.linear.y = forward_speed;
        
        if (current_altitude_target_ > base_altitude_)
        {
          cmd.linear.z = -descend_speed;
          current_altitude_target_ -= descend_speed * 0.1;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "⬇️  DESCENDING | Alt: %.2fm → %.2fm", 
            current_altitude_target_, base_altitude_);
        }
        else
        {
          cmd.linear.z = 0.0;
          current_altitude_target_ = base_altitude_;
          
          RCLCPP_INFO(this->get_logger(), 
            "╔══════════════════════════════════════════╗");
          RCLCPP_INFO(this->get_logger(), 
            "║ Returned to base altitude: %.2fm         ║", base_altitude_);
          RCLCPP_INFO(this->get_logger(), 
            "║ Obstacle maneuver complete!              ║");
          RCLCPP_INFO(this->get_logger(), 
            "║ Resuming normal cruise                   ║");
          RCLCPP_INFO(this->get_logger(), 
            "╚══════════════════════════════════════════╝");
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
    if (depth_front_image_.empty()) return 100.0f;

    int rows = depth_front_image_.rows;
    int cols = depth_front_image_.cols;

    float center_distance = getMinDepth(depth_front_image_, 
                                        rows*0.3, rows*0.7, 
                                        cols*0.3, cols*0.7);
    
    return center_distance;
  }

  float analyzeDownCamera()
  {
    if (depth_down_image_.empty()) return 100.0f;

    int rows = depth_down_image_.rows;
    int cols = depth_down_image_.cols;

    float ground_distance = getMinDepth(depth_down_image_,
                                        rows*0.3, rows*0.7,
                                        cols*0.3, cols*0.7);
    
    return ground_distance;
  }

  float getMinDepth(const cv::Mat& image, int r1, int r2, int c1, int c2)
  {
    float min_val = 100.0f;

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
  rclcpp::spin(std::make_shared<DualCamObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}ar **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualCamObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}