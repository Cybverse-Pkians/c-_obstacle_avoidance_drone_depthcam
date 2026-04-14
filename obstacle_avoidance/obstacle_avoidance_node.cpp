#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// TF2 Dependencies for Mapping
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

// ================= CONFIGURATION =================
const double GRID_RESOLUTION = 0.1; // 1 pixel = 10cm
const int GRID_SIZE = 1000;         // 100m x 100m map
const double SAFETY_BUFFER = 1.0;   // Keep 1m away from mapped obstacles
// =================================================

enum class FlightState { WARMUP, GUIDED, ARMING, TAKEOFF, MISSION };
enum class AvoidanceMode { CRUISING, STRAFING, MAPPING_CLEARANCE, RETURNING };

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
    {
        // Subscribers
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/realsense/depth_image", 10,
            std::bind(&ObstacleAvoidanceNode::depthCallback, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&ObstacleAvoidanceNode::stateCallback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // Service Clients
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

        // TF2 Setup (Essential for Mapping)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize Global Map (Black = Empty, White = Obstacle)
        global_map_ = cv::Mat::zeros(GRID_SIZE, GRID_SIZE, CV_8UC1);
        
        control_timer_ = this->create_wall_timer(100ms, std::bind(&ObstacleAvoidanceNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), ">>> VOXEL MAPPING NODE STARTED <<<");
    }

private:
    // ROS Handles
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data
    cv::Mat depth_image_;
    cv::Mat global_map_; // The Occupancy Grid
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Twist cmd_vel_;

    // Logic
    FlightState flight_state_ = FlightState::WARMUP;
    AvoidanceMode avoidance_mode_ = AvoidanceMode::CRUISING;
    int state_step_ = 0;
    rclcpp::Time last_depth_time_;
    double min_obstacle_dist_ = 100.0;
    int strafe_dir_ = 1; 

    // Helper: Map Coordinate Conversion
    cv::Point2i worldToMap(double x, double y) {
        int c = (int)(x / GRID_RESOLUTION) + (GRID_SIZE / 2);
        int r = (int)(y / GRID_RESOLUTION) + (GRID_SIZE / 2); // Inverted Y in images usually, but we keep it simple
        return cv::Point2i(c, r);
    }

    // --- MAPPING FUNCTION ---
    void updateOccupancyMap()
    {
        if(depth_image_.empty()) return;

        // 1. Get Drone Position
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; // No transform yet
        }

        double drone_x = t.transform.translation.x;
        double drone_y = t.transform.translation.y;
        double drone_yaw = 0.0; 
        
        // Extract Yaw from Quaternion
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(drone_yaw, drone_yaw, drone_yaw); // Hack: variable reuse, getting Roll/Pitch/Yaw
        // Actually correct way:
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, drone_yaw);

        // 2. Project Depth Points to Map
        // We only scan a horizontal line from the depth image to save CPU
        int row = depth_image_.rows / 2;
        for(int col = 0; col < depth_image_.cols; col+=4) 
        {
            float dist = depth_image_.at<float>(row, col);
            if(dist < 0.2 || dist > 6.0) continue; // Ignore far noise

            // Calculate Angle relative to drone
            float fov_rad = 1.518; // ~87 degrees
            float angle_offset = ((float)col / depth_image_.cols - 0.5f) * fov_rad;
            float total_angle = drone_yaw + angle_offset;

            // Polar -> Cartesian (World)
            double obs_x = drone_x + dist * cos(total_angle);
            double obs_y = drone_y + dist * sin(total_angle);

            // Mark in Map
            cv::Point2i pt = worldToMap(obs_x, obs_y);
            if(pt.x >= 0 && pt.x < GRID_SIZE && pt.y >= 0 && pt.y < GRID_SIZE) {
                cv::circle(global_map_, pt, 2, cv::Scalar(255), -1); // 2px radius obstacle
            }
        }
    }

    // --- CHECK MAP FUNCTION ---
    // Returns TRUE if the area next to the drone is clear in the map
    bool isSideClear(double drone_x, double drone_y)
    {
        // Check a box area next to the drone (towards the original path)
        // If we strafed LEFT, we check RIGHT.
        double check_dist = 1.5; 
        double check_x = drone_x; 
        double check_y = drone_y - (strafe_dir_ * check_dist); // Look towards original path

        cv::Point2i pt = worldToMap(check_x, check_y);
        
        // Check a 5x5 pixel area around that point
        int search_rad = 5;
        for(int r = pt.y - search_rad; r <= pt.y + search_rad; r++) {
            for(int c = pt.x - search_rad; c <= pt.x + search_rad; c++) {
                if(r>=0 && r<GRID_SIZE && c>=0 && c<GRID_SIZE) {
                    if(global_map_.at<uchar>(r, c) > 128) return false; // Found an obstacle!
                }
            }
        }
        return true;
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image_ = cv_ptr->image;
            last_depth_time_ = this->now();
            
            // Simple safety distance check
            float min_dist = 100.0;
            int r = depth_image_.rows / 2;
            for(int c=0; c<depth_image_.cols; ++c) {
                float val = depth_image_.at<float>(r, c);
                if(val > 0.2 && val < min_dist) min_dist = val;
            }
            min_obstacle_dist_ = min_dist;

        } catch (...) {}
    }

    // --- MAIN LOOP ---
    void controlLoop()
    {
        updateOccupancyMap(); // <--- ALWAYS UPDATE MAP

        switch(flight_state_) {
            case FlightState::WARMUP: 
                if(current_state_.connected && ++state_step_ > 20) flight_state_ = FlightState::GUIDED;
                break;
            case FlightState::GUIDED: 
                if(current_state_.mode != "GUIDED" && state_step_++ % 10 == 0) {
                     auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                     req->custom_mode = "GUIDED";
                     mode_client_->async_send_request(req);
                } else if(current_state_.mode == "GUIDED") flight_state_ = FlightState::ARMING;
                break;
            case FlightState::ARMING: 
                if(!current_state_.armed && state_step_++ % 10 == 0) {
                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;
                    arm_client_->async_send_request(req);
                } else if(current_state_.armed) { state_step_=0; flight_state_ = FlightState::TAKEOFF; }
                break;
            case FlightState::TAKEOFF: 
                if(state_step_++ == 0) {
                    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    req->altitude = 2.0;
                    takeoff_client_->async_send_request(req);
                }
                if(state_step_ > 100) flight_state_ = FlightState::MISSION; // Simple timer wait
                break;
            case FlightState::MISSION: handleMission(); break;
        }

        if(flight_state_ == FlightState::MISSION || flight_state_ == FlightState::TAKEOFF)
            vel_pub_->publish(cmd_vel_);
    }

    void handleMission()
    {
        if((this->now() - last_depth_time_).seconds() > 1.0) {
            cmd_vel_ = geometry_msgs::msg::Twist(); return;
        }

        // Need current position for clearing logic
        geometry_msgs::msg::TransformStamped t;
        try { t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero); } 
        catch (...) { return; }
        double drone_x = t.transform.translation.x;
        double drone_y = t.transform.translation.y;

        switch(avoidance_mode_)
        {
            case AvoidanceMode::CRUISING:
                cmd_vel_.linear.x = 0.8; cmd_vel_.linear.y = 0.0; // Moving X-forward
                if(min_obstacle_dist_ < 3.0) {
                    RCLCPP_WARN(this->get_logger(), "Obstacle! Mapping and Strafing...");
                    avoidance_mode_ = AvoidanceMode::STRAFING;
                }
                break;

            case AvoidanceMode::STRAFING:
                cmd_vel_.linear.x = 0.0; cmd_vel_.linear.y = 0.6 * strafe_dir_; // Strafe Y
                if(min_obstacle_dist_ > 4.0) {
                    RCLCPP_INFO(this->get_logger(), "Visual Clear. Checking Map Memory...");
                    avoidance_mode_ = AvoidanceMode::MAPPING_CLEARANCE;
                }
                break;

            case AvoidanceMode::MAPPING_CLEARANCE:
                cmd_vel_.linear.x = 0.8; cmd_vel_.linear.y = 0.0; // Move forward
                
                // CRITICAL VOXEL CHECK:
                // Is the area next to us (where the truck should be) clear in the map?
                if(isSideClear(drone_x, drone_y)) {
                    RCLCPP_INFO(this->get_logger(), "Map Memory confirms path clear! Returning.");
                    avoidance_mode_ = AvoidanceMode::RETURNING;
                } else {
                     // The map remembers the truck is still there! Keep going forward.
                     if(state_step_ % 10 == 0) RCLCPP_INFO(this->get_logger(), "Map says: Truck still there...");
                }
                break;

            case AvoidanceMode::RETURNING:
                cmd_vel_.linear.x = 0.4; cmd_vel_.linear.y = -0.6 * strafe_dir_;
                // Simplified return logic (timer or distance based)
                if(min_obstacle_dist_ > 5.0) avoidance_mode_ = AvoidanceMode::CRUISING; 
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}