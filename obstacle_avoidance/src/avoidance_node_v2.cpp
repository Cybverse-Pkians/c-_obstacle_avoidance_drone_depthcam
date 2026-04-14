/*
 * obstacle_avoidance_node_v2.cpp — FRESH IMPLEMENTATION
 *
 * FLOW:
 * 1. Detect obstacle → go STRAIGHT initially
 * 2. Turn LEFT when lateral position is safe
 * 3. Continue STRAIGHT while checking right wall
 * 4. Use voxelization to remember wall (2D costmap)
 * 5. Go 5m extra STRAIGHT as safety offset
 * 6. Return to NORMAL path
 * 7. Calculate OBB dimensions and save to CSV (final only)
 */

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
#include <unordered_map>
#include <stack>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <set>

using namespace std::chrono_literals;

// ============================================================
//  CSV LOGGER
// ============================================================

class CSVLogger {
public:
  CSVLogger(const std::string& filename) : file_path_(filename) {
    std::ofstream file(file_path_, std::ios::trunc);
    if (file.is_open()) {
      file << "timestamp_ms,obstacle_id,rel_x_m,rel_y_m,rel_z_m,"
            << "length_m,breadth_m,height_m,"
            << "center_x_m,center_y_m,center_z_m,"
            << "confidence,voxel_count\n";
      file.close();
    }
  }

  void log(long long ts_ms, int obs_id, float rel_x, float rel_y, float rel_z,
           float len, float breadth, float height,
           float cx, float cy, float cz, float conf, int vox_cnt) {
    std::ofstream file(file_path_, std::ios::app);
    if (!file.is_open()) return;
    
    file << ts_ms << "," << obs_id << ","
         << std::fixed << std::setprecision(3)
         << rel_x << "," << rel_y << "," << rel_z << ","
         << len << "," << breadth << "," << height << ","
         << cx << "," << cy << "," << cz << ","
         << std::setprecision(2) << conf << "," << vox_cnt << "\n";
    file.close();
  }

private:
  std::string file_path_;
};

// ============================================================
//  2D VOXEL COSTMAP — Simplified to top-down view
// ============================================================

struct VoxelKey2D {
  int x, y;
  bool operator==(const VoxelKey2D& o) const { return x==o.x && y==o.y; }
};

namespace std {
  template<> struct hash<VoxelKey2D> {
    size_t operator()(const VoxelKey2D& k) const {
      size_t h = std::hash<int>()(k.x);
      h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };
}

struct Voxel2D {
  int hits{0};
  rclcpp::Time last_seen;
  float z_min{1e6f}, z_max{-1e6f};  // height range for this voxel
};

class CostmapND {
public:
  CostmapND(float resolution, float decay_s) : res_(resolution), decay_s_(decay_s) {}

  void add(float x, float y, float z, rclcpp::Time ts) {
    auto k = key(x, y);
    auto& v = grid_[k];
    v.hits = std::min(v.hits + 1, 50);
    v.last_seen = ts;
    v.z_min = std::min(v.z_min, z);
    v.z_max = std::max(v.z_max, z);
  }

  float confidence(float x, float y, rclcpp::Time now_t) const {
    auto it = grid_.find(key(x, y));
    if (it == grid_.end()) return 0.f;
    double age = (now_t - it->second.last_seen).seconds();
    float eff_decay = decay_s_ * (it->second.hits >= 10 ? 2.0f : 1.0f);
    return (float)std::exp(-age / eff_decay);
  }

  bool occupied(float x, float y, rclcpp::Time now_t) const {
    return confidence(x, y, now_t) > 0.3f;
  }

  // Get all occupied cells as clusters
  std::vector<std::vector<std::pair<float, float>>> getClusters(rclcpp::Time now_t, float min_conf = 0.3f) const {
    std::vector<std::vector<std::pair<float, float>>> clusters;
    std::set<VoxelKey2D> visited;
    
    for (const auto& entry : grid_) {
      if (visited.count(entry.first)) continue;
      if (confidence(entry.first.x * res_, entry.first.y * res_, now_t) < min_conf) continue;
      
      std::vector<std::pair<float, float>> cluster;
      std::vector<VoxelKey2D> to_visit;
      to_visit.push_back(entry.first);
      
      while (!to_visit.empty()) {
        auto key = to_visit.back();
        to_visit.pop_back();
        
        if (visited.count(key)) continue;
        visited.insert(key);
        
        if (confidence(key.x * res_, key.y * res_, now_t) < min_conf) continue;
        
        cluster.push_back({key.x * res_, key.y * res_});
        
        // Check 8-neighbors
        for (int dx = -1; dx <= 1; dx++) {
          for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            VoxelKey2D nkey = {key.x + dx, key.y + dy};
            if (!visited.count(nkey)) to_visit.push_back(nkey);
          }
        }
      }
      
      if (!cluster.empty()) clusters.push_back(cluster);
    }
    return clusters;
  }

  void cleanup(rclcpp::Time now_t) {
    for (auto it = grid_.begin(); it != grid_.end(); )
      if ((now_t - it->second.last_seen).seconds() > decay_s_ * 3.5f)
        it = grid_.erase(it);
      else ++it;
  }

  size_t size() const { return grid_.size(); }

private:
  VoxelKey2D key(float x, float y) const {
    return {(int)std::floor(x / res_), (int)std::floor(y / res_)};
  }
  
  std::unordered_map<VoxelKey2D, Voxel2D> grid_;
  float res_, decay_s_;
};

// ============================================================
//  OBB (Oriented Bounding Box) CALCULATION
// ============================================================

struct OBB {
  float length{0.f};
  float breadth{0.f};
  float height{0.f};
  float center_x{0.f}, center_y{0.f}, center_z{0.f};
  int voxel_count{0};
  float confidence{0.f};
  bool valid{false};
};

OBB computeOBB(const std::vector<std::pair<float, float>>& points_2d) {
  OBB obb;
  if (points_2d.size() < 3) return obb;
  
  // Compute centroid
  float cx = 0.f, cy = 0.f;
  for (const auto& p : points_2d) {
    cx += p.first;
    cy += p.second;
  }
  cx /= points_2d.size();
  cy /= points_2d.size();
  
  // PCA: Compute covariance matrix
  float cov_xx = 0.f, cov_yy = 0.f, cov_xy = 0.f;
  for (const auto& p : points_2d) {
    float dx = p.first - cx;
    float dy = p.second - cy;
    cov_xx += dx * dx;
    cov_yy += dy * dy;
    cov_xy += dx * dy;
  }
  cov_xx /= points_2d.size();
  cov_yy /= points_2d.size();
  cov_xy /= points_2d.size();
  
  // Eigenvalues & eigenvectors
  float trace = cov_xx + cov_yy;
  float det = cov_xx * cov_yy - cov_xy * cov_xy;
  float lambda1 = trace / 2.f + std::sqrt(trace * trace / 4.f - det);
  float lambda2 = trace / 2.f - std::sqrt(trace * trace / 4.f - det);
  
  if (std::abs(lambda1 - lambda2) < 1e-6f) {
    // Degenerate: Use AABB
    float x_min = points_2d[0].first, x_max = x_min;
    float y_min = points_2d[0].second, y_max = y_min;
    for (const auto& p : points_2d) {
      x_min = std::min(x_min, p.first);
      x_max = std::max(x_max, p.first);
      y_min = std::min(y_min, p.second);
      y_max = std::max(y_max, p.second);
    }
    obb.length = x_max - x_min;
    obb.breadth = y_max - y_min;
    obb.valid = true;
    obb.voxel_count = points_2d.size();
    return obb;
  }
  
  // Primary eigenvector (direction of max spread)
  float vx = cov_xy;
  float vy = lambda1 - cov_xx;
  float norm = std::sqrt(vx * vx + vy * vy);
  if (norm > 1e-6f) {
    vx /= norm;
    vy /= norm;
  } else {
    vx = 1.f; vy = 0.f;
  }
  
  float ux = -vy;  // orthogonal direction
  float uy = vx;
  
  // Project all points onto these axes
  float d1_min = 1e6f, d1_max = -1e6f;
  float d2_min = 1e6f, d2_max = -1e6f;
  
  for (const auto& p : points_2d) {
    float px = p.first - cx;
    float py = p.second - cy;
    float d1 = px * vx + py * vy;
    float d2 = px * ux + py * uy;
    d1_min = std::min(d1_min, d1);
    d1_max = std::max(d1_max, d1);
    d2_min = std::min(d2_min, d2);
    d2_max = std::max(d2_max, d2);
  }
  
  float len1 = d1_max - d1_min;
  float len2 = d2_max - d2_min;
  
  obb.length = std::max(len1, len2);
  obb.breadth = std::min(len1, len2);
  obb.center_x = cx;
  obb.center_y = cy;
  obb.voxel_count = points_2d.size();
  obb.valid = true;
  
  return obb;
}

// ============================================================
//  ENUMS & STRUCTURES
// ============================================================

enum class FlightState { WARMUP, GUIDED, ARMING, TAKEOFF, MISSION };

enum class AvoidanceMode {
  CRUISING,
  STRAIGHT_FORWARD,     // Initial forward when obstacle detected
  TURN_LEFT,            // Turn left until safe
  WALL_FOLLOWING,       // Go straight while wall on right exists
  EXTRA_STRAIGHT_5M,    // 5m safety offset
  RETURNING             // Return to original path
};

struct ObstacleContext {
  int obstacle_id{-1};
  float forward_dist{0.f};
  float lateral_offset{0.f};
  AvoidanceMode mode{AvoidanceMode::STRAIGHT_FORWARD};
  int clear_cnt{0};
  OBB final_measurement;
};

struct SceneAnalysis {
  float front_min{200.f};
  float right_min{200.f};
};

// ============================================================
//  MAIN NODE
// ============================================================

class SideObstacleAvoidance : public rclcpp::Node {
public:
  SideObstacleAvoidance()
    : Node("side_obstacle_avoidance"), 
      costmap_(0.1f, 60.f),  // 0.1m cells, 60s decay
      csv_logger_("/home/avengers/obstacle_data.csv")
  {
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      std::bind(&SideObstacleAvoidance::depthCb, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&SideObstacleAvoidance::stateCb, this, std::placeholders::_1));
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort().durability_volatile();
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos,
      std::bind(&SideObstacleAvoidance::posCb, this, std::placeholders::_1));

    vel_pub_  = create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arm_cl_   = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_cl_  = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    tkoff_cl_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    ctrl_tmr_  = create_wall_timer(100ms, std::bind(&SideObstacleAvoidance::controlLoop, this));
    cmd_tmr_   = create_wall_timer(50ms,  std::bind(&SideObstacleAvoidance::publishCmd, this));
    clean_tmr_ = create_wall_timer(5s,    [this]{ costmap_.cleanup(now()); });

    fstate_ = FlightState::WARMUP;
    last_depth_t_ = last_upd_t_ = now();

    RCLCPP_INFO(get_logger(), "═════════════════════════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  Obstacle Avoidance v2 — FRESH IMPLEMENTATION");
    RCLCPP_INFO(get_logger(), "  ✓ Straight → Left → Wall Follow → 5M → Return");
    RCLCPP_INFO(get_logger(), "  ✓ 2D Voxel Costmap (top-down view)");
    RCLCPP_INFO(get_logger(), "  ✓ OBB dimension measurement (PCA)");
    RCLCPP_INFO(get_logger(), "  ✓ Final CSV logging only");
    RCLCPP_INFO(get_logger(), "═════════════════════════════════════════════════════════");
  }

private:
  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          vel_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr         arm_cl_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr             mode_cl_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr          tkoff_cl_;
  rclcpp::TimerBase::SharedPtr ctrl_tmr_, cmd_tmr_, clean_tmr_;

  // Flight state
  FlightState fstate_;
  int warm_ctr_{0}, state_ctr_{0};
  int next_obstacle_id_{0};

  cv::Mat      depth_img_;
  rclcpp::Time last_depth_t_, last_upd_t_;
  mavros_msgs::msg::State    mav_state_;
  geometry_msgs::msg::Twist  cur_cmd_;

  // Position
  double dx_{0}, dy_{0}, dyaw_{0};
  double alt_{0}, base_alt_{2.0}, tkoff_alt_{0};
  double traveled_{0};

  // Scene and stack
  SceneAnalysis scene_;
  std::stack<ObstacleContext> stk_;
  static constexpr int kMaxStackDepth = 4;

  CostmapND costmap_;
  CSVLogger csv_logger_;

  // TUNING PARAMETERS
  static constexpr float kFwdSpd       = 1.0f;
  static constexpr float kStrafeSpd    = 0.6f;
  static constexpr float kTriggerDist  = 3.5f;  // Front obstacle distance
  static constexpr float kWallGoneDist = 7.0f;  // Wall gone from FOV
  static constexpr float kLeftTurnDist = 2.5f;  // Lateral offset before going straight
  static constexpr float kExtraStraight= 5.0f;  // 5m safety offset
  static constexpr float kReturnDist   = 2.5f;  // Return to original path

  // Camera intrinsics
  static constexpr float kFx = 320.f;
  static constexpr float kFy = 320.f;

  // ─────────────────────────────────────────────────────────────────────────
  //  CALLBACKS
  // ─────────────────────────────────────────────────────────────────────────

  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto p = cv_bridge::toCvCopy(msg);
      if (msg->encoding == "32FC1")       depth_img_ = p->image.clone();
      else if (msg->encoding == "16UC1")  p->image.convertTo(depth_img_, CV_32F, 0.001f);
      last_depth_t_ = now();
      if (fstate_ == FlightState::MISSION) {
        updateCostmap();
        analyzeScene();
      }
    } catch (...) { RCLCPP_ERROR(get_logger(), "Depth decode failed"); }
  }

  void stateCb(const mavros_msgs::msg::State::SharedPtr m) { mav_state_ = *m; }

  void posCb(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    alt_ = m->pose.position.z;
    dx_  = m->pose.position.x;
    dy_  = m->pose.position.y;
    double qw = m->pose.orientation.w, qx = m->pose.orientation.x,
           qy = m->pose.orientation.y, qz = m->pose.orientation.z;
    dyaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  COSTMAP UPDATE — Project depth to 2D world grid
  // ─────────────────────────────────────────────────────────────────────────

  void updateCostmap() {
    if (depth_img_.empty()) return;
    int rows = depth_img_.rows, cols = depth_img_.cols;
    float cx = cols / 2.f, cy = rows / 2.f;
    rclcpp::Time ts = now();
    float csy = std::cos(dyaw_), sny = std::sin(dyaw_);
    
    for (int r = 0; r < rows; r += 4) {
      for (int c = 0; c < cols; c += 4) {
        float d = depth_img_.at<float>(r, c);
        if (d < 0.25f || d > 12.f) continue;

        // Depth camera frame
        float xb = d;
        float yb = -(c - cx) * d / kFx;
        float zb = -(r - cy) * d / kFy;

        // World frame
        float xw = dx_ + (xb * csy - yb * sny);
        float yw = dy_ + (xb * sny + yb * csy);
        float zw = alt_ + zb;

        // Only add if roughly at drone altitude
        if (std::abs(zw - alt_) < 1.5f && zw > 0.2f) {
          costmap_.add(xw, yw, zw, ts);
        }
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  SCENE ANALYSIS
  // ─────────────────────────────────────────────────────────────────────────

  void analyzeScene() {
    if (depth_img_.empty()) return;
    SceneAnalysis s;
    int rows = depth_img_.rows, cols = depth_img_.cols;
    int r0 = (int)(rows * 0.15f), r1 = (int)(rows * 0.55f);

    s.front_min = bandMin(r0, r1, (int)(cols*0.35f), (int)(cols*0.65f));
    s.right_min = bandMin(r0, r1, (int)(cols*0.78f),  cols);

    scene_ = s;
  }

  float bandMin(int r0, int r1, int c0, int c1) const {
    if (depth_img_.empty()) return 200.f;
    r0=std::max(0,r0); r1=std::min(depth_img_.rows,r1);
    c0=std::max(0,c0); c1=std::min(depth_img_.cols,c1);
    float cy = depth_img_.rows / 2.f;
    float mn = 200.f; int valid = 0;
    for (int r=r0;r<r1;r++) for (int c=c0;c<c1;c++) {
      float d = depth_img_.at<float>(r,c);
      if (d < 0.25f || d > 12.f) continue;
      float ph = alt_ - (((float)r - cy) * d / kFy);
      if (std::abs(ph - alt_) < 1.0f && ph > 0.2f) { if (d < mn) mn = d; valid++; }
    }
    return (valid < 5) ? 200.f : mn;
  }

  bool isNewObstacleDetected() const {
    return scene_.front_min < kTriggerDist;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  CONTROL LOOP
  // ─────────────────────────────────────────────────────────────────────────

  void controlLoop() {
    switch (fstate_) {
      case FlightState::WARMUP:  warmup();    break;
      case FlightState::GUIDED:  setGuided(); break;
      case FlightState::ARMING:  doArm();     break;
      case FlightState::TAKEOFF: doTakeoff(); break;
      case FlightState::MISSION: mission();   break;
    }
  }

  void warmup() {
    cur_cmd_ = geometry_msgs::msg::Twist();
    warm_ctr_++;
    if (!mav_state_.connected) { warm_ctr_=0; return; }
    if (warm_ctr_ % 10 == 0) RCLCPP_INFO(get_logger(), "Warmup... connected");
    if (warm_ctr_ > 40) { state_ctr_=0; fstate_=FlightState::GUIDED;
      RCLCPP_INFO(get_logger(), "→ GUIDED"); }
  }

  void setGuided() {
    state_ctr_++;
    if (state_ctr_ == 1) {
      if (!mode_cl_->wait_for_service(1s)) { state_ctr_=0; return; }
      auto r=std::make_shared<mavros_msgs::srv::SetMode::Request>();
      r->custom_mode="GUIDED"; mode_cl_->async_send_request(r);
    }
    if (mav_state_.mode=="GUIDED") {
      state_ctr_=0; fstate_=FlightState::ARMING;
      RCLCPP_INFO(get_logger(), "GUIDED → ARMING");
    } else if (state_ctr_>50) state_ctr_=0;
  }

  void doArm() {
    state_ctr_++;
    if (state_ctr_ == 1) {
      if (!arm_cl_->wait_for_service(1s)) { state_ctr_=0; return; }
      auto r=std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      r->value=true; arm_cl_->async_send_request(r);
    }
    if (mav_state_.armed) {
      tkoff_alt_=alt_; state_ctr_=0; fstate_=FlightState::TAKEOFF;
      RCLCPP_INFO(get_logger(), "Armed → TAKEOFF");
    } else if (state_ctr_>50) state_ctr_=0;
  }

  void doTakeoff() {
    double rel = alt_ - tkoff_alt_;
    if (rel < 0.3) {
      if (state_ctr_ % 50 == 0) {
        if (!tkoff_cl_->wait_for_service(1s)) return;
        auto r=std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        r->altitude = base_alt_; tkoff_cl_->async_send_request(r);
      }
      state_ctr_++; return;
    }
    geometry_msgs::msg::Twist cmd;
    double err = base_alt_ - rel;
    cmd.linear.z = (err>0.5)?0.7:(err>0.1)?0.2:0.0;
    cur_cmd_ = cmd; state_ctr_++;
    if (rel >= (base_alt_ - 0.2)) {
      RCLCPP_INFO(get_logger(), "Takeoff → MISSION");
      last_upd_t_ = now(); traveled_ = 0.0;
      while (!stk_.empty()) stk_.pop();
      fstate_ = FlightState::MISSION;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  MISSION
  // ─────────────────────────────────────────────────────────────────────────

  void mission() {
    if (depth_img_.empty() || (now()-last_depth_t_).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"No depth!");
      stopDrone(); return;
    }
    rclcpp::Time t = now();
    double dt = (t - last_upd_t_).seconds();
    last_upd_t_ = t;

    processAvoidance(dt);
    maintainAlt();

    traveled_ += (std::abs(cur_cmd_.linear.x) + std::abs(cur_cmd_.linear.y)) * dt;

    std::string ms = "CRUISING";
    if (!stk_.empty()) {
      auto& c = stk_.top();
      switch (c.mode) {
        case AvoidanceMode::STRAIGHT_FORWARD: ms = "STRAIGHT_FWD"; break;
        case AvoidanceMode::TURN_LEFT:        ms = "LEFT_TURN";   break;
        case AvoidanceMode::WALL_FOLLOWING:   ms = "WALL_FOLLOW"; break;
        case AvoidanceMode::EXTRA_STRAIGHT_5M: ms = "EXTRA_5M";   break;
        case AvoidanceMode::RETURNING:        ms = "RETURN";      break;
        default: break;
      }
      ms += "[" + std::to_string(stk_.size()) + "]";
    }

    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "Dist:%.1f/300m | %s | Vx:%.2f Vy:%.2f | F:%.2f R:%.2f | Costmap:%zu",
      traveled_, ms.c_str(),
      cur_cmd_.linear.x, cur_cmd_.linear.y,
      scene_.front_min, scene_.right_min,
      costmap_.size());

    if (traveled_ >= 300.0) {
      stopDrone();
      RCLCPP_INFO(get_logger(), "===== MISSION COMPLETE =====");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  AVOIDANCE STATE MACHINE
  // ─────────────────────────────────────────────────────────────────────────

  void processAvoidance(double dt) {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = cmd.linear.z = 0.0;

    // ===== CRUISING =====
    if (stk_.empty()) {
      if (isNewObstacleDetected()) {
        RCLCPP_INFO(get_logger(),
          "╔════════════════════════════════╗\n"
          "║ OBSTACLE detected at %.2fm    ║\n"
          "╚════════════════════════════════╝", scene_.front_min);
        ObstacleContext ctx;
        ctx.obstacle_id = next_obstacle_id_++;
        stk_.push(ctx);
      } else {
        cmd.linear.x = 0.0;
        cmd.linear.y = kFwdSpd;
      }
      cur_cmd_ = cmd; return;
    }

    ObstacleContext& ctx = stk_.top();

    // State machine
    switch (ctx.mode) {

    // ==================================================================
    //  STRAIGHT_FORWARD — Go straight initially
    // ==================================================================
    case AvoidanceMode::STRAIGHT_FORWARD:
    {
      cmd.linear.x = 0.f;
      cmd.linear.y = kFwdSpd;
      ctx.forward_dist += kFwdSpd * dt;

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "⬇ STRAIGHT_FWD %.2f | front=%.2f",
        ctx.forward_dist, scene_.front_min);

      // Still seeing obstacle at left? Switch to LEFT_TURN
      if (scene_.front_min < kTriggerDist && ctx.forward_dist > 0.5f) {
        RCLCPP_INFO(get_logger(), "→ Turning LEFT");
        ctx.mode = AvoidanceMode::TURN_LEFT;
      }
      break;
    }

    // ==================================================================
    //  TURN_LEFT — Shift left until safe
    // ==================================================================
    case AvoidanceMode::TURN_LEFT:
    {
      cmd.linear.x = kStrafeSpd;  // LEFT = +y
      cmd.linear.y = 0.f;
      ctx.lateral_offset += kStrafeSpd * dt;

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "← LEFT | lat=%.2f/%.2fm | front=%.2f",
        ctx.lateral_offset, kLeftTurnDist, scene_.front_min);

      // Safe when obstacle out of FOV AND lateral distance reached
      bool obs_out = (scene_.front_min >= kTriggerDist);
      bool lat_done = (ctx.lateral_offset >= kLeftTurnDist);

      if (obs_out && lat_done) {
        RCLCPP_INFO(get_logger(), "✔ Left turn complete → WALL_FOLLOWING");
        ctx.mode = AvoidanceMode::WALL_FOLLOWING;
      }
      break;
    }

    // ==================================================================
    //  WALL_FOLLOWING — Go straight while right wall visible
    // ==================================================================
    case AvoidanceMode::WALL_FOLLOWING:
    {
      cmd.linear.x = 0.f;
      cmd.linear.y = kFwdSpd;
      ctx.forward_dist += kFwdSpd * dt;

      bool cam_sees_wall = (scene_.right_min < kWallGoneDist);
      bool vox_sees_wall = costmap_.occupied(dx_ + 3.0f, dy_ - 2.0f, now());  // Check right-front

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "⬆ WALL_FOLLOW fwd=%.2f | cam=%s vox=%s | right=%.2f",
        ctx.forward_dist, cam_sees_wall ? "Y" : "N", vox_sees_wall ? "Y" : "N",
        scene_.right_min);

      // Exit when both camera AND voxels say wall is gone
      if (!cam_sees_wall && !vox_sees_wall) {
        RCLCPP_INFO(get_logger(), "✔ Wall gone → EXTRA_STRAIGHT_5M");
        ctx.forward_dist = 0.f;
        ctx.mode = AvoidanceMode::EXTRA_STRAIGHT_5M;
      }
      break;
    }

    // ==================================================================
    //  EXTRA_STRAIGHT_5M — 5m safety offset
    // ==================================================================
    case AvoidanceMode::EXTRA_STRAIGHT_5M:
    {
      cmd.linear.x = 0.f;
      cmd.linear.y = kFwdSpd;
      ctx.forward_dist += kFwdSpd * dt;

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "⬆ EXTRA_5M %.2f/%.2fm",
        ctx.forward_dist, kExtraStraight);

      if (ctx.forward_dist >= kExtraStraight) {
        RCLCPP_INFO(get_logger(), "✔ 5M offset done → RETURNING");
        ctx.lateral_offset = 0.f;
        ctx.clear_cnt = 0;
        ctx.mode = AvoidanceMode::RETURNING;
      }
      break;
    }

    // ==================================================================
    //  RETURNING — Return to original path
    // ==================================================================
    case AvoidanceMode::RETURNING:
    {
      bool still_front = (scene_.front_min < kTriggerDist);
      bool still_right = (scene_.right_min < kWallGoneDist);

      if (still_front || still_right) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
          "⚠ RETURN: blocked (front=%.2f, right=%.2f)", 
          scene_.front_min, scene_.right_min);
        cmd.linear.x=0.f; cmd.linear.y=kFwdSpd*0.6f;
        ctx.clear_cnt=0; break;
      }

      ctx.clear_cnt++;
      if (ctx.clear_cnt < 10) {
        cmd.linear.x=0.f; cmd.linear.y=kFwdSpd*0.5f; break;
      }

      // Start returning
      cmd.linear.x = -kStrafeSpd;  // RIGHT = -y to return
      cmd.linear.y = kFwdSpd * 0.4f;
      ctx.lateral_offset += kStrafeSpd * dt;

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "→ RETURN RIGHT | %.2f/%.2fm",
        ctx.lateral_offset, kLeftTurnDist);

      if (ctx.lateral_offset >= kLeftTurnDist) {
        RCLCPP_INFO(get_logger(),
          "╔══════════════════════════════════╗\n"
          "║ ✓ Returned to pre-obstacle path ║\n"
          "╚══════════════════════════════════╝");
        
        // Extract and log final measurement
        auto clusters = costmap_.getClusters(now(), 0.4f);
        if (!clusters.empty()) {
          OBB obb = computeOBB(clusters[0]);
          if (obb.valid) {
            float rel_x = obb.center_x - dx_;
            float rel_y = obb.center_y - dy_;
            long long ts_ms = (long long)(now().seconds() * 1000.0);
            csv_logger_.log(ts_ms, ctx.obstacle_id, rel_x, rel_y, 0.f,
                           obb.length, obb.breadth, 0.f,
                           obb.center_x, obb.center_y, alt_,
                           0.9f, clusters[0].size());
            RCLCPP_INFO(get_logger(),
              "📊 FINAL OBS %d: L=%.2fm W=%.2fm | Voxels=%zu",
              ctx.obstacle_id, obb.length, obb.breadth, clusters[0].size());
          }
        }
        
        stk_.pop();
        if (stk_.empty()) RCLCPP_INFO(get_logger(), "✓ Stack empty — CRUISING");
      }
      break;
    }

    default: break;
    }

    cur_cmd_ = cmd;
  }

  void maintainAlt() {
    double err = base_alt_ - (alt_ - tkoff_alt_);
    cur_cmd_.linear.z = (std::abs(err) > 0.1)
                        ? std::clamp(0.5 * err, -0.5, 0.5) : 0.0;
  }

  void stopDrone() {
    cur_cmd_ = geometry_msgs::msg::Twist();
    maintainAlt();
  }

  void publishCmd() {
    if (fstate_==FlightState::TAKEOFF && (alt_-tkoff_alt_)<0.3) return;
    if (fstate_==FlightState::MISSION || fstate_==FlightState::TAKEOFF)
      vel_pub_->publish(cur_cmd_);
  }
};

// ============================================================
//  ENTRY POINT
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SideObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
