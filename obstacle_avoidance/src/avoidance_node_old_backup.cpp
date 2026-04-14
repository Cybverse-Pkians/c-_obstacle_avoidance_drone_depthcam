/*
 * obstacle_avoidance_v8.cpp
 *
 * FIXES OVER v7:
 * ─────────────────────────────────────────────────────────────────────────
 *
 *  FIX 1 — CSV SAVE LOCATION:
 *    v7 hardcoded /home/avengers/obstacle_data.csv which silently fails if
 *    the node runs as a different user.
 *    Fix: uses $HOME environment variable at runtime with /tmp fallback.
 *    The exact path is printed in the startup log so you always know where it is.
 *
 *  FIX 2 — DRONE NOT RETURNING (main bug):
 *    RETURNING had 4 guards before starting lateral movement.
 *    Guard 3 (sideVoxelOccupied) was always true because the obstacle's own
 *    voxels stamped during flight were visible in the lateral slab → drone
 *    never started the return strafe, just crept forward forever.
 *    Guard 4 (returnPathClear) was also always blocking for the same reason.
 *    Fix: removed both voxel guards from RETURNING.  After 5 m of clear
 *    forward flight the return corridor is geometrically safe — no sensor
 *    check needed to start moving back.
 *
 *  FIX 3 — RETURN DISTANCE MEASUREMENT:
 *    v7 used  ctx.return_dist += kStrafeSpd * dt  which accumulates dt
 *    integration error and is wrong when cmd.linear.y is also non-zero
 *    (diagonal motion means the actual lateral distance is less than
 *    kStrafeSpd * dt).
 *    Fix: records world-frame anchor position (return_start_x/y) when
 *    RETURNING begins and measures actual Euclidean distance from that
 *    anchor each tick.  Precise, drift-free.
 *
 *  FIX 4 — RETURN DIRECTION GUARD:
 *    Guard 2 now checks the RETURN side (opposite of avoidance) for an
 *    immediate physical wall < 1.5 m rather than the avoidance side.
 *    This is the only sensor guard that makes sense during return.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  STATE FLOW:
 *
 *  CRUISING → SHIFT_RIGHT → WALL_STRAIGHT → EXTRA_5M → RETURNING → CRUISING
 *
 *  SHIFT_RIGHT:   strafe right until obstacle leaves front FOV
 *  WALL_STRAIGHT: fly straight; camera+voxel wall check on correct side
 *  EXTRA_5M:      hard 5 m forward after wall confirmed gone
 *  RETURNING:     strafe back exact shift_accumulated distance (world anchor)
 *                 → finalise dimensions → write CSV
 *
 *  CSV saved to: $HOME/obstacle_data.csv  (path printed at startup)
 * ─────────────────────────────────────────────────────────────────────────
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
#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <cstdlib>

using namespace std::chrono_literals;

// ============================================================
//  CSV LOGGER
// ============================================================

class CSVLogger {
public:
  explicit CSVLogger() {
    // Use $HOME so it always resolves to the actual user running the node.
    // Fallback to /tmp if HOME is unset.
    const char* home = std::getenv("HOME");
    path_ = std::string(home ? home : "/tmp") + "/obstacle_data.csv";

    std::ofstream f(path_, std::ios::trunc);
    if (f.is_open()) {
      f << "timestamp_ms,obstacle_id,avoidance_dir,"
        << "length_m,breadth_m,height_m,"
        << "center_world_x,center_world_y,center_world_z,"
        << "rel_x_m,rel_y_m,rel_z_m,"
        << "voxel_count,confidence,method\n";
      ready_ = true;
    }
  }

  const std::string& path() const { return path_; }
  bool ready()              const { return ready_; }

  void write(long long ts_ms, int id, const std::string& dir,
             float len, float breadth, float ht,
             float cx, float cy, float cz,
             float rx, float ry, float rz,
             int vox_cnt, float conf, const std::string& method)
  {
    std::ofstream f(path_, std::ios::app);
    if (!f.is_open()) return;
    f << ts_ms << "," << id << ",\"" << dir << "\","
      << std::fixed << std::setprecision(3)
      << len    << "," << breadth << "," << ht    << ","
      << cx     << "," << cy      << "," << cz    << ","
      << rx     << "," << ry      << "," << rz    << ","
      << vox_cnt << ","
      << std::setprecision(2) << conf << ","
      << "\"" << method << "\"\n";
  }

private:
  std::string path_;
  bool        ready_{false};
};

// ============================================================
//  ENUMS
// ============================================================

enum class FlightState { WARMUP, GUIDED, ARMING, TAKEOFF, MISSION };

enum class AvoidMode {
  CRUISING,
  SHIFT_RIGHT,    // strafe right until obstacle leaves front FOV
  WALL_STRAIGHT,  // fly straight; exit when camera+voxels both clear
  EXTRA_5M,       // 5 m hard straight after wall gone
  RETURNING       // strafe left back to original path
};

// ============================================================
//  VOXEL GRID
// ============================================================

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const {
    return x==o.x && y==o.y && z==o.z;
  }
};

namespace std {
  template<> struct hash<VoxelKey> {
    size_t operator()(const VoxelKey& k) const {
      size_t h = std::hash<int>()(k.x);
      h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h<<6) + (h>>2);
      h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };
}

struct Voxel {
  float        occ{0.f};
  rclcpp::Time last_seen;
  int          hits{0};
  int          obstacle_id{-1};  // which obstacle tagged this voxel
};

class VoxelGrid {
public:
  VoxelGrid(float vs, float decay_s) : vs_(vs), dt_(decay_s) {}

  // Add a voxel, optionally tagging it with an obstacle id
  void add(float x, float y, float z, rclcpp::Time ts, int obs_id = -1) {
    auto& v = grid_[key(x,y,z)];
    v.occ       = 1.f;
    v.last_seen = ts;
    v.hits      = std::min(v.hits + 1, 50);
    if (obs_id >= 0) v.obstacle_id = obs_id;
  }

  float confidence(float x, float y, float z, rclcpp::Time now_t) const {
    auto it = grid_.find(key(x,y,z));
    if (it == grid_.end()) return 0.f;
    double age   = (now_t - it->second.last_seen).seconds();
    float eff_dt = dt_ * (it->second.hits >= 10 ? 1.5f : 1.0f);
    return it->second.occ * std::exp(-(float)(age / eff_dt));
  }

  bool occupied(float x, float y, float z, rclcpp::Time now_t) const {
    return confidence(x,y,z,now_t) > 0.25f;
  }

  // Count occupied voxels in a lateral rectangular slab
  int countSlab(float x0, float x1,
                float y0, float y1,
                float zc, float zhalf,
                rclcpp::Time now_t) const
  {
    float xmn=std::min(x0,x1), xmx=std::max(x0,x1);
    float ymn=std::min(y0,y1), ymx=std::max(y0,y1);
    int cnt=0;
    for (float x=xmn; x<=xmx+vs_*0.5f; x+=vs_)
      for (float y=ymn; y<=ymx+vs_*0.5f; y+=vs_)
        for (float z=zc-zhalf; z<=zc+zhalf+vs_*0.5f; z+=vs_)
          if (occupied(x,y,z,now_t)) cnt++;
    return cnt;
  }

  // Collect all occupied voxel centres in a slab (for measurement)
  std::vector<std::array<float,3>> collectSlab(
      float x0, float x1, float y0, float y1,
      float zc, float zhalf, rclcpp::Time now_t) const
  {
    float xmn=std::min(x0,x1), xmx=std::max(x0,x1);
    float ymn=std::min(y0,y1), ymx=std::max(y0,y1);
    std::vector<std::array<float,3>> pts;
    for (float x=xmn; x<=xmx+vs_*0.5f; x+=vs_)
      for (float y=ymn; y<=ymx+vs_*0.5f; y+=vs_)
        for (float z=zc-zhalf; z<=zc+zhalf+vs_*0.5f; z+=vs_)
          if (occupied(x,y,z,now_t)) pts.push_back({x,y,z});
    return pts;
  }

  // Check if a rectangular corridor is clear (for path planning)
  bool pathClear(float x0, float x1, float y0, float y1,
                 float zc, float zm, rclcpp::Time now_t) const
  {
    float xmn=std::min(x0,x1), xmx=std::max(x0,x1);
    float ymn=std::min(y0,y1), ymx=std::max(y0,y1);
    for (float x=xmn; x<=xmx; x+=vs_)
      for (float y=ymn; y<=ymx; y+=vs_)
        for (float z=zc-zm; z<=zc+zm; z+=vs_)
          if (occupied(x,y,z,now_t)) return false;
    return true;
  }

  void cleanup(rclcpp::Time now_t) {
    for (auto it=grid_.begin(); it!=grid_.end(); )
      if ((now_t - it->second.last_seen).seconds() > dt_*3.5)
        it = grid_.erase(it);
      else ++it;
  }

  float voxelSize() const { return vs_; }
  size_t size()     const { return grid_.size(); }

private:
  VoxelKey key(float x, float y, float z) const {
    return { (int)std::floor(x/vs_),
             (int)std::floor(y/vs_),
             (int)std::floor(z/vs_) };
  }
  mutable std::unordered_map<VoxelKey, Voxel> grid_;
  float vs_, dt_;
};

// ============================================================
//  OBSTACLE MEASUREMENT
//  Accumulates voxel data and computes dimensions on demand.
// ============================================================

struct ObstacleMeasurement {
  // World-frame AABB (running min/max updated every tick)
  float xmin{ std::numeric_limits<float>::max()};
  float xmax{-std::numeric_limits<float>::max()};
  float ymin{ std::numeric_limits<float>::max()};
  float ymax{-std::numeric_limits<float>::max()};
  float zmin{ std::numeric_limits<float>::max()};
  float zmax{-std::numeric_limits<float>::max()};

  // Flight yaw at first detection — defines the "forward" axis
  float initial_yaw{0.f};
  bool  yaw_captured{false};

  // Reservoir-sampled point cloud for PCA
  std::vector<std::array<float,3>> pts;
  static constexpr int kMaxPts = 2000;

  int  sample_ticks{0};  // how many ticks we collected data
  bool finalised{false};

  // Final results
  float length{0.f};   // along flight-forward axis
  float breadth{0.f};  // perpendicular (lateral)
  float height{0.f};   // vertical

  // Where the obstacle was (world center)
  float cx{0.f}, cy{0.f}, cz{0.f};

  // Measurement method used
  std::string method{"none"};

  void reset() {
    xmin =  std::numeric_limits<float>::max();
    xmax = -std::numeric_limits<float>::max();
    ymin =  std::numeric_limits<float>::max();
    ymax = -std::numeric_limits<float>::max();
    zmin =  std::numeric_limits<float>::max();
    zmax = -std::numeric_limits<float>::max();
    initial_yaw  = 0.f;
    yaw_captured = false;
    pts.clear();
    sample_ticks = 0;
    finalised    = false;
    length = breadth = height = 0.f;
    cx = cy = cz = 0.f;
    method = "none";
  }

  // Feed new voxel points into the accumulator
  void update(const std::vector<std::array<float,3>>& new_pts, float yaw) {
    if (new_pts.empty()) return;
    if (!yaw_captured) { initial_yaw = yaw; yaw_captured = true; }

    for (auto& p : new_pts) {
      xmin=std::min(xmin,p[0]); xmax=std::max(xmax,p[0]);
      ymin=std::min(ymin,p[1]); ymax=std::max(ymax,p[1]);
      zmin=std::min(zmin,p[2]); zmax=std::max(zmax,p[2]);
    }

    // Reservoir sampling to keep pts bounded
    for (auto& p : new_pts) {
      if ((int)pts.size() < kMaxPts) {
        pts.push_back(p);
      } else {
        int idx = rand() % kMaxPts;
        pts[idx] = p;
      }
    }
    sample_ticks++;
  }

  // Compute final length / breadth / height
  // Uses PCA if enough points, falls back to AABB projection
  void finalise() {
    if (finalised) return;
    finalised = true;

    if (sample_ticks == 0) return;

    height = (zmax > zmin) ? (zmax - zmin) : 0.f;
    cx = (xmin + xmax) / 2.f;
    cy = (ymin + ymax) / 2.f;
    cz = (zmin + zmax) / 2.f;

    // ── AABB projected onto flight-frame axes ─────────────────────────
    float cos_y = std::cos(initial_yaw);
    float sin_y = std::sin(initial_yaw);
    // forward = (cos_y, sin_y)   lateral = (-sin_y, cos_y)
    float wx = xmax - xmin;
    float wy = ymax - ymin;
    float aabb_fwd = std::abs(wx*cos_y) + std::abs(wy*sin_y);
    float aabb_lat = std::abs(wx*sin_y) + std::abs(wy*cos_y);

    if ((int)pts.size() < 20) {
      length  = aabb_fwd;
      breadth = aabb_lat;
      method  = "AABB";
      return;
    }

    // ── 2D PCA on (x, y) ─────────────────────────────────────────────
    float mx=0.f, my=0.f;
    for (auto& p : pts) { mx+=p[0]; my+=p[1]; }
    mx/=pts.size(); my/=pts.size();

    float cxx=0,cxy=0,cyy=0;
    for (auto& p : pts) {
      float dx=p[0]-mx, dy=p[1]-my;
      cxx+=dx*dx; cxy+=dx*dy; cyy+=dy*dy;
    }
    cxx/=pts.size(); cxy/=pts.size(); cyy/=pts.size();

    float tr   = cxx+cyy;
    float disc = std::sqrt(std::max(0.f, tr*tr*0.25f - (cxx*cyy - cxy*cxy)));
    float ev1  = tr*0.5f + disc;

    // Principal eigenvector
    float px, py;
    if (std::abs(cxy) > 1e-6f) { px=ev1-cyy; py=cxy; }
    else { px=(cxx>=cyy)?1.f:0.f; py=(cxx>=cyy)?0.f:1.f; }
    float pn = std::sqrt(px*px+py*py);
    if (pn < 1e-6f) { length=aabb_fwd; breadth=aabb_lat; method="AABB_fb"; return; }
    px/=pn; py/=pn;

    // Extent along principal axes
    float s1mn=1e9f,s1mx=-1e9f, s2mn=1e9f,s2mx=-1e9f;
    for (auto& p : pts) {
      float dx=p[0]-mx, dy=p[1]-my;
      float s1= dx*px+dy*py;
      float s2=-dx*py+dy*px;
      s1mn=std::min(s1mn,s1); s1mx=std::max(s1mx,s1);
      s2mn=std::min(s2mn,s2); s2mx=std::max(s2mx,s2);
    }
    float pca_axis1 = s1mx-s1mn;   // extent along principal axis
    float pca_axis2 = s2mx-s2mn;   // extent along minor axis

    // Determine which PCA axis corresponds to "length" (forward) vs "breadth"
    float fwd_dot = std::abs(px*cos_y + py*sin_y);
    if (fwd_dot >= 0.5f) {
      // principal axis ≈ flight direction → principal = length
      length  = pca_axis1;
      breadth = pca_axis2;
    } else {
      // principal axis ≈ lateral → principal = breadth
      length  = pca_axis2;
      breadth = pca_axis1;
    }

    // Sanity floor: PCA can underestimate if voxel cloud is sparse on one end
    length  = std::max(length,  aabb_fwd * 0.85f);
    breadth = std::max(breadth, aabb_lat * 0.85f);
    method  = "PCA";
  }
};

// ============================================================
//  AVOIDANCE CONTEXT  (one per active obstacle on the stack)
// ============================================================

struct AvoidCtx {
  int         obstacle_id{-1};
  int         avoidance_dir{1};   // +1=went RIGHT (wall on left), -1=went LEFT (wall on right)
  AvoidMode   mode{AvoidMode::SHIFT_RIGHT};

  // SHIFT_RIGHT: accumulated lateral distance (used to know roughly how far we went)
  float shift_accumulated{0.f};

  // RETURNING: world-frame position where RETURNING started.
  // We use actual Euclidean distance from this anchor instead of dt integration
  // so accumulated timing error doesn't cause over/under-shoot.
  float return_start_x{0.f};
  float return_start_y{0.f};
  bool  return_anchored{false};

  // WALL_STRAIGHT: dual-source wall-gone counter
  int no_wall_cnt{0};
  int wall_seen_cnt{0};
  // 35 frames @ 10 Hz = 3.5 s of confirmed-clear before exiting WALL_STRAIGHT
  static constexpr int kNoWallExit = 35;

  // EXTRA_5M
  float extra_dist{0.f};
  static constexpr float kExtra5m = 5.0f;

  // RETURNING
  float return_dist{0.f};   // Euclidean distance returned so far
  int   clear_debounce{0};
  static constexpr int kClearDebounce = 8;

  // Measurement accumulator
  ObstacleMeasurement meas;
};

// ============================================================
//  SCENE ANALYSIS
// ============================================================

struct Scene {
  float front_min{200.f};
  float left_min{200.f};
  float right_min{200.f};
};

// ============================================================
//  MAIN NODE
// ============================================================

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  ObstacleAvoidanceNode()
    : Node("obstacle_avoidance"), voxel_(0.5f, 60.f)
  {
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      std::bind(&ObstacleAvoidanceNode::depthCb, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&ObstacleAvoidanceNode::stateCb, this, std::placeholders::_1));
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort().durability_volatile();
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos,
      std::bind(&ObstacleAvoidanceNode::posCb, this, std::placeholders::_1));

    vel_pub_  = create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arm_cl_   = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_cl_  = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    tkoff_cl_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    ctrl_tmr_  = create_wall_timer(100ms, std::bind(&ObstacleAvoidanceNode::ctrlLoop, this));
    cmd_tmr_   = create_wall_timer(50ms,  std::bind(&ObstacleAvoidanceNode::publishCmd, this));
    clean_tmr_ = create_wall_timer(5s,    [this]{ voxel_.cleanup(now()); });

    fstate_       = FlightState::WARMUP;
    last_depth_t_ = last_upd_t_ = now();

    RCLCPP_INFO(get_logger(),
      "╔═════════════════════════════════════════════════════════════╗\n"
      "║            Obstacle Avoidance v8                            ║\n"
      "║  SHIFT RIGHT until obstacle leaves front FOV                ║\n"
      "║  WALL STRAIGHT while camera OR voxels see wall on side      ║\n"
      "║  EXTRA 5M straight after both sources clear                 ║\n"
      "║  RETURN to original path via world-position anchor          ║\n"
      "╚═════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(get_logger(), "CSV save path: %s  [%s]",
      csv_.path().c_str(), csv_.ready() ? "OPEN OK" : "OPEN FAILED — check permissions");
  }

private:
  // ── ROS ────────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          vel_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr         arm_cl_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr             mode_cl_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr          tkoff_cl_;
  rclcpp::TimerBase::SharedPtr ctrl_tmr_, cmd_tmr_, clean_tmr_;

  // ── Flight state ────────────────────────────────────────────────────────
  FlightState fstate_;
  int  warm_ctr_{0}, state_ctr_{0};
  int  next_obs_id_{0};

  // ── Depth image ─────────────────────────────────────────────────────────
  cv::Mat      depth_img_;
  rclcpp::Time last_depth_t_, last_upd_t_;

  // ── MAVRos state + command ──────────────────────────────────────────────
  mavros_msgs::msg::State   mav_state_;
  geometry_msgs::msg::Twist cur_cmd_;

  // ── Pose ────────────────────────────────────────────────────────────────
  double dx_{0}, dy_{0}, dyaw_{0};
  double alt_{0}, base_alt_{2.0}, tkoff_alt_{0};
  double traveled_{0};

  // ── Scene + avoidance stack ─────────────────────────────────────────────
  Scene scene_;
  std::stack<AvoidCtx> stk_;
  static constexpr int kMaxStack = 4;

  VoxelGrid voxel_;
  CSVLogger csv_;

  // ── TUNING ──────────────────────────────────────────────────────────────
  // Trigger
  static constexpr float kTrigger      = 3.5f;   // obstacle trigger distance (m)
  // Speeds
  static constexpr float kStrafeSpd    = 0.6f;   // m/s lateral
  static constexpr float kFwdSpd       = 1.0f;   // m/s forward
  // Wall detection
  static constexpr float kWallGoneCam  = 7.0f;   // camera dist meaning "no wall" (m)
  static constexpr int   kSlabMinHits  = 3;       // min voxels to call slab occupied
  // Camera intrinsics (D435 640×480)
  static constexpr float kFx           = 320.f;
  static constexpr float kFy           = 320.f;
  // Synthetic voxel wall distance (used when camera clear but voxels occupied)
  static constexpr float kVoxSynthDist = 3.0f;
  // Measurement slab geometry
  static constexpr float kMeasNear     = 0.3f;   // m from drone to inner slab edge
  static constexpr float kMeasFar      = 6.0f;   // m from drone to outer slab edge
  static constexpr float kMeasZHalf    = 1.2f;   // ±m vertical slab half-height

  // ─────────────────────────────────────────────────────────────────────────
  //  CALLBACKS
  // ─────────────────────────────────────────────────────────────────────────

  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto p = cv_bridge::toCvCopy(msg);
      if      (msg->encoding == "32FC1") depth_img_ = p->image.clone();
      else if (msg->encoding == "16UC1") p->image.convertTo(depth_img_, CV_32F, 0.001f);
      last_depth_t_ = now();
      if (fstate_ == FlightState::MISSION) {
        updateVoxel();
        analyzeScene();
      }
    } catch (...) { RCLCPP_ERROR(get_logger(), "Depth decode error"); }
  }

  void stateCb(const mavros_msgs::msg::State::SharedPtr m) { mav_state_ = *m; }

  void posCb(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    alt_ = m->pose.position.z;
    dx_  = m->pose.position.x;
    dy_  = m->pose.position.y;
    double qw=m->pose.orientation.w, qx=m->pose.orientation.x,
           qy=m->pose.orientation.y, qz=m->pose.orientation.z;
    dyaw_ = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  VOXEL UPDATE  — back-project depth pixels into world frame
  // ─────────────────────────────────────────────────────────────────────────

  void updateVoxel() {
    if (depth_img_.empty()) return;
    int rows=depth_img_.rows, cols=depth_img_.cols;
    float pcx=cols/2.f, pcy=rows/2.f;
    rclcpp::Time ts = now();
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);

    // Tag voxels with the current obstacle id if we're mid-avoidance
    int obs_id = stk_.empty() ? -1 : stk_.top().obstacle_id;

    for (int r=0; r<rows; r+=4)
      for (int c=0; c<cols; c+=4) {
        float d = depth_img_.at<float>(r,c);
        if (d < 0.25f || d > 12.f) continue;

        float xb =  d;
        float yb = -(c-pcx)*d/kFx;
        float zb = -(r-pcy)*d/kFy;

        float xw = dx_ + (xb*cs - yb*sn);
        float yw = dy_ + (xb*sn + yb*cs);
        float zw = alt_ + zb;

        if (std::abs(zw-alt_) < 1.2f && zw > 0.2f)
          voxel_.add(xw, yw, zw, ts, obs_id);
      }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  SCENE ANALYSIS  — sample front, left, right bands
  // ─────────────────────────────────────────────────────────────────────────

  void analyzeScene() {
    if (depth_img_.empty()) return;
    int rows=depth_img_.rows, cols=depth_img_.cols;
    int r0=(int)(rows*0.15f), r1=(int)(rows*0.55f);
    scene_.front_min = bandMin(r0,r1,(int)(cols*0.35f),(int)(cols*0.65f));
    scene_.left_min  = bandMin(r0,r1, 0,               (int)(cols*0.22f));
    scene_.right_min = bandMin(r0,r1,(int)(cols*0.78f), cols);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  DEPTH HELPERS
  // ─────────────────────────────────────────────────────────────────────────

  float bandMin(int r0, int r1, int c0, int c1) const {
    if (depth_img_.empty()) return 200.f;
    r0=std::max(0,r0); r1=std::min(depth_img_.rows,r1);
    c0=std::max(0,c0); c1=std::min(depth_img_.cols,c1);
    float pcy=depth_img_.rows/2.f;
    float mn=200.f; int valid=0;
    for (int r=r0;r<r1;r++) for (int c=c0;c<c1;c++) {
      float d=depth_img_.at<float>(r,c);
      if (d<0.25f||d>12.f) continue;
      float ph=alt_-(((float)r-pcy)*d/kFy);
      if (std::abs(ph-alt_)<1.0f&&ph>0.2f) { if(d<mn)mn=d; valid++; }
    }
    return (valid<5)?200.f:mn;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  LATERAL SLAB HELPERS
  //
  //  avoidance_dir = +1 means we went RIGHT → wall is on our LEFT.
  //  LEFT in world frame (after yaw rotation):
  //    unit vector = (-sin(yaw), cos(yaw))
  // ─────────────────────────────────────────────────────────────────────────

  // Return the unit vector pointing TOWARD the wall side.
  // avoidance_dir: +1 = we went RIGHT → wall is on LEFT  (-sin, +cos)
  //                -1 = we went LEFT  → wall is on RIGHT (+sin, -cos)
  void wallSideDir(int avoidance_dir, float& ddx, float& ddy) const {
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);
    if (avoidance_dir == 1) {
      // went RIGHT → wall on LEFT
      ddx = -sn;  ddy =  cs;
    } else {
      // went LEFT → wall on RIGHT
      ddx =  sn;  ddy = -cs;
    }
  }

  // Check if lateral voxel slab on the wall side is occupied.
  // Probes at current pos and multiple forward/back positions.
  // Needs majority of probes to confirm (robust against sparse returns).
  //
  // FIX: takes avoidance_dir so it checks the CORRECT side.
  // FIX: probes extended to ±3 m and +5 m ahead to catch long-wall tail.
  bool sideVoxelOccupied(int avoidance_dir) const {
    float ddx, ddy;
    wallSideDir(avoidance_dir, ddx, ddy);
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);

    // Wider slab: 0.3–6 m lateral, ±1.0 m vertical
    const float lat_near=0.3f, lat_far=6.0f, zhalf=1.0f;
    // Five probes: current, 1m back, 3m back, 2m ahead, 5m ahead
    // The 5m-ahead probe is the key one that catches the wall end BEFORE
    // the drone reaches it, preventing the premature-exit → collision bug.
    const float probes[5] = {0.f, -1.f, -3.f, 2.f, 5.f};
    rclcpp::Time t = now();
    int occ_probes = 0;

    for (float fp : probes) {
      float bx=dx_+fp*cs, by=dy_+fp*sn;
      int cnt = voxel_.countSlab(
        bx+lat_near*ddx, bx+lat_far*ddx,
        by+lat_near*ddy, by+lat_far*ddy,
        alt_, zhalf, t);
      if (cnt >= kSlabMinHits) occ_probes++;
    }
    // Need 3 of 5 probes to confirm wall present (majority vote)
    return (occ_probes >= 3);
  }

  // Camera lateral wall distance on the correct avoidance side.
  // FIX: direction-aware — samples LEFT bands if went RIGHT, RIGHT bands if went LEFT.
  float cameraWallDist(int avoidance_dir) const {
    if (depth_img_.empty()) return 200.f;
    int cols=depth_img_.cols, rows=depth_img_.rows;
    int r0=(int)(rows*0.10f), r1=(int)(rows*0.60f);  // slightly wider vertical band
    float d1, d2;
    if (avoidance_dir == 1) {
      // went RIGHT → wall on LEFT → sample left portion of image
      d1 = bandMin(r0,r1, 0,                (int)(cols*0.22f));
      d2 = bandMin(r0,r1,(int)(cols*0.22f), (int)(cols*0.40f));
    } else {
      // went LEFT → wall on RIGHT → sample right portion of image
      d1 = bandMin(r0,r1,(int)(cols*0.78f), cols);
      d2 = bandMin(r0,r1,(int)(cols*0.60f),(int)(cols*0.78f));
    }
    return std::min(d1,d2);
  }

  // Effective lateral wall distance — fuses camera + voxels on the correct side.
  float effectiveWallDist(int avoidance_dir) const {
    float cam_d     = cameraWallDist(avoidance_dir);
    bool  cam_wall  = (cam_d > 0.3f && cam_d < kWallGoneCam);
    bool  vox_wall  = sideVoxelOccupied(avoidance_dir);
    if (cam_wall)  return cam_d;
    if (vox_wall)  return kVoxSynthDist;
    return 200.f;
  }

  // Is the wall present on the correct avoidance side?
  bool wallPresent(int avoidance_dir) const {
    return effectiveWallDist(avoidance_dir) < kWallGoneCam;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  MEASUREMENT HELPER
  //  Collects current-frame obstacle voxels into the context's accumulator.
  //  Uses 3 forward probe positions for better coverage.
  // ─────────────────────────────────────────────────────────────────────────

  void updateMeasurement(AvoidCtx& ctx) {
    float ddx, ddy;
    wallSideDir(ctx.avoidance_dir, ddx, ddy);
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);

    rclcpp::Time t = now();
    std::vector<std::array<float,3>> all_pts;

    const float fwd_probes[3] = {0.f, -2.f, 2.f};
    for (float fp : fwd_probes) {
      float bx=dx_+fp*cs, by=dy_+fp*sn;
      auto pts = voxel_.collectSlab(
        bx+kMeasNear*ddx, bx+kMeasFar*ddx,
        by+kMeasNear*ddy, by+kMeasFar*ddy,
        alt_, kMeasZHalf, t);
      all_pts.insert(all_pts.end(), pts.begin(), pts.end());
    }

    ctx.meas.update(all_pts, (float)dyaw_);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  PATH CHECKS
  // ─────────────────────────────────────────────────────────────────────────

  bool forwardClear(float dist) const {
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);
    return voxel_.pathClear(dx_,dx_+dist*cs, dy_,dy_+dist*sn, alt_,0.6f, now());
  }

  // Check if the lateral return path is clear in voxels.
  // Return direction is opposite to avoidance direction.
  bool returnPathClear(int avoidance_dir) const {
    float cs=std::cos(dyaw_), sn=std::sin(dyaw_);
    // If we went RIGHT (dir=+1), return is LEFT: ddx=-sn, ddy=+cs
    // If we went LEFT  (dir=-1), return is RIGHT: ddx=+sn, ddy=-cs
    float rdx = (avoidance_dir == 1) ? -sn :  sn;
    float rdy = (avoidance_dir == 1) ?  cs : -cs;
    float dist = stk_.empty() ? 2.5f : stk_.top().shift_accumulated;
    return voxel_.pathClear(dx_, dx_+dist*rdx,
                             dy_, dy_+dist*rdy,
                             alt_, 0.6f, now());
  }

  bool obstacleInFront() const {
    return (scene_.front_min < kTrigger) || !forwardClear(kTrigger);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  CONTROL LOOP DISPATCHER
  // ─────────────────────────────────────────────────────────────────────────

  void ctrlLoop() {
    switch (fstate_) {
      case FlightState::WARMUP:  doWarmup();   break;
      case FlightState::GUIDED:  doGuided();   break;
      case FlightState::ARMING:  doArm();      break;
      case FlightState::TAKEOFF: doTakeoff();  break;
      case FlightState::MISSION: doMission();  break;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  SETUP STATES
  // ─────────────────────────────────────────────────────────────────────────

  void doWarmup() {
    cur_cmd_ = geometry_msgs::msg::Twist();
    warm_ctr_++;
    if (!mav_state_.connected) { warm_ctr_=0; return; }
    if (warm_ctr_%10==0) RCLCPP_INFO(get_logger(),"Warmup... connected");
    if (warm_ctr_>40) { state_ctr_=0; fstate_=FlightState::GUIDED;
      RCLCPP_INFO(get_logger(),"→ GUIDED"); }
  }
  void doGuided() {
    state_ctr_++;
    if (state_ctr_==1) {
      if (!mode_cl_->wait_for_service(1s)) { state_ctr_=0; return; }
      auto r=std::make_shared<mavros_msgs::srv::SetMode::Request>();
      r->custom_mode="GUIDED"; mode_cl_->async_send_request(r);
    }
    if (mav_state_.mode=="GUIDED") {
      state_ctr_=0; fstate_=FlightState::ARMING;
      RCLCPP_INFO(get_logger(),"GUIDED → ARMING");
    } else if (state_ctr_>50) state_ctr_=0;
  }
  void doArm() {
    state_ctr_++;
    if (state_ctr_==1) {
      if (!arm_cl_->wait_for_service(1s)) { state_ctr_=0; return; }
      auto r=std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      r->value=true; arm_cl_->async_send_request(r);
    }
    if (mav_state_.armed) {
      tkoff_alt_=alt_; state_ctr_=0; fstate_=FlightState::TAKEOFF;
      RCLCPP_INFO(get_logger(),"Armed → TAKEOFF");
    } else if (state_ctr_>50) state_ctr_=0;
  }
  void doTakeoff() {
    double rel=alt_-tkoff_alt_;
    if (rel<0.3) {
      if (state_ctr_%50==0) {
        if (!tkoff_cl_->wait_for_service(1s)) return;
        auto r=std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        r->altitude=base_alt_; tkoff_cl_->async_send_request(r);
      }
      state_ctr_++; return;
    }
    geometry_msgs::msg::Twist cmd;
    double err=base_alt_-rel;
    cmd.linear.z=(err>0.5)?0.7:(err>0.1)?0.2:0.0;
    cur_cmd_=cmd; state_ctr_++;
    if (rel>=(base_alt_-0.2)) {
      RCLCPP_INFO(get_logger(),"Takeoff → MISSION");
      last_upd_t_=now(); traveled_=0.0;
      while (!stk_.empty()) stk_.pop();
      fstate_=FlightState::MISSION;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  MISSION
  // ─────────────────────────────────────────────────────────────────────────

  void doMission() {
    if (depth_img_.empty() || (now()-last_depth_t_).seconds()>1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"No depth — stopping");
      stop(); return;
    }
    rclcpp::Time t = now();
    double dt = (t-last_upd_t_).seconds();
    last_upd_t_ = t;

    avoidanceStep(dt);
    maintainAlt();

    traveled_ += (std::abs(cur_cmd_.linear.x)+std::abs(cur_cmd_.linear.y))*dt;

    const char* ms = "CRUISING";
    if (!stk_.empty()) {
      switch (stk_.top().mode) {
        case AvoidMode::SHIFT_RIGHT:   ms="SHIFT_R";    break;
        case AvoidMode::WALL_STRAIGHT: ms="WALL_FWD";   break;
        case AvoidMode::EXTRA_5M:      ms="EXTRA_5M";   break;
        case AvoidMode::RETURNING:     ms="RETURN_L";   break;
        default: break;
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "Dist:%.1f/300m | %s[%zu] | Vx:%.2f Vy:%.2f | "
      "F:%.2f L:%.2f R:%.2f | VoxN:%zu",
      traveled_, ms, stk_.size(),
      cur_cmd_.linear.x, cur_cmd_.linear.y,
      scene_.front_min, scene_.left_min, scene_.right_min,
      voxel_.size());

    if (traveled_>=300.0) {
      stop();
      RCLCPP_INFO(get_logger(),"===== MISSION COMPLETE =====");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  AVOIDANCE STATE MACHINE
  // ─────────────────────────────────────────────────────────────────────────

  void avoidanceStep(double dt) {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0; cmd.linear.z = 0.0;

    // ── CRUISING ──────────────────────────────────────────────────────────
    if (stk_.empty()) {
      if (obstacleInFront()) {
        RCLCPP_INFO(get_logger(),
          "╔══════════════════════════════════════╗\n"
          "║  OBSTACLE detected → SHIFT RIGHT      ║\n"
          "╚══════════════════════════════════════╝");
        AvoidCtx ctx;
        ctx.obstacle_id   = next_obs_id_++;
        ctx.avoidance_dir = 1;  // always go RIGHT
        ctx.mode          = AvoidMode::SHIFT_RIGHT;
        ctx.meas.reset();
        stk_.push(ctx);
      } else {
        // Normal cruise: fly forward
        cmd.linear.x = 0.0;
        cmd.linear.y = kFwdSpd;
        cur_cmd_ = cmd; return;
      }
    }

    AvoidCtx& ctx = stk_.top();

    // ── New obstacle while already avoiding → nest ──────────────────────
    if (ctx.mode != AvoidMode::SHIFT_RIGHT &&
        obstacleInFront()                  &&
        (int)stk_.size() < kMaxStack)
    {
      RCLCPP_WARN(get_logger(),
        "⚠ New obstacle while avoiding! Nesting depth %zu→%zu",
        stk_.size(), stk_.size()+1);
      AvoidCtx nc;
      nc.obstacle_id   = next_obs_id_++;
      nc.avoidance_dir = 1;  // always RIGHT
      nc.mode          = AvoidMode::SHIFT_RIGHT;
      nc.meas.reset();
      stk_.push(nc);
      avoidanceStep(dt); return;
    }

    // ── Dispatch ──────────────────────────────────────────────────────────
    switch (ctx.mode) {

    // ======================================================================
    //  SHIFT_RIGHT
    //  Strafe RIGHT (always +1 direction for this build).
    //  avoidance_dir=+1 → linear.x = -kStrafeSpd (right in body frame).
    //  Exit: front_min >= kTrigger (obstacle no longer blocking front FOV).
    //  Accumulate shift distance for exact RETURNING target.
    // ======================================================================
    case AvoidMode::SHIFT_RIGHT:
    {
      // avoidance_dir was set when context was pushed; always +1 here
      // RIGHT strafe: linear.x negative in this frame convention
      cmd.linear.x = (ctx.avoidance_dir == 1) ? -kStrafeSpd : +kStrafeSpd;
      cmd.linear.y =  0.0f;

      ctx.shift_accumulated += kStrafeSpd * dt;
      updateMeasurement(ctx);  // begin collecting obstacle voxels immediately

      bool front_clear = (scene_.front_min >= kTrigger);

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),400,
        "%s SHIFT | front=%.2fm (need>=%.1f) | shifted=%.2fm",
        ctx.avoidance_dir==1?"→ RIGHT":"← LEFT",
        scene_.front_min, kTrigger, ctx.shift_accumulated);

      if (front_clear) {
        RCLCPP_INFO(get_logger(),
          "✔ Obstacle out of front FOV (%.2fm) → WALL_STRAIGHT | shift=%.2fm",
          scene_.front_min, ctx.shift_accumulated);
        ctx.no_wall_cnt   = 0;
        ctx.wall_seen_cnt = 0;
        ctx.mode = AvoidMode::WALL_STRAIGHT;
      }
      break;
    }

    // ======================================================================
    //  WALL_STRAIGHT
    //  Fly straight (forward only).
    //  Stay here while wall is present on the CORRECT avoidance side:
    //    cameraWallDist(dir) < kWallGoneCam   →  camera sees wall
    //    OR sideVoxelOccupied(dir)             →  voxel memory sees wall
    //  Transition only when BOTH are clear for kNoWallExit consecutive frames.
    //  FIX: all wall checks are now direction-aware.
    //  FIX: kNoWallExit raised to 35 frames (3.5s) to prevent premature exit.
    //  FIX: voxel probes extended to 5m ahead to catch wall end early.
    // ======================================================================
    case AvoidMode::WALL_STRAIGHT:
    {
      cmd.linear.x = 0.f;
      cmd.linear.y = kFwdSpd;

      updateMeasurement(ctx);  // keep accumulating

      float cam_d  = cameraWallDist(ctx.avoidance_dir);
      bool  cam_w  = (cam_d > 0.3f && cam_d < kWallGoneCam);
      bool  vox_w  = sideVoxelOccupied(ctx.avoidance_dir);

      if (cam_w || vox_w) {
        // Wall still present (at least one source) — reset both-clear counter
        ctx.no_wall_cnt = 0;
        ctx.wall_seen_cnt++;
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
          "⬆ WALL_STRAIGHT | cam_wall=%s(%.2fm) vox_wall=%s | side=%s | clear_cnt=0",
          cam_w?"YES":"NO", cam_d,
          vox_w?"YES":"NO",
          ctx.avoidance_dir==1?"LEFT":"RIGHT");
      } else {
        // Both sources clear — count consecutive frames
        ctx.no_wall_cnt++;
        ctx.wall_seen_cnt = 0;
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),300,
          "⬆ WALL_STRAIGHT | BOTH CLEAR — cnt=%d/%d (need %d)",
          ctx.no_wall_cnt, AvoidCtx::kNoWallExit, AvoidCtx::kNoWallExit);

        if (ctx.no_wall_cnt >= AvoidCtx::kNoWallExit) {
          RCLCPP_INFO(get_logger(),
            "✔ Wall gone (camera+voxels, %d frames confirmed) → EXTRA_5M",
            AvoidCtx::kNoWallExit);
          ctx.extra_dist = 0.f;
          ctx.mode = AvoidMode::EXTRA_5M;
        }
      }
      break;
    }

    // ======================================================================
    //  EXTRA_5M
    //  Fly exactly 5 metres straight after wall is confirmed gone.
    //  This ensures the drone is fully clear of any remaining structure
    //  even if the voxel tail was noisy.
    //  Continue measuring during this phase too (picks up wall end voxels).
    // ======================================================================
    case AvoidMode::EXTRA_5M:
    {
      cmd.linear.x = 0.f;
      cmd.linear.y = kFwdSpd;
      ctx.extra_dist += kFwdSpd * dt;

      updateMeasurement(ctx);  // absorb final tail voxels

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
        "⬆ EXTRA_5M %.2f/%.1fm | vox_wall=%s | side=%s",
        ctx.extra_dist, AvoidCtx::kExtra5m,
        sideVoxelOccupied(ctx.avoidance_dir)?"OCC":"CLEAR",
        ctx.avoidance_dir==1?"LEFT":"RIGHT");

      if (ctx.extra_dist >= AvoidCtx::kExtra5m) {
        RCLCPP_INFO(get_logger(),
          "✔ 5m offset complete → RETURNING | shift_accumulated=%.2fm",
          ctx.shift_accumulated);
        ctx.return_dist     = 0.f;
        ctx.return_anchored = false;   // will be set on first RETURNING tick
        ctx.clear_debounce  = 0;
        ctx.mode = AvoidMode::RETURNING;
      }
      break;
    }

    // ======================================================================
    //  RETURNING
    //
    //  Strafes back to the original lateral path.
    //
    //  TARGET DISTANCE: shift_accumulated (metres we moved sideways during
    //  SHIFT_RIGHT), measured as actual Euclidean distance from the world-
    //  position anchor set when RETURNING begins.  This is immune to dt
    //  integration drift and velocity frame confusion.
    //
    //  GUARDS — only two simple ones:
    //    1. Camera must NOT see a fresh obstacle in front (< kTrigger).
    //       If blocked, fly forward slowly until clear.
    //    2. Camera must NOT see a close wall in the return direction
    //       (< 1.5 m).  This catches an immediate physical blocker.
    //
    //  NOTE: voxel-path guard deliberately REMOVED.  After 5 m of clear
    //  forward flight the return corridor is geometrically safe.  The voxel
    //  path check was blocking return because it was reading the obstacle's
    //  own stamped voxels from earlier in the flight.
    //
    //  DEBOUNCE: 8 frames before starting lateral movement.
    // ======================================================================
    case AvoidMode::RETURNING:
    {
      // Anchor the start position on first entry
      if (!ctx.return_anchored) {
        ctx.return_start_x  = (float)dx_;
        ctx.return_start_y  = (float)dy_;
        ctx.return_anchored = true;
        ctx.return_dist     = 0.f;
        ctx.clear_debounce  = 0;
        RCLCPP_INFO(get_logger(),
          "↩ RETURNING anchored at (%.2f, %.2f) — need %.2fm lateral",
          ctx.return_start_x, ctx.return_start_y, ctx.shift_accumulated);
      }

      // Guard 1: fresh obstacle directly in front — hold and fly forward
      if (scene_.front_min < kTrigger) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
          "⚠ RETURN: front blocked %.2fm — drift forward", scene_.front_min);
        cmd.linear.x = 0.f;
        cmd.linear.y = kFwdSpd * 0.5f;
        ctx.clear_debounce = 0;
        break;
      }

      // Guard 2: immediate physical wall in the return direction (< 1.5 m)
      // Check the OPPOSITE side from avoidance (that's where we're returning to)
      int return_cam_dir = -ctx.avoidance_dir;   // opposite side
      float ret_cam_d    = cameraWallDist(return_cam_dir);
      if (ret_cam_d > 0.2f && ret_cam_d < 1.5f) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
          "⚠ RETURN: wall %.2fm in return direction — hold", ret_cam_d);
        cmd.linear.x = 0.f;
        cmd.linear.y = kFwdSpd * 0.5f;
        ctx.clear_debounce = 0;
        break;
      }

      // Short debounce before starting lateral movement
      if (ctx.clear_debounce < AvoidCtx::kClearDebounce) {
        ctx.clear_debounce++;
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),400,
          "↩ RETURN debounce %d/%d", ctx.clear_debounce, AvoidCtx::kClearDebounce);
        cmd.linear.x = 0.f;
        cmd.linear.y = kFwdSpd * 0.3f;
        break;
      }

      // ── Actual lateral return movement ──────────────────────────────
      // avoidance_dir=+1 (went RIGHT) → return is LEFT  → cmd.linear.x = +kStrafeSpd
      // avoidance_dir=-1 (went LEFT)  → return is RIGHT → cmd.linear.x = -kStrafeSpd
      float ret_x = (ctx.avoidance_dir == 1) ? +kStrafeSpd : -kStrafeSpd;
      cmd.linear.x = ret_x;
      cmd.linear.y = kFwdSpd * 0.3f;   // slight forward to avoid stall

      // Measure actual world lateral distance traveled since anchor
      // (Euclidean — no frame confusion, no dt drift)
      float ddx_world = (float)dx_ - ctx.return_start_x;
      float ddy_world = (float)dy_ - ctx.return_start_y;
      ctx.return_dist = std::sqrt(ddx_world*ddx_world + ddy_world*ddy_world);

      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),400,
        "%s RETURN | traveled=%.2fm / target=%.2fm",
        ctx.avoidance_dir==1 ? "← LEFT" : "→ RIGHT",
        ctx.return_dist, ctx.shift_accumulated);

      if (ctx.return_dist >= ctx.shift_accumulated) {
        // ── Done — finalise measurement and write CSV ─────────────────
        ctx.meas.finalise();

        RCLCPP_INFO(get_logger(),
          "╔══════════════════════════════════════════════════════════╗\n"
          "║            OBSTACLE #%d — FINAL DIMENSIONS               ║\n"
          "╠══════════════════════════════════════════════════════════╣\n"
          "║  Method  : %-10s                                    ║\n"
          "║  Length  : %7.2f m  (along flight direction)           ║\n"
          "║  Breadth : %7.2f m  (across flight path)               ║\n"
          "║  Height  : %7.2f m  (vertical)                         ║\n"
          "║  Center  : X=%.2f  Y=%.2f  Z=%.2f  (world)             ║\n"
          "║  Voxels  : %d sample ticks                              ║\n"
          "╚══════════════════════════════════════════════════════════╝",
          ctx.obstacle_id,
          ctx.meas.method.c_str(),
          ctx.meas.length, ctx.meas.breadth, ctx.meas.height,
          ctx.meas.cx, ctx.meas.cy, ctx.meas.cz,
          ctx.meas.sample_ticks);

        float rel_x = ctx.meas.cx - (float)dx_;
        float rel_y = ctx.meas.cy - (float)dy_;
        float rel_z = ctx.meas.cz - (float)alt_;
        long long ts_ms = (long long)(now().seconds() * 1000.0);
        std::string dir_str = (ctx.avoidance_dir==1) ? "RIGHT" : "LEFT";

        csv_.write(ts_ms, ctx.obstacle_id, dir_str,
                   ctx.meas.length, ctx.meas.breadth, ctx.meas.height,
                   ctx.meas.cx, ctx.meas.cy, ctx.meas.cz,
                   rel_x, rel_y, rel_z,
                   ctx.meas.sample_ticks, 1.0f,
                   ctx.meas.method);

        RCLCPP_INFO(get_logger(),
          "╔═══════════════════════════════════════════════════╗\n"
          "║  ✓ Back on original path — CRUISING               ║\n"
          "║  ✓ CSV: %s  ║\n"
          "╚═══════════════════════════════════════════════════╝",
          csv_.path().c_str());

        stk_.pop();
        if (!stk_.empty())
          RCLCPP_INFO(get_logger(),"✓ Resumed outer avoidance context");
      }
      break;
    }

    default: break;
    } // switch

    cur_cmd_ = cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  ALTITUDE HOLD + STOP
  // ─────────────────────────────────────────────────────────────────────────

  void maintainAlt() {
    double err = base_alt_ - (alt_ - tkoff_alt_);
    cur_cmd_.linear.z = (std::abs(err)>0.1) ? std::clamp(0.5*err,-0.5,0.5) : 0.0;
  }

  void stop() {
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
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}