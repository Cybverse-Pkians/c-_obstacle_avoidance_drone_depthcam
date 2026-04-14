/*
 * obstacle_avoidance_v22_trackfix.cpp
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  BASE: obstacle_avoidance_v21_yawslant.cpp
 *
 *  DIAGNOSED FROM LOG 00000185.BIN (SITL ArduCopter v4.7-dev, Gazebo):
 *  ─────────────────────────────────────────────────────────────────────────
 *
 *  Flight lasted 31.2s. Drone started at (0,0) heading North (yaw≈0°).
 *  Slant wall detected at t=49s. Wall angle estimated ≈60° → avoid_yaw=60°.
 *  Avoidance ran, drone returned to cruise at t=65s with 0.8m East offset.
 *  At t=68s drone spun 46° uncontrolled → physical collision with wall.
 *
 *  THREE ROOT CAUSES FOUND:
 *
 *  BUG 1 — initial_yaw captured DURING YAW_ALIGN rotation (wrong value)
 *  ──────────────────────────────────────────────────────────────────────
 *  ctx.initial_yaw is set when the obstacle is first detected:
 *      ctx.initial_yaw = (float)dyaw_;
 *  At t=49s dyaw_≈0° — looks correct. BUT the scan phase runs for 3 frames
 *  (~0.3s) and YAW_ALIGN immediately starts rotating the drone toward 60°.
 *  For INLINE obstacles pushed during WALL_FWD, initial_yaw is captured
 *  when dyaw_ is already rotated mid-manoeuvre (e.g. 334° from log).
 *  The yaw-back target then becomes 334° instead of 0°, so the drone
 *  returns to the wrong heading and DIAG_IN deposits a permanent offset.
 *
 *  FIX A: Add node-level mission_yaw_ set ONCE at takeoff completion.
 *          Never changes. All yaw-back targets use mission_yaw_.
 *          ctx.initial_yaw keeps its local meaning (heading at detect time)
 *          but yaw-back and DIAG_IN return both use mission_yaw_.
 *
 *  BUG 2 — kYawReturnTol = 8° (0.14 rad) — too loose
 *  ──────────────────────────────────────────────────
 *  Log shows yaw settled at 334° with DesYaw=336° (2° PID error).
 *  With 8° tolerance the phase B completion fired at 334° instead of
 *  waiting for true alignment to 0° (mission_yaw_). This 6° heading error
 *  accumulated into a 1.5m lateral position error over the return diagonal.
 *
 *  FIX B: Tighten kYawReturnTol to 3° (0.052 rad).
 *          Add kYawReturnHoldFrames=5: must hold within 3° for 0.5s
 *          before accepting alignment as complete.
 *
 *  BUG 3 — return_dist measured in initial_yaw frame, not mission_yaw_ frame
 *  ──────────────────────────────────────────────────────────────────────────
 *  DIAG_IN measures how far the drone has moved laterally using:
 *      cs0 = cos(ctx.initial_yaw), sn0 = sin(ctx.initial_yaw)
 *  When initial_yaw differs from mission_yaw_ (inline obstacles, or
 *  if the drone yawed before detection), the lateral axis is wrong.
 *  The drone stops the return too early or too late → residual offset.
 *
 *  FIX C: Replace ctx.initial_yaw with mission_yaw_ in DIAG_IN lateral
 *          measurement and in the finalise() call.
 *
 *  RESULT (expected):
 *    • Yaw-back always targets true mission heading (mission_yaw_)
 *    • Return diagonal measured on true mission lateral axis
 *    • Track restoration error < 0.1m
 *    • No second collision because drone is back on the cleared corridor
 *
 *  ALL v21 CHANGES RETAINED:
 *    YAW_ALIGN phase, avoid_yaw_target, bandMinPerp, estimateWallAngle,
 *    cameraLeftDist 0-55%, voxel slab 1.2m, feedPixel wall_angle.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/*
 * [v21 original header retained below for reference]
 *
 *  BASE: obstacle_avoidance_v20_slant.cpp
 *
 *  ROOT CAUSE OF REMAINING COLLISION IN v20:
 *  ─────────────────────────────────────────
 *
 *  v20 computed wall_angle and rotated the VELOCITY VECTOR by it, but the
 *  drone's HEADING (dyaw_) never changed.  The yaw PID actively fought
 *  against the rotated velocity and pulled the nose back to initial_yaw,
 *  which means the drone was still pointing straight at the slant face
 *  while trying to crab sideways with rotated velocity commands.
 *
 *  Two cascading problems caused collisions:
 *
 *    A) The yaw PID target was ctx.initial_yaw.  After applyWallNormal()
 *       rotated (vlat, vfwd) the resulting body-frame commands had a
 *       forward component that pressed the drone INTO the slant face while
 *       the PID simultaneously tried to restore the original heading.
 *       Net result: drone slid along the wall face instead of departing
 *       perpendicular to it.
 *
 *    B) obstacleInFrontCamera() / obstacleInFront() both use the drone's
 *       HEADING axis to check for frontal obstacles.  When the drone is
 *       still pointed at the slant wall, "front" == straight into the wall,
 *       so the collision check fires again mid-departure and pushes a new
 *       DIAG_OUT onto the stack — causing the drone to oscillate and
 *       eventually run out of lateral clearance.
 *
 *  THE FIX — YAW TO WALL NORMAL BEFORE DEPARTING:
 *  ────────────────────────────────────────────────
 *
 *  When wall_angle is estimated at the end of the DIAG_OUT scan phase,
 *  we set a new yaw target:
 *
 *      avoid_yaw_target_ = initial_yaw + wall_angle
 *
 *  This rotates the drone's nose to face the wall NORMAL direction
 *  (perpendicular to the slant face).  A dedicated YAW_ALIGN sub-phase
 *  is inserted between the scan phase and the diagonal departure:
 *
 *      Phase 0: CRUISING → obstacle detected → push DIAG_OUT
 *      Phase 1: diag_scan_frames 1–3 → estimate wall_angle, measure bspan
 *      Phase 2: YAW_ALIGN → rotate nose to wall normal, hold position
 *               (zero velocity, only angular.z to yaw PID new target)
 *               Exit when |dyaw_ - avoid_yaw_target_| < kYawAlignTol (5°)
 *               OR after kYawAlignMaxFrames (15 frames = 1.5s) as safety
 *      Phase 3: diagonal departure — now vfwd is truly away from wall
 *               vlat is truly parallel to wall face
 *               NO applyWallNormal() needed — the yaw handles it
 *
 *  CONSEQUENCES:
 *
 *    • The yaw PID target for ALL subsequent states (WALL_FWD, EXTRA_5M,
 *      DIAG_IN) is updated to avoid_yaw_target_ instead of initial_yaw.
 *      This means the drone flies parallel to the wall face throughout the
 *      entire avoidance manoeuvre — exactly the geometry needed.
 *
 *    • obstacleInFrontCamera() and obstacleInFront() now correctly see
 *      "front" as the wall-normal direction, so the collision check does
 *      not false-fire during the lateral departure.
 *
 *    • DIAG_IN return: the drone yaws back to initial_yaw during EXTRA_5M
 *      so the return diagonal is symmetric and the drone re-aligns with
 *      the original mission track.
 *
 *  NEW FIELDS IN AvoidCtx:
 *    avoid_yaw_target  — yaw target aligned to wall normal (= initial_yaw
 *                        for perpendicular walls, initial_yaw+wall_angle
 *                        for slant walls)
 *    yaw_aligned       — true once YAW_ALIGN phase completes
 *    yaw_align_frames  — safety counter
 *
 *  NEW CONSTANT:
 *    kYawAlignTol      = 5° (0.087 rad) — acceptable yaw error to exit align
 *    kYawAlignMaxFrames = 15            — hard timeout (1.5 s at 10 Hz)
 *    kYawReturnTol     = 8° (0.14 rad)  — tolerance to finish yaw-back in EXTRA_5M
 *
 *  ALL v20 CHANGES RETAINED:
 *    bandMinPerp, estimateWallAngle, applyWallNormal (kept but wall_angle=0
 *    effectively because yaw already handles it — left for safety/fallback),
 *    cameraLeftDist 0-55%, voxel slab 1.2m, feedPixel wall_angle, CSV log.
 *
 *  ALL v19 CHANGES RETAINED:
 *    kAvoidFwdSpd, kWallCruiseSpd, kDiagTotalSpd, front_wide rotor coverage,
 *    inline obstacle logic, hysteresis thresholds, voxel decay, yaw PID.
 *
 * ═══════════════════════════════════════════════════════════════════════════
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
#include <stack>
#include <string>
#include <optional>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════
//  CSV LOGGER
// ═══════════════════════════════════════════════════════════════

class CSVLogger {
public:
  CSVLogger() {
    const char* home = std::getenv("HOME");
    path_ = std::string(home ? home : "/tmp") + "/obstacle_data.csv";
    std::ofstream f(path_, std::ios::trunc);
    if (f.is_open()) {
      f << "timestamp_ms,obstacle_id,type,"
        << "length_m,breadth_m,height_m,"
        << "center_world_x,center_world_y,center_world_z,"
        << "n_pts,method,wall_angle_deg\n";
      ready_ = true;
    }
  }
  const std::string& path()  const { return path_; }
  bool               ready() const { return ready_; }

  void write(long long ts_ms, int id, const std::string& type,
             float len, float breadth, float ht,
             float cx, float cy, float cz,
             int n_pts, const std::string& method,
             float wall_angle_deg = 0.f)
  {
    std::ofstream f(path_, std::ios::app);
    if (!f.is_open()) return;
    f << ts_ms << ',' << id << ",\"" << type << "\","
      << std::fixed << std::setprecision(3)
      << len << ',' << breadth << ',' << ht << ','
      << cx  << ',' << cy      << ',' << cz << ','
      << n_pts << ",\"" << method << "\","
      << std::setprecision(1) << wall_angle_deg << "\n";
  }

private:
  std::string path_;
  bool        ready_{false};
};

// ═══════════════════════════════════════════════════════════════
//  ENUMS
// ═══════════════════════════════════════════════════════════════

enum class FlightState { WARMUP, GUIDED, ARMING, TAKEOFF, MISSION };

enum class AvoidMode {
  CRUISING,
  DIAG_OUT,
  WALL_FWD,
  EXTRA_5M,
  DIAG_IN
};

// ═══════════════════════════════════════════════════════════════
//  VOXEL GRID
// ═══════════════════════════════════════════════════════════════

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
      h ^= std::hash<int>()(k.y) + 0x9e3779b9u + (h<<6)+(h>>2);
      h ^= std::hash<int>()(k.z) + 0x9e3779b9u + (h<<6)+(h>>2);
      return h;
    }
  };
}

struct VoxelCell {
  rclcpp::Time last_seen;
  int          hits{0};
  int          obstacle_id{-1};
};

class VoxelGrid {
public:
  VoxelGrid(float vs, float decay) : vs_(vs), decay_(decay) {}

  void add(float x, float y, float z, rclcpp::Time ts, int obs_id=-1) {
    auto& c = grid_[key(x,y,z)];
    c.last_seen = ts;
    c.hits      = std::min(c.hits+1, 60);
    if (obs_id >= 0) c.obstacle_id = obs_id;
  }

  bool occupied(float x, float y, float z, rclcpp::Time now_t) const {
    auto it = grid_.find(key(x,y,z));
    if (it==grid_.end()) return false;
    double age = (now_t - it->second.last_seen).seconds();
    return std::exp(-(float)(age / decay_)) > 0.20f;
  }

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
          if (occupied(x,y,z,now_t)) ++cnt;
    return cnt;
  }

  bool pathClear(float x0, float x1,
                 float y0, float y1,
                 float zc, float zhalf,
                 rclcpp::Time now_t) const
  {
    float xmn=std::min(x0,x1), xmx=std::max(x0,x1);
    float ymn=std::min(y0,y1), ymx=std::max(y0,y1);
    for (float x=xmn; x<=xmx; x+=vs_)
      for (float y=ymn; y<=ymx; y+=vs_)
        for (float z=zc-zhalf; z<=zc+zhalf; z+=vs_)
          if (occupied(x,y,z,now_t)) return false;
    return true;
  }

  std::vector<std::array<float,3>> collectInRegion(
      float cx, float cy, float radius, rclcpp::Time now_t) const {
    std::vector<std::array<float,3>> pts;
    pts.reserve(256);
    for (auto& [k,c] : grid_) {
      float wx=(k.x+0.5f)*vs_, wy=(k.y+0.5f)*vs_, wz=(k.z+0.5f)*vs_;
      float dx=wx-cx, dy=wy-cy;
      if (dx*dx+dy*dy > radius*radius) continue;
      if (occupied(wx,wy,wz,now_t)) pts.push_back({wx,wy,wz});
    }
    return pts;
  }

  void cleanup(rclcpp::Time now_t) {
    for (auto it=grid_.begin(); it!=grid_.end(); )
      if ((now_t-it->second.last_seen).seconds() > decay_*2.0)
        it=grid_.erase(it);
      else ++it;
  }

  float  voxelSize() const { return vs_; }
  size_t size()      const { return grid_.size(); }

private:
  VoxelKey key(float x, float y, float z) const {
    return {(int)std::floor(x/vs_),(int)std::floor(y/vs_),(int)std::floor(z/vs_)};
  }
  mutable std::unordered_map<VoxelKey,VoxelCell> grid_;
  float vs_, decay_;
};

// ═══════════════════════════════════════════════════════════════
//  HYSTERESIS COUNTER
// ═══════════════════════════════════════════════════════════════

struct Hysteresis {
  int threshold{3};
  int count{0};
  bool update(bool cond) {
    if (cond) { count = std::min(count + 1, threshold + 1); }
    else      { count = 0; }
    return count >= threshold;
  }
  bool active() const { return count >= threshold; }
  void reset()  { count = 0; }
};

// ═══════════════════════════════════════════════════════════════
//  SCENE
// ═══════════════════════════════════════════════════════════════

struct Scene {
  // All frontal bands store FORWARD-PROJECTED clearance (bandMinPerp).
  float front_min  {200.f};
  float front_left {200.f};
  float front_right{200.f};
  float front_wide {200.f};
  // Side/top/bottom use raw slant range (bandMin).
  float left_min  {200.f};
  float right_min {200.f};
  float top_min   {200.f};
  float bottom_min{200.f};
};

// ═══════════════════════════════════════════════════════════════
//  MEASUREMENT ACCUMULATOR
// ═══════════════════════════════════════════════════════════════

struct MeasAccum {
  float lat_min{ 1e9f};
  float lat_max{-1e9f};
  int   breadth_n{0};

  float fwd_min{ 1e9f};
  float fwd_max{-1e9f};
  int   fwd_n{0};

  float wall_seed_x{0.f};
  float wall_seed_y{0.f};

  float length{0.f};
  float breadth{0.f};
  float height{0.f};
  float cx{0.f}, cy{0.f}, cz{0.f};
  int   n_pts{0};

  void reset() {
    lat_min=1e9f; lat_max=-1e9f; breadth_n=0;
    fwd_min=1e9f; fwd_max=-1e9f; fwd_n=0;
    wall_seed_x=wall_seed_y=0.f;
    length=breadth=height=0.f;
    cx=cy=cz=0.f; n_pts=0;
  }

  void feedPixel(float wx, float wy, float sn, float cs,
                 float d=0.f, float ref_d=0.f, float gate=0.6f,
                 float wall_angle=0.f)
  {
    if (ref_d > 0.f && d > ref_d + gate) return;
    float wa_cs = std::cos(wall_angle), wa_sn = std::sin(wall_angle);
    float lat_nx = (-sn)*wa_cs - cs*wa_sn;
    float lat_ny = (-sn)*wa_sn + cs*wa_cs;
    float lat = wx*lat_nx + wy*lat_ny;
    if (std::abs(lat) > 8.f) return;
    lat_min = std::min(lat_min, lat);
    lat_max = std::max(lat_max, lat);
    ++breadth_n;
  }

  void feedWallFwdFrame(const cv::Mat& depth_img,
                        float dx, float dy, float alt,
                        float dyaw, float fx, float fy,
                        float depth_max, float depth_gate)
  {
    if (depth_img.empty()) return;
    int rows=depth_img.rows, cols=depth_img.cols;
    float pcx=cols/2.f, pcy=rows/2.f;
    float cs=std::cos(dyaw), sn=std::sin(dyaw);

    bool have_lat = (lat_max > lat_min && breadth_n > 0);
    float lat_lo  = have_lat ? lat_min - 0.4f : -1e9f;
    float lat_hi  = have_lat ? lat_max + 0.4f :  1e9f;

    int c_max = (int)(cols * 0.55f);

    for (int r = (int)(rows*0.08f); r < (int)(rows*0.92f); r += 3)
      for (int c = 0; c < c_max; c += 3) {
        float d = depth_img.at<float>(r, c);
        if (d < 0.2f || d > depth_max) continue;
        float xb = d, yb = -(c - pcx)*d/fx, zb = -(r - pcy)*d/fy;
        float wx = dx + (xb*cs - yb*sn);
        float wy = dy + (xb*sn + yb*cs);
        float wz = alt + zb;
        if (wz < 0.1f || wz > 5.5f) continue;
        if (d > depth_max) continue;
        float lat = wx*(-sn) + wy*cs;
        if (lat < lat_lo || lat > lat_hi) continue;
        float fwd = wx*cs + wy*sn;
        fwd_min = std::min(fwd_min, fwd);
        fwd_max = std::max(fwd_max, fwd);
        ++fwd_n;
      }
  }

  void finalise(const std::vector<std::array<float,3>>& pts,
                float fwd_cs, float fwd_sn)
  {
    breadth = (lat_max > lat_min && breadth_n > 0)
              ? (lat_max - lat_min) : 0.f;
    length  = (fwd_max > fwd_min && fwd_n > 0)
              ? (fwd_max - fwd_min) : 0.f;
    n_pts = (int)pts.size();
    if (pts.empty()) { height = 0.f; return; }
    float lat_lo = (lat_min < 1e8f) ? lat_min - 0.5f : -1e9f;
    float lat_hi = (lat_max > -1e8f) ? lat_max + 0.5f :  1e9f;
    float zmin=1e9f, zmax=-1e9f;
    float sx=0, sy=0, sz=0;
    int used = 0;
    for (auto& p : pts) {
      float lat = p[0]*(-fwd_sn) + p[1]*fwd_cs;
      if (lat < lat_lo || lat > lat_hi) continue;
      zmin = std::min(zmin, p[2]);
      zmax = std::max(zmax, p[2]);
      sx += p[0]; sy += p[1]; sz += p[2];
      ++used;
    }
    height = std::max(0.f, zmax - zmin);
    n_pts  = used;
    if (used > 0) { cx=sx/used; cy=sy/used; cz=sz/used; }
  }
};

// ═══════════════════════════════════════════════════════════════
//  AVOIDANCE CONTEXT
// ═══════════════════════════════════════════════════════════════

struct AvoidCtx {
  int       obstacle_id{-1};
  AvoidMode mode{AvoidMode::DIAG_OUT};

  // initial_yaw: heading when obstacle was first detected.
  // Used to restore the drone's original mission track in DIAG_IN.
  float     initial_yaw{0.f};

  // v21: avoid_yaw_target = initial_yaw + wall_angle
  // This is the heading the drone yaws TO during YAW_ALIGN so that
  // "forward" means "away from the wall face" throughout the manoeuvre.
  // For perpendicular walls wall_angle==0 so avoid_yaw_target==initial_yaw
  // and behaviour is identical to v20/v19.
  float     avoid_yaw_target{0.f};

  // v20: estimated wall angle (radians) relative to drone heading.
  // Positive = wall face opens left; negative = opens right.
  float     wall_angle{0.f};

  // v21 YAW_ALIGN phase tracking
  bool  yaw_aligned{false};       // true once alignment complete
  int   yaw_align_frames{0};      // safety counter
  static constexpr int   kYawAlignMaxFrames = 15;    // 1.5s at 10Hz
  static constexpr float kYawAlignTol       = 0.087f; // 5° in radians

  float diag_start_x{0.f}, diag_start_y{0.f};
  bool  diag_anchored{false};
  float diag_lateral{0.f};
  float diag_lateral_target{0.f};
  float diag_fwd{0.f};
  float diag_angle{0.f};
  float diag_speed{0.f};
  int   diag_scan_frames{0};
  static constexpr int kDiagScanFrames = 3;

  int  no_wall_cnt{0};
  bool wall_seen_this_run{false};
  static constexpr int kNoWallExit = 8;

  float shift_accumulated{0.f};

  Hysteresis front_block_hyst;

  bool  is_inline{false};

  float extra_dist{0.f};
  static constexpr float kExtra5m = 0.5f;

  // v21: yaw_returned tracks whether the drone has yawed back to mission_yaw_
  // during EXTRA_5M so DIAG_IN flies on the original track.
  bool  yaw_returned{false};
  int   yaw_return_frames{0};
  int   yaw_return_hold_frames{0};   // v22: must hold <3° for this many frames
  static constexpr int   kYawReturnMaxFrames  = 25;  // 2.5s hard timeout (was 15)
  static constexpr int   kYawReturnHoldFrames =  5;  // 0.5s hold at <3°
  static constexpr float kYawReturnTol        = 0.052f; // 3° (was 8° = 0.14f)

  float return_start_x{0.f}, return_start_y{0.f};
  bool  return_anchored{false};
  float return_dist{0.f};
  int   return_debounce{0};
  static constexpr int kReturnDebounce = 8;

  MeasAccum meas;
};

// ═══════════════════════════════════════════════════════════════
//  ALTITUDE DODGE STATE
// ═══════════════════════════════════════════════════════════════

struct AltDodge {
  bool  active{false};
  float offset{0.f};
  int   hold_cnt{0};
  static constexpr int kHoldFrames = 25;
  void start(float off) { active=true; offset=off; hold_cnt=0; }
  void tick()  { if (active) ++hold_cnt; }
  bool done()  const { return hold_cnt >= kHoldFrames; }
  void finish(){ active=false; offset=0.f; hold_cnt=0; }
};

// ═══════════════════════════════════════════════════════════════
//  MAIN NODE
// ═══════════════════════════════════════════════════════════════

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  ObstacleAvoidanceNode()
    : Node("obstacle_avoidance"), voxel_(kVoxelSize, kVoxDecay)
  {
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/realsense/depth_image", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr m){ depthCb(m); });

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr m){ mav_state_=*m; });

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort().durability_volatile();
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){ posCb(m); });

    vel_pub_  = create_publisher<geometry_msgs::msg::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    arm_cl_   = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_cl_  = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    tkoff_cl_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    ctrl_tmr_  = create_wall_timer(100ms, [this]{ ctrlLoop(); });
    cmd_tmr_   = create_wall_timer( 50ms, [this]{ publishCmd(); });
    clean_tmr_ = create_wall_timer(  2s,  [this]{ voxel_.cleanup(now()); });

    fstate_ = FlightState::WARMUP;
    last_depth_t_ = last_upd_t_ = now();

    RCLCPP_INFO(get_logger(),
      "╔═══════════════════════════════════════════════════════════════════════╗\n"
      "║  Obstacle Avoidance v22-trackfix                                     ║\n"
      "╠═══════════════════════════════════════════════════════════════════════╣\n"
      "║  v22: mission_yaw_ — set ONCE at takeoff, never overwritten          ║\n"
      "║       all yaw-back and DIAG_IN return use mission_yaw_               ║\n"
      "║       FIX: initial_yaw captured mid-rotation (log: 334° not 0°)     ║\n"
      "║  v22: kYawReturnTol 8°→3° + 5-frame hold before DIAG_IN            ║\n"
      "║       FIX: loose tolerance left 6° heading error → 1.5m offset      ║\n"
      "║  v22: DIAG_IN lateral measured in mission_yaw_ frame                 ║\n"
      "║       FIX: wrong frame caused premature return diagonal stop         ║\n"
      "║  v21: YAW_ALIGN, avoid_yaw_target, EXTRA_5M yaw-back retained       ║\n"
      "║  v21: obstacleInFront() uses avoid_yaw_target axis during avoidance  ║\n"
      "║       so collision check no longer fires falsely mid-departure       ║\n"
      "╠═══════════════════════════════════════════════════════════════════════╣\n"
      "║  v20: bandMinPerp  estimateWallAngle  cameraLeftDist 55%  slab 1.2m  ║\n"
      "║  v19: kAvoidFwdSpd=0.5  kWallCruiseSpd=2.0  front_wide rotor-safe   ║\n"
      "║  v18: kTriggerDist=3.8m  kDiagScanFrames=3  kCrawlDist=1.5m        ║\n"
      "╚═══════════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(get_logger(), "CSV → %s  [%s]",
      csv_.path().c_str(), csv_.ready() ? "OPEN OK" : "OPEN FAILED");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          vel_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr         arm_cl_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr             mode_cl_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr          tkoff_cl_;
  rclcpp::TimerBase::SharedPtr ctrl_tmr_, cmd_tmr_, clean_tmr_;

  FlightState fstate_;
  int  warm_ctr_{0}, state_ctr_{0};
  int  next_obs_id_{0};

  cv::Mat      depth_img_;
  rclcpp::Time last_depth_t_, last_upd_t_;

  mavros_msgs::msg::State   mav_state_;
  geometry_msgs::msg::Twist cur_cmd_;

  double dx_{0}, dy_{0}, dyaw_{0};
  double alt_{0}, base_alt_{2.0}, tkoff_alt_{0};
  double traveled_{0};

  // v22 FIX A: mission_yaw_ is set ONCE when the drone reaches mission
  // altitude and never changed again.  It represents the true intended
  // forward heading of the mission.  All yaw-back targets and DIAG_IN
  // lateral measurements use this value so that avoidance always restores
  // the drone to the ORIGINAL track, not to whatever heading was current
  // when the obstacle happened to be detected.
  float mission_yaw_{0.f};

  Scene  scene_;
  std::stack<AvoidCtx> stk_;
  AltDodge alt_dodge_;

  float yaw_pid_integral_{0.f};
  float yaw_pid_prev_err_{0.f};

  VoxelGrid voxel_;
  CSVLogger csv_;

  // ══════════════════════════════════════════════════════════════
  //  TUNING CONSTANTS
  // ══════════════════════════════════════════════════════════════

  static constexpr float kFwdSpd        = 2.0f;
  static constexpr float kAvoidFwdSpd   = 0.5f;
  static constexpr float kWallCruiseSpd = 2.0f;
  static constexpr float kDiagTotalSpd  = kFwdSpd;

  static constexpr float kDepthMax    = 4.0f;
  static constexpr float kTriggerDist = 3.8f;
  static constexpr float kWallGoneCam = 3.8f;
  static constexpr float kSlowDist    = 3.5f;
  static constexpr float kCrawlDist   = 1.5f;
  static constexpr float kStrafeSpd   = 0.8f;

  static constexpr float kDiagAngMin  = 40.f * (float)M_PI / 180.f;
  static constexpr float kDiagAngMax  = 75.f * (float)M_PI / 180.f;
  static constexpr float kWallAngMax  = 60.f * (float)M_PI / 180.f;

  static constexpr float kVoxDecay    = 8.0f;
  static constexpr float kVoxelSize   = 0.4f;
  static constexpr int   kSlabMinHits = 2;

  static constexpr float kFx         = 320.f;
  static constexpr float kFy         = 320.f;

  static constexpr float kRollFF     = 0.10f;
  static constexpr float kPitchFF    = 0.10f;

  static constexpr float kYawKp      = 1.0f;
  static constexpr float kYawKi      = 0.04f;
  static constexpr float kYawKd      = 0.2f;
  static constexpr float kYawMaxOut  = 0.7f;

  static constexpr float kHeightCal  = 1.136f;

  // ══════════════════════════════════════════════════════════════
  //  HELPER: normalise angle to [-π, π]
  // ══════════════════════════════════════════════════════════════

  static float normAngle(float a) {
    while (a >  (float)M_PI) a -= 2.f*(float)M_PI;
    while (a < -(float)M_PI) a += 2.f*(float)M_PI;
    return a;
  }

  // ══════════════════════════════════════════════════════════════
  //  YAW PID
  // ══════════════════════════════════════════════════════════════

  float yawPID(float target, float current, double dt) {
    float err = normAngle(target - current);
    yaw_pid_integral_ += err * (float)dt;
    yaw_pid_integral_  = std::clamp(yaw_pid_integral_, -0.5f, 0.5f);
    float deriv = (err - yaw_pid_prev_err_) / (float)std::max(dt, 0.001);
    yaw_pid_prev_err_ = err;
    float out = kYawKp*err + kYawKi*yaw_pid_integral_ + kYawKd*deriv;
    return std::clamp(out, -kYawMaxOut, kYawMaxOut);
  }

  void resetYawPID() {
    yaw_pid_integral_ = 0.f;
    yaw_pid_prev_err_ = 0.f;
  }

  // ══════════════════════════════════════════════════════════════
  //  DEPTH CALLBACK
  // ══════════════════════════════════════════════════════════════

  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto p = cv_bridge::toCvCopy(msg);
      if      (msg->encoding=="32FC1") depth_img_ = p->image.clone();
      else if (msg->encoding=="16UC1") p->image.convertTo(depth_img_,CV_32F,0.001f);
      last_depth_t_ = now();
      if (fstate_==FlightState::MISSION) {
        updateVoxelMap();
        analyzeScene();
      }
    } catch(...) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Depth decode error");
    }
  }

  void posCb(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    dx_  = m->pose.position.x;
    dy_  = m->pose.position.y;
    alt_ = m->pose.position.z;
    double qw=m->pose.orientation.w, qx=m->pose.orientation.x,
           qy=m->pose.orientation.y, qz=m->pose.orientation.z;
    dyaw_ = std::atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz));
  }

  // ══════════════════════════════════════════════════════════════
  //  VOXEL MAP UPDATE
  // ══════════════════════════════════════════════════════════════

  void updateVoxelMap() {
    if (depth_img_.empty()) return;
    int   rows=depth_img_.rows, cols=depth_img_.cols;
    float pcx=cols/2.f, pcy=rows/2.f;
    float cs=(float)std::cos(dyaw_), sn=(float)std::sin(dyaw_);
    rclcpp::Time ts=now();
    int obs_id = stk_.empty() ? -1 : stk_.top().obstacle_id;
    for (int r=0; r<rows; r+=4)
      for (int c=0; c<cols; c+=4) {
        float d=depth_img_.at<float>(r,c);
        if (d<0.3f || d>kDepthMax) continue;
        float xb=d, yb=-(c-pcx)*d/kFx, zb=-(r-pcy)*d/kFy;
        float xw=(float)dx_+(xb*cs-yb*sn);
        float yw=(float)dy_+(xb*sn+yb*cs);
        float zw=(float)alt_+zb;
        if (std::abs(zw-(float)alt_)<2.5f && zw>0.15f)
          voxel_.add(xw,yw,zw,ts,obs_id);
      }
  }

  // ══════════════════════════════════════════════════════════════
  //  SCENE ANALYSIS
  // ══════════════════════════════════════════════════════════════

  void analyzeScene() {
    if (depth_img_.empty()) return;
    int rows=depth_img_.rows, cols=depth_img_.cols;
    int r0=(int)(rows*0.12f), r1=(int)(rows*0.58f);
    scene_.front_min   = bandMinPerp(r0,r1,(int)(cols*0.32f),(int)(cols*0.68f));
    scene_.front_left  = bandMinPerp(r0,r1,(int)(cols*0.10f),(int)(cols*0.45f));
    scene_.front_right = bandMinPerp(r0,r1,(int)(cols*0.55f),(int)(cols*0.90f));
    scene_.front_wide  = std::min({scene_.front_min,
                                   scene_.front_left,
                                   scene_.front_right});
    scene_.left_min   = bandMin(r0,r1, 0,               (int)(cols*0.22f));
    scene_.right_min  = bandMin(r0,r1,(int)(cols*0.78f), cols);
    scene_.top_min    = bandMin(0,              (int)(rows*0.25f),
                                (int)(cols*0.30f),(int)(cols*0.70f));
    scene_.bottom_min = bandMin((int)(rows*0.75f), rows,
                                (int)(cols*0.30f),(int)(cols*0.70f));
  }

  float bandMin(int r0, int r1, int c0, int c1) const {
    if (depth_img_.empty()) return 200.f;
    r0=std::max(0,r0); r1=std::min(depth_img_.rows,r1);
    c0=std::max(0,c0); c1=std::min(depth_img_.cols,c1);
    float pcy=depth_img_.rows/2.f;
    float mn=200.f; int valid=0;
    for (int r=r0;r<r1;r++) for (int c=c0;c<c1;c++) {
      float d=depth_img_.at<float>(r,c);
      if (d<0.3f || d>kDepthMax) continue;
      float ph=(float)alt_-((float)(r-pcy)*d/kFy);
      if (std::abs(ph-(float)alt_)<1.0f && ph>0.15f) { if(d<mn)mn=d; ++valid; }
    }
    return (valid<5)?200.f:mn;
  }

  float bandMinPerp(int r0, int r1, int c0, int c1) const {
    if (depth_img_.empty()) return 200.f;
    r0=std::max(0,r0); r1=std::min(depth_img_.rows,r1);
    c0=std::max(0,c0); c1=std::min(depth_img_.cols,c1);
    float pcx = depth_img_.cols / 2.f;
    float pcy = depth_img_.rows / 2.f;
    float cs  = (float)std::cos(dyaw_);
    float sn  = (float)std::sin(dyaw_);
    float mn = 200.f; int valid = 0;
    for (int r=r0; r<r1; r++) for (int c=c0; c<c1; c++) {
      float d = depth_img_.at<float>(r,c);
      if (d < 0.3f || d > kDepthMax) continue;
      float xb = d;
      float yb = -(c - pcx)*d / kFx;
      float zb = -(r - pcy)*d / kFy;
      float wx = (float)dx_ + (xb*cs - yb*sn);
      float wy = (float)dy_ + (xb*sn + yb*cs);
      float wz = (float)alt_ + zb;
      if (std::abs(wz - (float)alt_) > 1.0f || wz < 0.15f) continue;
      float fwd = (wx - (float)dx_)*cs + (wy - (float)dy_)*sn;
      if (fwd > 0.1f) { if (fwd < mn) mn = fwd; ++valid; }
    }
    return (valid < 5) ? 200.f : mn;
  }

  // ══════════════════════════════════════════════════════════════
  //  WALL ANGLE ESTIMATOR  (unchanged from v20)
  // ══════════════════════════════════════════════════════════════

  float estimateWallAngle(float ref_d) const {
    if (depth_img_.empty()) return 0.f;
    int rows = depth_img_.rows, cols = depth_img_.cols;
    float pcx = cols / 2.f, pcy = rows / 2.f;
    float cs = (float)std::cos(dyaw_), sn = (float)std::sin(dyaw_);
    float sf=0.f, sl=0.f, sfl=0.f, sl2=0.f;
    int n = 0;
    for (int r=(int)(rows*0.10f); r<(int)(rows*0.90f); r+=3)
      for (int c=0; c<cols; c+=3) {
        float d = depth_img_.at<float>(r,c);
        if (d < 0.3f || d > ref_d + 0.6f) continue;
        float xb = d, yb = -(c-pcx)*d/kFx, zb = -(r-pcy)*d/kFy;
        float wx = (float)dx_ + (xb*cs - yb*sn);
        float wy = (float)dy_ + (xb*sn + yb*cs);
        float wz = (float)alt_ + zb;
        if (wz < 0.1f || wz > 5.5f) continue;
        float fwd = (wx-(float)dx_)*cs + (wy-(float)dy_)*sn;
        float lat = (wx-(float)dx_)*(-sn) + (wy-(float)dy_)*cs;
        if (fwd < 0.1f || fwd > ref_d + 0.3f) continue;
        sf += fwd; sl += lat; sfl += fwd*lat; sl2 += lat*lat;
        ++n;
      }
    if (n < 30) return 0.f;
    float denom = (float)n * sl2 - sl * sl;
    if (std::abs(denom) < 1e-4f) return 0.f;
    float m = ((float)n * sfl - sf * sl) / denom;
    if (std::abs(m) < 1e-3f) return 0.f;
    float angle = std::atan(-1.f / m);
    return std::clamp(angle, -kWallAngMax, kWallAngMax);
  }

  // ══════════════════════════════════════════════════════════════
  //  LEFT WALL DETECTION  (widened to 0-55% from v20)
  // ══════════════════════════════════════════════════════════════

  float cameraLeftDist() const {
    if (depth_img_.empty()) return 200.f;
    int rows=depth_img_.rows, cols=depth_img_.cols;
    int r0=(int)(rows*0.10f), r1=(int)(rows*0.60f);
    float d1=bandMin(r0,r1, 0,                (int)(cols*0.28f));
    float d2=bandMin(r0,r1,(int)(cols*0.28f), (int)(cols*0.55f));
    return std::min(d1,d2);
  }

  bool leftVoxelOccupied() const {
    float cs=(float)std::cos(dyaw_), sn=(float)std::sin(dyaw_);
    float ldx=-sn, ldy=+cs;
    const float lat_near=0.3f, lat_far=3.5f, zhalf=1.0f;
    const float probes[4]={0.f, 1.f, 2.f, 3.f};
    rclcpp::Time t=now();
    int occ=0;
    for (float fp : probes) {
      float bx=(float)dx_+fp*cs, by=(float)dy_+fp*sn;
      int cnt=voxel_.countSlab(bx+lat_near*ldx, bx+lat_far*ldx,
                               by+lat_near*ldy, by+lat_far*ldy,
                               (float)alt_, zhalf, t);
      if (cnt>=kSlabMinHits) ++occ;
    }
    return (occ>=3);
  }

  bool leftWallPresent() const {
    float cd=cameraLeftDist();
    return (cd>0.3f && cd<kWallGoneCam) || leftVoxelOccupied();
  }

  // ══════════════════════════════════════════════════════════════
  //  FRONT OBSTACLE CHECKS
  //
  //  v21: obstacleInFront() uses the ACTIVE YAW TARGET as its forward axis.
  //  During avoidance the active yaw target is avoid_yaw_target (wall normal
  //  direction).  After the manoeuvre it reverts to the drone's current heading.
  //  This stops false collision triggers mid-departure on slant walls.
  // ══════════════════════════════════════════════════════════════

  bool obstacleInFrontCamera() const {
    return scene_.front_wide < kTriggerDist;
  }

  // activeYaw returns the current avoid_yaw_target if we are in avoidance,
  // otherwise returns dyaw_ (current heading).
  float activeYaw() const {
    if (!stk_.empty()) return stk_.top().avoid_yaw_target;
    return (float)dyaw_;
  }

  bool obstacleInFront() const {
    if (scene_.front_wide < kTriggerDist) return true;
    // v21: use avoid_yaw_target axis, not raw dyaw_, for the voxel corridor check.
    float ya = activeYaw();
    float cs=(float)std::cos(ya), sn=(float)std::sin(ya);
    return !voxel_.pathClear(
      (float)dx_, (float)dx_+kTriggerDist*cs,
      (float)dy_, (float)dy_+kTriggerDist*sn,
      (float)alt_, 1.2f, now());
  }

  // ══════════════════════════════════════════════════════════════
  //  ALTITUDE DODGE
  // ══════════════════════════════════════════════════════════════

  bool tryAltitudeDodge() {
    if (alt_dodge_.active) { alt_dodge_.tick(); return true; }
    if (alt_dodge_.done()) { alt_dodge_.finish(); return false; }
    if (!obstacleInFrontCamera()) return false;
    bool only_top    = (scene_.top_min    < kTriggerDist + 0.8f) && (scene_.bottom_min > 3.5f);
    bool only_bottom = (scene_.bottom_min < kTriggerDist + 0.8f) && (scene_.top_min    > 3.5f);
    if (only_top) {
      RCLCPP_INFO(get_logger(), "⬇ ALT DODGE descend 0.6m (top=%.2fm)", scene_.top_min);
      alt_dodge_.start(-0.6f); return true;
    } else if (only_bottom) {
      RCLCPP_INFO(get_logger(), "⬆ ALT DODGE ascend 0.6m (bot=%.2fm)", scene_.bottom_min);
      alt_dodge_.start(+0.6f); return true;
    }
    return false;
  }

  // ══════════════════════════════════════════════════════════════
  //  ADAPTIVE FORWARD SPEED
  // ══════════════════════════════════════════════════════════════

  float adaptiveFwdSpd() const {
    if (scene_.front_wide < kCrawlDist) return kFwdSpd * 0.20f;
    if (scene_.front_wide < kSlowDist)  return kFwdSpd * 0.60f;
    return kFwdSpd;
  }

  // ══════════════════════════════════════════════════════════════
  //  BREADTH PIXEL SCAN
  // ══════════════════════════════════════════════════════════════

  void scanBreadthPixels(MeasAccum& meas, float iy, float ref_d,
                         float wall_angle = 0.f) {
    if (depth_img_.empty()) return;
    int rows=depth_img_.rows, cols=depth_img_.cols;
    float pcx=cols/2.f, pcy=rows/2.f;
    float cs=std::cos(iy), sn=std::sin(iy);
    int r0=(int)(rows*0.05f), r1=(int)(rows*0.95f);
    for (int r=r0; r<r1; r+=2)
      for (int c=0; c<cols; c+=2) {
        float d=depth_img_.at<float>(r,c);
        if (d<0.2f || d>kDepthMax) continue;
        float xb=d, yb=-(c-pcx)*d/kFx, zb=-(r-pcy)*d/kFy;
        float wx=(float)dx_+(xb*cs-yb*sn);
        float wy=(float)dy_+(xb*sn+yb*cs);
        float wz=(float)alt_+zb;
        if (wz<0.05f) continue;
        meas.feedPixel(wx, wy, sn, cs, d, ref_d, 0.6f, wall_angle);
      }
  }

  // ══════════════════════════════════════════════════════════════
  //  applyWallNormal  (kept from v20 as safety fallback)
  // ══════════════════════════════════════════════════════════════

  std::pair<float,float> applyWallNormal(float vlat, float vfwd,
                                          float wall_angle) const {
    if (std::abs(wall_angle) < 0.01f) return {vlat, vfwd};
    float wcs = std::cos(wall_angle), wsn = std::sin(wall_angle);
    float vlat_r =  vlat * wcs - vfwd * wsn;
    float vfwd_r =  vlat * wsn + vfwd * wcs;
    return {vlat_r, vfwd_r};
  }

  // ══════════════════════════════════════════════════════════════
  //  FLIGHT STATE MACHINE
  // ══════════════════════════════════════════════════════════════

  void ctrlLoop() {
    switch (fstate_) {
      case FlightState::WARMUP:  doWarmup();  break;
      case FlightState::GUIDED:  doGuided();  break;
      case FlightState::ARMING:  doArm();     break;
      case FlightState::TAKEOFF: doTakeoff(); break;
      case FlightState::MISSION: doMission(); break;
    }
  }

  void doWarmup() {
    cur_cmd_=geometry_msgs::msg::Twist();
    if (!mav_state_.connected){warm_ctr_=0;return;}
    if (++warm_ctr_%10==0) RCLCPP_INFO(get_logger(),"Warmup %d/40",warm_ctr_);
    if (warm_ctr_>40){state_ctr_=0;fstate_=FlightState::GUIDED;
      RCLCPP_INFO(get_logger(),"→ GUIDED");}
  }
  void doGuided() {
    if (++state_ctr_==1){
      if (!mode_cl_->wait_for_service(1s)){state_ctr_=0;return;}
      auto r=std::make_shared<mavros_msgs::srv::SetMode::Request>();
      r->custom_mode="GUIDED"; mode_cl_->async_send_request(r);
    }
    if (mav_state_.mode=="GUIDED"){state_ctr_=0;fstate_=FlightState::ARMING;
      RCLCPP_INFO(get_logger(),"GUIDED → ARMING");}
    else if (state_ctr_>50) state_ctr_=0;
  }
  void doArm() {
    if (++state_ctr_==1){
      if (!arm_cl_->wait_for_service(1s)){state_ctr_=0;return;}
      auto r=std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      r->value=true; arm_cl_->async_send_request(r);
    }
    if (mav_state_.armed){tkoff_alt_=alt_;state_ctr_=0;
      fstate_=FlightState::TAKEOFF;RCLCPP_INFO(get_logger(),"Armed → TAKEOFF");}
    else if (state_ctr_>50) state_ctr_=0;
  }
  void doTakeoff() {
    double rel=alt_-tkoff_alt_;
    if (rel<0.3){
      if (state_ctr_++%50==0){
        if (!tkoff_cl_->wait_for_service(1s)) return;
        auto r=std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        r->altitude=base_alt_; tkoff_cl_->async_send_request(r);
      }
      return;
    }
    geometry_msgs::msg::Twist cmd;
    double err=base_alt_-rel;
    cmd.linear.z=(err>0.5)?0.7:(err>0.1)?0.25:0.0;
    cur_cmd_=cmd;
    if (rel>=base_alt_-0.2){
      RCLCPP_INFO(get_logger(),"Takeoff → MISSION (alt=%.2f m)",(float)rel);
      mission_yaw_ = (float)dyaw_;   // v22: capture true mission heading ONCE
      RCLCPP_INFO(get_logger(),"mission_yaw_ = %.1f°",
                  mission_yaw_ * 180.f / (float)M_PI);
      last_upd_t_=now(); traveled_=0.0;
      while(!stk_.empty()) stk_.pop();
      resetYawPID();
      fstate_=FlightState::MISSION;
    }
  }

  // ══════════════════════════════════════════════════════════════
  //  MISSION LOOP
  // ══════════════════════════════════════════════════════════════

  void doMission() {
    if (depth_img_.empty()||(now()-last_depth_t_).seconds()>1.0){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"No depth — stopping");
      stopHold(); return;
    }
    rclcpp::Time t=now();
    double dt=(t-last_upd_t_).seconds();
    last_upd_t_=t;
    if (dt<=0.0||dt>0.5) dt=0.1;

    avoidanceStep(dt);
    maintainAlt();
    traveled_ += std::abs(cur_cmd_.linear.y)*dt;

    static const char* mnames[]={
      "CRUISING","DIAG_OUT","WALL_FWD","EXTRA_5M","DIAG_IN"};
    const char* ms=mnames[0];
    if (!stk_.empty()) ms=mnames[(int)stk_.top().mode];

    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
      "Dist:%.1fm | %-10s[%zu] | Vx:%+.2f Vy:%.2f | "
      "Fc:%.2f Fw:%.2f L:%.2f R:%.2f | Vox:%zu | WA:%.1f° AY:%.1f°",
      traveled_,ms,stk_.size(),
      cur_cmd_.linear.x, cur_cmd_.linear.y,
      scene_.front_min, scene_.front_wide,
      scene_.left_min, scene_.right_min,
      voxel_.size(),
      stk_.empty() ? 0.f : stk_.top().wall_angle        * 180.f / (float)M_PI,
      stk_.empty() ? 0.f : stk_.top().avoid_yaw_target  * 180.f / (float)M_PI);

    if (traveled_>=300.0){
      stopHold();
      RCLCPP_INFO(get_logger(),"===== MISSION COMPLETE =====");
    }
  }

  // ══════════════════════════════════════════════════════════════
  //  AVOIDANCE STATE MACHINE
  //
  //  VELOCITY CONVENTION:
  //    cmd.linear.y = forward  (+Y = nose direction)
  //    cmd.linear.x = lateral  (+X = left)
  //    cmd.linear.z = alt hold
  //
  //  v21 YAW STRATEGY:
  //    CRUISING          → yaw target = dyaw_ (current)
  //    DIAG_OUT scan     → yaw target = initial_yaw  (hold still)
  //    DIAG_OUT YAW_ALIGN→ yaw target = avoid_yaw_target (= initial_yaw + wall_angle)
  //                        zero velocity, pure yaw rotation
  //    DIAG_OUT depart   → yaw target = avoid_yaw_target
  //                        vfwd/vlat now relative to wall-normal heading
  //    WALL_FWD          → yaw target = avoid_yaw_target
  //    EXTRA_5M fwd run  → yaw target = avoid_yaw_target
  //    EXTRA_5M yaw-back → yaw target = initial_yaw (transition back)
  //    DIAG_IN           → yaw target = initial_yaw  (return on original track)
  // ══════════════════════════════════════════════════════════════

  void avoidanceStep(double dt) {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z=0.f; cmd.angular.x=0.f; cmd.angular.y=0.f;
    cmd.linear.z=0.f;

    // ── CRUISING ─────────────────────────────────────────────────────────
    if (stk_.empty()) {
      if (obstacleInFront()) {
        if (!tryAltitudeDodge()) {
          RCLCPP_INFO(get_logger(),
            "╔════════════════════════════════════════════════════════════╗\n"
            "║  OBSTACLE @ %.2fm (wide=%.2fm) — INSTANT BRAKE → DIAG_OUT ║\n"
            "╚════════════════════════════════════════════════════════════╝",
            scene_.front_min, scene_.front_wide);
          AvoidCtx ctx;
          ctx.obstacle_id       = next_obs_id_++;
          ctx.initial_yaw       = (float)dyaw_;
          ctx.avoid_yaw_target  = (float)dyaw_; // refined after wall angle scan
          ctx.mode              = AvoidMode::DIAG_OUT;
          ctx.meas.reset();
          ctx.front_block_hyst.threshold = 3;
          ctx.diag_anchored    = false;
          ctx.diag_scan_frames = 0;
          ctx.diag_lateral     = 0.f;
          ctx.diag_fwd         = 0.f;
          ctx.diag_angle       = kDiagAngMax;
          ctx.diag_speed       = kDiagTotalSpd;
          ctx.wall_angle       = 0.f;
          ctx.yaw_aligned      = false;
          ctx.yaw_align_frames = 0;
          ctx.yaw_returned     = false;
          ctx.yaw_return_frames= 0;
          ctx.yaw_return_hold_frames = 0;
          stk_.push(ctx);
          resetYawPID();
          cmd.linear.x  = 0.f;
          cmd.linear.y  = kAvoidFwdSpd;
          cmd.angular.y = -kPitchFF * kAvoidFwdSpd;
          cur_cmd_ = cmd; return;
        } else {
          cmd.linear.x = 0.f;
          cmd.linear.y = kAvoidFwdSpd;
          cur_cmd_ = cmd; return;
        }
      } else {
        cmd.linear.x  = 0.f;
        cmd.linear.y  = kFwdSpd;
        cmd.angular.y = -kPitchFF * kFwdSpd;
        cur_cmd_ = cmd; return;
      }
    }

    AvoidCtx& ctx = stk_.top();

    // ── YAW COMPENSATION ─────────────────────────────────────────────────
    // During scan and YAW_ALIGN phases use initial_yaw as PID target to
    // keep the drone steady while we measure the wall.
    // After alignment, switch to avoid_yaw_target so the drone's nose
    // points along the wall normal throughout departure and wall-follow.
    // In DIAG_IN the target is initial_yaw so the drone returns to track.
    float yaw_target;
    switch (ctx.mode) {
      case AvoidMode::DIAG_IN:
        yaw_target = ctx.initial_yaw;
        break;
      default:
        // DIAG_OUT scan phase: hold initial_yaw.
        // DIAG_OUT YAW_ALIGN + depart + WALL_FWD + EXTRA_5M: avoid_yaw_target.
        yaw_target = (ctx.yaw_aligned) ? ctx.avoid_yaw_target : ctx.initial_yaw;
        break;
    }

    float yaw_corr = yawPID(yaw_target, (float)dyaw_, dt);

    // yaw_cos/sin used by applyYawComp — relative to yaw_target so velocity
    // commands are expressed in the frame the drone is currently trying to hold.
    float yaw_err_now = normAngle(yaw_target - (float)dyaw_);
    float yaw_cos = std::cos(yaw_err_now), yaw_sin = std::sin(yaw_err_now);

    auto applyYawComp = [&](float vx, float vy) -> std::pair<float,float> {
      return { vx*yaw_cos - vy*yaw_sin,  vx*yaw_sin + vy*yaw_cos };
    };

    switch (ctx.mode) {

    // ════════════════════════════════════════════════════════════════════
    //  STATE 1: DIAG_OUT
    //
    //  Sub-phases:
    //    1a — scan (diag_scan_frames 0→kDiagScanFrames): measure wall, hold
    //    1b — YAW_ALIGN: rotate to avoid_yaw_target, zero velocity
    //    1c — depart: fly diagonal away from wall at wall-normal heading
    // ════════════════════════════════════════════════════════════════════
    case AvoidMode::DIAG_OUT:
    {
      // ── Sub-phase 1a: SCAN ────────────────────────────────────────────
      if (ctx.diag_scan_frames < AvoidCtx::kDiagScanFrames) {
        ++ctx.diag_scan_frames;

        float wa = estimateWallAngle(scene_.front_wide);
        ctx.wall_angle = ((ctx.diag_scan_frames - 1) * ctx.wall_angle + wa)
                         / (float)ctx.diag_scan_frames;

        scanBreadthPixels(ctx.meas, ctx.initial_yaw, scene_.front_wide,
                          ctx.wall_angle);

        // Hold position: forward-only at kAvoidFwdSpd, yaw on initial_yaw
        auto [scvx, scvy] = applyYawComp(0.f, kAvoidFwdSpd);
        cmd.linear.x  = scvx;
        cmd.linear.y  = scvy;
        cmd.angular.z = yaw_corr;
        cmd.angular.y = -kPitchFF * kAvoidFwdSpd;

        if (ctx.diag_scan_frames == AvoidCtx::kDiagScanFrames) {
          // Compute geometry
          float bspan = (ctx.meas.lat_max > ctx.meas.lat_min && ctx.meas.breadth_n > 10)
                       ? (ctx.meas.lat_max - ctx.meas.lat_min) : 2.0f;
          ctx.diag_lateral_target = bspan * 0.5f + 1.0f;
          float fwd_avail = std::max(scene_.front_wide - kCrawlDist, 0.3f);
          ctx.diag_angle  = std::clamp(std::atan2(ctx.diag_lateral_target, fwd_avail),
                                       kDiagAngMin, kDiagAngMax);
          ctx.diag_speed  = kDiagTotalSpd;
          ctx.diag_anchored = false;

          // v21 KEY: set avoid_yaw_target to wall normal direction.
          // avoid_yaw_target = initial_yaw + wall_angle
          // After yaw-align the drone's "forward" IS the wall-normal direction,
          // so plain (0, vfwd) commands go straight away from the wall.
          ctx.avoid_yaw_target = normAngle(ctx.initial_yaw + ctx.wall_angle);

          RCLCPP_INFO(get_logger(),
            "╔══════════════════════════════════════════════════════════════════╗\n"
            "║  DIAG_OUT #%d | bspan=%.2fm lat_tgt=%.2fm wall_ang=%.1f°        ║\n"
            "║  geo_angle=%.1f°  fwd_avail=%.2fm                               ║\n"
            "║  v21: avoid_yaw_target=%.1f° — yawing to wall normal now        ║\n"
            "╚══════════════════════════════════════════════════════════════════╝",
            ctx.obstacle_id, bspan, ctx.diag_lateral_target,
            ctx.wall_angle * 180.f/(float)M_PI,
            ctx.diag_angle * 180.f/(float)M_PI, fwd_avail,
            ctx.avoid_yaw_target * 180.f/(float)M_PI);
        }
        break;
      }

      // ── Sub-phase 1b: YAW_ALIGN ───────────────────────────────────────
      // Drone rotates to avoid_yaw_target (wall normal direction).
      // Zero translation velocity — only yaw.
      // Exit when within kYawAlignTol or after kYawAlignMaxFrames (safety).
      if (!ctx.yaw_aligned) {
        ++ctx.yaw_align_frames;
        float yaw_err = std::abs(normAngle(ctx.avoid_yaw_target - (float)dyaw_));
        bool aligned  = (yaw_err < AvoidCtx::kYawAlignTol);
        bool timeout  = (ctx.yaw_align_frames >= AvoidCtx::kYawAlignMaxFrames);

        // Pure yaw rotation — stop translation so the drone doesn't drift
        // into the wall during alignment.
        cmd.linear.x  = 0.f;
        cmd.linear.y  = 0.f;
        cmd.angular.z = yawPID(ctx.avoid_yaw_target, (float)dyaw_, dt);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
          "↻ YAW_ALIGN #%d | err=%.1f° target=%.1f° frame=%d/%d",
          ctx.obstacle_id,
          yaw_err * 180.f/(float)M_PI,
          ctx.avoid_yaw_target * 180.f/(float)M_PI,
          ctx.yaw_align_frames, AvoidCtx::kYawAlignMaxFrames);

        if (aligned || timeout) {
          ctx.yaw_aligned = true;
          resetYawPID(); // fresh PID integrator for departure
          RCLCPP_INFO(get_logger(),
            "✔ YAW_ALIGN #%d done | err=%.1f° (%s) → departing wall-normal",
            ctx.obstacle_id,
            yaw_err * 180.f/(float)M_PI,
            timeout ? "TIMEOUT" : "OK");
        }
        break;
      }

      // ── Sub-phase 1c: DEPART ──────────────────────────────────────────
      // Drone is now facing the wall normal.  vfwd pushes away from wall,
      // vlat moves along the wall face.  No applyWallNormal needed — yaw
      // has already aligned the coordinate frame.
      if (!ctx.diag_anchored) {
        ctx.diag_start_x  = (float)dx_;
        ctx.diag_start_y  = (float)dy_;
        ctx.diag_anchored = true;
        RCLCPP_INFO(get_logger(),
          "→ DIAG_OUT #%d depart | geo_angle=%.1f° lat_target=%.2fm "
          "avoid_yaw=%.1f°",
          ctx.obstacle_id,
          ctx.diag_angle    * 180.f/(float)M_PI,
          ctx.diag_lateral_target,
          ctx.avoid_yaw_target * 180.f/(float)M_PI);
      }

      float vfwd = kAvoidFwdSpd;
      float vlat = std::sqrt(std::max(0.f,
                    kDiagTotalSpd*kDiagTotalSpd - vfwd*vfwd));

      // No applyWallNormal() here — the yaw alignment has already rotated
      // the drone so that its body +X is along the wall face and +Y is
      // away from the wall.  Plain vlat/vfwd are correct.
      auto [dfvx, dfvy] = applyYawComp(vlat, vfwd);
      cmd.linear.x  = dfvx;
      cmd.linear.y  = dfvy;
      cmd.angular.z = yaw_corr;
      cmd.angular.x = kRollFF * vlat;
      cmd.angular.y = -kPitchFF * vfwd;

      ctx.diag_fwd += vfwd * (float)dt;

      // Lateral progress measured in initial_yaw frame so we know when
      // the drone has moved far enough from the original path centreline.
      float cs0 = std::cos(ctx.initial_yaw), sn0 = std::sin(ctx.initial_yaw);
      float ddx = (float)dx_ - ctx.diag_start_x;
      float ddy = (float)dy_ - ctx.diag_start_y;
      ctx.diag_lateral      = std::abs(ddx * (-sn0) + ddy * cs0);
      ctx.shift_accumulated = ctx.diag_lateral;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
        "↗ DIAG_OUT #%d | lat=%.2f/%.2fm | vfwd=%.2f vlat=%.2f | "
        "F=%.2fm | AY=%.1f°",
        ctx.obstacle_id, ctx.diag_lateral, ctx.diag_lateral_target,
        vfwd, vlat, scene_.front_min,
        ctx.avoid_yaw_target * 180.f/(float)M_PI);

      if (ctx.diag_lateral >= ctx.diag_lateral_target &&
          scene_.front_wide >= kTriggerDist) {
        RCLCPP_INFO(get_logger(),
          "✔ DIAG_OUT #%d done lat=%.2fm → WALL_FWD",
          ctx.obstacle_id, ctx.diag_lateral);
        ctx.no_wall_cnt        = 0;
        ctx.wall_seen_this_run = false;
        ctx.meas.wall_seed_x   = (float)dx_;
        ctx.meas.wall_seed_y   = (float)dy_;
        ctx.meas.fwd_min = 1e9f;
        ctx.meas.fwd_max = -1e9f;
        ctx.meas.fwd_n   = 0;
        ctx.front_block_hyst.reset();
        ctx.mode = AvoidMode::WALL_FWD;
      }
      break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STATE 2: WALL_FWD
    //  Drone yaw = avoid_yaw_target throughout (wall-normal heading).
    //  Moving "forward" in avoid_yaw_target frame = moving along wall face.
    // ════════════════════════════════════════════════════════════════════
    case AvoidMode::WALL_FWD:
    {
      float cam_d   = cameraLeftDist();
      bool  cam_w   = (cam_d > 0.3f && cam_d < kWallGoneCam);
      bool  vox_w   = leftVoxelOccupied();
      bool  wall_on = cam_w || vox_w;

      if (wall_on) {
        ctx.wall_seen_this_run = true;
        ctx.no_wall_cnt = 0;
        ctx.meas.feedWallFwdFrame(depth_img_,
          (float)dx_, (float)dy_, (float)alt_, (float)dyaw_,
          kFx, kFy, kDepthMax, 1.0f);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "⬆ WALL_FWD | cam=%s(%.2fm) vox=%s | fwd_span=%.2fm (n=%d) | "
          "%.1fm/s AY=%.1f°",
          cam_w?"WALL":"clr", cam_d, vox_w?"WALL":"clr",
          (ctx.meas.fwd_max > ctx.meas.fwd_min)
            ? ctx.meas.fwd_max - ctx.meas.fwd_min : 0.f,
          ctx.meas.fwd_n, kWallCruiseSpd,
          ctx.avoid_yaw_target * 180.f/(float)M_PI);
      } else {
        ++ctx.no_wall_cnt;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
          "⬆ WALL_FWD | CLEAR cnt=%d/%d | fwd_span=%.2fm",
          ctx.no_wall_cnt, AvoidCtx::kNoWallExit,
          (ctx.meas.fwd_max > ctx.meas.fwd_min)
            ? ctx.meas.fwd_max - ctx.meas.fwd_min : 0.f);
        if (ctx.wall_seen_this_run && ctx.no_wall_cnt >= AvoidCtx::kNoWallExit) {
          RCLCPP_INFO(get_logger(),
            "✔ Wall gone → EXTRA_5M (fwd_span=%.2fm, %d px)",
            (ctx.meas.fwd_max > ctx.meas.fwd_min)
              ? ctx.meas.fwd_max - ctx.meas.fwd_min : 0.f,
            ctx.meas.fwd_n);
          ctx.extra_dist = 0.f;
          ctx.yaw_returned      = false;
          ctx.yaw_return_frames = 0;
          ctx.mode = AvoidMode::EXTRA_5M;
          break;
        }
      }

      bool front_blocked = obstacleInFrontCamera();
      bool hyst_fired    = ctx.front_block_hyst.update(front_blocked);

      if (hyst_fired) {
        if (tryAltitudeDodge()) {
          auto [advx,advy] = applyYawComp(0.f, kWallCruiseSpd);
          cmd.linear.x  = advx;
          cmd.linear.y  = advy;
          cmd.angular.z = yaw_corr;
          break;
        }
        RCLCPP_WARN(get_logger(),
          "⚠ WALL_FWD: front %.2fm blocked → pushing inline DIAG_OUT #%d",
          scene_.front_min, next_obs_id_);
        AvoidCtx ic;
        ic.obstacle_id       = next_obs_id_++;
        ic.initial_yaw       = (float)dyaw_;
        ic.avoid_yaw_target  = (float)dyaw_; // will be refined in scan
        ic.mode              = AvoidMode::DIAG_OUT;
        ic.is_inline         = true;
        ic.meas.reset();
        ic.front_block_hyst.threshold = AvoidCtx::kDiagScanFrames + 1;
        ic.diag_anchored    = false;
        ic.diag_scan_frames = 0;
        ic.diag_lateral     = 0.f;
        ic.diag_fwd         = 0.f;
        ic.diag_angle       = kDiagAngMax;
        ic.diag_speed       = kDiagTotalSpd;
        ic.wall_angle       = 0.f;
        ic.yaw_aligned      = false;
        ic.yaw_align_frames = 0;
        ic.yaw_returned     = false;
        ic.yaw_return_frames= 0;
        ic.yaw_return_hold_frames = 0;
        stk_.push(ic);
        ctx.front_block_hyst.reset();
        avoidanceStep(dt);
        return;
      }

      float spd = (scene_.front_wide < kCrawlDist)
                  ? kWallCruiseSpd * 0.3f
                  : kWallCruiseSpd;
      auto [wvx,wvy] = applyYawComp(0.f, spd);
      cmd.linear.x  = wvx;
      cmd.linear.y  = wvy;
      cmd.angular.z = yaw_corr;
      cmd.angular.y = -kPitchFF * spd;
      break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STATE 3: EXTRA_5M
    //
    //  v21 TWO-PHASE:
    //    Phase A: fly forward at kWallCruiseSpd (avoid_yaw_target heading)
    //             until kExtra5m is covered.
    //    Phase B: yaw back to initial_yaw while holding position.
    //             Once within kYawReturnTol (or timeout) → DIAG_IN.
    //  This ensures DIAG_IN starts with the drone already pointing along
    //  the original mission track so the return diagonal is symmetric.
    // ════════════════════════════════════════════════════════════════════
    case AvoidMode::EXTRA_5M:
    {
      if (!ctx.yaw_returned) {
        // Phase A: forward run
        if (ctx.extra_dist < AvoidCtx::kExtra5m) {
          auto [evx, evy] = applyYawComp(0.f, kWallCruiseSpd);
          cmd.linear.x  = evx;
          cmd.linear.y  = evy;
          cmd.angular.z = yaw_corr;
          cmd.angular.y = -kPitchFF * kWallCruiseSpd;
          ctx.extra_dist += (float)(kWallCruiseSpd * dt);

          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,
            "⬆ EXTRA_A %.2f/%.1fm (%.1fm/s) AY=%.1f°",
            ctx.extra_dist, AvoidCtx::kExtra5m, kWallCruiseSpd,
            ctx.avoid_yaw_target * 180.f/(float)M_PI);
        } else {
          // Phase B: yaw back to mission_yaw_ (v22: was initial_yaw)
          // v22 FIX A: use mission_yaw_ so we always return to the true
          //            mission heading regardless of when initial_yaw was set.
          // v22 FIX B: must hold within 3° for kYawReturnHoldFrames (0.5s)
          //            before accepting alignment. Prevents premature exit
          //            while PID is still settling.
          ++ctx.yaw_return_frames;
          float ye = std::abs(normAngle(mission_yaw_ - (float)dyaw_));
          bool in_tol = (ye < AvoidCtx::kYawReturnTol);
          bool tout   = (ctx.yaw_return_frames >= AvoidCtx::kYawReturnMaxFrames);

          if (in_tol) ++ctx.yaw_return_hold_frames;
          else        ctx.yaw_return_hold_frames = 0;  // reset hold if drifts out

          bool held = (ctx.yaw_return_hold_frames >= AvoidCtx::kYawReturnHoldFrames);

          cmd.linear.x  = 0.f;
          cmd.linear.y  = 0.f;
          cmd.angular.z = yawPID(mission_yaw_, (float)dyaw_, dt);

          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),300,
            "↺ EXTRA_B yaw-back | err=%.1f° hold=%d/%d frame=%d/%d",
            ye * 180.f/(float)M_PI,
            ctx.yaw_return_hold_frames, AvoidCtx::kYawReturnHoldFrames,
            ctx.yaw_return_frames, AvoidCtx::kYawReturnMaxFrames);

          if (held || tout) {
            ctx.yaw_returned = true;
            resetYawPID();
            RCLCPP_INFO(get_logger(),
              "✔ EXTRA_5M yaw-back done | err=%.1f° → DIAG_IN (%s)",
              ye * 180.f/(float)M_PI,
              tout ? "TIMEOUT" : "OK");
            ctx.return_anchored = false;
            ctx.return_dist     = 0.f;
            ctx.return_debounce = 0;
            ctx.mode = AvoidMode::DIAG_IN;
          }
        }
      }
      break;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STATE 4: DIAG_IN
    //  Drone yaw = initial_yaw (restored in EXTRA_5M phase B).
    //  Return diagonal is symmetric to departure: vlat rightward, vfwd fwd.
    // ════════════════════════════════════════════════════════════════════
    case AvoidMode::DIAG_IN:
    {
      if (!ctx.return_anchored) {
        ctx.return_start_x  = (float)dx_;
        ctx.return_start_y  = (float)dy_;
        ctx.return_anchored = true;
        ctx.return_dist     = 0.f;
        ctx.return_debounce = 0;
        RCLCPP_INFO(get_logger(),
          "↘ DIAG_IN #%d anchored return=%.2fm "
          "mission_yaw=%.1f° (initial_yaw=%.1f°)",
          ctx.obstacle_id, ctx.shift_accumulated,
          mission_yaw_           * 180.f/(float)M_PI,
          ctx.initial_yaw        * 180.f/(float)M_PI);
      }

      if (obstacleInFrontCamera()) {
        auto [gvx,gvy] = applyYawComp(0.f, kWallCruiseSpd);
        cmd.linear.x  = gvx;
        cmd.linear.y  = gvy;
        cmd.angular.z = yaw_corr;
        cmd.angular.y = -kPitchFF * kWallCruiseSpd;
        ctx.return_debounce = 0;
        break;
      }

      if (ctx.return_debounce < AvoidCtx::kReturnDebounce) {
        ++ctx.return_debounce;
        auto [dbvx,dbvy] = applyYawComp(0.f, kWallCruiseSpd * 0.4f);
        cmd.linear.x  = dbvx;
        cmd.linear.y  = dbvy;
        cmd.angular.z = yaw_corr;
        break;
      }

      // Return diagonal: mirror of DIAG_OUT departure.
      // vlat is rightward (negative lateral) to undo the leftward shift.
      // No wall normal rotation needed — drone is back on initial_yaw.
      float vfwd = kAvoidFwdSpd;
      float vlat = std::sqrt(std::max(0.f,
                    kDiagTotalSpd*kDiagTotalSpd - vfwd*vfwd));

      auto [rivx, rivy] = applyYawComp(-vlat, vfwd);
      cmd.linear.x  = rivx;
      cmd.linear.y  = rivy;
      cmd.angular.z = yaw_corr;
      cmd.angular.x = kRollFF * (-vlat);
      cmd.angular.y = -kPitchFF * vfwd;

      // v22 FIX C: measure lateral return progress in mission_yaw_ frame.
      float cs0 = std::cos(mission_yaw_), sn0 = std::sin(mission_yaw_);
      float ddx_r = (float)dx_ - ctx.return_start_x;
      float ddy_r = (float)dy_ - ctx.return_start_y;
      ctx.return_dist = std::abs(ddx_r * (-sn0) + ddy_r * cs0);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
        "↘ DIAG_IN #%d | return=%.2f/%.2fm",
        ctx.obstacle_id, ctx.return_dist, ctx.shift_accumulated);

      if (ctx.return_dist >= ctx.shift_accumulated) {
        if (ctx.is_inline && obstacleInFrontCamera()) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 400,
            "↘ DIAG_IN inline #%d | lateral done F=%.2fm — holding",
            ctx.obstacle_id, scene_.front_min);
          break;
        }

        // ── Finalise ──────────────────────────────────────────────────────
        // v22 FIX C: use mission_yaw_ for the obstacle OBB axes
        float fcs = std::cos(mission_yaw_), fsn = std::sin(mission_yaw_);

        float col_cx = (ctx.meas.wall_seed_x + (float)dx_) * 0.5f;
        float col_cy = (ctx.meas.wall_seed_y + (float)dy_) * 0.5f;
        auto  pts    = voxel_.collectInRegion(col_cx, col_cy, 25.f, now());

        bool is_wall       = ctx.wall_seen_this_run && !ctx.is_inline;
        bool is_inline_obs = ctx.is_inline;

        ctx.meas.finalise(pts, fcs, fsn);

        float cal_height  = ctx.meas.height * kHeightCal;
        float cal_length  = ctx.meas.length;
        float cal_breadth = ctx.meas.breadth;
        std::string obs_type = is_inline_obs ? "inline"
                             : (is_wall      ? "wall" : "standalone");
        float wall_angle_deg = ctx.wall_angle * 180.f / (float)M_PI;

        RCLCPP_INFO(get_logger(),
          "╔════════════════════════════════════════════════════════════╗\n"
          "║  OBSTACLE #%d (%s) — FINAL DIMENSIONS                     ║\n"
          "╠════════════════════════════════════════════════════════════╣\n"
          "║  Length  : %7.2f m  (%d fwd-px)                           ║\n"
          "║  Breadth : %7.2f m  (%d br-px)                            ║\n"
          "║  Height  : %7.2f m  (×%.3f, %d vox)                       ║\n"
          "║  Centre  : X=%.2f Y=%.2f Z=%.2f                           ║\n"
          "║  WallAngle: %.1f°   AvoidYaw: %.1f°                       ║\n"
          "╚════════════════════════════════════════════════════════════╝",
          ctx.obstacle_id, obs_type.c_str(),
          cal_length,  ctx.meas.fwd_n,
          cal_breadth, ctx.meas.breadth_n,
          cal_height,  kHeightCal, ctx.meas.n_pts,
          ctx.meas.cx, ctx.meas.cy, ctx.meas.cz,
          wall_angle_deg,
          ctx.avoid_yaw_target * 180.f/(float)M_PI);

        long long ts_ms = (long long)(now().seconds() * 1000.0);
        csv_.write(ts_ms, ctx.obstacle_id, obs_type,
                   cal_length, cal_breadth, cal_height,
                   ctx.meas.cx, ctx.meas.cy, ctx.meas.cz,
                   ctx.meas.n_pts,
                   "pixel-fwd-OBB+pixel-lat-OBB+vox-ht*HtCal",
                   wall_angle_deg);
        RCLCPP_INFO(get_logger(), "✓ CSV → %s", csv_.path().c_str());

        bool was_inline = ctx.is_inline;
        stk_.pop();
        if (was_inline && !stk_.empty()) {
          RCLCPP_INFO(get_logger(),
            "✓ DIAG_IN inline done → resuming WALL_FWD (stack=%zu)", stk_.size());
          stk_.top().front_block_hyst.reset();
          stk_.top().no_wall_cnt = 0;
        } else {
          RCLCPP_INFO(get_logger(), "✓ DIAG_IN done → CRUISING");
        }
      }
      break;
    }

    default: break;
    } // switch

    cur_cmd_ = cmd;
  }

  // ══════════════════════════════════════════════════════════════
  //  ALTITUDE HOLD
  // ══════════════════════════════════════════════════════════════

  void maintainAlt() {
    double target = base_alt_ + (alt_dodge_.active ? alt_dodge_.offset : 0.0);
    double err = target - (alt_ - tkoff_alt_);
    cur_cmd_.linear.z = (std::abs(err)>0.1) ? std::clamp(0.5*err,-0.5,0.5) : 0.0;
  }

  void stopHold() {
    cur_cmd_=geometry_msgs::msg::Twist(); maintainAlt();
  }

  void publishCmd() {
    if (fstate_==FlightState::TAKEOFF&&(alt_-tkoff_alt_)<0.3) return;
    if (fstate_==FlightState::MISSION||fstate_==FlightState::TAKEOFF)
      vel_pub_->publish(cur_cmd_);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}