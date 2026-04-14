/*
 * fake_depth_node.cpp
 *
 * Synthetic 32FC1 depth image publisher for bench-testing
 * side_obstacle_avoidance_v4/v5 WITHOUT a real RealSense camera.
 *
 * Mirrors the FakeHCSR04Node pattern exactly:
 *   ros2 param set /fake_depth_node <param> <value>
 * takes effect immediately at the next frame (10 Hz).
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  PARAMETERS  (all settable at runtime via ros2 param set)
 * ─────────────────────────────────────────────────────────────────────────
 *
 *  BACKGROUND
 *    background_dist   float   10.0    Default depth everywhere (m). Keep
 *                                      > kWallGoneCamera (7 m) so avoidance
 *                                      node sees "all clear" by default.
 *
 *  FRONT OBSTACLE
 *    front_dist        float   -1.0    Distance of front obstacle (m).
 *                                      -1 = disabled.
 *    front_width_frac  float    0.35   Fraction of image width occupied
 *                                      (centre columns). 0.35 = full trigger.
 *    front_duration    float    3.0    How long override stays active (s).
 *
 *  LEFT WALL  (columns 0–22% of image)
 *    left_dist         float   -1.0    Distance of left wall (m). -1 = off.
 *    left_duration     float    5.0    Duration (s). Set large for long wall.
 *    left_slant        bool     false  If true, wall is slanted: depth ramps
 *                                      from left_dist at left edge to
 *                                      left_dist + slant_delta at 22% col.
 *
 *  RIGHT WALL  (columns 78–100% of image)
 *    right_dist        float   -1.0    Distance of right wall (m). -1 = off.
 *    right_duration    float    5.0    Duration (s).
 *    right_slant       bool     false  Same as left_slant but for right.
 *
 *  SLANT
 *    slant_delta       float    2.0    Depth difference (m) across slanted
 *                                      wall from near to far edge.
 *
 *  NOISE
 *    noise_stddev      float    0.0    Gaussian noise stddev (m) added to
 *                                      all valid pixels. 0 = clean.
 *
 *  PUBLISH
 *    publish_rate      int      10     Hz. Match avoidance node subscription.
 *    image_width       int      640    Must match avoidance node kFx assumption.
 *    image_height      int      480    Must match avoidance node kFy assumption.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  QUICK REFERENCE — BENCH TEST COMMANDS
 * ─────────────────────────────────────────────────────────────────────────
 *
 *  # Inject front obstacle at 2.5 m for 3 s → triggers LATERAL_SHIFT
 *  ros2 param set /fake_depth_node front_dist 2.5
 *
 *  # Left wall at 3 m for 8 s → tests WALL_FOLLOWING
 *  ros2 param set /fake_depth_node left_dist 3.0
 *  ros2 param set /fake_depth_node left_duration 8.0
 *
 *  # Right wall slanted → tests slant detection + gradient logic
 *  ros2 param set /fake_depth_node right_dist 2.0
 *  ros2 param set /fake_depth_node right_slant true
 *  ros2 param set /fake_depth_node slant_delta 2.5
 *
 *  # Simultaneous: front + left wall (tests nested avoidance)
 *  ros2 param set /fake_depth_node front_dist 2.5
 *  ros2 param set /fake_depth_node left_dist 3.0
 *
 *  # Clear everything manually
 *  ros2 param set /fake_depth_node front_dist -1.0
 *  ros2 param set /fake_depth_node left_dist  -1.0
 *  ros2 param set /fake_depth_node right_dist -1.0
 *
 *  # Watch what the avoidance node sees
 *  ros2 topic echo /realsense/depth_image --no-arr   (header only)
 *  ros2 run image_view image_view --ros-args -r image:=/realsense/depth_image
 * ─────────────────────────────────────────────────────────────────────────
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

// ============================================================
//  Helper — clamp a float to valid depth range
// ============================================================
static float clampDepth(float d) {
  return std::clamp(d, 0.02f, 20.0f);
}

// ============================================================
//  Node
// ============================================================
class FakeDepthNode : public rclcpp::Node
{
public:
  FakeDepthNode() : Node("fake_depth_node")
  {
    // ── Declare all parameters ────────────────────────────────────────
    background_dist_   = declare_parameter<double>("background_dist",   10.0);

    // Front
    front_dist_        = declare_parameter<double>("front_dist",        -1.0);
    front_width_frac_  = declare_parameter<double>("front_width_frac",   0.35);
    front_duration_    = declare_parameter<double>("front_duration",      3.0);

    // Left wall
    left_dist_         = declare_parameter<double>("left_dist",          -1.0);
    left_duration_     = declare_parameter<double>("left_duration",       5.0);
    left_slant_        = declare_parameter<bool>  ("left_slant",         false);

    // Right wall
    right_dist_        = declare_parameter<double>("right_dist",         -1.0);
    right_duration_    = declare_parameter<double>("right_duration",      5.0);
    right_slant_       = declare_parameter<bool>  ("right_slant",        false);

    // Shared slant
    slant_delta_       = declare_parameter<double>("slant_delta",         2.0);

    // Noise & publish
    noise_stddev_      = declare_parameter<double>("noise_stddev",        0.0);
    publish_rate_      = declare_parameter<int>   ("publish_rate",        10);
    image_width_       = declare_parameter<int>   ("image_width",        640);
    image_height_      = declare_parameter<int>   ("image_height",       480);

    // ── Publisher ─────────────────────────────────────────────────────
    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/realsense/depth_image",
      rclcpp::SensorDataQoS());

    // ── Parameter callback (runtime changes) ──────────────────────────
    param_cb_ = add_on_set_parameters_callback(
      std::bind(&FakeDepthNode::onParamChange, this, std::placeholders::_1));

    // ── Timer ─────────────────────────────────────────────────────────
    int period_ms = 1000 / std::max(1, publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&FakeDepthNode::timerCb, this));

    rng_.seed(std::random_device{}());

    RCLCPP_INFO(get_logger(),
      "=======================================================");
    RCLCPP_INFO(get_logger(),
      " fake_depth_node — synthetic 32FC1 depth publisher");
    RCLCPP_INFO(get_logger(),
      " Image: %dx%d @ %d Hz  |  background: %.1f m",
      image_width_, image_height_, publish_rate_, background_dist_);
    RCLCPP_INFO(get_logger(),
      " Inject obstacles at runtime:");
    RCLCPP_INFO(get_logger(),
      "   ros2 param set /fake_depth_node front_dist 2.5");
    RCLCPP_INFO(get_logger(),
      "   ros2 param set /fake_depth_node left_dist  3.0");
    RCLCPP_INFO(get_logger(),
      "   ros2 param set /fake_depth_node right_dist 3.0");
    RCLCPP_INFO(get_logger(),
      "=======================================================");
  }

private:

  // ── Parameter change callback ──────────────────────────────────────────
  rcl_interfaces::msg::SetParametersResult
  onParamChange(const std::vector<rclcpp::Parameter> &params)
  {
    for (const auto &p : params) {
      const std::string &n = p.get_name();

      // ── Background ───────────────────────────────────────────────────
      if (n == "background_dist")  background_dist_ = p.as_double();

      // ── Front obstacle ───────────────────────────────────────────────
      else if (n == "front_dist") {
        front_dist_ = p.as_double();
        if (front_dist_ >= 0.0) {
          front_active_until_ = now() + rclcpp::Duration::from_seconds(front_duration_);
          RCLCPP_INFO(get_logger(),
            "[FRONT] Obstacle at %.2f m for %.1f s → triggers avoidance if < %.1f m",
            front_dist_, front_duration_, 3.5);
        } else {
          RCLCPP_INFO(get_logger(), "[FRONT] Cleared");
        }
      }
      else if (n == "front_width_frac") front_width_frac_ = p.as_double();
      else if (n == "front_duration") {
        front_duration_ = p.as_double();
        // If front is already active, extend it
        if (front_dist_ >= 0.0)
          front_active_until_ = now() + rclcpp::Duration::from_seconds(front_duration_);
      }

      // ── Left wall ────────────────────────────────────────────────────
      else if (n == "left_dist") {
        left_dist_ = p.as_double();
        if (left_dist_ >= 0.0) {
          left_active_until_ = now() + rclcpp::Duration::from_seconds(left_duration_);
          RCLCPP_INFO(get_logger(),
            "[LEFT]  Wall at %.2f m for %.1f s  slant=%s",
            left_dist_, left_duration_, left_slant_ ? "ON" : "OFF");
        } else {
          RCLCPP_INFO(get_logger(), "[LEFT]  Cleared");
        }
      }
      else if (n == "left_duration") {
        left_duration_ = p.as_double();
        if (left_dist_ >= 0.0)
          left_active_until_ = now() + rclcpp::Duration::from_seconds(left_duration_);
      }
      else if (n == "left_slant") {
        left_slant_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "[LEFT]  Slant %s", left_slant_ ? "ON" : "OFF");
      }

      // ── Right wall ───────────────────────────────────────────────────
      else if (n == "right_dist") {
        right_dist_ = p.as_double();
        if (right_dist_ >= 0.0) {
          right_active_until_ = now() + rclcpp::Duration::from_seconds(right_duration_);
          RCLCPP_INFO(get_logger(),
            "[RIGHT] Wall at %.2f m for %.1f s  slant=%s",
            right_dist_, right_duration_, right_slant_ ? "ON" : "OFF");
        } else {
          RCLCPP_INFO(get_logger(), "[RIGHT] Cleared");
        }
      }
      else if (n == "right_duration") {
        right_duration_ = p.as_double();
        if (right_dist_ >= 0.0)
          right_active_until_ = now() + rclcpp::Duration::from_seconds(right_duration_);
      }
      else if (n == "right_slant") {
        right_slant_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "[RIGHT] Slant %s", right_slant_ ? "ON" : "OFF");
      }

      // ── Slant delta ──────────────────────────────────────────────────
      else if (n == "slant_delta") slant_delta_ = p.as_double();

      // ── Noise ────────────────────────────────────────────────────────
      else if (n == "noise_stddev") noise_stddev_ = p.as_double();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // ── Timer callback — builds and publishes the depth image ──────────────
  void timerCb()
  {
    rclcpp::Time t = now();

    // Evaluate which obstacles are currently active
    bool front_on = (front_dist_ >= 0.0) && (t < front_active_until_);
    bool left_on  = (left_dist_  >= 0.0) && (t < left_active_until_);
    bool right_on = (right_dist_ >= 0.0) && (t < right_active_until_);

    int W = image_width_, H = image_height_;

    // Column ranges — mirror exactly what analyzeScene() samples:
    //   front:  cols 35%–65%
    //   left:   cols 0–22%  (and 22%–38% secondary band)
    //   right:  cols 78%–100%  (and 62%–78% secondary band)
    int front_c0 = (int)(W * 0.35f),  front_c1 = (int)(W * 0.65f);
    int left_c0  = 0,                  left_c1  = (int)(W * 0.38f); // fill both L bands
    int right_c0 = (int)(W * 0.62f),  right_c1 = W;                // fill both R bands

    // Row range — mirror analyzeScene() band:  rows 15%–55%
    int r0 = (int)(H * 0.15f), r1 = (int)(H * 0.55f);

    // Allocate float image, fill with background
    std::vector<float> img(W * H, (float)background_dist_);

    std::normal_distribution<float> noise_dist(0.f, (float)noise_stddev_);

    for (int r = 0; r < H; ++r) {
      bool in_band = (r >= r0 && r < r1);  // row inside detection band

      for (int c = 0; c < W; ++c) {
        float d = (float)background_dist_;

        // ── FRONT obstacle ───────────────────────────────────────────
        if (front_on && in_band && c >= front_c0 && c < front_c1) {
          // Apply front width fraction centred on image
          int half_w = (int)(W * front_width_frac_ * 0.5f);
          int centre  = W / 2;
          if (c >= centre - half_w && c < centre + half_w)
            d = std::min(d, (float)front_dist_);
        }

        // ── LEFT wall ────────────────────────────────────────────────
        if (left_on && in_band && c >= left_c0 && c < left_c1) {
          float base = (float)left_dist_;
          if (left_slant_) {
            // Depth ramps from left_dist at c=0 to left_dist+slant_delta at c=left_c1
            // This creates a depth gradient that analyzeScene()'s 8-strip
            // weighted-centroid regression will detect as a slant.
            float t_col = (float)c / (float)(left_c1 - 1);
            base += (float)(slant_delta_ * t_col);
          }
          d = std::min(d, base);
        }

        // ── RIGHT wall ───────────────────────────────────────────────
        if (right_on && in_band && c >= right_c0 && c < right_c1) {
          float base = (float)right_dist_;
          if (right_slant_) {
            // Depth ramps from right_dist+slant_delta at c=right_c0
            // to right_dist at c=W-1
            float t_col = (float)(c - right_c0) / (float)(W - 1 - right_c0);
            base += (float)(slant_delta_ * (1.0f - t_col));
          }
          d = std::min(d, base);
        }

        // ── Noise ────────────────────────────────────────────────────
        if (noise_stddev_ > 0.f)
          d += noise_dist(rng_);

        img[r * W + c] = clampDepth(d);
      }
    }

    // ── Pack into ROS Image message ──────────────────────────────────────
    sensor_msgs::msg::Image msg;
    msg.header.stamp    = t;
    msg.header.frame_id = "camera_link";
    msg.height          = H;
    msg.width           = W;
    msg.encoding        = "32FC1";
    msg.is_bigendian    = false;
    msg.step            = W * sizeof(float);
    msg.data.resize(W * H * sizeof(float));
    std::memcpy(msg.data.data(), img.data(), msg.data.size());

    pub_->publish(msg);

    // ── Throttled status log ─────────────────────────────────────────────
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Publishing depth | front=%s(%.1fm) left=%s(%.1fm%s) right=%s(%.1fm%s) bg=%.1fm",
      front_on ? "ON" : "off", front_dist_,
      left_on  ? "ON" : "off", left_dist_,  left_slant_  ? ",slant" : "",
      right_on ? "ON" : "off", right_dist_, right_slant_ ? ",slant" : "",
      background_dist_);
  }

  // ── Members ──────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Background
  double background_dist_;

  // Front
  double front_dist_;
  double front_width_frac_;
  double front_duration_;
  rclcpp::Time front_active_until_{0, 0, RCL_ROS_TIME};

  // Left
  double left_dist_;
  double left_duration_;
  bool   left_slant_;
  rclcpp::Time left_active_until_{0, 0, RCL_ROS_TIME};

  // Right
  double right_dist_;
  double right_duration_;
  bool   right_slant_;
  rclcpp::Time right_active_until_{0, 0, RCL_ROS_TIME};

  // Slant
  double slant_delta_;

  // Noise & config
  double noise_stddev_;
  int    publish_rate_;
  int    image_width_;
  int    image_height_;

  std::mt19937 rng_;
};

// ============================================================
//  ENTRY POINT
// ============================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDepthNode>());
  rclcpp::shutdown();
  return 0;
}