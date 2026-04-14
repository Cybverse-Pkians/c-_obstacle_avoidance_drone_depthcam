#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class DepthFinalSineHoldOA : public rclcpp::Node
{
public:
    DepthFinalSineHoldOA()
        : Node("depth_final_sine_hold_oa")
    {
        vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/drone1/setpoint_velocity/cmd_vel", 10);

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/depth_cam/depth/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&DepthFinalSineHoldOA::depthCb, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            20ms, std::bind(&DepthFinalSineHoldOA::controlLoop, this));

        cv::namedWindow("OA DEBUG", cv::WINDOW_NORMAL);
        last_time_ = now();

        RCLCPP_INFO(get_logger(), "FINAL sine-hold-sine OA started");
    }

private:
    /* ================= STATE ================= */
    enum Phase { STRAIGHT, SINE_OUT, HOLD, SINE_BACK };
    Phase phase_{STRAIGHT};

    /* ================= SPEED ================= */
    const double V_FORWARD_ = 3.0;
    const double V_AVOID_   = 1.0;

    /* ================= DEPTH ================= */
    const double DETECT_DIST_ = 3.5;
    const double CLEAR_DIST_  = 3.5;
    const double DEPTH_NEAR_  = 0.25;

    /* ================= CAMERA ================= */
    const int IMG_W_ = 640;
    const double HFOV_ = 1.047;

    /* ================= SAFETY ================= */
    const double SAFETY_MARGIN_ = 0.90;
    const double A_MIN_ = 0.8;
    const double A_MAX_ = 4.5;

    const double MIN_HOLD_TIME_ = 0.25;
    const double MAX_HOLD_TIME_ = 1.5;

    const int NEW_OBS_PX_THRESH_ = 40;

    /* ================= SINE ================= */
    const double SINE_LEN_ = 4.0;
    double sine_progress_{0.0};
    double sine_amplitude_{1.0};
    double achieved_offset_{0.0};

    /* ================= DIRECTION ================= */
    int avoid_dir_{1};                 // +1 = LEFT, -1 = RIGHT
    bool avoid_dir_locked_{false};

    /* ================= OBSTACLE TAG ================= */
    int locked_obs_center_px_{-1};

    /* ================= DEPTH DATA ================= */
    double min_depth_{10.0};
    int obs_width_px_{0};
    int obs_center_px_{-1};
    int free_left_px_{0};
    int free_right_px_{0};

    /* ================= TIME ================= */
    rclcpp::Time last_time_;
    rclcpp::Time hold_start_time_;

    /* ================= ROS ================= */
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ================= DEPTH CALLBACK ================= */
    void depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::TYPE_32FC1);

        const cv::Mat &depth = cv_ptr->image;
        int h = depth.rows;
        int w = depth.cols;

        int r0 = int(0.3 * h);
        int r1 = int(0.6 * h);
        cv::Mat roi = depth(cv::Range(r0, r1), cv::Range::all());

        min_depth_ = 10.0;
        free_left_px_ = free_right_px_ = 0;

        std::vector<int> blocked;

        for (int c = w / 4; c < 3 * w / 4; c++) {
            double d;
            cv::minMaxLoc(roi.col(c), &d, nullptr);

            if (d > DEPTH_NEAR_ && d < DETECT_DIST_) {
                blocked.push_back(c);
                min_depth_ = std::min(min_depth_, d);
            } else {
                (c < w / 2) ? free_left_px_++ : free_right_px_++;
            }
        }

        if (!blocked.empty()) {
            obs_width_px_ = largestBlock(blocked);
            obs_center_px_ = blocked[blocked.size() / 2];
        } else {
            obs_width_px_ = 0;
            obs_center_px_ = -1;
        }

        drawDebug(roi, blocked);
    }

    /* ================= CONTROL LOOP ================= */
    void controlLoop()
    {
        auto t = now();
        double dt = (t - last_time_).seconds();
        last_time_ = t;

        double vx = 0.0;
        double vy = 0.0;

        /* ---------- STRAIGHT ---------- */
        if (phase_ == STRAIGHT) {
            vx = V_FORWARD_;

            if (min_depth_ < DETECT_DIST_) {
                planAvoidance();
                phase_ = SINE_OUT;
            }
        }

        /* ---------- SINE OUT ---------- */
        else if (phase_ == SINE_OUT) {
            sine_progress_ += V_AVOID_ * dt;

            double s = std::clamp(sine_progress_ / SINE_LEN_, 0.0, 1.0);
            achieved_offset_ = sine_amplitude_ * std::sin(M_PI * s / 2.0);

            double dydx =
                avoid_dir_ * sine_amplitude_ *
                (M_PI / (2.0 * SINE_LEN_)) *
                std::cos(M_PI * s / 2.0);

            vx = V_AVOID_ / std::sqrt(1.0 + dydx * dydx);

            
            vy = -vx * dydx;

            if (s >= 1.0) {
                phase_ = HOLD;
                hold_start_time_ = t;
            }
        }

        /* ---------- HOLD ---------- */
        else if (phase_ == HOLD) {
            vx = V_AVOID_;
            vy = 0.0;

            double hold_time = (t - hold_start_time_).seconds();

            bool new_obstacle =
                (obs_center_px_ >= 0 &&
                 std::abs(obs_center_px_ - locked_obs_center_px_)
                    > NEW_OBS_PX_THRESH_);

            if (new_obstacle) {
                planAvoidance();
                phase_ = SINE_OUT;
            }
            else if ((min_depth_ > CLEAR_DIST_ &&
                      achieved_offset_ > 0.85 * sine_amplitude_ &&
                      hold_time > MIN_HOLD_TIME_) ||
                     hold_time > MAX_HOLD_TIME_) {

                sine_progress_ = 0.0;
                phase_ = SINE_BACK;
            }
        }

        /* ---------- SINE BACK ---------- */
        else if (phase_ == SINE_BACK) {
            sine_progress_ += V_AVOID_ * dt;

            double s = std::clamp(sine_progress_ / SINE_LEN_, 0.0, 1.0);

            double dydx =
                -avoid_dir_ * sine_amplitude_ *
                (M_PI / (2.0 * SINE_LEN_)) *
                std::cos(M_PI * s / 2.0);

            vx = V_AVOID_ / std::sqrt(1.0 + dydx * dydx);

            /* 🔥 FINAL FIX APPLIED HERE TOO */
            vy = -vx * dydx;

            if (s >= 1.0) {
                phase_ = STRAIGHT;
                avoid_dir_locked_ = false;
            }
        }

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = t;

        /* YOUR AXIS MAPPING (unchanged) */
        cmd.twist.linear.x = vy;
        cmd.twist.linear.y = vx;
        cmd.twist.angular.z = 0.0;

        vel_pub_->publish(cmd);
    }

    /* ================= AVOIDANCE ================= */
    void planAvoidance()
    {
        sine_progress_ = 0.0;
        achieved_offset_ = 0.0;

        locked_obs_center_px_ = obs_center_px_;

        if (!avoid_dir_locked_) {
            avoid_dir_ =
                (free_right_px_ > free_left_px_) ? -1 : 1;
            avoid_dir_locked_ = true;
        }

        sine_amplitude_ = computeAmplitude();
    }

    double computeAmplitude()
    {
        if (obs_width_px_ <= 0)
            return A_MIN_;

        double angle =
            (double(obs_width_px_) / IMG_W_) * HFOV_;

        double width_m =
            2.0 * min_depth_ * std::tan(angle / 2.0);

        return std::clamp(
            SAFETY_MARGIN_ + width_m / 2.0,
            A_MIN_, A_MAX_);
    }

    int largestBlock(const std::vector<int>& v)
    {
        int best = 1, cur = 1;
        for (size_t i = 1; i < v.size(); i++) {
            cur = (v[i] == v[i - 1] + 1) ? cur + 1 : 1;
            best = std::max(best, cur);
        }
        return best;
    }

    /* ================= GUI ================= */
    void drawDebug(const cv::Mat &roi,
                   const std::vector<int>& blocked)
    {
        cv::Mat viz;
        cv::normalize(roi, viz, 0, 255, cv::NORM_MINMAX);
        viz.convertTo(viz, CV_8UC1);
        cv::applyColorMap(viz, viz, cv::COLORMAP_JET);

        for (int c : blocked)
            cv::line(viz, {c,0}, {c,viz.rows}, {0,0,0}, 1);

        cv::putText(
            viz,
            "Phase: " + phaseName() +
            " Dir: " + (avoid_dir_ > 0 ? "LEFT" : "RIGHT"),
            {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
            {255,255,255}, 2);

        cv::imshow("OA DEBUG", viz);
        cv::waitKey(1);
    }

    std::string phaseName()
    {
        switch (phase_) {
            case STRAIGHT: return "STRAIGHT";
            case SINE_OUT: return "SINE_OUT";
            case HOLD:     return "HOLD";
            case SINE_BACK:return "SINE_BACK";
        }
        return "UNKNOWN";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthFinalSineHoldOA>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
