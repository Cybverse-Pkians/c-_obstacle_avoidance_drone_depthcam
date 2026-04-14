#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class DepthHalfSineAdaptiveOA : public rclcpp::Node
{
public:
    DepthHalfSineAdaptiveOA() : Node("depth_half_sine_adaptive_oa")
    {
        vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/drone1/setpoint_velocity/cmd_vel", 10);

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/depth_cam/depth/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&DepthHalfSineAdaptiveOA::depthCb, this, std::placeholders::_1));

        last_time_ = now();

        timer_ = create_wall_timer(
            20ms, std::bind(&DepthHalfSineAdaptiveOA::controlLoop, this));

        cv::namedWindow("SMART Half-Sine OA", cv::WINDOW_NORMAL);

        RCLCPP_INFO(get_logger(),
            "SMART Half-Sine OA started (avoid → slow → widen → stop)");
    }

private:
    /* ================= STATE ================= */
    enum Mode { STRAIGHT, AVOIDING, STOPPED };
    Mode mode_{STRAIGHT};

    /* ================= SPEED ================= */
    const double V_NOMINAL_ = 2.0;
    const double V_MIN_ = 0.5;
    double V_cmd_{2.0};

    /* ================= DEPTH / DISTANCE ================= */
    const double DETECT_DIST_ = 6.0;
    const double CLEAR_DIST_  = 7.0;
    const double DEPTH_NEAR_  = 0.2;

    /* ================= GEOMETRY ================= */
    const double HFOV_ = 1.047; // ~60 deg
    const double SAFETY_MARGIN_ = 0.6;
    const double A_MIN_ = 0.6;
    const double A_MAX_ = 3.0;

    /* ================= DYNAMICS ================= */
    const double L_MIN_ = 6.0;
    const double L_GAIN_ = 4.0;
    const double MAX_LAT_ACC_ = 2.0;
    const double MAX_LAT_VEL_ = 1.5;

    /* ================= DEPTH DATA ================= */
    double min_depth_{10.0};
    int obs_width_px_{0};
    int free_left_px_{0};
    int free_right_px_{0};

    /* ================= PATH ================= */
    double progress_{0.0};
    double amplitude_{1.0};
    double avoid_len_{8.0};
    int avoid_dir_{1};

    rclcpp::Time stop_start_;

    /* ================= ROS ================= */
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    /* ================= DEPTH CALLBACK ================= */
    void depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::TYPE_32FC1);

        const cv::Mat &depth = cv_ptr->image;
        int h = depth.rows;
        int w = depth.cols;

        int r0 = static_cast<int>(0.25 * h);
        int r1 = static_cast<int>(0.55 * h);
        cv::Mat roi = depth(cv::Range(r0, r1), cv::Range::all());

        std::vector<int> blocked;
        min_depth_ = 10.0;
        free_left_px_ = 0;
        free_right_px_ = 0;

        for (int c = w / 4; c < 3 * w / 4; c++) {
            double d;
            cv::minMaxLoc(roi.col(c), &d, nullptr);

            if (d > DEPTH_NEAR_ && d < DETECT_DIST_) {
                blocked.push_back(c);
                min_depth_ = std::min(min_depth_, d);
            } else {
                if (c < w / 2) free_left_px_++;
                else free_right_px_++;
            }
        }

        obs_width_px_ = largestBlock(blocked);
        drawGUI(roi, blocked);
    }

    /* ================= FEASIBILITY CHECK ================= */
    bool feasible(double A, double L, double V)
    {
        double kappa = A * M_PI * M_PI / (L * L);
        double alat = V * V * kappa;
        return alat <= MAX_LAT_ACC_;
    }

    /* ================= CONTROL LOOP ================= */
    void controlLoop()
    {
        auto t = now();
        double dt = (t - last_time_).seconds();
        last_time_ = t;

        double vx = V_cmd_;
        double vy = 0.0;

        /* ========== START AVOIDANCE ========== */
        if (mode_ == STRAIGHT && min_depth_ < DETECT_DIST_) {
            mode_ = AVOIDING;
            progress_ = 0.0;

            avoid_dir_ = (free_right_px_ > free_left_px_) ? -1 : 1;
            amplitude_ = computeAmplitude();
            V_cmd_ = V_NOMINAL_;

            bool solution_found = false;

            for (double v = V_NOMINAL_; v >= V_MIN_; v *= 0.85) {
                double L = std::max(L_MIN_, L_GAIN_ * v);

                for (double A = amplitude_; A <= A_MAX_; A += 0.25) {
                    if (feasible(A, L, v)) {
                        V_cmd_ = v;
                        avoid_len_ = L;
                        amplitude_ = A;
                        solution_found = true;
                        break;
                    }
                }
                if (solution_found) break;
            }

            if (!solution_found) {
                mode_ = STOPPED;
                stop_start_ = t;
            }
        }

        /* ========== AVOIDANCE ========== */
        if (mode_ == AVOIDING) {
            progress_ += V_cmd_ * dt;

            double s = std::clamp(progress_ / avoid_len_, 0.0, 1.0);
            double dydx =
                avoid_dir_ * amplitude_ *
                (M_PI / avoid_len_) * std::cos(M_PI * s);

            vx = V_cmd_ / std::sqrt(1.0 + dydx * dydx);
            vy = std::clamp(vx * dydx, -MAX_LAT_VEL_, MAX_LAT_VEL_);

            if (progress_ >= avoid_len_ && min_depth_ > CLEAR_DIST_) {
                mode_ = STRAIGHT;
                V_cmd_ = V_NOMINAL_;
            }
        }

        /* ========== STOP MODE ========== */
        if (mode_ == STOPPED) {
            vx = 0.0;
            vy = 0.0;

            if ((t - stop_start_).seconds() > 0.5 && min_depth_ > 1.2) {
                mode_ = STRAIGHT;
                V_cmd_ = V_MIN_;
            }
        }

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = t;

        /* AXIS FIX (YOUR SYSTEM) */
        cmd.twist.linear.x = vy;   // roll
        cmd.twist.linear.y = vx;   // pitch
        cmd.twist.linear.z = 0.0;
        cmd.twist.angular.z = 0.0;

        vel_pub_->publish(cmd);
    }

    /* ================= HELPERS ================= */
    double computeAmplitude()
    {
        if (obs_width_px_ <= 0) return A_MIN_;
        double angle = (obs_width_px_ / 640.0) * HFOV_;
        double width = 2.0 * min_depth_ * std::tan(angle / 2.0);
        return std::clamp(SAFETY_MARGIN_ + width / 2.0, A_MIN_, A_MAX_);
    }

    int largestBlock(const std::vector<int>& v)
    {
        if (v.empty()) return 0;
        int best = 1, cur = 1;
        for (size_t i = 1; i < v.size(); i++) {
            cur = (v[i] == v[i-1] + 1) ? cur + 1 : 1;
            best = std::max(best, cur);
        }
        return best;
    }

    /* ================= GUI ================= */
    void drawGUI(const cv::Mat &roi, const std::vector<int>& blocked)
    {
        cv::Mat viz;
        cv::normalize(roi, viz, 0, 255, cv::NORM_MINMAX);
        viz.convertTo(viz, CV_8UC1);
        cv::applyColorMap(viz, viz, cv::COLORMAP_JET);

        for (int c : blocked)
            cv::line(viz, {c,0}, {c,viz.rows}, {0,0,0}, 1);

        cv::putText(viz, "Speed: " + std::to_string(V_cmd_),
            {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);

        cv::putText(viz, "Free L/R: " +
            std::to_string(free_left_px_) + "/" +
            std::to_string(free_right_px_),
            {10,60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);

        cv::putText(viz, "Mode: " +
            std::string(mode_ == STRAIGHT ? "STRAIGHT" :
                        mode_ == AVOIDING ? "AVOIDING" : "STOPPED"),
            {10,90}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);

        cv::imshow("SMART Half-Sine OA", viz);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthHalfSineAdaptiveOA>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
