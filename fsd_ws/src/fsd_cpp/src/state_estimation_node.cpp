// BLOCK 3 — STATE ESTIMATION (C++). Port of the Python reference node.
// /imu/data + /wheel_speeds (+ /localization/correction, optional)
//     -> /odometry/filtered (50 Hz dead reckoning with honest covariance
//        growth; IMU yaw RATE only, never absolute yaw).
//
// Dead reckoning alone is metres out after a lap, which is fatal once the
// planner works off a whole-lap map. Block 4 measures the offset between what
// the cameras see and where the map says those cones are, and this node folds
// it in SLEW-LIMITED: at most max_correction_speed_mps of position and
// max_correction_yaw_rate of heading per second, never a jump. Both reasons
// matter — a jump would break the pure-pursuit lookahead mid-corner, and the
// safety supervisor treats a >1 m pose discontinuity as a failure (correctly:
// a real jump means something is badly wrong).

#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fsd_msgs/msg/wheel_speeds.hpp>
#include <fsd_msgs/msg/pose_correction.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::PoseCorrection;

class StateEstimationNode : public rclcpp::Node
{
public:
  StateEstimationNode() : Node("state_estimation")
  {
    declare_parameter<double>("wheel_radius_m", 0.228);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<bool>("use_map_correction", true);
    declare_parameter<double>("max_correction_speed_mps", 1.0);
    declare_parameter<double>("max_correction_yaw_rate", 0.3);
    declare_parameter<double>("correction_reject_m", 2.0);
    r_ = get_parameter("wheel_radius_m").as_double();
    use_corr_ = get_parameter("use_map_correction").as_bool();
    corr_v_ = get_parameter("max_correction_speed_mps").as_double();
    corr_w_ = get_parameter("max_correction_yaw_rate").as_double();
    corr_reject_ = get_parameter("correction_reject_m").as_double();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", fsd::qos_reliable(10),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        wz_ = m->angular_velocity.z;
        have_imu_ = true;
      });
    wheels_sub_ = create_subscription<fsd_msgs::msg::WheelSpeeds>(
      "/wheel_speeds", fsd::qos_reliable(10),
      [this](fsd_msgs::msg::WheelSpeeds::ConstSharedPtr m) {
        v_ = 0.5 * (m->rl + m->rr) * r_;
        have_wheels_ = true;
      });
    corr_sub_ = create_subscription<PoseCorrection>(
      "/localization/correction", fsd::qos_reliable(10),
      [this](PoseCorrection::ConstSharedPtr m) { on_correction(m); });
    pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/filtered", fsd::qos_reliable(10));

    const double rate = get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               [this]() { step(); });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "state_estimation");
  }

private:
  void on_correction(PoseCorrection::ConstSharedPtr m)
  {
    if (!use_corr_ || !m->valid) {
      return;
    }
    if (std::hypot(m->dx, m->dy) > corr_reject_) {
      return;   // defence in depth; Block 4 gates this too
    }
    // REPLACED, never accumulated: each solve measures the current total
    // offset, which already reflects whatever has been worked off so far.
    pend_x_ = m->dx;
    pend_y_ = m->dy;
    pend_yaw_ = m->dyaw;
    dist_since_fix_ = 0.0;
    have_fix_ = true;
  }

  void apply_correction(double dt)
  {
    const double step_xy = corr_v_ * dt;
    const double d = std::hypot(pend_x_, pend_y_);
    if (d > 1e-9) {
      const double f = std::min(1.0, step_xy / d);
      x_ += pend_x_ * f;
      y_ += pend_y_ * f;
      pend_x_ -= pend_x_ * f;
      pend_y_ -= pend_y_ * f;
    }
    const double step_w = corr_w_ * dt;
    const double aw = std::clamp(pend_yaw_, -step_w, step_w);
    yaw_ = fsd::wrap_angle(yaw_ + aw);
    pend_yaw_ -= aw;
  }

  void step()
  {
    const rclcpp::Time now = this->now();
    const double t = now.seconds();
    if (last_t_ < 0.0) {
      last_t_ = t;
      return;
    }
    const double dt = t - last_t_;
    last_t_ = t;
    if (dt <= 0.0 || dt > 0.5 || !have_imu_ || !have_wheels_) {
      return;
    }

    yaw_ = fsd::wrap_angle(yaw_ + wz_ * dt);
    x_ += v_ * std::cos(yaw_) * dt;
    y_ += v_ * std::sin(yaw_) * dt;
    dist_ += std::abs(v_) * dt;
    dist_since_fix_ += std::abs(v_) * dt;
    apply_correction(dt);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    fsd::quaternion_from_yaw(msg.pose.pose.orientation, yaw_);
    msg.twist.twist.linear.x = v_;
    msg.twist.twist.angular.z = wz_;

    // Honest covariance: ~1% of distance in position, gyro drift in yaw. Once
    // the map is fixing the pose, growth restarts from the last fix instead of
    // from the start of the run — that IS the benefit of localizing, and
    // downstream consumers should see it.
    const double ref = have_fix_ ? dist_since_fix_ : dist_;
    const double pos_var = std::max(0.01, std::pow(0.01 * ref, 2));
    const double yaw_var = std::max(0.005, std::pow(0.002 * ref, 2));
    msg.pose.covariance[0] = pos_var;
    msg.pose.covariance[7] = pos_var;
    msg.pose.covariance[14] = 1e6;
    msg.pose.covariance[21] = 1e6;
    msg.pose.covariance[28] = 1e6;
    msg.pose.covariance[35] = yaw_var;
    msg.twist.covariance[0] = 0.04;
    msg.twist.covariance[35] = 0.01;

    pub_->publish(msg);
  }

  double r_, corr_v_, corr_w_, corr_reject_;
  bool use_corr_{true};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, v_{0.0}, wz_{0.0}, dist_{0.0};
  double dist_since_fix_{0.0};
  double pend_x_{0.0}, pend_y_{0.0}, pend_yaw_{0.0};
  bool have_fix_{false};
  double last_t_{-1.0};
  bool have_imu_{false}, have_wheels_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<fsd_msgs::msg::WheelSpeeds>::SharedPtr wheels_sub_;
  rclcpp::Subscription<PoseCorrection>::SharedPtr corr_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
