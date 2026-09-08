// BLOCK 6 — MOTION PLANNING & CONTROL (C++). Fixed 50 Hz output.
// Velocity profile (curvature limit + backward braking pass + forward
// traction pass) -> Pure Pursuit lateral + PI longitudinal. Staleness
// policy per spec: >0.5 s path -> decay speed (DEGRADED); >2 s path or
// >0.2 s odom -> emergency_stop (ERROR).
//
// SPEED REGIME. v_max_mps here is the absolute ceiling this car is allowed to
// see, and the dynamics limits (a_lat/a_brake/a_accel) are tyre properties — a
// mode does not change what the tyres can do. What the mode changes is how
// much of the ceiling is usable, and Block 5 owns that call, because only the
// planner knows whether the path ahead is a sensor-horizon-limited exploration
// path, a fully mapped racing line, or a corridor with something in it. That
// arrives as /planning/speed_limit. A cap of zero means stop and hold — a
// legitimate driving state, reported DEGRADED, not an error, so a blocked
// track does not latch the EBS.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fsd_msgs/msg/path_point_array.hpp>
#include <fsd_msgs/msg/speed_limit.hpp>
#include <fsd_msgs/msg/vehicle_cmd.hpp>
#include <fsd_msgs/msg/vehicle_status.hpp>
#include <fsd_msgs/msg/heartbeat.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::Heartbeat;
using fsd_msgs::msg::SpeedLimit;
using fsd_msgs::msg::VehicleCmd;
using fsd_msgs::msg::VehicleStatus;

class MotionControlNode : public rclcpp::Node
{
public:
  MotionControlNode() : Node("motion_control")
  {
    declare_parameter<double>("wheelbase_m", 1.53);
    declare_parameter<double>("max_steering_rad", 0.35);
    declare_parameter<double>("v_max_mps", 5.0);
    declare_parameter<double>("a_lat_max", 4.0);
    declare_parameter<double>("a_brake_max", 5.0);
    declare_parameter<double>("a_accel_max", 3.0);
    declare_parameter<double>("torque_max_nm", 30.0);
    declare_parameter<double>("kp_speed", 12.0);
    declare_parameter<double>("ki_speed", 2.0);
    declare_parameter<double>("lookahead_gain_s", 0.8);
    declare_parameter<double>("lookahead_min_m", 2.0);
    declare_parameter<double>("lookahead_max_m", 8.0);
    declare_parameter<double>("creep_speed_mps", 1.5);
    declare_parameter<double>("creep_timeout_s", 15.0);
    // Used only when no fresh /planning/speed_limit exists (older planner, bag
    // replay, planner restart): the timid v1 speed, never the race one.
    declare_parameter<double>("v_no_cap_mps", 5.0);
    declare_parameter<double>("speed_limit_timeout_s", 1.0);
    declare_parameter<double>("cap_ramp_mps2", 2.0);

    L_ = get_parameter("wheelbase_m").as_double();
    steer_max_ = get_parameter("max_steering_rad").as_double();
    v_max_ = get_parameter("v_max_mps").as_double();
    a_lat_ = get_parameter("a_lat_max").as_double();
    a_brk_ = get_parameter("a_brake_max").as_double();
    a_acc_ = get_parameter("a_accel_max").as_double();
    tq_max_ = get_parameter("torque_max_nm").as_double();
    kp_ = get_parameter("kp_speed").as_double();
    ki_ = get_parameter("ki_speed").as_double();
    kv_ = get_parameter("lookahead_gain_s").as_double();
    ld_min_ = get_parameter("lookahead_min_m").as_double();
    ld_max_ = get_parameter("lookahead_max_m").as_double();
    creep_speed_ = get_parameter("creep_speed_mps").as_double();
    creep_timeout_ = get_parameter("creep_timeout_s").as_double();
    v_no_cap_ = get_parameter("v_no_cap_mps").as_double();
    cap_timeout_ = get_parameter("speed_limit_timeout_s").as_double();
    cap_ramp_ = get_parameter("cap_ramp_mps2").as_double();
    decayed_vmax_ = v_max_;
    allowed_ = std::min(v_max_, v_no_cap_);

    path_sub_ = create_subscription<fsd_msgs::msg::PathPointArray>(
      "/planning/path", fsd::qos_reliable(5),
      [this](fsd_msgs::msg::PathPointArray::ConstSharedPtr m) { on_path(m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", fsd::qos_reliable(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_ = m;
        odom_t_ = now().seconds();
      });
    status_sub_ = create_subscription<VehicleStatus>(
      "/vehicle/status", fsd::qos_reliable(10),
      [this](VehicleStatus::ConstSharedPtr m) {
        as_driving_ = (m->as_state == VehicleStatus::AS_DRIVING);
      });
    cap_sub_ = create_subscription<SpeedLimit>(
      "/planning/speed_limit", fsd::qos_reliable(5),
      [this](SpeedLimit::ConstSharedPtr m) {
        cap_v_ = m->v_max_mps;
        cap_detail_ = m->detail;
        cap_t_ = now().seconds();
      });
    pub_ = create_publisher<VehicleCmd>("/control/cmd", fsd::qos_reliable(1));
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { tick(); });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "motion_control");
  }

private:
  struct PathPt
  {
    double x, y, curvature;
  };

  void on_path(fsd_msgs::msg::PathPointArray::ConstSharedPtr msg)
  {
    if (msg->points.size() < 2) {
      return;  // empty = planner ERROR; staleness policy reacts
    }
    path_.clear();
    for (const auto & p : msg->points) {
      path_.push_back({p.x, p.y, p.curvature});
    }
    v_profile_ = velocity_profile();
    path_t_ = now().seconds();
    decayed_vmax_ = v_max_;
    last_near_ = -1;   // new path: next tick does one full scan
  }

  // O(1) amortized nearest-path-point: search ±20 points around the last
  // result (the car advances a few points per tick at most). Full scan on
  // a fresh path or when the windowed result is implausibly far away.
  size_t nearest_index(double px, double py)
  {
    const auto d2_at = [&](size_t i) {
      const double dx = path_[i].x - px;
      const double dy = path_[i].y - py;
      return dx * dx + dy * dy;
    };
    size_t best_i = 0;
    double best = 1e18;
    if (last_near_ >= 0) {
      const size_t lo = static_cast<size_t>(std::max<long>(0, last_near_ - 20));
      const size_t hi = std::min(path_.size(), static_cast<size_t>(last_near_) + 21);
      for (size_t i = lo; i < hi; ++i) {
        const double d2 = d2_at(i);
        if (d2 < best) {
          best = d2;
          best_i = i;
        }
      }
      if (best < 4.0 * 4.0) {        // plausible: within 4 m of the path
        last_near_ = static_cast<long>(best_i);
        return best_i;
      }
    }
    best = 1e18;
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d2 = d2_at(i);
      if (d2 < best) {
        best = d2;
        best_i = i;
      }
    }
    last_near_ = static_cast<long>(best_i);
    return best_i;
  }

  std::vector<double> velocity_profile() const
  {
    const size_t n = path_.size();
    std::vector<double> v(n), ds(n - 1);
    for (size_t i = 0; i < n; ++i) {
      v[i] = std::min(v_max_, std::sqrt(a_lat_ / std::max(std::abs(path_[i].curvature), 1e-3)));
    }
    for (size_t i = 0; i + 1 < n; ++i) {
      ds[i] = std::hypot(path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y);
    }
    for (size_t i = n - 1; i-- > 0;) {
      v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_brk_ * ds[i]));
    }
    for (size_t i = 1; i < n; ++i) {
      v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * a_acc_ * ds[i - 1]));
    }
    return v;
  }

  void tick()
  {
    const double t = now().seconds();
    VehicleCmd cmd;
    cmd.header.stamp = now();

    if (!odom_) {
      pub_->publish(cmd);   // no odometry yet: no motion
      return;
    }
    const double odom_age = t - odom_t_;
    const double path_age = t - path_t_;

    // Creep-start: once the GO signal is received (AS_DRIVING) but no track
    // has been mapped into a path yet, crawl straight forward so the camera
    // approaches the first cones and the map can bootstrap. The safety
    // supervisor still EBSes if the car moves > 3 s with ZERO mapped cones,
    // so a blind runaway is bounded. Steering held at zero.
    if (path_.empty()) {
      if (creep_start_t_ < 0.0 && as_driving_) {
        creep_start_t_ = t;                       // begin creep window
      }
      const bool within_cap = creep_start_t_ >= 0.0 &&
                              (t - creep_start_t_) < creep_timeout_;
      if (as_driving_ && odom_age <= 0.2 && within_cap) {
        const double v = odom_->twist.twist.linear.x;
        const double err = creep_speed_ - v;
        cmd.torque_request = static_cast<float>(
          std::clamp(kp_ * err, 0.0, tq_max_ * 0.5));  // gentle
        cmd.steering_angle = 0.0f;
        hb_->set_status(Heartbeat::STATUS_DEGRADED, "creep-start: seeking track");
      } else if (as_driving_ && !within_cap) {
        cmd.brake_cmd = 0.3f;                      // give up, stop
        hb_->set_status(Heartbeat::STATUS_ERROR, "creep timeout, no track found");
      } else {
        hb_->set_status(Heartbeat::STATUS_OK, "waiting for GO / path");
      }
      pub_->publish(cmd);
      return;
    }
    creep_start_t_ = -1.0;   // a path exists: reset the creep window
    if (path_age > 2.0 || odom_age > 0.2) {
      cmd.emergency_stop = true;
      cmd.brake_cmd = 1.0f;
      hb_->set_status(Heartbeat::STATUS_ERROR, "stale inputs");
      pub_->publish(cmd);
      return;
    }
    const double allowed = speed_ceiling(t, 1.0 / 50.0);
    const bool holding = cap_fresh_ && cap_v_ <= 0.0;
    if (path_age > 0.5) {
      decayed_vmax_ = std::max(0.0, decayed_vmax_ - 2.0 / 50.0);
      hb_->set_status(Heartbeat::STATUS_DEGRADED, "path stale, decaying speed");
    } else if (holding) {
      hb_->set_status(Heartbeat::STATUS_DEGRADED,
                      cap_detail_.empty() ? "held at zero by the planner"
                                          : cap_detail_);
    } else {
      hb_->set_status(Heartbeat::STATUS_OK);
    }

    const double px = odom_->pose.pose.position.x;
    const double py = odom_->pose.pose.position.y;
    const double pyaw = fsd::yaw_from_quaternion(odom_->pose.pose.orientation);
    const double v = odom_->twist.twist.linear.x;

    // Sticky nearest-point search: the car moves monotonically along the
    // path between ticks, so search a window around the previous index —
    // O(1) amortized instead of a full scan every 20 ms. Falls back to a
    // full scan when the local result looks wrong (car far off path).
    size_t i_near = nearest_index(px, py);

    // ---- Pure Pursuit
    const double ld = std::clamp(kv_ * v, ld_min_, ld_max_);
    size_t i_tgt = path_.size() - 1;
    double acc = 0.0;
    for (size_t i = i_near; i + 1 < path_.size(); ++i) {
      acc += std::hypot(path_[i + 1].x - path_[i].x, path_[i + 1].y - path_[i].y);
      if (acc >= ld) {
        i_tgt = i + 1;
        break;
      }
    }
    const double dx = path_[i_tgt].x - px;
    const double dy = path_[i_tgt].y - py;
    const double tx = dx * std::cos(pyaw) + dy * std::sin(pyaw);
    const double ty = -dx * std::sin(pyaw) + dy * std::cos(pyaw);
    const double alpha = std::atan2(ty, std::max(tx, 1e-6));
    const double ld_act = std::max(std::hypot(tx, ty), 1e-3);
    const double steer = std::atan2(2.0 * L_ * std::sin(alpha), ld_act);
    cmd.steering_angle = static_cast<float>(std::clamp(steer, -steer_max_, steer_max_));

    // ---- Longitudinal PI
    const size_t i_v = std::min(i_near + 2, v_profile_.size() - 1);
    double v_target = std::min({v_profile_[i_v], decayed_vmax_, allowed});
    if (!as_driving_) {
      v_target = 0.0;
    }
    const double err = v_target - v;
    if (holding && v < 0.3) {
      // Stopped in front of something: hold the brake instead of trickling
      // torque, and drop the integrator.
      integ_ = 0.0;
      cmd.torque_request = 0.0f;
      cmd.brake_cmd = 0.3f;
    } else if (err >= -0.3) {
      integ_ = std::clamp(integ_ + err / 50.0, -5.0, 5.0);
      const double tq = kp_ * err + ki_ * integ_;
      cmd.torque_request = static_cast<float>(std::clamp(tq, 0.0, tq_max_));
      cmd.brake_cmd = 0.0f;
    } else {
      integ_ = 0.0;
      cmd.torque_request = 0.0f;
      cmd.brake_cmd = static_cast<float>(std::clamp(-err * 0.25, 0.0, 1.0));
    }
    pub_->publish(cmd);
  }

  // The usable ceiling this tick: the planner's cap, ramped UP so a regime
  // change is a smooth pull rather than a step, but applied DOWNWARD
  // immediately — a lower cap is always a braking request.
  double speed_ceiling(double t, double dt)
  {
    cap_fresh_ = cap_t_ > 0.0 && (t - cap_t_) <= cap_timeout_;
    const double target = std::min(v_max_, cap_fresh_ ? cap_v_ : v_no_cap_);
    if (target < allowed_) {
      allowed_ = target;
    } else {
      allowed_ = std::min(target, allowed_ + cap_ramp_ * dt);
    }
    return allowed_;
  }

  double L_, steer_max_, v_max_, a_lat_, a_brk_, a_acc_, tq_max_;
  double kp_, ki_, kv_, ld_min_, ld_max_, creep_speed_, creep_timeout_;
  double integ_{0.0}, decayed_vmax_;
  double creep_start_t_{-1.0};
  double path_t_{-1.0}, odom_t_{-1.0};
  double v_no_cap_{5.0}, cap_timeout_{1.0}, cap_ramp_{2.0};
  double cap_v_{0.0}, cap_t_{-1.0}, allowed_{0.0};
  bool cap_fresh_{false};
  std::string cap_detail_;
  long last_near_{-1};
  bool as_driving_{true};   // adapter/bridge overrides via /vehicle/status

  std::vector<PathPt> path_;
  std::vector<double> v_profile_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

  rclcpp::Subscription<fsd_msgs::msg::PathPointArray>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<SpeedLimit>::SharedPtr cap_sub_;
  rclcpp::Publisher<VehicleCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}
