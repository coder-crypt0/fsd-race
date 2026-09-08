// BLOCK 8 — SAFETY SUPERVISOR (C++). Heartbeat watchdog + plausibility
// checks; latched /safety/ebs_trigger with 10 Hz keepalive (false when
// healthy) so a dead supervisor is itself a trigger condition downstream.

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fsd_msgs/msg/heartbeat.hpp>
#include <fsd_msgs/msg/cone_map.hpp>
#include <fsd_msgs/msg/vehicle_cmd.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::Heartbeat;

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode() : Node("safety_supervisor")
  {
    declare_parameter<std::vector<std::string>>("required_nodes", {
      "stereo_cone", "state_estimation", "cone_mapping",
      "path_planning", "motion_control"});
    declare_parameter<double>("max_steering_rad", 0.35);
    // How long the car may move with an empty map before this is an emergency.
    // This covers driving blind, but it must not fire during BOOTSTRAP: the
    // mapper needs N_CONFIRM sightings of a cone before it publishes anything,
    // so a car that has just started creeping legitimately has an empty map for
    // a few seconds. At 3 s it was killing runs at t=6 s, before the first cone
    // was ever confirmed. 8 s is ~20 m at exploration speed and still well
    // inside the 30 s standstill limit the rules impose (FB2027 D2.6.1).
    declare_parameter<double>("blind_timeout_s", 8.0);
    required_ = get_parameter("required_nodes").as_string_array();
    steer_max_ = get_parameter("max_steering_rad").as_double();
    blind_timeout_ = get_parameter("blind_timeout_s").as_double();
    start_t_ = now().seconds();

    hb_sub_ = create_subscription<Heartbeat>(
      "/safety/heartbeat", fsd::qos_reliable(10),
      [this](Heartbeat::ConstSharedPtr m) {
        last_hb_[m->node_id] = now().seconds();
        if (m->status == Heartbeat::STATUS_ERROR) {
          trigger("node " + m->node_id + " reports ERROR: " + m->message);
        }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", fsd::qos_reliable(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(m); });
    map_sub_ = create_subscription<fsd_msgs::msg::ConeMap>(
      "/mapping/track", fsd::qos_reliable(5),
      [this](fsd_msgs::msg::ConeMap::ConstSharedPtr m) {
        cone_count_ = m->cones.size();
      });
    cmd_sub_ = create_subscription<fsd_msgs::msg::VehicleCmd>(
      "/control/cmd", fsd::qos_reliable(1),
      [this](fsd_msgs::msg::VehicleCmd::ConstSharedPtr m) {
        if (!std::isfinite(m->steering_angle) ||
            !std::isfinite(m->torque_request) || !std::isfinite(m->brake_cmd))
        {
          trigger("NaN/inf in control command");
        } else if (std::abs(m->steering_angle) > steer_max_ * 1.05) {
          trigger("steering command exceeds physical limit");
        }
      });

    pub_ = create_publisher<std_msgs::msg::Bool>(
      "/safety/ebs_trigger", fsd::qos_reliable(10));
    check_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                     [this]() { check(); });
    keepalive_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                         [this]() { keepalive(); });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "safety_supervisor");
  }

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr m)
  {
    const double x = m->pose.pose.position.x;
    const double y = m->pose.pose.position.y;
    const double v = m->twist.twist.linear.x;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(v)) {
      trigger("NaN/inf in odometry");
      return;
    }
    if (have_pose_) {
      const double jump = std::hypot(x - last_x_, y - last_y_);
      if (jump > 1.0) {
        trigger("pose jump " + std::to_string(jump) + " m between odom updates");
      }
    }
    last_x_ = x;
    last_y_ = y;
    have_pose_ = true;
    speed_ = v;
  }

  void check()
  {
    const double t = now().seconds();
    if (t - start_t_ > 5.0) {   // startup grace
      for (const auto & id : required_) {
        const auto it = last_hb_.find(id);
        if (it == last_hb_.end()) {
          trigger("node " + id + " never sent a heartbeat");
        } else if (t - it->second > 0.5) {
          trigger("node " + id + " heartbeat missing");
        }
      }
    }
    if (speed_ > 0.5 && cone_count_ == 0) {
      if (blind_since_ < 0.0) {
        blind_since_ = t;
      } else if (t - blind_since_ > blind_timeout_) {
        trigger("moving with zero confirmed cones for > " +
                std::to_string(static_cast<int>(blind_timeout_)) + " s");
      }
    } else {
      blind_since_ = -1.0;
    }
  }

  void keepalive()
  {
    std_msgs::msg::Bool msg;
    msg.data = triggered_;
    pub_->publish(msg);
  }

  void trigger(const std::string & reason)
  {
    if (triggered_) {
      return;
    }
    triggered_ = true;   // latched until restart
    RCLCPP_ERROR(get_logger(), "EBS TRIGGERED: %s", reason.c_str());
    hb_->set_status(Heartbeat::STATUS_ERROR, "EBS: " + reason);
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub_->publish(msg);
  }

  std::vector<std::string> required_;
  double steer_max_, start_t_;
  std::map<std::string, double> last_hb_;
  bool triggered_{false}, have_pose_{false};
  double last_x_{0.0}, last_y_{0.0}, speed_{0.0}, blind_since_{-1.0};
  double blind_timeout_{8.0};
  size_t cone_count_{0};

  rclcpp::Subscription<Heartbeat>::SharedPtr hb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<fsd_msgs::msg::ConeMap>::SharedPtr map_sub_;
  rclcpp::Subscription<fsd_msgs::msg::VehicleCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr check_timer_, keepalive_timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
