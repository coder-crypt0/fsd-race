// FSDS ADAPTER — glue between the FSDS ROS2 bridge and the fsd_msgs
// contracts. Replaces Block 7 while running against the simulator.
//
//   /control/cmd (VehicleCmd)  ->  /control_command (fs_msgs/ControlCommand)
//   /wheel_states (WheelStates) ->  /wheel_speeds (fsd_msgs/WheelSpeeds)
//   /signal/go                 ->  AS_READY -> AS_DRIVING
//   /signal/finished           ->  AS_FINISHED
//   /safety/ebs_trigger        ->  latched full brake (AS_EMERGENCY)
//   50 Hz                      ->  /vehicle/status (fsd_msgs/VehicleStatus)
//
// Topic names verified against the FSDS ros2 bridge v2.2.0 running live:
// the main bridge node publishes at the root namespace (/imu, /control_command,
// /signal/go, /signal/finished, /wheel_states), while the camera nodes use
// absolute frame ids (/fsds/cam_left/image_color) — those are remapped in
// fsds.launch.py.
//
// Speed source is /wheel_states (per-wheel RPM), the legitimate wheel-encoder
// equivalent — the same signal class the real car's encoders provide. FSDS's
// GSS ground-speed sensor is not used (not always published, and a magic
// ground-speed sensor is not what the real car has).

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fsd_msgs/msg/vehicle_cmd.hpp>
#include <fsd_msgs/msg/vehicle_status.hpp>
#include <fsd_msgs/msg/wheel_speeds.hpp>
#include <fs_msgs/msg/control_command.hpp>
#include <fs_msgs/msg/go_signal.hpp>
#include <fs_msgs/msg/finished_signal.hpp>
#include <fs_msgs/msg/wheel_states.hpp>

#include "fsd_cpp/common.hpp"

using fsd_msgs::msg::VehicleCmd;
using fsd_msgs::msg::VehicleStatus;

namespace
{
constexpr double kRpmToRadPerSec = 2.0 * M_PI / 60.0;
}

class FsdsAdapterNode : public rclcpp::Node
{
public:
  FsdsAdapterNode() : Node("fsds_adapter")
  {
    declare_parameter<double>("max_steering_rad", 0.35);
    declare_parameter<double>("torque_max_nm", 30.0);
    // Our convention: +steering = left. AirSim/FSDS: +1 = right lock.
    // If the car steers mirror-image in the sim, flip this to +1.0.
    declare_parameter<double>("steering_sign", -1.0);
    // Skip waiting for the GO signal during bench bring-up.
    declare_parameter<bool>("auto_go", false);

    steer_max_ = get_parameter("max_steering_rad").as_double();
    tq_max_ = get_parameter("torque_max_nm").as_double();
    steer_sign_ = get_parameter("steering_sign").as_double();
    go_ = get_parameter("auto_go").as_bool();

    cmd_sub_ = create_subscription<VehicleCmd>(
      "/control/cmd", fsd::qos_reliable(1),
      [this](VehicleCmd::ConstSharedPtr m) { on_cmd(m); });
    ebs_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/ebs_trigger", fsd::qos_reliable(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data) {
          ebs_ = true;
        }
      });
    go_sub_ = create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", fsd::qos_reliable(5),
      [this](fs_msgs::msg::GoSignal::ConstSharedPtr) {
        if (!go_) {
          RCLCPP_INFO(get_logger(), "GO signal received");
        }
        go_ = true;
      });
    finished_sub_ = create_subscription<fs_msgs::msg::FinishedSignal>(
      "/signal/finished", fsd::qos_reliable(5),
      [this](fs_msgs::msg::FinishedSignal::ConstSharedPtr) { finished_ = true; });

    wheels_sub_ = create_subscription<fs_msgs::msg::WheelStates>(
      "/wheel_states", fsd::qos_best_effort(5),
      [this](fs_msgs::msg::WheelStates::ConstSharedPtr m) {
        fsd_msgs::msg::WheelSpeeds w;
        w.header.stamp = m->header.stamp;
        w.fl = static_cast<float>(m->fl_rpm * kRpmToRadPerSec);
        w.fr = static_cast<float>(m->fr_rpm * kRpmToRadPerSec);
        w.rl = static_cast<float>(m->rl_rpm * kRpmToRadPerSec);
        w.rr = static_cast<float>(m->rr_rpm * kRpmToRadPerSec);
        wheels_pub_->publish(w);
      });

    control_pub_ = create_publisher<fs_msgs::msg::ControlCommand>(
      "/control_command", fsd::qos_reliable(1));
    wheels_pub_ = create_publisher<fsd_msgs::msg::WheelSpeeds>(
      "/wheel_speeds", fsd::qos_reliable(10));
    status_pub_ = create_publisher<VehicleStatus>(
      "/vehicle/status", fsd::qos_reliable(10));

    status_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                      [this]() { publish_status(); });
    hb_ = std::make_unique<fsd::HeartbeatEmitter>(this, "fsds_adapter");
  }

private:
  void on_cmd(VehicleCmd::ConstSharedPtr m)
  {
    fs_msgs::msg::ControlCommand out;
    out.header.stamp = now();
    if (ebs_ || m->emergency_stop || !go_ || finished_) {
      out.throttle = 0.0f;
      out.steering = 0.0f;
      out.brake = 1.0f;
    } else {
      out.throttle = static_cast<float>(
        std::clamp(static_cast<double>(m->torque_request) / tq_max_, 0.0, 1.0));
      out.steering = static_cast<float>(std::clamp(
        steer_sign_ * m->steering_angle / steer_max_, -1.0, 1.0));
      out.brake = static_cast<float>(
        std::clamp(static_cast<double>(m->brake_cmd), 0.0, 1.0));
    }
    last_steer_ = m->steering_angle;
    control_pub_->publish(out);
  }

  void publish_status()
  {
    VehicleStatus s;
    s.header.stamp = now();
    s.actual_steering_angle = static_cast<float>(last_steer_);
    if (ebs_) {
      s.as_state = VehicleStatus::AS_EMERGENCY;
    } else if (finished_) {
      s.as_state = VehicleStatus::AS_FINISHED;
    } else if (go_) {
      s.as_state = VehicleStatus::AS_DRIVING;
    } else {
      s.as_state = VehicleStatus::AS_READY;
    }
    status_pub_->publish(s);
  }

  double steer_max_, tq_max_, steer_sign_;
  bool go_{false}, finished_{false}, ebs_{false};
  double last_steer_{0.0};

  rclcpp::Subscription<VehicleCmd>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ebs_sub_;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr go_sub_;
  rclcpp::Subscription<fs_msgs::msg::FinishedSignal>::SharedPtr finished_sub_;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr wheels_sub_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::Publisher<fsd_msgs::msg::WheelSpeeds>::SharedPtr wheels_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<fsd::HeartbeatEmitter> hb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FsdsAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
