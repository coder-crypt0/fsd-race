#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <fsd_msgs/msg/callback_timing.hpp>

namespace fsd {

// Optional best-effort diagnostics: at most 5 reports/s per instrumented callback.
// Measures execution, not queue wait or causal camera-to-actuator latency.
class CallbackTimer {
public:
  explicit CallbackTimer(rclcpp::Node * node) : node_(node) {
    if (node_->declare_parameter<bool>("enable_timing", false)) {
      pub_ = node_->create_publisher<fsd_msgs::msg::CallbackTiming>(
        "/diagnostics/callback_timing", rclcpp::QoS(20).best_effort());
    }
  }

  class Scope {
  public:
    Scope(CallbackTimer & owner, double input_stamp) : owner_(owner) {
      index_ = ++owner_.count_;
      if (!owner_.pub_) { return; }
      started_ = Clock::now();
      sampled_ = started_ - owner_.last_ >= std::chrono::milliseconds(200);
      if (sampled_) {
        owner_.last_ = started_;
        input_stamp_ = input_stamp;
        received_ = owner_.node_->now().seconds();
      }
    }
    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;
    ~Scope() {
      if (!sampled_ || !rclcpp::ok()) { return; }
      const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - started_).count();
      fsd_msgs::msg::CallbackTiming msg;
      msg.header.stamp = owner_.node_->now();
      msg.component = owner_.node_->get_name();
      msg.has_input_stamp = input_stamp_ > 0.0;
      if (msg.has_input_stamp) {
        msg.input_stamp = rclcpp::Time(static_cast<int64_t>(input_stamp_ * 1e9));
        msg.input_age_ms = (received_ - input_stamp_) * 1000.0;
      }
      msg.callback_ms = elapsed;
      msg.callback_index = index_;
      owner_.pub_->publish(msg);
    }
  private:
    using Clock = std::chrono::steady_clock;
    CallbackTimer & owner_;
    Clock::time_point started_{};
    double input_stamp_{0.0}, received_{0.0};
    uint64_t index_{0};
    bool sampled_{false};
  };

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<fsd_msgs::msg::CallbackTiming>::SharedPtr pub_;
  std::chrono::steady_clock::time_point last_{};
  uint64_t count_{0};
};
}  // namespace fsd
