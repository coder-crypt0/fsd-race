// Shared helpers for all fsd_cpp nodes: QoS per spec §2.2, the mandatory
// heartbeat emitter, timestamped pose buffer, and small math utilities.
#pragma once

#include <cmath>
#include <deque>
#include <string>
#include <tuple>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fsd_msgs/msg/heartbeat.hpp>

namespace fsd
{

inline rclcpp::QoS qos_reliable(size_t depth = 5)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
}

inline rclcpp::QoS qos_best_effort(size_t depth = 1)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
}

inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline void quaternion_from_yaw(geometry_msgs::msg::Quaternion & q, double yaw)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
}

inline double stamp_to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// ---------------------------------------------------------------- heartbeat
class HeartbeatEmitter
{
public:
  HeartbeatEmitter(rclcpp::Node * node, const std::string & node_id,
                   double rate_hz = 10.0)
  : node_(node), id_(node_id)
  {
    pub_ = node->create_publisher<fsd_msgs::msg::Heartbeat>(
      "/safety/heartbeat", qos_reliable(10));
    timer_ = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      [this]() { tick(); });
  }

  void set_status(uint8_t status, const std::string & text = "")
  {
    status_ = status;
    text_ = text;
  }

private:
  void tick()
  {
    fsd_msgs::msg::Heartbeat m;
    m.header.stamp = node_->now();
    m.node_id = id_;
    m.status = status_;
    m.message = text_;
    pub_->publish(m);
  }

  rclcpp::Node * node_;
  std::string id_;
  uint8_t status_{fsd_msgs::msg::Heartbeat::STATUS_OK};
  std::string text_;
  rclcpp::Publisher<fsd_msgs::msg::Heartbeat>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// --------------------------------------------------------------- pose buffer
struct Pose2D
{
  double x, y, yaw;
};

class PoseBuffer
{
public:
  explicit PoseBuffer(size_t maxlen = 300) : maxlen_(maxlen) {}

  void add(double t, double x, double y, double yaw)
  {
    if (!buf_.empty() && t <= std::get<0>(buf_.back())) {
      return;
    }
    buf_.emplace_back(t, x, y, yaw);
    while (buf_.size() > maxlen_) {
      buf_.pop_front();
    }
  }

  std::optional<Pose2D> query(double t) const
  {
    if (buf_.empty()) {
      return std::nullopt;
    }
    if (t <= std::get<0>(buf_.front())) {
      const auto & [t0, x, y, yaw] = buf_.front();
      (void)t0;
      return Pose2D{x, y, yaw};
    }
    if (t >= std::get<0>(buf_.back())) {
      const auto & [t0, x, y, yaw] = buf_.back();
      (void)t0;
      return Pose2D{x, y, yaw};
    }
    size_t lo = 0, hi = buf_.size() - 1;
    while (hi - lo > 1) {
      size_t mid = (lo + hi) / 2;
      if (std::get<0>(buf_[mid]) <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    const auto & [t0, x0, y0, yaw0] = buf_[lo];
    const auto & [t1, x1, y1, yaw1] = buf_[hi];
    if (t1 <= t0) {
      return Pose2D{x1, y1, yaw1};
    }
    const double a = (t - t0) / (t1 - t0);
    return Pose2D{
      x0 + a * (x1 - x0),
      y0 + a * (y1 - y0),
      wrap_angle(yaw0 + a * wrap_angle(yaw1 - yaw0))};
  }

private:
  size_t maxlen_;
  std::deque<std::tuple<double, double, double, double>> buf_;
};

}  // namespace fsd
