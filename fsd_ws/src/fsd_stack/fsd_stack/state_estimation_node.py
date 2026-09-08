"""BLOCK 3 — STATE ESTIMATION (owner: Mapping & Planning).

/imu/data + /wheel_speeds (+ /gps/fix optional, unused in v1)
        + /localization/correction (Block 4, optional)
        -> /odometry/filtered (nav_msgs/Odometry, odom frame)

v1 is a dead-reckoning fusion: wheel odometry for speed, IMU gyro for yaw
rate, integrated at 50 Hz with honest covariance growth. The BNO055's
absolute heading drifts over a lap, so only its RATE is used, per spec.

Dead reckoning alone is metres out after a lap, which is fatal once the
planner works off a whole-lap map. Block 4 measures the offset between what
the cameras see and where the map says those cones are, and this node folds
that in as a SLEW-LIMITED correction: at most `max_correction_speed_mps` of
position and `max_correction_yaw_rate` of heading per second, never a jump.
That matters for two reasons — a jump would break the pure-pursuit lookahead
mid-corner, and the safety supervisor treats a >1 m pose discontinuity as a
failure (correctly: a real jump means something is badly wrong).

Drop-in upgrade path: replace this node with robot_localization's ekf_node
(same output topic/frame contract) once a tuned ekf.yaml exists.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from fsd_msgs.msg import WheelSpeeds, PoseCorrection

from .common import HeartbeatEmitter, qos_reliable, quaternion_from_yaw, wrap_angle


class StateEstimationNode(Node):
    def __init__(self):
        super().__init__('state_estimation')
        self.declare_parameter('wheel_radius_m', 0.228)   # Hoosier R18 ~18" OD
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('use_map_correction', True)
        self.declare_parameter('max_correction_speed_mps', 1.0)
        self.declare_parameter('max_correction_yaw_rate', 0.3)
        self.declare_parameter('correction_reject_m', 2.0)

        self._r = float(self.get_parameter('wheel_radius_m').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._use_corr = bool(self.get_parameter('use_map_correction').value)
        self._corr_v = float(self.get_parameter('max_correction_speed_mps').value)
        self._corr_w = float(self.get_parameter('max_correction_yaw_rate').value)
        self._corr_reject = float(self.get_parameter('correction_reject_m').value)

        # State: pose in odom, zeroed at node start (AS Ready re-zero = restart
        # this node, or call the reset by relaunching — v1 keeps it simple).
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0          # m/s from wheels
        self._wz = 0.0         # rad/s from IMU gyro
        self._dist = 0.0       # total distance, drives covariance growth
        self._dist_since_fix = 0.0   # distance since the last map fix
        self._have_wheels = False
        self._have_imu = False
        self._last_t = None
        # Correction still to be worked off, in odom frame. REPLACED by each
        # new solve (it measures the current total offset), never accumulated.
        self._pend = [0.0, 0.0, 0.0]
        self._have_fix = False

        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_reliable(10))
        self.create_subscription(WheelSpeeds, '/wheel_speeds', self._on_wheels,
                                 qos_reliable(10))
        self.create_subscription(PoseCorrection, '/localization/correction',
                                 self._on_correction, qos_reliable(10))
        self._pub = self.create_publisher(Odometry, '/odometry/filtered',
                                          qos_reliable(10))
        self.create_timer(1.0 / rate, self._step)
        self._hb = HeartbeatEmitter(self, 'state_estimation')

    def _on_imu(self, msg: Imu):
        self._wz = msg.angular_velocity.z
        self._have_imu = True

    def _on_wheels(self, msg: WheelSpeeds):
        # Rear axle mean — front wheels scrub under steering.
        self._v = 0.5 * (msg.rl + msg.rr) * self._r
        self._have_wheels = True

    def _on_correction(self, msg: PoseCorrection):
        if not (self._use_corr and msg.valid):
            return
        if math.hypot(msg.dx, msg.dy) > self._corr_reject:
            return  # defence in depth; Block 4 gates this too
        self._pend = [float(msg.dx), float(msg.dy), float(msg.dyaw)]
        self._dist_since_fix = 0.0
        self._have_fix = True

    def _apply_correction(self, dt):
        """Work off part of the pending correction, rate-limited."""
        step = self._corr_v * dt
        d = math.hypot(self._pend[0], self._pend[1])
        if d > 1e-9:
            f = min(1.0, step / d)
            ax, ay = self._pend[0] * f, self._pend[1] * f
            self._x += ax
            self._y += ay
            self._pend[0] -= ax
            self._pend[1] -= ay
        step_w = self._corr_w * dt
        aw = max(-step_w, min(step_w, self._pend[2]))
        self._yaw = wrap_angle(self._yaw + aw)
        self._pend[2] -= aw

    def _step(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._last_t is None:
            self._last_t = t
            return
        dt = t - self._last_t
        self._last_t = t
        if dt <= 0.0 or dt > 0.5:
            return
        if not (self._have_wheels and self._have_imu):
            return  # never integrate garbage; supervisor sees missing odom

        self._yaw = wrap_angle(self._yaw + self._wz * dt)
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt
        self._dist += abs(self._v) * dt
        self._dist_since_fix += abs(self._v) * dt
        self._apply_correction(dt)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        quaternion_from_yaw(msg.pose.pose.orientation, self._yaw)
        msg.twist.twist.linear.x = self._v
        msg.twist.twist.angular.z = self._wz

        # Honest covariance: ~1% of distance in position, gyro drift in yaw.
        # Once the map is fixing the pose, the growth restarts from the last
        # fix instead of from the start of the run — that IS the benefit of
        # localizing, and downstream consumers should see it.
        ref = self._dist_since_fix if self._have_fix else self._dist
        pos_var = max(0.01, (0.01 * ref) ** 2)
        yaw_var = max(0.005, (0.002 * ref) ** 2)
        cov = [0.0] * 36
        cov[0] = pos_var          # x
        cov[7] = pos_var          # y
        cov[14] = 1e6             # z unobserved
        cov[21] = 1e6             # roll unobserved
        cov[28] = 1e6             # pitch unobserved
        cov[35] = yaw_var         # yaw
        msg.pose.covariance = cov
        tcov = [0.0] * 36
        tcov[0] = 0.04            # vx
        tcov[35] = 0.01           # wz
        msg.twist.covariance = tcov

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
