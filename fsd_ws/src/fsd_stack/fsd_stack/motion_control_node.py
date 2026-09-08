"""BLOCK 6 — MOTION PLANNING & CONTROL (owner: Controls & Actuation).

/planning/path + /odometry/filtered + /planning/speed_limit
        -> /control/cmd (fsd_msgs/VehicleCmd)

Longitudinal: curvature-limited velocity profile (backward braking pass +
forward traction pass) then PI on speed error -> torque / brake.
Lateral: Pure Pursuit with speed-scaled lookahead.

Output is a FIXED 50 Hz timer — never event-driven. Staleness policy per
spec: path > 0.5 s old -> hold path, decay speed, DEGRADED; path > 2 s or
odom > 0.2 s -> emergency_stop, ERROR.

SPEED REGIME. `v_max_mps` here is the absolute ceiling this car is allowed to
see, and the dynamics limits (a_lat/a_brake/a_accel) are tyre properties — a
mode does not change what the tyres can do. What the mode changes is how much
of the ceiling is usable, and Block 5 owns that call, because only the planner
knows whether the path in front of the car is a sensor-horizon-limited
exploration path, a fully mapped racing line, or a corridor with something in
it. That arrives as /planning/speed_limit. A cap of zero means stop and hold —
a legitimate driving state, reported DEGRADED, not an error, so a blocked
track does not latch the EBS.

v1 dynamics limits are deliberately timid. Raise a_lat/v_max in the params
file only after three consecutive clean runs at the current setting.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from fsd_msgs.msg import (PathPointArray, SpeedLimit, VehicleCmd, VehicleStatus,
                          Heartbeat)

from .common import HeartbeatEmitter, qos_reliable, yaw_from_quaternion


class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control')
        self.declare_parameter('wheelbase_m', 1.53)      # PLACEHOLDER — measure!
        self.declare_parameter('max_steering_rad', 0.35)
        self.declare_parameter('v_max_mps', 5.0)         # absolute ceiling
        # Used only when no fresh /planning/speed_limit exists (older planner,
        # bag replay, planner restart): the timid v1 speed, never the race one.
        self.declare_parameter('v_no_cap_mps', 5.0)
        self.declare_parameter('speed_limit_timeout_s', 1.0)
        self.declare_parameter('cap_ramp_mps2', 2.0)
        self.declare_parameter('a_lat_max', 4.0)
        self.declare_parameter('a_brake_max', 5.0)
        self.declare_parameter('a_accel_max', 3.0)
        self.declare_parameter('torque_max_nm', 30.0)
        self.declare_parameter('kp_speed', 12.0)         # Nm per m/s error
        self.declare_parameter('ki_speed', 2.0)
        self.declare_parameter('lookahead_gain_s', 0.8)
        self.declare_parameter('lookahead_min_m', 2.0)
        self.declare_parameter('lookahead_max_m', 8.0)
        self.declare_parameter('creep_speed_mps', 1.5)
        self.declare_parameter('creep_timeout_s', 15.0)

        gp = lambda n: float(self.get_parameter(n).value)
        self._L = gp('wheelbase_m')
        self._steer_max = gp('max_steering_rad')
        self._v_max = gp('v_max_mps')
        self._v_no_cap = gp('v_no_cap_mps')
        self._cap_timeout = gp('speed_limit_timeout_s')
        self._cap_ramp = gp('cap_ramp_mps2')
        self._a_lat = gp('a_lat_max')
        self._a_brk = gp('a_brake_max')
        self._a_acc = gp('a_accel_max')
        self._tq_max = gp('torque_max_nm')
        self._kp = gp('kp_speed')
        self._ki = gp('ki_speed')
        self._kv = gp('lookahead_gain_s')
        self._ld_min = gp('lookahead_min_m')
        self._ld_max = gp('lookahead_max_m')
        self._creep_speed = gp('creep_speed_mps')
        self._creep_timeout = gp('creep_timeout_s')

        self._path = None            # list of (x, y, heading, curvature)
        self._v_profile = None       # per-point target speed
        self._path_t = None
        self._odom = None
        self._odom_t = None
        self._integ = 0.0
        self._decayed_vmax = self._v_max
        self._last_near = None
        self._cap = None             # (v, reason, detail)
        self._cap_t = None
        self._allowed = min(self._v_max, self._v_no_cap)   # ramped ceiling
        self._creep_start_t = None

        self.create_subscription(PathPointArray, '/planning/path',
                                 self._on_path, qos_reliable(5))
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._on_odom, qos_reliable(10))
        self.create_subscription(VehicleStatus, '/vehicle/status',
                                 self._on_status, qos_reliable(10))
        self.create_subscription(SpeedLimit, '/planning/speed_limit',
                                 self._on_speed_limit, qos_reliable(5))
        self._pub = self.create_publisher(VehicleCmd, '/control/cmd',
                                          qos_reliable(1))
        self.create_timer(1.0 / 50.0, self._tick)   # the 50 Hz control loop
        self._hb = HeartbeatEmitter(self, 'motion_control')
        self._as_driving = True   # sim default; CAN bridge feeds real state

    def _on_status(self, msg: VehicleStatus):
        self._as_driving = (msg.as_state == VehicleStatus.AS_DRIVING)

    def _on_speed_limit(self, msg: SpeedLimit):
        self._cap = (float(msg.v_max_mps), msg.reason, msg.detail)
        self._cap_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_path(self, msg: PathPointArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.points) < 2:
            return  # empty path = planner ERROR; staleness policy handles it
        self._path = [(p.x, p.y, p.heading, p.curvature) for p in msg.points]
        self._v_profile = self._velocity_profile(msg.points)
        self._path_t = now
        self._decayed_vmax = self._v_max
        self._last_near = None   # new path: next tick does one full scan

    def _on_odom(self, msg: Odometry):
        self._odom = msg
        self._odom_t = self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------- velocity plan
    def _velocity_profile(self, points):
        n = len(points)
        v = [0.0] * n
        for i, p in enumerate(points):
            v_curve = math.sqrt(self._a_lat / max(abs(p.curvature), 1e-3))
            v[i] = min(self._v_max, v_curve)
        ds = [math.hypot(points[i + 1].x - points[i].x,
                         points[i + 1].y - points[i].y) for i in range(n - 1)]
        # Backward pass: braking feasibility.
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * self._a_brk * ds[i]))
        # Forward pass: traction feasibility.
        for i in range(1, n):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * self._a_acc * ds[i - 1]))
        return v

    def _speed_ceiling(self, now, dt):
        """The usable ceiling this tick: the planner's cap, ramped up so a
        regime change is a smooth pull rather than a step, but applied
        DOWNWARD immediately — a lower cap is always a braking request."""
        fresh = (self._cap is not None and self._cap_t is not None
                 and now - self._cap_t <= self._cap_timeout)
        target = min(self._v_max, self._cap[0] if fresh else self._v_no_cap)
        if target < self._allowed:
            self._allowed = target
        else:
            self._allowed = min(target, self._allowed + self._cap_ramp * dt)
        return self._allowed, fresh

    # ----------------------------------------------------------- 50 Hz tick
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / 50.0
        cmd = VehicleCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self._odom is None:
            self._pub.publish(cmd)   # all zeros: no motion on startup
            return
        odom_age = now - self._odom_t

        # Creep-start: once the GO signal is received (AS_DRIVING) but no track
        # has been mapped into a path yet, crawl straight forward so the camera
        # approaches the first cones and the map can bootstrap. The safety
        # supervisor still EBSes if the car moves > 3 s with ZERO mapped cones,
        # so a blind runaway is bounded. Steering held at zero.
        if self._path is None:
            if self._creep_start_t is None and self._as_driving:
                self._creep_start_t = now
            within_cap = (self._creep_start_t is not None
                          and now - self._creep_start_t < self._creep_timeout)
            if self._as_driving and odom_age <= 0.2 and within_cap:
                v = self._odom.twist.twist.linear.x
                cmd.torque_request = max(0.0, min(self._tq_max * 0.5,
                                                  self._kp * (self._creep_speed - v)))
                cmd.steering_angle = 0.0
                self._hb.set_status(Heartbeat.STATUS_DEGRADED,
                                    'creep-start: seeking track')
            elif self._as_driving and not within_cap:
                cmd.brake_cmd = 0.3
                self._hb.set_status(Heartbeat.STATUS_ERROR,
                                    'creep timeout, no track found')
            else:
                self._hb.set_status(Heartbeat.STATUS_OK, 'waiting for GO / path')
            self._pub.publish(cmd)
            return
        self._creep_start_t = None   # a path exists: reset the creep window

        path_age = now - self._path_t
        if path_age > 2.0 or odom_age > 0.2:
            cmd.emergency_stop = True
            cmd.brake_cmd = 1.0
            self._hb.set_status(Heartbeat.STATUS_ERROR,
                                f'stale inputs: path {path_age:.1f}s odom {odom_age:.2f}s')
            self._pub.publish(cmd)
            return

        allowed, cap_fresh = self._speed_ceiling(now, dt)
        holding = cap_fresh and self._cap[0] <= 0.0
        if path_age > 0.5:
            # Hold path, decay allowed speed at 2 m/s^2.
            self._decayed_vmax = max(0.0, self._decayed_vmax - 2.0 / 50.0)
            self._hb.set_status(Heartbeat.STATUS_DEGRADED, 'path stale, decaying speed')
        elif holding:
            self._hb.set_status(Heartbeat.STATUS_DEGRADED,
                                self._cap[2] or 'held at zero by the planner')
        else:
            self._hb.set_status(Heartbeat.STATUS_OK)

        px = self._odom.pose.pose.position.x
        py = self._odom.pose.pose.position.y
        pyaw = yaw_from_quaternion(self._odom.pose.pose.orientation)
        v = self._odom.twist.twist.linear.x

        # Nearest path point — sticky index: search a window around the
        # previous result (O(1) amortized), full scan on new path or when
        # the windowed result is implausibly far (> 4 m off path).
        i_near = self._nearest_index(px, py)

        # ---------------- Pure Pursuit
        ld = min(max(self._kv * v, self._ld_min), self._ld_max)
        target = self._path[-1]
        acc = 0.0
        for i in range(i_near, len(self._path) - 1):
            acc += math.hypot(self._path[i + 1][0] - self._path[i][0],
                              self._path[i + 1][1] - self._path[i][1])
            if acc >= ld:
                target = self._path[i + 1]
                break
        dx, dy = target[0] - px, target[1] - py
        # Target in vehicle frame.
        tx = dx * math.cos(pyaw) + dy * math.sin(pyaw)
        ty = -dx * math.sin(pyaw) + dy * math.cos(pyaw)
        alpha = math.atan2(ty, max(tx, 1e-6))
        ld_actual = max(math.hypot(tx, ty), 1e-3)
        steer = math.atan2(2.0 * self._L * math.sin(alpha), ld_actual)
        cmd.steering_angle = max(-self._steer_max, min(self._steer_max, steer))

        # ---------------- Longitudinal PI
        # Target speed: profile a little ahead of the nearest point, under the
        # decay policy and the planner's cap.
        i_tgt = min(i_near + 2, len(self._v_profile) - 1)
        v_target = min(self._v_profile[i_tgt], self._decayed_vmax, allowed)
        if not self._as_driving:
            v_target = 0.0
        err = v_target - v
        if holding and v < 0.3:
            # Stopped in front of something: hold the brake instead of
            # trickling torque, and drop the integrator.
            self._integ = 0.0
            cmd.torque_request = 0.0
            cmd.brake_cmd = 0.3
        elif err >= -0.3:                      # accelerate / hold
            self._integ = max(min(self._integ + err / 50.0, 5.0), -5.0)
            tq = self._kp * err + self._ki * self._integ
            cmd.torque_request = max(0.0, min(self._tq_max, tq))
            cmd.brake_cmd = 0.0
        else:                                  # brake, never torque+brake
            self._integ = 0.0
            cmd.torque_request = 0.0
            cmd.brake_cmd = max(0.0, min(1.0, -err * 0.25))

        self._pub.publish(cmd)

    def _nearest_index(self, px, py):
        def d2(i):
            p = self._path[i]
            return (p[0] - px) ** 2 + (p[1] - py) ** 2

        if self._last_near is not None:
            lo = max(0, self._last_near - 20)
            hi = min(len(self._path), self._last_near + 21)
            i_best = min(range(lo, hi), key=d2)
            if d2(i_best) < 16.0:          # within 4 m: plausible
                self._last_near = i_best
                return i_best
        i_best = min(range(len(self._path)), key=d2)
        self._last_near = i_best
        return i_best


def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
