"""BLOCK 8 — SAFETY SUPERVISOR (owner: Controls & Actuation).

Watches /safety/heartbeat from every required node plus plausibility checks
on the data streams. Any violation latches /safety/ebs_trigger = True.
Publishes a 10 Hz keepalive (False) when healthy — the STM32 treats a
missing keepalive as a trigger condition, so a dead supervisor is itself
an EBS trigger.

This is the SOFTWARE stop path (path 3 of 3). It must never be the only
one: the RES/SDC hardware path and the STM32 command-timeout firmware path
exist independently of everything in this file.
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from fsd_msgs.msg import Heartbeat, ConeMap, VehicleCmd

from .common import HeartbeatEmitter, qos_reliable

HEARTBEAT_TIMEOUT_S = 0.5
BLIND_SPEED_MPS = 0.5
BLIND_GRACE_S = 3.0
MAX_POSE_JUMP_M = 1.0


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        # Which nodes MUST be alive. Set per launch config: the sim launch
        # excludes perception nodes because the sim publishes ideal cones.
        self.declare_parameter('required_nodes', [
            'cone_detection', 'cone_localization', 'state_estimation',
            'cone_mapping', 'path_planning', 'motion_control'])
        self.declare_parameter('max_steering_rad', 0.35)

        self._required = list(self.get_parameter('required_nodes').value)
        self._steer_max = float(self.get_parameter('max_steering_rad').value)

        self._last_hb = {}          # node_id -> (t, status)
        self._triggered = False
        self._start_t = self.get_clock().now().nanoseconds * 1e-9
        self._last_pose = None
        self._speed = 0.0
        self._cone_count = 0
        self._blind_since = None

        self.create_subscription(Heartbeat, '/safety/heartbeat',
                                 self._on_hb, qos_reliable(10))
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._on_odom, qos_reliable(10))
        self.create_subscription(ConeMap, '/mapping/track',
                                 self._on_map, qos_reliable(5))
        self.create_subscription(VehicleCmd, '/control/cmd',
                                 self._on_cmd, qos_reliable(1))

        self._pub = self.create_publisher(Bool, '/safety/ebs_trigger',
                                          qos_reliable(10))
        self.create_timer(0.05, self._check)      # 20 Hz checks
        self.create_timer(0.1, self._keepalive)   # 10 Hz keepalive
        self._hb = HeartbeatEmitter(self, 'safety_supervisor')

    # ------------------------------------------------------------- inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_hb(self, msg: Heartbeat):
        self._last_hb[msg.node_id] = (self._now(), msg.status)
        if msg.status == Heartbeat.STATUS_ERROR:
            self._trigger(f'node {msg.node_id} reports ERROR: {msg.message}')

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear.x
        if any(not math.isfinite(f) for f in (p.x, p.y, v)):
            self._trigger('NaN/inf in odometry')
            return
        if self._last_pose is not None:
            jump = math.hypot(p.x - self._last_pose[0], p.y - self._last_pose[1])
            if jump > MAX_POSE_JUMP_M:
                self._trigger(f'pose jump {jump:.2f} m between odom updates')
        self._last_pose = (p.x, p.y)
        self._speed = v

    def _on_map(self, msg: ConeMap):
        self._cone_count = len(msg.cones)

    def _on_cmd(self, msg: VehicleCmd):
        if any(not math.isfinite(f) for f in
               (msg.steering_angle, msg.torque_request, msg.brake_cmd)):
            self._trigger('NaN/inf in control command')
            return
        if abs(msg.steering_angle) > self._steer_max * 1.05:
            self._trigger(f'steering command {msg.steering_angle:.2f} rad '
                          f'exceeds physical limit')

    # ------------------------------------------------------------- checks
    def _check(self):
        now = self._now()
        # Give nodes a startup grace period before demanding heartbeats.
        if now - self._start_t > 5.0:
            for node_id in self._required:
                seen = self._last_hb.get(node_id)
                if seen is None:
                    self._trigger(f'node {node_id} never sent a heartbeat')
                elif now - seen[0] > HEARTBEAT_TIMEOUT_S:
                    self._trigger(f'node {node_id} heartbeat missing '
                                  f'{now - seen[0]:.2f} s')

        # Moving blind: speed > threshold with zero confirmed cones.
        if self._speed > BLIND_SPEED_MPS and self._cone_count == 0:
            if self._blind_since is None:
                self._blind_since = now
            elif now - self._blind_since > BLIND_GRACE_S:
                self._trigger('moving with zero confirmed cones for > 3 s')
        else:
            self._blind_since = None

    def _keepalive(self):
        msg = Bool()
        msg.data = self._triggered
        self._pub.publish(msg)

    def _trigger(self, reason):
        if not self._triggered:
            self._triggered = True   # latched until process restart, per spec
            self.get_logger().error(f'EBS TRIGGERED: {reason}')
            self._hb.set_status(Heartbeat.STATUS_ERROR, f'EBS: {reason}')
            msg = Bool()
            msg.data = True
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
