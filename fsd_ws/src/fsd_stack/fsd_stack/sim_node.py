"""KINEMATIC SIMULATOR — replaces the car AND Blocks 1-2 for integration
testing. Not part of the competition stack.

Generates an elliptical cone track (blue left, yellow right of travel
direction), runs a kinematic bicycle model, and publishes exactly the
topics the real sensors/perception would:

  /perception/cones   Cone3DArray, 30 Hz  (ideal detections in FOV + noise)
  /imu/data           sensor_msgs/Imu, 100 Hz (gyro + noise)
  /wheel_speeds       fsd_msgs/WheelSpeeds, 50 Hz
  /vehicle/status     fsd_msgs/VehicleStatus, 50 Hz (AS_DRIVING)
  /sim/ground_truth   nav_msgs/Odometry, 50 Hz (for error metrics only —
                      NOTHING downstream may subscribe to this)
  /sim/obstacles      fsd_msgs/ConeMap, 1 Hz (ground-truth obstacle positions,
                      for measuring clearance in tests — also off limits to
                      the stack)

Obstacles are objects placed ON the racing line at chosen fractions of the
lap, reported through /perception/cones as COLOR_UNKNOWN objects exactly like
any other detection — the stack gets no privileged knowledge that they are
special. `obstacle_appear_s` delays them so a run can map a clean track first
and then meet something in the corridor at race speed, which is the case that
actually exercises the avoidance logic.

Subscribes /control/cmd and /safety/ebs_trigger and behaves like Block 7:
command timeout of 100 ms -> zero torque + brake, EBS trigger -> full stop.
"""

import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from fsd_msgs.msg import (Cone3D, Cone3DArray, ConeMap, ConeMapEntry,
                          WheelSpeeds, VehicleCmd, VehicleStatus,
                          ConeDetection2D)

from .common import qos_reliable, quaternion_from_yaw, wrap_angle

PHYS_HZ = 100.0
WHEEL_RADIUS = 0.228
MASS = 220.0
TORQUE_TO_ACCEL = 0.07      # m/s^2 per Nm (Emrax through ~3.5:1 final drive)
BRAKE_ACCEL = 8.0           # m/s^2 at brake_cmd = 1.0
EBS_ACCEL = 10.0
DRAG = 0.05                 # per-second speed decay coefficient
STEER_SLEW = math.radians(90.0)   # rad/s at tire
CMD_TIMEOUT_S = 0.1


def centerline_at(fraction, lateral=0.0, a=20.0, b=12.0, n_dense=2000):
    """Point at `fraction` of the way around the elliptical centerline,
    offset `lateral` metres to the left of travel. Arclength-parameterised,
    so a fraction means the same thing all the way round."""
    pts = []
    s = 0.0
    last = (a, 0.0)
    for i in range(n_dense + 1):
        t = 2.0 * math.pi * i / n_dense
        p = (a * math.cos(t), b * math.sin(t))
        s += math.hypot(p[0] - last[0], p[1] - last[1])
        tx, ty = -a * math.sin(t), b * math.cos(t)
        nrm = math.hypot(tx, ty)
        pts.append((p[0], p[1], tx / nrm, ty / nrm, s))
        last = p
    target = (fraction % 1.0) * pts[-1][4]
    for x, y, tx, ty, sc in pts:
        if sc >= target:
            # Left normal of the travel tangent.
            return x - lateral * ty, y + lateral * tx
    x, y, tx, ty, _ = pts[-1]
    return x - lateral * ty, y + lateral * tx


def build_track(a=20.0, b=12.0, spacing=3.0, half_width=1.75):
    """Elliptical centerline, cones every ~spacing meters of arclength.
    Returns list of (x, y, color). Counterclockwise: blue outside-left,
    yellow inside-right relative to travel."""
    cones = []
    n_dense = 2000
    pts = []
    for i in range(n_dense):
        t = 2.0 * math.pi * i / n_dense
        pts.append((a * math.cos(t), b * math.sin(t), t))
    s_acc = 0.0
    last = pts[0]
    next_at = 0.0
    for p in pts + [pts[0]]:
        s_acc += math.hypot(p[0] - last[0], p[1] - last[1])
        if s_acc >= next_at:
            t = p[2]
            # Tangent of CCW ellipse.
            tx, ty = -a * math.sin(t), b * math.cos(t)
            norm = math.hypot(tx, ty)
            tx, ty = tx / norm, ty / norm
            # Left normal.
            nx, ny = -ty, tx
            cones.append((p[0] + half_width * nx, p[1] + half_width * ny,
                          ConeDetection2D.COLOR_BLUE))
            cones.append((p[0] - half_width * nx, p[1] - half_width * ny,
                          ConeDetection2D.COLOR_YELLOW))
            next_at += spacing
        last = p
    return cones


class SimNode(Node):
    def __init__(self):
        super().__init__('sim')
        self.declare_parameter('noise_pos_m', 0.05)
        self.declare_parameter('fov_range_m', 12.0)
        self.declare_parameter('fov_half_angle_deg', 60.0)
        # Obstacles on the line. Negative fractions mean "none" — a plain
        # empty list has no inferable parameter type in ROS 2.
        self.declare_parameter('obstacle_lap_fractions', [-1.0])
        self.declare_parameter('obstacle_lateral_m', [0.0])
        self.declare_parameter('obstacle_appear_s', 0.0)
        self._noise = float(self.get_parameter('noise_pos_m').value)
        self._fov_r = float(self.get_parameter('fov_range_m').value)
        self._fov_a = math.radians(
            float(self.get_parameter('fov_half_angle_deg').value))

        self._track = build_track()

        fractions = [float(f) for f in
                     self.get_parameter('obstacle_lap_fractions').value
                     if float(f) >= 0.0]
        laterals = [float(v) for v in
                    self.get_parameter('obstacle_lateral_m').value]
        self._obstacle_appear = float(self.get_parameter('obstacle_appear_s').value)
        self._obstacles = [
            centerline_at(f, laterals[i] if i < len(laterals) else 0.0)
            for i, f in enumerate(fractions)]
        self._t0 = None

        # Start on the centerline at t=0 heading along CCW travel (+y tangent).
        self._x, self._y = 20.0, 0.0
        self._yaw = math.pi / 2.0
        self._v = 0.0
        self._steer = 0.0
        self._accel = 0.0

        self._cmd = VehicleCmd()
        self._cmd_t = None
        self._ebs = False
        self._tick_count = 0

        self.create_subscription(VehicleCmd, '/control/cmd', self._on_cmd,
                                 qos_reliable(1))
        self.create_subscription(Bool, '/safety/ebs_trigger', self._on_ebs,
                                 qos_reliable(10))

        self._cones_pub = self.create_publisher(Cone3DArray, '/perception/cones',
                                                qos_reliable(5))
        self._imu_pub = self.create_publisher(Imu, '/imu/data', qos_reliable(10))
        self._wheels_pub = self.create_publisher(WheelSpeeds, '/wheel_speeds',
                                                 qos_reliable(10))
        self._status_pub = self.create_publisher(VehicleStatus, '/vehicle/status',
                                                 qos_reliable(10))
        self._gt_pub = self.create_publisher(Odometry, '/sim/ground_truth',
                                             qos_reliable(10))
        self._obs_pub = self.create_publisher(ConeMap, '/sim/obstacles',
                                              qos_reliable(5))

        self.create_timer(1.0 / PHYS_HZ, self._physics)
        self.create_timer(1.0, self._publish_obstacle_truth)
        self.get_logger().info(
            f'Sim up: {len(self._track)} cones, elliptical track, '
            f'{len(self._obstacles)} obstacle(s)'
            + (f' appearing at t+{self._obstacle_appear:.0f}s'
               if self._obstacles and self._obstacle_appear > 0.0 else '')
            + '. Waiting for /control/cmd...')

    def _on_cmd(self, msg):
        self._cmd = msg
        self._cmd_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_ebs(self, msg):
        if msg.data:
            if not self._ebs:
                self.get_logger().warn('EBS triggered in sim — full stop')
            self._ebs = True

    # ---------------------------------------------------------- physics
    def _physics(self):
        dt = 1.0 / PHYS_HZ
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now

        cmd = self._cmd
        timed_out = self._cmd_t is None or (now - self._cmd_t) > CMD_TIMEOUT_S

        # Steering actuator: slew-limited tracking of the command.
        target = 0.0 if timed_out else cmd.steering_angle
        d = max(-STEER_SLEW * dt, min(STEER_SLEW * dt, target - self._steer))
        self._steer += d

        # Longitudinal.
        if self._ebs or (not timed_out and cmd.emergency_stop):
            a = -EBS_ACCEL
        elif timed_out:
            a = -BRAKE_ACCEL * 0.5   # firmware safe state: brake gently
        else:
            a = (cmd.torque_request * TORQUE_TO_ACCEL
                 - cmd.brake_cmd * BRAKE_ACCEL)
        a -= DRAG * self._v
        self._accel = a
        self._v = max(0.0, self._v + a * dt)

        # Kinematic bicycle.
        wz = self._v / 1.53 * math.tan(self._steer)
        self._yaw = wrap_angle(self._yaw + wz * dt)
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt

        self._tick_count += 1
        self._publish_imu(wz)                     # 100 Hz
        if self._tick_count % 2 == 0:             # 50 Hz
            self._publish_wheels()
            self._publish_status()
            self._publish_ground_truth()
        if self._tick_count % 3 == 0:             # ~33 Hz
            self._publish_cones()

    # ---------------------------------------------------------- outputs
    def _publish_imu(self, wz):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        quaternion_from_yaw(m.orientation, self._yaw)
        m.angular_velocity.z = wz + random.gauss(0.0, 0.005)
        m.linear_acceleration.x = self._accel + random.gauss(0.0, 0.05)
        self._imu_pub.publish(m)

    def _publish_wheels(self):
        m = WheelSpeeds()
        m.header.stamp = self.get_clock().now().to_msg()
        w = self._v / WHEEL_RADIUS
        m.fl = m.fr = m.rl = m.rr = w + random.gauss(0.0, 0.02)
        self._wheels_pub.publish(m)

    def _publish_status(self):
        m = VehicleStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.actual_steering_angle = self._steer
        m.motor_rpm = self._v / WHEEL_RADIUS * 60.0 / (2.0 * math.pi) * 3.5
        m.as_state = (VehicleStatus.AS_EMERGENCY if self._ebs
                      else VehicleStatus.AS_DRIVING)
        self._status_pub.publish(m)

    def _publish_ground_truth(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = self._x
        m.pose.pose.position.y = self._y
        quaternion_from_yaw(m.pose.pose.orientation, self._yaw)
        m.twist.twist.linear.x = self._v
        self._gt_pub.publish(m)

    def _obstacles_visible(self):
        """Obstacles currently in the world, honouring the reveal delay."""
        if not self._obstacles or self._t0 is None:
            return []
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._t0 < self._obstacle_appear:
            return []
        return self._obstacles

    def _publish_obstacle_truth(self):
        msg = ConeMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for i, (x, y) in enumerate(self._obstacles):
            e = ConeMapEntry()
            e.id = i + 1
            e.color = ConeDetection2D.COLOR_UNKNOWN
            e.x = float(x)
            e.y = float(y)
            e.side = ConeMapEntry.SIDE_UNKNOWN
            msg.cones.append(e)
        self._obs_pub.publish(msg)

    def _publish_cones(self):
        out = Cone3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        cos_y, sin_y = math.cos(self._yaw), math.sin(self._yaw)
        idx = 0
        objects = [(x, y, ConeDetection2D.COLOR_UNKNOWN)
                   for (x, y) in self._obstacles_visible()]
        for cx, cy, color in list(self._track) + objects:
            dx, dy = cx - self._x, cy - self._y
            lx = dx * cos_y + dy * sin_y       # forward
            ly = -dx * sin_y + dy * cos_y      # left
            r = math.hypot(lx, ly)
            if not (0.5 < lx and r < self._fov_r):
                continue
            if abs(math.atan2(ly, lx)) > self._fov_a:
                continue
            sigma = self._noise + 0.02 * r
            c = Cone3D()
            c.id = idx
            idx += 1
            c.color = color
            c.confidence = 0.9
            c.x = lx + random.gauss(0.0, sigma)
            c.y = ly + random.gauss(0.0, sigma)
            c.z = 0.0
            c.depth_sigma = sigma
            out.cones.append(c)
        self._cones_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
