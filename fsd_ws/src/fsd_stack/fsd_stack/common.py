"""Shared helpers for every fsd_stack node: QoS profiles per the interface
spec (Section 2.2), the mandatory heartbeat emitter, and a timestamped pose
buffer for interpolating ego pose at sensor timestamps."""

import math
from collections import deque

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from fsd_msgs.msg import Heartbeat


def qos_reliable(depth=5):
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def qos_best_effort(depth=1):
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(q, yaw):
    """Fill an existing geometry_msgs/Quaternion in-place from a yaw angle."""
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class HeartbeatEmitter:
    """Every node instantiates one of these. Publishes /safety/heartbeat at
    10 Hz (spec requires >= 5 Hz). Call set_status() to flag degradation."""

    def __init__(self, node, node_id, rate_hz=10.0):
        self._node = node
        self._id = node_id
        self._status = Heartbeat.STATUS_OK
        self._text = ''
        self._pub = node.create_publisher(Heartbeat, '/safety/heartbeat', qos_reliable(10))
        self._timer = node.create_timer(1.0 / rate_hz, self._tick)

    def set_status(self, status, text=''):
        self._status = status
        self._text = text

    def _tick(self):
        m = Heartbeat()
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.node_id = self._id
        m.status = self._status
        m.message = self._text
        self._pub.publish(m)


class PoseBuffer:
    """Ring buffer of (t, x, y, yaw). query(t) returns the pose linearly
    interpolated at time t (clamped to buffer ends), or None when empty.
    This is the spec-required 'interpolate to detection timestamp' mechanism
    without pulling in a full tf2 buffer."""

    def __init__(self, maxlen=300):
        self._buf = deque(maxlen=maxlen)

    def add(self, t, x, y, yaw):
        # Ignore out-of-order inserts; odometry is monotonic in practice.
        if self._buf and t <= self._buf[-1][0]:
            return
        self._buf.append((t, x, y, yaw))

    def query(self, t):
        if not self._buf:
            return None
        buf = self._buf
        if t <= buf[0][0]:
            return buf[0][1:]
        if t >= buf[-1][0]:
            return buf[-1][1:]
        # Binary search for the bracketing pair.
        lo, hi = 0, len(buf) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if buf[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, yaw0 = buf[lo]
        t1, x1, y1, yaw1 = buf[hi]
        if t1 <= t0:
            return buf[hi][1:]
        a = (t - t0) / (t1 - t0)
        x = x0 + a * (x1 - x0)
        y = y0 + a * (y1 - y0)
        yaw = yaw0 + a * wrap_angle(yaw1 - yaw0)
        return (x, y, wrap_angle(yaw))
