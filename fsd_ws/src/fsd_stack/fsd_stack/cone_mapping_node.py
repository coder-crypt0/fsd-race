"""BLOCK 4 — CONE MAPPING (owner: Mapping & Planning).

/perception/cones + /odometry/filtered
        -> /mapping/track              (fsd_msgs/ConeMap)
        -> /mapping/status             (fsd_msgs/TrackStatus)
        -> /localization/correction    (fsd_msgs/PoseCorrection)

Implements the normative data-association algorithm from the interface spec:
  1. match against confirmed map (spatial gate, Kalman position update)
  2. else match/accumulate in the tentative buffer (N_CONFIRM promotion)
  3. garbage-collect stale tentative entries only

Hard rules honoured here:
  * color is NEVER an association gate — resolved by majority vote
  * confirmed cones are NEVER deleted mid-run
  * uniform-grid spatial index, no O(n^2) scans
  * detections are transformed with the pose INTERPOLATED at their timestamp

On top of the map, this node owns the two things that make a *known* track
usable at speed:

  * MAP-RELATIVE LOCALIZATION — a 2D rigid fit (Kabsch) of the current
    observations onto the confirmed landmarks. Dead reckoning alone drifts
    metres per lap, which used to re-map the same physical cone as a new
    landmark on every lap (measured: 186 landmarks for 70 real cones). The
    correction is published, not applied here; Block 3 applies it
    slew-limited so the pose stays continuous.
  * LAP COUNTING — start-line crossings from the pose at node start, with an
    arming distance so the line can only be counted once per lap. One
    completed lap = the whole track has been seen = MODE_KNOWN, which is what
    licenses race speed downstream.
"""

import math
from collections import Counter

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from fsd_msgs.msg import (Cone3DArray, ConeMap, ConeMapEntry, PoseCorrection,
                          TrackStatus)

from .common import (HeartbeatEmitter, PoseBuffer, qos_reliable,
                     stamp_to_sec, wrap_angle, yaw_from_quaternion)


def solve_pose_correction(pose, dets_body, query_fn, gate=1.0, min_inliers=6,
                          iters=3, min_spread_m2=8.0, max_trans_m=0.6,
                          max_rot_rad=0.15, reject_trans_m=2.0,
                          reject_rot_rad=0.4):
    """Rigid 2D fit of body-frame detections onto the confirmed map.

    Iterated closest point with a closed-form Kabsch solve per iteration:
    transform the detections into odom with the working pose, pair each with
    its nearest landmark inside `gate`, solve for the (rotation, translation)
    that best maps observations onto landmarks, move the working pose by it,
    repeat.

    Returns (dx, dy, dyaw, n_inliers, rms) as a correction to `pose`, or None
    when the solve is not trustworthy. Guards, in order of what they catch:
      * min_inliers    — too few pairs to constrain 3 DoF
      * min_spread_m2  — pairs clustered in one spot: rotation ill-determined
      * reject_*       — solve so large it is more likely a mis-association
                         (cone spacing is ~3 m) than real drift
      * max_*          — plausible but big: clamp, converge over more frames
    """
    px, py, pyaw = pose
    wx, wy, wyaw = px, py, pyaw
    n_inliers, rms = 0, 0.0

    for _ in range(iters):
        cos_y, sin_y = math.cos(wyaw), math.sin(wyaw)
        pairs = []
        for bx, by in dets_body:
            qx = wx + bx * cos_y - by * sin_y
            qy = wy + bx * sin_y + by * cos_y
            near, nd2 = None, gate * gate
            for lx, ly in query_fn(qx, qy, gate):
                d2 = (lx - qx) ** 2 + (ly - qy) ** 2
                if d2 < nd2:
                    near, nd2 = (lx, ly), d2
            if near is not None:
                pairs.append((qx, qy, near[0], near[1]))

        n = len(pairs)
        if n < min_inliers:
            return None
        qmx = sum(p[0] for p in pairs) / n
        qmy = sum(p[1] for p in pairs) / n
        mmx = sum(p[2] for p in pairs) / n
        mmy = sum(p[3] for p in pairs) / n
        spread = sum((p[0] - qmx) ** 2 + (p[1] - qmy) ** 2 for p in pairs)
        if spread < min_spread_m2:
            return None

        # Kabsch in 2D: the optimal rotation is the argument of the sum of
        # cross/dot products of the centred point sets.
        num = sum((p[0] - qmx) * (p[3] - mmy) - (p[1] - qmy) * (p[2] - mmx)
                  for p in pairs)
        den = sum((p[0] - qmx) * (p[2] - mmx) + (p[1] - qmy) * (p[3] - mmy)
                  for p in pairs)
        dpsi = math.atan2(num, den)
        c, s = math.cos(dpsi), math.sin(dpsi)
        tx = mmx - (c * qmx - s * qmy)
        ty = mmy - (s * qmx + c * qmy)

        rss = 0.0
        for qx, qy, mx, my in pairs:
            rx = c * qx - s * qy + tx - mx
            ry = s * qx + c * qy + ty - my
            rss += rx * rx + ry * ry
        n_inliers, rms = n, math.sqrt(rss / n)

        # Move the working pose by the same rigid transform.
        wx, wy = c * wx - s * wy + tx, s * wx + c * wy + ty
        wyaw = wrap_angle(wyaw + dpsi)

    dx, dy = wx - px, wy - py
    dyaw = wrap_angle(wyaw - pyaw)
    trans = math.hypot(dx, dy)
    if trans > reject_trans_m or abs(dyaw) > reject_rot_rad:
        return None
    if trans > max_trans_m:
        scale = max_trans_m / trans
        dx, dy = dx * scale, dy * scale
    dyaw = max(-max_rot_rad, min(max_rot_rad, dyaw))
    return dx, dy, dyaw, n_inliers, rms


class LapCounter:
    """Start-line crossing detector.

    The line is the perpendicular to the heading at the pose where the node
    first saw odometry (both FSDS and the kinematic sim start the car on the
    start line; a mid-track start just measures laps from there instead).

    A crossing needs all three of: the car has driven at least `min_lap_m`
    since the last one (so the line cannot re-trigger while sitting on it),
    the along-track coordinate goes from behind the line to in front of it,
    and the car is inside `corridor_m` laterally — which is what rejects the
    far side of the track, where the same plane is crossed the other way.
    """

    def __init__(self, min_lap_m=20.0, corridor_m=6.0):
        self.min_lap_m = min_lap_m
        self.corridor_m = corridor_m
        self.origin = None            # (x, y, fx, fy)
        self.laps = 0
        self.lap_start_dist = 0.0
        self.track_length_m = 0.0
        self._prev_s = 0.0

    def update(self, x, y, yaw, dist):
        if self.origin is None:
            self.origin = (x, y, math.cos(yaw), math.sin(yaw))
            self.lap_start_dist = dist
            self._prev_s = 0.0
            return False
        ox, oy, fx, fy = self.origin
        s = (x - ox) * fx + (y - oy) * fy
        lat = -(x - ox) * fy + (y - oy) * fx
        crossed = False
        if (dist - self.lap_start_dist >= self.min_lap_m
                and self._prev_s < 0.0 <= s
                and abs(lat) <= self.corridor_m):
            self.laps += 1
            self.track_length_m = dist - self.lap_start_dist
            self.lap_start_dist = dist
            crossed = True
        self._prev_s = s
        return crossed


class Landmark:
    __slots__ = ('id', 'x', 'y', 'var', 'color_votes', 'side_votes',
                 'obs_count', 'first_seen', 'last_seen')

    def __init__(self, x, y, var, color, side, t):
        self.id = 0
        self.x = x
        self.y = y
        self.var = var                    # scalar position variance (m^2)
        self.color_votes = Counter([color])
        self.side_votes = Counter([side])
        self.obs_count = 1
        self.first_seen = t
        self.last_seen = t

    def update(self, x, y, meas_var, color, side, t):
        k = self.var / (self.var + meas_var)   # scalar Kalman gain
        self.x += k * (x - self.x)
        self.y += k * (y - self.y)
        self.var *= (1.0 - k)
        self.var = max(self.var, 1e-4)
        self.color_votes[color] += 1
        self.side_votes[side] += 1
        self.obs_count += 1
        self.last_seen = t


class GridIndex:
    """Uniform hash grid over 2D points for O(1) radius queries."""

    def __init__(self, cell=2.0):
        self._cell = cell
        self._cells = {}

    def _key(self, x, y):
        return (int(math.floor(x / self._cell)), int(math.floor(y / self._cell)))

    def insert(self, item):
        self._cells.setdefault(self._key(item.x, item.y), []).append(item)

    def rebuild(self, items):
        self._cells.clear()
        for it in items:
            self.insert(it)

    def query_radius(self, x, y, r):
        out = []
        kx, ky = self._key(x, y)
        span = int(math.ceil(r / self._cell))
        for ix in range(kx - span, kx + span + 1):
            for iy in range(ky - span, ky + span + 1):
                for it in self._cells.get((ix, iy), []):
                    if (it.x - x) ** 2 + (it.y - y) ** 2 <= r * r:
                        out.append(it)
        return out


class ConeMappingNode(Node):
    def __init__(self):
        super().__init__('cone_mapping')
        self.declare_parameter('search_radius_m', 1.2)
        self.declare_parameter('gate_threshold_m', 0.5)   # Euclidean bring-up gate
        self.declare_parameter('n_confirm', 3)
        self.declare_parameter('tentative_timeout_s', 1.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        # A detection whose own range uncertainty is comparable to the
        # association gate cannot be told apart from a neighbour, so it must
        # not be allowed to CREATE a landmark — it lands outside the gate of
        # the cone it really is, and three such in a row get promoted into a
        # permanent duplicate that no later observation can remove. Such
        # detections still update landmarks they do match, and still feed
        # localization; the cone gets mapped properly when the car is closer.
        self.declare_parameter('max_spawn_sigma_m', 0.25)
        # Map-relative localization.
        self.declare_parameter('localize_enable', True)
        self.declare_parameter('localize_gate_m', 1.0)
        self.declare_parameter('localize_min_inliers', 6)
        self.declare_parameter('localize_min_obs', 5)
        self.declare_parameter('localize_min_landmarks', 12)
        # Lap counting / mode.
        self.declare_parameter('min_lap_distance_m', 20.0)
        self.declare_parameter('start_corridor_m', 6.0)
        self.declare_parameter('laps_to_known', 1)

        gp = lambda n: self.get_parameter(n).value
        self._search_r = float(gp('search_radius_m'))
        self._gate = float(gp('gate_threshold_m'))
        self._n_confirm = int(gp('n_confirm'))
        self._tent_timeout = float(gp('tentative_timeout_s'))
        self._max_spawn_var = float(gp('max_spawn_sigma_m')) ** 2
        self._loc_enable = bool(gp('localize_enable'))
        self._loc_gate = float(gp('localize_gate_m'))
        self._loc_min_inliers = int(gp('localize_min_inliers'))
        self._loc_min_obs = int(gp('localize_min_obs'))
        self._loc_min_landmarks = int(gp('localize_min_landmarks'))
        self._laps_to_known = int(gp('laps_to_known'))

        self._confirmed = []           # list[Landmark], never shrinks mid-run
        self._tentative = []           # list[Landmark]
        self._grid_confirmed = GridIndex()
        self._grid_tentative = GridIndex()
        self._next_id = 1
        self._poses = PoseBuffer()

        self._laps = LapCounter(float(gp('min_lap_distance_m')),
                                float(gp('start_corridor_m')))
        self._dist = 0.0
        self._last_odom_t = None
        self._loc_inliers = 0
        self._loc_rms = 0.0

        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom,
                                 qos_reliable(10))
        self.create_subscription(Cone3DArray, '/perception/cones',
                                 self._on_cones, qos_reliable(5))
        self._pub = self.create_publisher(ConeMap, '/mapping/track', qos_reliable(5))
        self._status_pub = self.create_publisher(TrackStatus, '/mapping/status',
                                                 qos_reliable(5))
        self._corr_pub = self.create_publisher(PoseCorrection,
                                               '/localization/correction',
                                               qos_reliable(10))
        self.create_timer(1.0 / float(gp('publish_rate_hz')), self._publish_map)
        self._hb = HeartbeatEmitter(self, 'cone_mapping')

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        t = stamp_to_sec(msg.header.stamp)
        self._poses.add(t, x, y, yaw)

        # Odometer from speed, not from position deltas: localization
        # corrections move the pose without the car having travelled.
        if self._last_odom_t is not None:
            dt = t - self._last_odom_t
            if 0.0 < dt < 0.5:
                self._dist += abs(msg.twist.twist.linear.x) * dt
        self._last_odom_t = t
        if self._laps.update(x, y, yaw, self._dist):
            self.get_logger().info(
                f'lap {self._laps.laps} complete: {self._laps.track_length_m:.1f} m'
                + (' — track KNOWN, race speed unlocked'
                   if self._laps.laps >= self._laps_to_known else ''))

    def _on_cones(self, msg: Cone3DArray):
        pose = self._poses.query(stamp_to_sec(msg.header.stamp))
        if pose is None:
            return  # no odometry yet
        px, py, pyaw = pose

        # --- map-relative localization, BEFORE the map is updated with this
        # frame: the fit must be against landmarks the frame did not create.
        if (self._loc_enable
                and len(self._confirmed) >= self._loc_min_landmarks
                and len(msg.cones) >= self._loc_min_inliers):
            fit = solve_pose_correction(
                (px, py, pyaw), [(d.x, d.y) for d in msg.cones],
                self._query_localization_targets,
                gate=self._loc_gate, min_inliers=self._loc_min_inliers)
            out = PoseCorrection()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'odom'
            if fit is None:
                out.valid = False
                self._loc_inliers, self._loc_rms = 0, 0.0
            else:
                dx, dy, dyaw, n, rms = fit
                out.dx, out.dy, out.dyaw = float(dx), float(dy), float(dyaw)
                out.n_inliers = int(n)
                out.rms_m = float(rms)
                out.valid = True
                self._loc_inliers, self._loc_rms = n, rms
                # Use the corrected pose for THIS frame's association too —
                # the correction is measured now, waiting a cycle only feeds
                # the map a pose we already know is off.
                px, py, pyaw = px + dx, py + dy, wrap_angle(pyaw + dyaw)
            self._corr_pub.publish(out)

        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)
        t = self.get_clock().now().nanoseconds * 1e-9

        for d in msg.cones:
            # base_link -> odom
            ox = px + d.x * cos_y - d.y * sin_y
            oy = py + d.x * sin_y + d.y * cos_y
            meas_var = max(d.depth_sigma, 0.05) ** 2
            # Side of the car this cone was seen on (sign of lateral coord).
            side = ConeMapEntry.SIDE_LEFT if d.y > 0 else ConeMapEntry.SIDE_RIGHT

            # 1. Confirmed map first.
            best, best_d2 = None, self._gate ** 2
            for c in self._grid_confirmed.query_radius(ox, oy, self._search_r):
                d2 = (c.x - ox) ** 2 + (c.y - oy) ** 2
                if d2 < best_d2:
                    best, best_d2 = c, d2
            if best is not None:
                best.update(ox, oy, meas_var, d.color, side, t)
                continue

            # 2. Tentative buffer.
            best, best_d2 = None, self._gate ** 2
            for c in self._grid_tentative.query_radius(ox, oy, self._search_r):
                d2 = (c.x - ox) ** 2 + (c.y - oy) ** 2
                if d2 < best_d2:
                    best, best_d2 = c, d2
            if best is not None:
                best.update(ox, oy, meas_var, d.color, side, t)
                if best.obs_count >= self._n_confirm:
                    best.id = self._next_id
                    self._next_id += 1
                    self._tentative.remove(best)
                    self._confirmed.append(best)
            elif meas_var <= self._max_spawn_var:
                self._tentative.append(
                    Landmark(ox, oy, meas_var, d.color, side, t))

        # 3. Garbage-collect tentative only. Confirmed is sacred.
        self._tentative = [
            c for c in self._tentative
            if not (t - c.first_seen > self._tent_timeout
                    and c.obs_count < self._n_confirm)]

        # Rebuild indices (cheap at FS scale: <300 landmarks at 30 Hz).
        self._grid_confirmed.rebuild(self._confirmed)
        self._grid_tentative.rebuild(self._tentative)

    def _query_localization_targets(self, x, y, r):
        """Landmarks eligible as localization targets: confirmed and seen
        enough times that their position is settled."""
        return [(c.x, c.y)
                for c in self._grid_confirmed.query_radius(x, y, r)
                if c.obs_count >= self._loc_min_obs]

    def _publish_map(self):
        msg = ConeMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for c in self._confirmed:
            e = ConeMapEntry()
            e.id = c.id
            e.color = c.color_votes.most_common(1)[0][0]
            e.x = c.x
            e.y = c.y
            e.side = c.side_votes.most_common(1)[0][0]
            e.observation_count = min(c.obs_count, 65535)
            msg.cones.append(e)
        self._pub.publish(msg)

        st = TrackStatus()
        st.header.stamp = msg.header.stamp
        st.header.frame_id = 'odom'
        st.mode = (TrackStatus.MODE_KNOWN
                   if self._laps.laps >= self._laps_to_known
                   else TrackStatus.MODE_EXPLORE)
        st.laps_completed = min(self._laps.laps, 65535)
        st.loop_closed = self._laps.laps > 0
        st.lap_distance_m = float(self._dist - self._laps.lap_start_dist)
        st.track_length_m = float(self._laps.track_length_m)
        st.confirmed_cones = min(len(self._confirmed), 65535)
        st.localized_cones = min(self._loc_inliers, 65535)
        st.localization_rms_m = float(self._loc_rms)
        self._status_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
