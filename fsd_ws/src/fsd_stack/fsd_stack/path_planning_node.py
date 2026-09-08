"""BLOCK 5 — PATH PLANNING (owner: Mapping & Planning).

/mapping/track + /odometry/filtered + /mapping/status
        -> /planning/path        (fsd_msgs/PathPointArray)
        -> /planning/speed_limit (fsd_msgs/SpeedLimit)

Two regimes, selected by what Block 4 knows about the track:

EXPLORE (first lap) — the spec pipeline: local cone window -> Delaunay
triangulation -> keep edges joining opposite-side cones (color first, side
field as fallback) -> edge midpoints -> greedy forward ordering -> cubic
smoothing spline -> resample at 0.5 m with heading + curvature. Speed is
capped by the sensor horizon: you may not drive faster than you can see.

KNOWN (map closed, i.e. a full lap counted) — the same midpoint extraction
over the WHOLE map, ordered into a closed loop, then a bounded
minimum-curvature optimization turns that centerline into a racing line
(apexes cut, corners opened out). The loop is built once and cached; each
cycle publishes the next `known_window_ahead_m` of it, wrapping past the
start line. Nothing bounds speed to the camera range any more — that is what
makes fast laps legitimate rather than reckless.

OBSTACLE AVOIDANCE runs in both regimes. Any mapped object sitting inside the
corridor (within `obstacle_gate_m` of the CENTERLINE — a knocked-in cone,
debris, a misplaced marker) is taken out of the track-boundary set and passed
on whichever side has room, by deforming the published path with a
raised-cosine lateral offset bounded by the corridor width. If no side leaves
enough gap the planner keeps publishing a valid path but caps speed at zero:
the car stops in front of the obstacle instead of guessing.

Detection deliberately uses the centerline and never the deformed or
optimized line. Measuring against a line that has already been pushed
sideways would flag the boundary cones it was pushed toward, and dropping
those from the boundary set would tear a hole in the corridor.

Fallbacks (normative):
  * too few cones          -> republish last valid path, DEGRADED
  * no valid path > 1.0 s  -> publish empty path, ERROR (supervisor decides)
"""

import math

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial import Delaunay
from scipy.interpolate import splprep, splev

from nav_msgs.msg import Odometry
from fsd_msgs.msg import (ConeMap, ConeMapEntry, ConeDetection2D, PathPoint,
                          PathPointArray, SpeedLimit, TrackStatus, Heartbeat)

from .common import (HeartbeatEmitter, qos_reliable, stamp_to_sec,
                     yaw_from_quaternion)

BLUEISH = (ConeDetection2D.COLOR_BLUE,)
YELLOWISH = (ConeDetection2D.COLOR_YELLOW,)


def _opposite(a: ConeMapEntry, b: ConeMapEntry) -> bool:
    """True if the two cones bound OPPOSITE sides of the track.

    Colour is definitive when both cones have a track colour: two blue cones
    are the same edge no matter what the side votes say. The side fallback
    exists only for cones whose colour cannot decide it (orange, unknown).

    Getting this wrong is expensive and was: the fallback used to be reached
    for same-colour pairs too, so two duplicate landmarks of ONE blue cone
    whose side votes had split (which happens whenever a cone passes nearly
    straight ahead and measurement noise flips the sign of its lateral
    coordinate) produced a midpoint sitting on the track boundary. That dragged
    the centerline into the cones, which then measured as objects inside the
    corridor, which removed them from the boundary set and tore holes in it.
    """
    a_track = a.color in BLUEISH or a.color in YELLOWISH
    b_track = b.color in BLUEISH or b.color in YELLOWISH
    if a_track and b_track:
        return a.color != b.color
    # Colour inconclusive (orange / unknown): fall back to the side estimate.
    if a.side != ConeMapEntry.SIDE_UNKNOWN and b.side != ConeMapEntry.SIDE_UNKNOWN:
        return a.side != b.side
    return False


def periodic_catmull_rom(chain, spacing, widths=None, resample=True):
    """Closed uniform Catmull-Rom through `chain`.

    `chain` must NOT repeat its first point at the end — the curve closes by
    wrapping the control-point indices, which is what keeps curvature
    continuous across the start line instead of showing a kink there.

    resample=True  -> walk each segment at ~`spacing` (turns a coarse midpoint
                      chain into a dense path).
    resample=False -> one sample per input point, i.e. heading and curvature
                      OF the given points. Used after the racing-line offsets
                      so the optimized line keeps index-for-index
                      correspondence with the centerline it came from.

    Returns (samples, sample_widths), each sample (x, y, heading, curvature),
    from analytic spline derivatives rather than finite differences.
    """
    n = len(chain)
    if n < 4:
        return [], []
    out, out_w = [], []
    for i in range(n):
        p0 = chain[(i - 1) % n]
        p1 = chain[i]
        p2 = chain[(i + 1) % n]
        p3 = chain[(i + 2) % n]
        w1 = widths[i] if widths else 0.0
        w2 = widths[(i + 1) % n] if widths else 0.0
        if resample:
            seg = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            steps = max(1, int(round(seg / spacing)))
        else:
            steps = 1
        for j in range(steps):           # j == steps belongs to the next segment
            t = j / steps
            t2, t3 = t * t, t * t * t
            ax = 2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]
            ay = 2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]
            bx = -p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]
            by = -p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]
            x = 0.5 * (2.0 * p1[0] + (-p0[0] + p2[0]) * t + ax * t2 + bx * t3)
            y = 0.5 * (2.0 * p1[1] + (-p0[1] + p2[1]) * t + ay * t2 + by * t3)
            dx = 0.5 * ((-p0[0] + p2[0]) + 2.0 * ax * t + 3.0 * bx * t2)
            dy = 0.5 * ((-p0[1] + p2[1]) + 2.0 * ay * t + 3.0 * by * t2)
            ddx = 0.5 * (2.0 * ax + 6.0 * bx * t)
            ddy = 0.5 * (2.0 * ay + 6.0 * by * t)
            denom = (dx * dx + dy * dy) ** 1.5
            k = (dx * ddy - dy * ddx) / denom if denom > 1e-9 else 0.0
            out.append((x, y, math.atan2(dy, dx), k))
            out_w.append(w1 + (w2 - w1) * t)
    return out, out_w


def smooth_closed_chain(pts, passes=1, weight=0.25):
    """Light Laplacian smoothing of a closed control-point chain.

    Delaunay midpoints are unevenly spaced (1.1-1.9 m on a 3 m cone pitch) and
    zig-zag by a few centimetres. Uniformly parameterised Catmull-Rom turns
    that jitter into curvature spikes — measured 0.51 1/m (a 2 m radius) on a
    track whose real minimum is 0.139 — and the velocity profile then brakes
    for corners that do not exist. One pass fixes it: peak curvature drops to
    0.17 1/m and the modelled lap time improves 6%.

    The cost is a small inward bias on the tightest corners (half the chord
    sagitta, ~8 cm here), which is why the caller still verifies cone
    clearance afterwards. More passes buy little and bias more.
    """
    out = list(pts)
    n = len(out)
    if n < 4 or passes <= 0:
        return out
    for _ in range(passes):
        prev = list(out)
        for i in range(n):
            a, b, c = prev[i - 1], prev[i], prev[(i + 1) % n]
            out[i] = ((1.0 - 2.0 * weight) * b[0] + weight * (a[0] + c[0]),
                      (1.0 - 2.0 * weight) * b[1] + weight * (a[1] + c[1]))
    return out


def optimize_raceline(pts, normals, bounds, iters=400, step=0.03, reg=0.002):
    """Bounded minimum-curvature line through a closed corridor.

    Each point may slide along its normal by alpha_i, |alpha_i| <= bounds_i.
    Minimizes sum |p_{i-1} - 2 p_i + p_{i+1}|^2 (discrete curvature energy at
    uniform spacing) plus reg * sum alpha_i^2, by projected gradient descent —
    a convex problem, so the projection cannot get stuck in a local minimum.
    The regularizer matters on straights, where curvature alone does not care
    where in the corridor the line runs; it pulls those stretches back to the
    middle instead of letting them wander onto a boundary.

    `step` must stay below 2 / lambda_max of the 4th-difference operator
    (= 32) for the iteration to contract; 0.03 leaves margin.
    """
    p0 = np.asarray(pts, dtype=float)
    nrm = np.asarray(normals, dtype=float)
    bnd = np.asarray(bounds, dtype=float)
    n = len(p0)
    if n < 8:
        return [0.0] * n
    a = np.zeros(n)
    for _ in range(iters):
        p = p0 + a[:, None] * nrm
        r = np.roll(p, 1, axis=0) - 2.0 * p + np.roll(p, -1, axis=0)
        g = np.roll(r, 1, axis=0) - 2.0 * r + np.roll(r, -1, axis=0)
        grad = 2.0 * np.sum(g * nrm, axis=1) + 2.0 * reg * a
        a = np.clip(a - step * grad, -bnd, bnd)
    return a.tolist()


def choose_pass(lat, bound, clearance):
    """Which side to pass an object on, and by how much to move the line.

    The object sits `lat` to the left of the line; we need |lat - alpha| >=
    `clearance` while |alpha| stays inside `bound` (the corridor budget).
    Passing on the right means alpha <= lat - clearance, on the left
    alpha >= lat + clearance. Prefer whichever moves the line less; if neither
    fits, take the side that gets closest and report the reduced clearance so
    the caller can decide to stop instead.

    Returns (alpha, clearance_achieved).
    """
    a_right = lat - clearance
    a_left = lat + clearance
    options = [a for a in (a_right, a_left) if abs(a) <= bound]
    if options:
        alpha = min(options, key=abs)
    else:
        pick = a_right if abs(a_right) < abs(a_left) else a_left
        alpha = max(-bound, min(bound, pick))
    return alpha, abs(lat - alpha)


def apply_offsets(samples, cum_s, shifts, blend):
    """Raised-cosine lateral offsets over +-blend.

    Where two objects overlap the strongest demand wins rather than the sum:
    opposing offsets must not cancel into a path that drives through both.
    """
    out_pts = []
    for i, (x, y, h, _k) in enumerate(samples):
        s = cum_s[i]
        best = 0.0
        for station, alpha in shifts:
            d = abs(s - station)
            if d >= blend:
                continue
            val = alpha * 0.5 * (1.0 + math.cos(math.pi * d / blend))
            if abs(val) > abs(best):
                best = val
        nx, ny = -math.sin(h), math.cos(h)
        out_pts.append((x + best * nx, y + best * ny))
    return resample_derivatives(out_pts, samples)


def path_stations(path_xy):
    """Cumulative arclength along a polyline."""
    cum = [0.0]
    for i in range(1, len(path_xy)):
        cum.append(cum[-1] + math.hypot(path_xy[i][0] - path_xy[i - 1][0],
                                        path_xy[i][1] - path_xy[i - 1][1]))
    return cum


def point_to_path(path_xy, cum_s, x, y):
    """Closest point on a polyline. Returns (station, signed_lateral,
    distance) with signed_lateral positive to the LEFT of travel, or None."""
    best = None
    for i in range(len(path_xy) - 1):
        x1, y1 = path_xy[i]
        x2, y2 = path_xy[i + 1]
        ex, ey = x2 - x1, y2 - y1
        seg2 = ex * ex + ey * ey
        if seg2 < 1e-12:
            continue
        t = ((x - x1) * ex + (y - y1) * ey) / seg2
        t = max(0.0, min(1.0, t))
        cx, cy = x1 + t * ex, y1 + t * ey
        d = math.hypot(x - cx, y - cy)
        if best is None or d < best[2]:
            seg = math.sqrt(seg2)
            lat = ((x - x1) * (-ey) + (y - y1) * ex) / seg
            best = (cum_s[i] + t * seg, lat, d)
    return best


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')
        self.declare_parameter('window_ahead_m', 25.0)
        self.declare_parameter('window_behind_m', 5.0)
        self.declare_parameter('edge_min_m', 1.5)
        self.declare_parameter('edge_max_m', 6.0)
        self.declare_parameter('sample_spacing_m', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('default_track_width_m', 3.5)
        # --- known-track racing line
        self.declare_parameter('raceline_enable', True)
        self.declare_parameter('known_window_ahead_m', 60.0)
        self.declare_parameter('known_window_behind_m', 3.0)
        # Lateral budget = track_width/2 - margin. 1.0 m keeps a 1.4 m wide
        # car ~0.3 m off the cones at full lock-to-the-inside.
        self.declare_parameter('raceline_margin_m', 1.0)
        self.declare_parameter('raceline_iters', 400)
        self.declare_parameter('raceline_step', 0.03)
        self.declare_parameter('raceline_reg', 0.002)
        self.declare_parameter('raceline_min_clearance_m', 0.9)
        # Applied to the global closed chain. The local window does not need it
        # because its scipy smoothing spline (s=0.5) already absorbs midpoint
        # jitter; the C++ stack, which fits Catmull-Rom in both regimes, applies
        # this to its local chain too.
        self.declare_parameter('chain_smooth_passes', 1)
        self.declare_parameter('chain_smooth_weight', 0.25)
        self.declare_parameter('global_rebuild_cone_delta', 3)
        self.declare_parameter('global_rebuild_min_period_s', 2.0)
        # --- obstacle avoidance
        # obstacle_gate_m MUST stay below half of the narrowest track width,
        # or genuine boundary cones start looking like obstacles.
        self.declare_parameter('obstacle_gate_m', 1.1)
        self.declare_parameter('obstacle_clearance_m', 0.9)
        self.declare_parameter('obstacle_min_clearance_m', 0.5)
        # A lateral offset A blended over a longitudinal distance D peaks at
        # pi^2*A*v^2/(2*D^2) of lateral acceleration, so a FIXED blend distance
        # becomes undriveable as speed rises (0.9 m over 5 m at 11 m/s asks for
        # 21 m/s^2). Making the blend a TIME instead makes the demand
        # speed-invariant: D = t*v gives pi^2*A/(2*t^2) = 7.3 m/s^2 at t = 0.8 s
        # for the widest offset this planner will ever command.
        self.declare_parameter('obstacle_blend_m', 5.0)
        self.declare_parameter('obstacle_blend_time_s', 0.8)
        self.declare_parameter('obstacle_blend_max_m', 12.0)
        self.declare_parameter('obstacle_edge_margin_m', 0.8)
        self.declare_parameter('obstacle_lookahead_m', 20.0)
        # Two cycles (0.2 s) is the shortest confirmation that still rejects a
        # one-frame ghost. At race speed every extra cycle costs over a metre
        # of the distance available to swerve in, and the mapper has already
        # required N_CONFIRM sightings before this cone existed at all.
        self.declare_parameter('obstacle_confirm_frames', 2)
        self.declare_parameter('obstacle_speed_mps', 3.0)
        self.declare_parameter('blocked_confirm_frames', 3)
        # Big orange cones mark the start/finish line and legitimately sit
        # near the corridor edge — never treat them as things to dodge.
        self.declare_parameter('obstacle_ignore_big_orange', True)
        # --- speed regimes (the cap published to Block 6)
        self.declare_parameter('v_explore_mps', 6.0)
        self.declare_parameter('v_race_mps', 15.0)
        self.declare_parameter('stationary_speed_mps', 0.5)

        gp = lambda n: float(self.get_parameter(n).value)
        gi = lambda n: int(self.get_parameter(n).value)
        gb = lambda n: bool(self.get_parameter(n).value)
        self._ahead = gp('window_ahead_m')
        self._behind = gp('window_behind_m')
        self._edge_min = gp('edge_min_m')
        self._edge_max = gp('edge_max_m')
        self._spacing = gp('sample_spacing_m')
        self._track_w = gp('default_track_width_m')
        self._race_enable = gb('raceline_enable')
        self._known_ahead = gp('known_window_ahead_m')
        self._known_behind = gp('known_window_behind_m')
        self._race_margin = gp('raceline_margin_m')
        self._race_iters = gi('raceline_iters')
        self._race_step = gp('raceline_step')
        self._race_reg = gp('raceline_reg')
        self._race_min_clear = gp('raceline_min_clearance_m')
        self._chain_smooth = gi('chain_smooth_passes')
        self._chain_weight = gp('chain_smooth_weight')
        self._rebuild_delta = gi('global_rebuild_cone_delta')
        self._rebuild_period = gp('global_rebuild_min_period_s')
        self._obs_gate = gp('obstacle_gate_m')
        self._obs_clear = gp('obstacle_clearance_m')
        self._obs_min_clear = gp('obstacle_min_clearance_m')
        self._obs_blend = gp('obstacle_blend_m')
        self._obs_blend_t = gp('obstacle_blend_time_s')
        self._obs_blend_max = gp('obstacle_blend_max_m')
        self._obs_edge_margin = gp('obstacle_edge_margin_m')
        self._obs_lookahead = gp('obstacle_lookahead_m')
        self._obs_confirm = gi('obstacle_confirm_frames')
        self._obs_speed = gp('obstacle_speed_mps')
        self._blocked_confirm = gi('blocked_confirm_frames')
        self._obs_ignore_big = gb('obstacle_ignore_big_orange')
        self._v_explore = gp('v_explore_mps')
        self._v_race = gp('v_race_mps')
        self._stationary_speed = gp('stationary_speed_mps')

        self._map = None
        self._pose = None                  # (x, y, yaw)
        self._speed = 0.0
        self._mode = TrackStatus.MODE_EXPLORE
        self._last_valid = None            # last good PathPointArray
        self._last_valid_t = None
        self._last_cap = (self._v_explore, SpeedLimit.REASON_EXPLORE, 'startup')

        self._obstacle_ids = set()
        self._obstacle_seen = {}           # cone id -> consecutive in-corridor frames
        self._blocked_frames = 0
        self._global = None                # (race, centre, widths, lap_length)
        self._global_cones = 0
        self._global_t = None
        self._global_obstacles = frozenset()
        self._global_failed_logged = False

        self.create_subscription(ConeMap, '/mapping/track', self._on_map,
                                 qos_reliable(5))
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom,
                                 qos_reliable(10))
        self.create_subscription(TrackStatus, '/mapping/status', self._on_status,
                                 qos_reliable(5))
        self._pub = self.create_publisher(PathPointArray, '/planning/path',
                                          qos_reliable(5))
        self._cap_pub = self.create_publisher(SpeedLimit, '/planning/speed_limit',
                                              qos_reliable(5))
        self.create_timer(1.0 / gp('publish_rate_hz'), self._plan)
        self._hb = HeartbeatEmitter(self, 'path_planning')

    def _on_map(self, msg):
        self._map = msg

    def _on_odom(self, msg):
        self._pose = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw_from_quaternion(msg.pose.pose.orientation))
        self._speed = abs(msg.twist.twist.linear.x)

    def _on_status(self, msg: TrackStatus):
        self._mode = msg.mode

    # ------------------------------------------------------------------ plan
    def _plan(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        path = self._compute_path()
        if path is not None:
            self._last_valid = path
            self._last_valid_t = now
            self._hb.set_status(Heartbeat.STATUS_OK)
            self._pub.publish(path)
            self._publish_cap()
            return

        # Fallback ladder. ERROR latches the EBS, so it is reserved for losing
        # a path the car is relying on RIGHT NOW — i.e. while moving. A
        # stationary car without a path is a waiting state, not an emergency,
        # and it is the normal state at the start line: the first blue/yellow
        # gate is often still out of pairing range, so validity flickers until
        # the car creeps far enough forward to see both edges. The
        # supervisor's moving-blind check still covers the dangerous case.
        if self._last_valid is None or self._speed < self._stationary_speed:
            self._hb.set_status(
                Heartbeat.STATUS_DEGRADED,
                'waiting for first valid path' if self._last_valid is None
                else 'no path while stationary')
            empty = PathPointArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = 'odom'
            self._pub.publish(empty)
        elif now - self._last_valid_t <= 1.0:
            self._hb.set_status(Heartbeat.STATUS_DEGRADED,
                                'no fresh path, holding last valid')
            self._pub.publish(self._last_valid)
        else:
            self._hb.set_status(Heartbeat.STATUS_ERROR,
                                'no valid path for > 1 s')
            empty = PathPointArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = 'odom'
            self._pub.publish(empty)
        self._publish_cap()

    def _publish_cap(self):
        v, reason, detail = self._last_cap
        m = SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.v_max_mps = float(v)
        m.reason = reason
        m.detail = detail
        self._cap_pub.publish(m)

    # --------------------------------------------------------------- compute
    def _compute_path(self):
        if self._map is None or self._pose is None:
            return None
        boundary = [c for c in self._map.cones if c.id not in self._obstacle_ids]

        samples = centre = widths = None
        source = 'local'
        if self._race_enable and self._mode == TrackStatus.MODE_KNOWN:
            got = self._known_window(boundary)
            if got is not None:
                samples, centre, widths = got
                source = 'race'
        if samples is None:
            got = self._local_window(boundary)
            if got is None:
                return None
            samples, widths = got
            centre = samples          # the local path IS the centerline

        samples, widths, cap = self._avoid(samples, centre, widths, source)

        msg = PathPointArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for (x, y, h, k), w in zip(samples, widths):
            p = PathPoint()
            p.x = float(x)
            p.y = float(y)
            p.heading = float(h)
            p.curvature = float(k)
            p.track_width = float(w if w > 0.0 else self._track_w)
            msg.points.append(p)
        self._last_cap = cap
        return msg

    # ------------------------------------------------------- EXPLORE regime
    def _local_window(self, cones_all):
        px, py, pyaw = self._pose
        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)

        cones = []
        for c in cones_all:
            dx, dy = c.x - px, c.y - py
            lx = dx * cos_y + dy * sin_y          # forward
            ly = -dx * sin_y + dy * cos_y         # left
            if -self._behind <= lx <= self._ahead and abs(ly) <= 15.0:
                cones.append(c)
        if len(cones) < 4:
            return None

        kept, widths = self._midpoints(cones)
        if kept is None:
            return None
        chain = self._order_midpoints(kept, px, py, pyaw)
        if chain is None or len(chain) < 4:
            return None
        ordered = kept[chain]
        w_ordered = [widths[i] for i in chain]
        return self._fit_and_sample(ordered, w_ordered)

    def _midpoints(self, cones):
        """Delaunay -> midpoints of opposite-side edges, deduped.
        Returns (np.array of points, list of corridor widths) or (None, None)."""
        pts = np.array([[c.x, c.y] for c in cones])
        try:
            tri = Delaunay(pts)
        except Exception:
            return None, None

        edges = set()
        for simplex in tri.simplices:
            for i in range(3):
                a, b = simplex[i], simplex[(i + 1) % 3]
                edges.add((min(a, b), max(a, b)))

        midpoints = []
        for a, b in edges:
            if not _opposite(cones[a], cones[b]):
                continue
            d = float(np.linalg.norm(pts[a] - pts[b]))
            if not (self._edge_min <= d <= self._edge_max):
                continue
            midpoints.append(((pts[a] + pts[b]) / 2.0, d))
        if len(midpoints) < 4:
            return None, None

        # Dedupe midpoints closer than 0.3 m (shared cones create near-dupes).
        kept, widths = [], []
        for m, w in midpoints:
            if all(np.linalg.norm(m - k) > 0.3 for k in kept):
                kept.append(m)
                widths.append(w)
        if len(kept) < 4:
            return None, None
        return np.array(kept), widths

    def _order_midpoints(self, pts, px, py, pyaw):
        """Greedy nearest-neighbour chain, starting near the car, walking
        forward. Rejects backward jumps and steps > edge_max."""
        n = len(pts)
        car = np.array([px, py])
        heading = np.array([math.cos(pyaw), math.sin(pyaw)])

        # Start: nearest midpoint that is not clearly behind the car.
        rel = pts - car
        fwd = rel @ heading
        cand = np.where(fwd > -2.0)[0]
        if cand.size == 0:
            return None
        start = int(cand[np.argmin(np.linalg.norm(rel[cand], axis=1))])

        used = {start}
        chain = [start]
        direction = heading.copy()
        while True:
            last = pts[chain[-1]]
            best, best_d = None, self._edge_max + 1.0
            for i in range(n):
                if i in used:
                    continue
                step = pts[i] - last
                d = float(np.linalg.norm(step))
                if d < 1e-6 or d > self._edge_max:
                    continue
                # Forward-progress constraint: no sharp reversals.
                if float(step @ direction) / d < -0.2:
                    continue
                if d < best_d:
                    best, best_d = i, d
            if best is None:
                break
            step = pts[best] - last
            direction = step / np.linalg.norm(step)
            used.add(best)
            chain.append(best)
        return chain

    def _fit_and_sample(self, pts, widths):
        """Smoothing spline through an OPEN chain -> uniform arclength
        samples. Returns (samples, sample_widths) or None."""
        xs, ys = pts[:, 0], pts[:, 1]
        k = min(3, len(pts) - 1)
        try:
            tck, _ = splprep([xs, ys], s=0.5, k=k)
        except Exception:
            return None

        u_dense = np.linspace(0.0, 1.0, 400)
        xd, yd = splev(u_dense, tck)
        seg = np.hypot(np.diff(xd), np.diff(yd))
        s_dense = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s_dense[-1])
        if total < 3.0:
            return None
        n_samples = max(int(total / self._spacing), 6)
        s_targets = np.linspace(0.0, total, n_samples)
        u_samples = np.interp(s_targets, s_dense, u_dense)

        x, y = splev(u_samples, tck)
        dx, dy = splev(u_samples, tck, der=1)
        ddx, ddy = splev(u_samples, tck, der=2)

        mean_w = float(np.mean(widths)) if len(widths) else self._track_w
        samples = []
        for i in range(n_samples):
            denom = (float(dx[i]) ** 2 + float(dy[i]) ** 2) ** 1.5
            kappa = ((float(dx[i]) * float(ddy[i]) - float(dy[i]) * float(ddx[i]))
                     / denom) if denom > 1e-9 else 0.0
            samples.append((float(x[i]), float(y[i]),
                            math.atan2(float(dy[i]), float(dx[i])), kappa))
        return samples, [mean_w] * n_samples

    # --------------------------------------------------------- KNOWN regime
    def _known_window(self, boundary):
        """Cached closed racing line, sliced to a forward window.
        Returns (race_window, centre_window, widths_window) or None."""
        now = self.get_clock().now().nanoseconds * 1e-9
        obs = frozenset(self._obstacle_ids)
        stale = (self._global is None
                 or obs != self._global_obstacles
                 or len(boundary) - self._global_cones >= self._rebuild_delta)
        if stale and (self._global is None
                      or self._global_t is None
                      or now - self._global_t >= self._rebuild_period):
            built = self._build_global(boundary)
            self._global_t = now
            if built is not None:
                self._global = built
                self._global_cones = len(boundary)
                self._global_obstacles = obs
                self._global_failed_logged = False
                self.get_logger().info(
                    f'global racing line: {len(built[0])} pts, '
                    f'{built[3]:.1f} m lap, from {len(boundary)} cones')
            elif not self._global_failed_logged:
                self._global_failed_logged = True
                self.get_logger().warn(
                    'known-track mode: could not close a global loop from the '
                    'map — staying on the local window')
        if self._global is None:
            return None

        race, centre, widths, _total = self._global
        px, py, _ = self._pose
        n = len(race)
        i_near = min(range(n), key=lambda i: (race[i][0] - px) ** 2
                     + (race[i][1] - py) ** 2)
        # A few points behind the car so Block 6's nearest-point search has
        # somewhere to sit even if the car is slightly ahead of the slice.
        back = int(round(self._known_behind / self._spacing))
        start = (i_near - back) % n
        r_out, c_out, w_out, acc = [], [], [], 0.0
        limit = self._known_ahead + self._known_behind
        prev = None
        for step in range(n):
            idx = (start + step) % n
            s = race[idx]
            if prev is not None:
                acc += math.hypot(s[0] - prev[0], s[1] - prev[1])
                if acc > limit:
                    break
            r_out.append(s)
            c_out.append(centre[idx])
            w_out.append(widths[idx])
            prev = s
        if len(r_out) < 6:
            return None
        return r_out, c_out, w_out

    def _build_global(self, boundary):
        """Whole-map closed centerline -> bounded minimum-curvature racing
        line. Returns (race, centre, widths, lap_length) or None."""
        if len(boundary) < 12:
            return None
        kept, widths = self._midpoints(boundary)
        if kept is None:
            return None
        px, py, pyaw = self._pose
        chain = self._order_midpoints(kept, px, py, pyaw)
        if chain is None or len(chain) < 12:
            return None
        # A real lap: the walk came back to where it started and used most of
        # the midpoints. Both checks matter — a shortcut across a hairpin can
        # close a loop while skipping half the track.
        loop = kept[chain]
        gap = float(np.linalg.norm(loop[-1] - loop[0]))
        if gap > self._edge_max or len(chain) < 0.75 * len(kept):
            return None

        ordered = smooth_closed_chain([(float(p[0]), float(p[1])) for p in loop],
                                      self._chain_smooth, self._chain_weight)
        w_ordered = [widths[i] for i in chain]
        centre, c_widths = periodic_catmull_rom(ordered, self._spacing, w_ordered)
        if len(centre) < 12:
            return None
        total = 0.0
        for i in range(len(centre)):
            j = (i + 1) % len(centre)
            total += math.hypot(centre[j][0] - centre[i][0],
                                centre[j][1] - centre[i][1])

        cones_xy = [(c.x, c.y) for c in boundary]
        base = [(s[0], s[1]) for s in centre]
        normals = [(-math.sin(s[2]), math.cos(s[2])) for s in centre]
        bounds = [max(0.0, w / 2.0 - self._race_margin) for w in c_widths]
        alpha = optimize_raceline(base, normals, bounds, self._race_iters,
                                  self._race_step, self._race_reg)
        shifted = [(base[i][0] + alpha[i] * normals[i][0],
                    base[i][1] + alpha[i] * normals[i][1])
                   for i in range(len(base))]
        # resample=False: one output sample per input point, so the racing
        # line stays index-aligned with the centerline it was derived from.
        race, _ = periodic_catmull_rom(shifted, self._spacing, c_widths,
                                       resample=False)
        if len(race) != len(centre):
            return None

        # Verify, then trust. If the optimized line does not keep clear of
        # the cones it was bounded by, the corridor width estimate was wrong
        # somewhere — fall back to the centerline rather than ship it.
        if min_clearance(race, cones_xy) < self._race_min_clear:
            self.get_logger().warn(
                'racing line failed the clearance check — using the '
                'centerline for this map')
            race = centre
        race = smooth_curvature(race)
        return race, centre, c_widths, total

    # ---------------------------------------------------- obstacle handling
    def _avoid(self, samples, centre, widths, source):
        """Flag in-corridor objects, deform the path around them, and decide
        the speed cap. Returns (samples, widths, cap)."""
        base_v = self._v_race if source == 'race' else self._v_explore
        base_reason = (SpeedLimit.REASON_RACE if source == 'race'
                       else SpeedLimit.REASON_EXPLORE)
        base_detail = ('known track, global racing line' if source == 'race'
                       else 'exploring: capped by sensor horizon')
        no_obstacles = (samples, widths, (base_v, base_reason, base_detail))

        path_xy = [(s[0], s[1]) for s in samples]
        cum_s = path_stations(path_xy)
        centre_xy = [(s[0], s[1]) for s in centre]
        centre_s = path_stations(centre_xy)

        obstacles = self._scan_obstacles(centre_xy, centre_s, path_xy, cum_s)
        if not obstacles:
            self._blocked_frames = 0
            return no_obstacles

        car = point_to_path(path_xy, cum_s, self._pose[0], self._pose[1])
        car_s = car[0] if car else 0.0

        shifts, blocked = [], []
        for cone, station, lat in obstacles:
            if station < car_s - 1.0 or station > car_s + self._obs_lookahead:
                continue
            i_w = min(range(len(cum_s)), key=lambda i: abs(cum_s[i] - station))
            w = widths[i_w] if widths[i_w] > 0.0 else self._track_w
            bound = max(0.0, w / 2.0 - self._obs_edge_margin)
            alpha, _achieved = choose_pass(lat, bound, self._obs_clear)
            shifts.append((station, alpha))

        if not shifts:
            self._blocked_frames = 0
            return no_obstacles

        blend = min(self._obs_blend_max,
                    max(self._obs_blend, self._obs_blend_t * self._speed))
        out = apply_offsets(samples, cum_s, shifts, blend)

        # Final gate: measure the clearance actually achieved. Overlapping
        # obstacles can fight each other, and geometry beats intent.
        out_xy = [(s[0], s[1]) for s in out]
        out_s = path_stations(out_xy)
        for cone, station, _lat in obstacles:
            if station < car_s - 1.0 or station > car_s + self._obs_lookahead:
                continue
            hit = point_to_path(out_xy, out_s, cone.x, cone.y)
            if hit is not None and hit[2] < self._obs_min_clear:
                blocked.append((cone, hit[2]))

        if blocked:
            self._blocked_frames += 1
            if self._blocked_frames >= self._blocked_confirm:
                cone, clear = min(blocked, key=lambda b: b[1])
                return out, widths, (
                    0.0, SpeedLimit.REASON_BLOCKED,
                    f'corridor blocked by cone {cone.id}: only {clear:.2f} m '
                    f'of gap — stopping')
        else:
            self._blocked_frames = 0

        return out, widths, (
            min(base_v, self._obs_speed), SpeedLimit.REASON_OBSTACLE,
            f'avoiding {len(shifts)} object(s) in the corridor')

    def _scan_obstacles(self, centre_xy, centre_s, path_xy, cum_s):
        """Find mapped objects inside the corridor.

        Membership is decided against the CENTERLINE; the station and lateral
        offset used to place the avoidance manoeuvre are measured on the path
        actually being published. Flags are sticky once confirmed on
        `obstacle_confirm_frames` consecutive cycles: a cone we have decided to
        drive around must not flicker back into the boundary set, which would
        make the path oscillate.
        """
        px, py, _ = self._pose
        reach = self._obs_lookahead + 10.0
        found, alive = [], set()
        for c in self._map.cones:
            if (self._obs_ignore_big
                    and c.color == ConeDetection2D.COLOR_ORANGE_BIG):
                continue
            if math.hypot(c.x - px, c.y - py) > reach:
                continue
            ref = point_to_path(centre_xy, centre_s, c.x, c.y)
            if ref is None or ref[2] > self._obs_gate:
                continue
            alive.add(c.id)
            if c.id not in self._obstacle_ids:
                seen = self._obstacle_seen.get(c.id, 0) + 1
                self._obstacle_seen[c.id] = seen
                if seen < self._obs_confirm:
                    continue
                self._obstacle_ids.add(c.id)
                self.get_logger().warn(
                    f'obstacle in corridor: cone {c.id} at '
                    f'({c.x:.1f}, {c.y:.1f}), {ref[2]:.2f} m off the centerline')
            hit = point_to_path(path_xy, cum_s, c.x, c.y)
            if hit is not None:
                found.append((c, hit[0], hit[1]))
        for cid in [k for k in self._obstacle_seen if k not in alive]:
            del self._obstacle_seen[cid]
        return found


def min_clearance(samples, cones_xy):
    """Smallest distance from any sample to any cone."""
    if not cones_xy:
        return float('inf')
    best = float('inf')
    for x, y, _, _ in samples:
        for cx, cy in cones_xy:
            d = math.hypot(x - cx, y - cy)
            if d < best:
                best = d
    return best


def smooth_curvature(samples, half=2, closed=True):
    """5-point moving average on curvature only. The car cannot respond to
    half-metre curvature spikes, and the velocity profile turns them into
    speed chatter. `closed` wraps the window (racing line); otherwise the
    window is clamped at the ends (a published forward window)."""
    n = len(samples)
    out = []
    for i in range(n):
        acc, cnt = 0.0, 0
        for j in range(i - half, i + half + 1):
            k = j % n if closed else min(max(j, 0), n - 1)
            acc += samples[k][3]
            cnt += 1
        out.append((samples[i][0], samples[i][1], samples[i][2], acc / cnt))
    return out


def resample_derivatives(pts, fallback):
    """Heading and curvature of an OPEN deformed polyline: three-point circle
    through each point's neighbours. No spline refit needed at 0.5 m spacing,
    and it degrades gracefully on straights.

    The first and last point have no two-sided neighbourhood, so their
    curvature is carried over from the adjacent point rather than left at
    zero. Left at zero it was the single largest source of disagreement with
    the analytic spline on identical geometry (0.059 1/m at the endpoint,
    bleeding three points inward through the moving average, against 0.001 1/m
    in the interior) — and a curvature of zero at the end of the window reads
    as "straight ahead, carry speed" to the velocity profile.
    """
    n = len(pts)
    if n < 3:
        return [(p[0], p[1], fallback[i][2], 0.0) for i, p in enumerate(pts)]
    out = []
    for i in range(n):
        x, y = pts[i]
        i0, i2 = max(0, i - 1), min(n - 1, i + 1)
        h = math.atan2(pts[i2][1] - pts[i0][1], pts[i2][0] - pts[i0][0])
        k = 0.0
        if i0 != i and i2 != i:
            x0, y0 = pts[i0]
            x2, y2 = pts[i2]
            a = math.hypot(x - x0, y - y0)
            b = math.hypot(x2 - x, y2 - y)
            c = math.hypot(x2 - x0, y2 - y0)
            area2 = (x - x0) * (y2 - y0) - (y - y0) * (x2 - x0)
            if a * b * c > 1e-9:
                k = 2.0 * area2 / (a * b * c)
        out.append([x, y, h, k])
    out[0][3] = out[1][3]
    out[n - 1][3] = out[n - 2][3]
    return smooth_curvature([tuple(o) for o in out], half=2, closed=False)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
