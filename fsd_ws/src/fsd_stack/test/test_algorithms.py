"""Offline verification of the algorithm cores in fsd_stack, with rclpy
stubbed out so the actual node modules import unmodified. Tests:
   1. sim track generation
   2. cone mapping data association (dedupe on repeated noisy observations)
   3. path planning (Delaunay -> midpoints -> ordering -> spline)
   4. velocity profile monotonic feasibility
   5. closed-loop: bicycle model + pure pursuit follows the planned path
   6. PoseBuffer interpolation
   7. map-relative localization recovers injected dead-reckoning drift
   8. slew-limited correction application never jumps the pose
   9. lap counting on a closed track
  10. global closed racing line: closes, clears the cones, laps faster
  11. obstacle avoidance: path clears an object on the line, stays in bounds
  12. blocked corridor: no feasible gap -> speed cap of zero
"""

import math
import random
import sys
import types
import numpy as np

random.seed(7)
np.random.seed(7)

# ---------------------------------------------------------------- stubs
def _stub_module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m

class _Enum:
    pass

# std_msgs-ish message stubs with the constants the code uses
class ConeDetection2D:
    COLOR_BLUE = 0; COLOR_YELLOW = 1; COLOR_ORANGE_SMALL = 2
    COLOR_ORANGE_BIG = 3; COLOR_UNKNOWN = 255
    SOURCE_YOLO = 0; SOURCE_HSV = 1

class ConeMapEntry:
    SIDE_UNKNOWN = 0; SIDE_LEFT = 1; SIDE_RIGHT = 2
    def __init__(self, x=0.0, y=0.0, color=0, side=0, cid=0):
        self.x, self.y, self.color, self.side = x, y, color, side
        self.id = cid; self.observation_count = 0

class Header:
    def __init__(self): self.stamp = None; self.frame_id = ''

class PathPoint:
    def __init__(self):
        self.x = self.y = self.heading = self.curvature = self.track_width = 0.0

class PathPointArray:
    def __init__(self): self.header = Header(); self.points = []

class Heartbeat:
    STATUS_OK = 0; STATUS_DEGRADED = 1; STATUS_ERROR = 2

class TrackStatus:
    MODE_EXPLORE = 0; MODE_KNOWN = 1
    def __init__(self):
        self.header = Header(); self.mode = 0; self.laps_completed = 0
        self.loop_closed = False; self.lap_distance_m = 0.0
        self.track_length_m = 0.0; self.confirmed_cones = 0
        self.localized_cones = 0; self.localization_rms_m = 0.0

class SpeedLimit:
    REASON_EXPLORE = 0; REASON_RACE = 1; REASON_OBSTACLE = 2; REASON_BLOCKED = 3
    def __init__(self):
        self.header = Header(); self.v_max_mps = 0.0
        self.reason = 0; self.detail = ''

class PoseCorrection:
    def __init__(self):
        self.header = Header(); self.dx = self.dy = self.dyaw = 0.0
        self.n_inliers = 0; self.rms_m = 0.0; self.valid = False

_stub_module('rclpy')
_stub_module('rclpy.node', Node=object)
_stub_module('rclpy.qos', QoSProfile=lambda **k: None,
             QoSReliabilityPolicy=_Enum, QoSHistoryPolicy=_Enum)
_Enum.RELIABLE = _Enum.BEST_EFFORT = _Enum.KEEP_LAST = None
_stub_module('fsd_msgs')
_stub_module('fsd_msgs.msg', ConeDetection2D=ConeDetection2D,
             ConeMapEntry=ConeMapEntry, PathPoint=PathPoint,
             PathPointArray=PathPointArray, Heartbeat=Heartbeat,
             TrackStatus=TrackStatus, SpeedLimit=SpeedLimit,
             PoseCorrection=PoseCorrection,
             ConeDetection2DArray=object, Cone3D=object, Cone3DArray=object,
             ConeMap=object, VehicleCmd=object, WheelSpeeds=object,
             VehicleStatus=object)
_stub_module('nav_msgs')
_stub_module('nav_msgs.msg', Odometry=object)
_stub_module('std_msgs')
_stub_module('std_msgs.msg', Bool=object)
_stub_module('sensor_msgs')
_stub_module('sensor_msgs.msg', Imu=object, Image=object)

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ------------------------------------------------- import real modules
from fsd_stack.common import PoseBuffer, wrap_angle
from fsd_stack.cone_mapping_node import (Landmark, GridIndex, LapCounter,
                                         solve_pose_correction)
from fsd_stack.path_planning_node import (PathPlanningNode, _opposite,
                                          min_clearance, optimize_raceline,
                                          periodic_catmull_rom)
from fsd_stack.state_estimation_node import StateEstimationNode

# ---------------------------------------------------- 1. track builder
def build_track(a=20.0, b=12.0, spacing=3.0, half_width=1.75):
    cones = []
    n_dense = 2000
    pts = [(a * math.cos(2*math.pi*i/n_dense), b * math.sin(2*math.pi*i/n_dense),
            2*math.pi*i/n_dense) for i in range(n_dense)]
    s_acc, next_at, last = 0.0, 0.0, pts[0]
    for p in pts + [pts[0]]:
        s_acc += math.hypot(p[0]-last[0], p[1]-last[1])
        if s_acc >= next_at:
            t = p[2]
            tx, ty = -a*math.sin(t), b*math.cos(t)
            n = math.hypot(tx, ty); tx, ty = tx/n, ty/n
            nx, ny = -ty, tx
            cones.append((p[0]+half_width*nx, p[1]+half_width*ny, 0))  # blue L
            cones.append((p[0]-half_width*nx, p[1]-half_width*ny, 1))  # yellow R
            next_at += spacing
        last = p
    return cones

track = build_track()
n_blue = sum(1 for c in track if c[2] == 0)
n_yellow = sum(1 for c in track if c[2] == 1)
assert n_blue == n_yellow and n_blue > 25, (n_blue, n_yellow)
print(f'1. track OK: {n_blue} blue + {n_yellow} yellow cones')

# --------------------------------------- 2. data association (no dupes)
SEARCH_R, GATE, N_CONFIRM = 1.2, 0.5, 3
confirmed, tentative = [], []
gc, gt = GridIndex(), GridIndex()
next_id = 1

def associate(ox, oy, var, color, side, t):
    global next_id
    best, bd = None, GATE**2
    for c in gc.query_radius(ox, oy, SEARCH_R):
        d2 = (c.x-ox)**2 + (c.y-oy)**2
        if d2 < bd: best, bd = c, d2
    if best is not None:
        best.update(ox, oy, var, color, side, t); return
    best, bd = None, GATE**2
    for c in gt.query_radius(ox, oy, SEARCH_R):
        d2 = (c.x-ox)**2 + (c.y-oy)**2
        if d2 < bd: best, bd = c, d2
    if best is not None:
        best.update(ox, oy, var, color, side, t)
        if best.obs_count >= N_CONFIRM:
            best.id = next_id; next_id += 1
            tentative.remove(best); confirmed.append(best)
    else:
        tentative.append(Landmark(ox, oy, var, color, side, t))
    gc.rebuild(confirmed); gt.rebuild(tentative)

# 20 noisy re-observations of every cone
for frame in range(20):
    for (cx, cy, col) in track[:40]:   # 40 physical cones
        associate(cx + random.gauss(0, 0.08), cy + random.gauss(0, 0.08),
                  0.01, col, 1, float(frame))
    gc.rebuild(confirmed); gt.rebuild(tentative)

assert len(confirmed) == 40, f'expected 40 confirmed, got {len(confirmed)}'
errs = []
for lm in confirmed:
    d = min(math.hypot(lm.x-cx, lm.y-cy) for (cx, cy, _) in track[:40])
    errs.append(d)
assert max(errs) < 0.15, max(errs)
print(f'2. association OK: 40/40 confirmed, zero dupes, max pos err {max(errs):.3f} m')

# --------------------------------------------- 3. path planning geometry
PLANNER_DEFAULTS = dict(
    _ahead=25.0, _behind=5.0, _edge_min=1.5, _edge_max=6.0, _spacing=0.5,
    _track_w=3.5, _race_enable=True, _known_ahead=60.0, _known_behind=3.0,
    _race_margin=1.0, _race_iters=400, _race_step=0.03, _race_reg=0.002,
    _race_min_clear=0.9, _chain_smooth=1, _chain_weight=0.25,
    _rebuild_delta=3, _rebuild_period=2.0,
    _obs_gate=1.1, _obs_clear=0.9, _obs_min_clear=0.5, _obs_blend=5.0,
    _obs_blend_t=0.8, _obs_blend_max=12.0, _speed=0.0,
    _obs_edge_margin=0.8, _obs_lookahead=20.0, _obs_confirm=2,
    _obs_speed=3.0, _blocked_confirm=3, _obs_ignore_big=True,
    _v_explore=6.0, _v_race=15.0, _stationary_speed=0.5,
    _map=None, _pose=None, _mode=TrackStatus.MODE_EXPLORE,
    _last_valid=None, _last_valid_t=None,
    _last_cap=(6.0, SpeedLimit.REASON_EXPLORE, 'startup'),
    _blocked_frames=0, _global=None, _global_cones=0, _global_t=None,
    _global_obstacles=frozenset(), _global_failed_logged=False,
)

class FakeClockNode:
    class _T:
        nanoseconds = 0
        def to_msg(self): return None
    def now(self): return self._T()

class FakeLogger:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass

def make_planner(**overrides):
    p = PathPlanningNode.__new__(PathPlanningNode)
    for k, v in PLANNER_DEFAULTS.items():
        setattr(p, k, v)
    p._obstacle_ids = set()
    p._obstacle_seen = {}
    for k, v in overrides.items():
        setattr(p, k, v)
    p.get_clock = lambda: FakeClockNode()
    p.get_logger = lambda: FakeLogger()
    return p

class FakeMap:
    def __init__(self, cones):
        self.cones = cones

def map_from(track_cones, extra=()):
    cones = [ConeMapEntry(x, y, color, 1 if color == 0 else 2, i + 1)
             for i, (x, y, color) in enumerate(track_cones)]
    base = len(cones)
    for j, (x, y, color, side) in enumerate(extra):
        cones.append(ConeMapEntry(x, y, color, side, base + j + 1))
    return FakeMap(cones)

planner = make_planner(_map=map_from(track), _pose=(20.0, 0.0, math.pi / 2.0))
path = planner._compute_path()
assert path is not None and len(path.points) >= 20, \
    f'path failed: {None if path is None else len(path.points)}'

# every path point must stay >= 0.75 m from every cone (spec acceptance)
cones_xy = [(cx, cy) for (cx, cy, _) in track]
local_clearance = min(
    min(math.hypot(p.x-cx, p.y-cy) for (cx, cy) in cones_xy)
    for p in path.points)
max_curv = max(abs(p.curvature) for p in path.points)
assert local_clearance >= 0.75, f'clearance {local_clearance:.2f}'
assert max_curv < 0.5, f'curvature {max_curv:.2f} implausible for this track'
# Same-colour cones are the SAME edge, whatever the side votes say. Duplicate
# landmarks of one cone routinely end up with split side votes (noise flips the
# sign of a nearly-straight-ahead lateral coordinate), and treating those as a
# gate pair puts a midpoint on the track boundary.
blue_l = ConeMapEntry(0.0, 0.0, ConeDetection2D.COLOR_BLUE, ConeMapEntry.SIDE_LEFT)
blue_r = ConeMapEntry(0.5, 0.0, ConeDetection2D.COLOR_BLUE, ConeMapEntry.SIDE_RIGHT)
yel_l = ConeMapEntry(0.0, 3.0, ConeDetection2D.COLOR_YELLOW, ConeMapEntry.SIDE_LEFT)
unk_l = ConeMapEntry(0.0, 0.0, ConeDetection2D.COLOR_UNKNOWN, ConeMapEntry.SIDE_LEFT)
unk_r = ConeMapEntry(0.5, 0.0, ConeDetection2D.COLOR_UNKNOWN, ConeMapEntry.SIDE_RIGHT)
assert not _opposite(blue_l, blue_r), 'two blue cones treated as opposite edges'
assert _opposite(blue_l, yel_l), 'blue/yellow must pair regardless of side'
assert _opposite(unk_l, unk_r), 'side must still decide when colour cannot'
print('3a. edge pairing OK: colour decides for track cones, side only when '
      'colour cannot')

print(f'3. planning OK: {len(path.points)} pts, min cone clearance '
      f'{local_clearance:.2f} m, max |curvature| {max_curv:.3f} 1/m '
      f'(ellipse min-R curvature = {20/(12**2):.3f})')

# ------------------------------------------------- 4. velocity profile
def velocity_profile(points, v_max=5.0, a_lat=4.0, a_brk=5.0, a_acc=3.0):
    n = len(points)
    v = [min(v_max, math.sqrt(a_lat / max(abs(p.curvature), 1e-3)))
         for p in points]
    ds = [math.hypot(points[i+1].x-points[i].x, points[i+1].y-points[i].y)
          for i in range(n-1)]
    for i in range(n-2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*a_brk*ds[i]))
    for i in range(1, n):
        v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2*a_acc*ds[i-1]))
    return v, ds

v, ds = velocity_profile(path.points)
for i in range(len(v)-1):
    assert v[i+1]**2 <= v[i]**2 + 2*3.0*ds[i] + 1e-6   # traction feasible
    assert v[i]**2 <= v[i+1]**2 + 2*5.0*ds[i] + 1e-6   # braking feasible
assert 0 < min(v) <= max(v) <= 5.0
print(f'4. velocity profile OK: v in [{min(v):.2f}, {max(v):.2f}] m/s, '
      'both passes feasible')

# ------------------------------------- 5. closed-loop pure pursuit test
L, dt = 1.53, 0.02
x, y, yaw, spd = 20.0, 0.0, math.pi/2, 0.0
pts = [(p.x, p.y) for p in path.points]
max_lat_err = 0.0
for step in range(1500):   # 30 s
    d2 = [(px-x)**2 + (py-y)**2 for (px, py) in pts]
    i_near = min(range(len(d2)), key=d2.__getitem__)
    max_lat_err = max(max_lat_err, math.sqrt(d2[i_near]))
    ld = min(max(0.8*spd, 2.0), 8.0)
    target, acc = pts[-1], 0.0
    for i in range(i_near, len(pts)-1):
        acc += math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
        if acc >= ld:
            target = pts[i+1]; break
    dx, dy = target[0]-x, target[1]-y
    tx = dx*math.cos(yaw) + dy*math.sin(yaw)
    ty = -dx*math.sin(yaw) + dy*math.cos(yaw)
    alpha = math.atan2(ty, max(tx, 1e-6))
    steer = max(-0.35, min(0.35, math.atan2(2*L*math.sin(alpha),
                                            max(math.hypot(tx, ty), 1e-3))))
    v_t = v[min(i_near+2, len(v)-1)]
    spd += max(-5.0, min(3.0, (v_t-spd)*2.0)) * dt
    spd = max(0.0, spd)
    yaw = wrap_angle(yaw + spd/L*math.tan(steer)*dt)
    x += spd*math.cos(yaw)*dt
    y += spd*math.sin(yaw)*dt
    # replan window: if car ran past path end, stop test early
    if i_near >= len(pts)-3:
        break

assert max_lat_err < 0.6, f'lateral error {max_lat_err:.2f} m'
assert spd > 1.0, f'car never got moving ({spd:.2f} m/s)'
print(f'5. closed loop OK: followed {i_near+1}/{len(pts)} path pts, '
      f'max lateral error {max_lat_err:.2f} m, speed {spd:.2f} m/s at end')

# -------------------------------------------------- 6. PoseBuffer
pb = PoseBuffer()
for i in range(10):
    pb.add(i*0.1, i*1.0, 0.0, 0.0)
xq, yq, _ = pb.query(0.45)
assert abs(xq - 4.5) < 1e-9
print('6. PoseBuffer interpolation OK')

# ------------------------------- 7. map-relative localization (ICP fit)
# A settled map of the whole track, then a pose that has drifted away from it.
lm_grid = GridIndex()
landmarks = []
for i, (cx, cy, col) in enumerate(track):
    lm = Landmark(cx, cy, 0.01, col, 1, 0.0)
    lm.obs_count = 20
    landmarks.append(lm)
lm_grid.rebuild(landmarks)

def query_targets(qx, qy, r):
    return [(c.x, c.y) for c in lm_grid.query_radius(qx, qy, r)
            if c.obs_count >= 5]

def detections_from(true_pose, fov_r=12.0, fov_a=math.radians(60.0), noise=0.0):
    tx, ty, tyaw = true_pose
    cos_y, sin_y = math.cos(tyaw), math.sin(tyaw)
    out = []
    for (cx, cy, _c) in track:
        dx, dy = cx - tx, cy - ty
        lx = dx * cos_y + dy * sin_y
        ly = -dx * sin_y + dy * cos_y
        if lx <= 0.5 or math.hypot(lx, ly) > fov_r:
            continue
        if abs(math.atan2(ly, lx)) > fov_a:
            continue
        out.append((lx + random.gauss(0.0, noise), ly + random.gauss(0.0, noise)))
    return out

true_pose = (14.0, 8.0, math.radians(140.0))
dets = detections_from(true_pose, noise=0.03)
assert len(dets) >= 8, f'only {len(dets)} detections in FOV'

drift = (0.45, -0.30, math.radians(1.2))
drifted = (true_pose[0] + drift[0], true_pose[1] + drift[1],
           true_pose[2] + drift[2])
fit = solve_pose_correction(drifted, dets, query_targets)
assert fit is not None, 'ICP refused a solvable case'
dx, dy, dyaw, n_in, rms = fit
fixed = (drifted[0] + dx, drifted[1] + dy, wrap_angle(drifted[2] + dyaw))
err_before = math.hypot(drift[0], drift[1])
err_after = math.hypot(fixed[0] - true_pose[0], fixed[1] - true_pose[1])
yaw_err_after = abs(wrap_angle(fixed[2] - true_pose[2]))
assert err_after < 0.10, f'position still {err_after:.3f} m out'
assert yaw_err_after < math.radians(0.5), f'yaw still {math.degrees(yaw_err_after):.2f} deg out'
assert n_in >= 8 and rms < 0.15, (n_in, rms)

# Guards: a shifted-by-one-cone mis-association must be refused, not applied.
huge = (true_pose[0] + 4.0, true_pose[1] + 3.0, true_pose[2])
assert solve_pose_correction(huge, dets, query_targets) is None, \
    'ICP accepted a gross mis-association'
assert solve_pose_correction(drifted, dets[:3], query_targets) is None, \
    'ICP accepted too few pairs'
print(f'7. localization OK: {err_before:.2f} m drift -> {err_after:.3f} m '
      f'({n_in} inliers, rms {rms:.3f} m); gross/underdetermined cases refused')

# --------------------------- 8. slew-limited correction application
est = StateEstimationNode.__new__(StateEstimationNode)
est._x = est._y = est._yaw = 0.0
est._corr_v, est._corr_w = 1.0, 0.3
est._pend = [0.45, -0.30, math.radians(1.2)]
step_dt = 1.0 / 50.0
max_step = 0.0
for _ in range(200):                       # 4 s of ticks
    bx, by = est._x, est._y
    est._apply_correction(step_dt)
    max_step = max(max_step, math.hypot(est._x - bx, est._y - by))
assert max_step <= est._corr_v * step_dt + 1e-9, f'jumped {max_step:.4f} m in one tick'
assert math.hypot(est._x - 0.45, est._y + 0.30) < 1e-6, 'correction not fully applied'
assert abs(est._yaw - math.radians(1.2)) < 1e-9
print(f'8. correction slew OK: applied in full, largest single step '
      f'{max_step*1000:.1f} mm (supervisor jump gate is 1000 mm)')

# ------------------------------------------------- 9. lap counting
laps = LapCounter(min_lap_m=20.0, corridor_m=6.0)
a_ax, b_ax = 20.0, 12.0
dist, prev = 0.0, (a_ax, 0.0)
n_steps = 4000
for i in range(n_steps * 3 + 1):           # three laps, ending ON the line
    t = 2.0 * math.pi * (i % n_steps) / n_steps
    px, py = a_ax * math.cos(t), b_ax * math.sin(t)
    dist += math.hypot(px - prev[0], py - prev[1])
    prev = (px, py)
    tx, ty = -a_ax * math.sin(t), b_ax * math.cos(t)
    laps.update(px, py, math.atan2(ty, tx), dist)
assert laps.laps == 3, f'counted {laps.laps} laps, expected 3'
assert abs(laps.track_length_m - 102.1) < 2.0, laps.track_length_m
print(f'9. lap counting OK: 3/3 laps, measured length '
      f'{laps.track_length_m:.1f} m (ellipse perimeter 102.1 m)')

# --------------------------------- 10. global closed racing line
racer = make_planner(_map=map_from(track), _pose=(20.0, 0.0, math.pi / 2.0),
                     _mode=TrackStatus.MODE_KNOWN)
built = racer._build_global(racer._map.cones)
assert built is not None, 'global loop could not be closed'
race, centre, widths, total = built
assert len(race) == len(centre), (len(race), len(centre))
assert abs(total - 102.1) < 4.0, f'lap length {total:.1f} m'
close_gap = math.hypot(race[0][0] - race[-1][0], race[0][1] - race[-1][1])
assert close_gap < 1.0, f'racing line does not close: {close_gap:.2f} m gap'
race_clear = min_clearance(race, cones_xy)
assert race_clear >= 0.9, f'racing line only {race_clear:.2f} m from a cone'

def closed_profile(samples, v_max=16.0, a_lat=8.0, a_brk=8.0, a_acc=3.0,
                   passes=4):
    n = len(samples)
    v = [min(v_max, math.sqrt(a_lat / max(abs(s[3]), 1e-3))) for s in samples]
    ds = [math.hypot(samples[(i+1) % n][0] - samples[i][0],
                     samples[(i+1) % n][1] - samples[i][1]) for i in range(n)]
    for _ in range(passes):
        for i in range(n - 1, -1, -1):
            v[i] = min(v[i], math.sqrt(v[(i+1) % n]**2 + 2*a_brk*ds[i]))
        for i in range(n):
            v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2*a_acc*ds[i-1]))
    return v, ds

def lap_time(samples):
    v, ds = closed_profile(samples)
    return sum(ds[i] / max(v[i], 0.1) for i in range(len(ds))), v

t_centre, v_centre = lap_time(centre)
t_race, v_race_prof = lap_time(race)
max_k_centre = max(abs(s[3]) for s in centre)
max_k_race = max(abs(s[3]) for s in race)
assert t_race < t_centre, (f'racing line ({t_race:.2f} s) not faster than the '
                           f'centerline ({t_centre:.2f} s)')
assert max_k_race < max_k_centre, (max_k_race, max_k_centre)
# Guard against the Catmull-Rom kink regression: on this track no real
# curvature exceeds 0.139 1/m, so anything near 0.5 means the midpoint chain
# jitter is leaking through as phantom corners the velocity profile brakes for.
assert max_k_centre < 0.25, (f'centerline curvature {max_k_centre:.3f} 1/m — '
                             f'chain smoothing is not doing its job')
print(f'10. racing line OK: closes to {close_gap:.2f} m, {race_clear:.2f} m '
      f'cone clearance, peak |k| {max_k_centre:.3f} -> {max_k_race:.3f} 1/m, '
      f'lap {t_centre:.2f} s -> {t_race:.2f} s '
      f'({100*(t_centre-t_race)/t_centre:.1f}% quicker)')

# Race speed must actually exceed the exploration cap somewhere on the lap,
# otherwise "fast mode" is a label with no effect.
assert max(v_race_prof) > 6.0 + 1.0, max(v_race_prof)
print(f'    profile peak speed {max(v_race_prof):.1f} m/s vs 6.0 m/s '
      f'exploration cap')

# ------------------------------------------- 11. obstacle avoidance
# One object sitting on the centerline, a tenth of a lap from the start.
from fsd_stack.sim_node import centerline_at
obs_x, obs_y = centerline_at(0.10, 0.0)
dodger = make_planner(
    _map=map_from(track, extra=[(obs_x, obs_y, ConeDetection2D.COLOR_UNKNOWN,
                                 ConeMapEntry.SIDE_UNKNOWN)]),
    _pose=(20.0, 0.0, math.pi / 2.0))
obs_path = None
for cycle in range(4):     # flag needs obstacle_confirm_frames consecutive hits
    obs_path = dodger._compute_path()
    assert obs_path is not None, f'planner lost the path on cycle {cycle}'
cap_v, cap_reason, cap_detail = dodger._last_cap
obs_pts = [(p.x, p.y) for p in obs_path.points]
gap = min(math.hypot(px - obs_x, py - obs_y) for (px, py) in obs_pts)
edge_gap = min(math.hypot(px - cx, py - cy)
               for (px, py) in obs_pts for (cx, cy) in cones_xy)
assert dodger._obstacle_ids, 'obstacle never flagged'
assert gap >= dodger._obs_min_clear, f'only {gap:.2f} m from the obstacle'
assert edge_gap >= 0.6, f'avoidance drove within {edge_gap:.2f} m of a boundary cone'
assert cap_reason == SpeedLimit.REASON_OBSTACLE, cap_reason
assert abs(cap_v - 3.0) < 1e-6, cap_v
print(f'11. obstacle avoidance OK: {gap:.2f} m clearance from the object '
      f'(gate {dodger._obs_min_clear:.1f} m), {edge_gap:.2f} m from the '
      f'boundary, speed capped to {cap_v:.1f} m/s')

# 11b. The swerve must stay inside the tyre limit at speed. The blend is a
# TIME, so the extra curvature it injects must not grow with v — check the
# lateral acceleration the deformation itself demands, over and above the
# curvature the track already has.
baseline = make_planner(_map=map_from(track), _pose=(20.0, 0.0, math.pi / 2.0))
base_path = baseline._compute_path()
A_LAT_LIMIT = 8.0

# The deformation path recomputes curvature with a different estimator than
# the spline. Prove the two agree on identical geometry first, or every number
# measured below is really measuring the estimator swap.
base_s = [(p.x, p.y, p.heading, p.curvature) for p in base_path.points]
from fsd_stack.path_planning_node import path_stations, apply_offsets, choose_pass
zero_shift = apply_offsets(
    base_s, path_stations([(s[0], s[1]) for s in base_s]), [], 5.0)
est_gap = max(abs(zero_shift[i][3] - base_s[i][3]) for i in range(len(base_s)))
assert est_gap < 0.01, (f'curvature estimators disagree by {est_gap:.4f} 1/m on '
                        f'identical geometry — phantom corners for the profile')

def swerve_accel(speed):
    p = make_planner(
        _map=map_from(track, extra=[(obs_x, obs_y, ConeDetection2D.COLOR_UNKNOWN,
                                     ConeMapEntry.SIDE_UNKNOWN)]),
        _pose=(20.0, 0.0, math.pi / 2.0), _speed=speed)
    out = None
    for _ in range(4):
        out = p._compute_path()
    assert len(out.points) == len(base_path.points), \
        (len(out.points), len(base_path.points))
    d_k = max(abs(out.points[i].curvature - base_path.points[i].curvature)
              for i in range(len(out.points)))
    return d_k * speed * speed, d_k

a_fast, dk_fast = swerve_accel(11.0)
a_slow, dk_slow = swerve_accel(3.0)
assert a_fast <= A_LAT_LIMIT, (f'swerve at 11 m/s demands {a_fast:.1f} m/s^2, '
                               f'over the {A_LAT_LIMIT:.0f} m/s^2 tyre limit')
assert dk_fast < dk_slow, ('blend did not stretch with speed: '
                           f'dk {dk_fast:.4f} at 11 m/s vs {dk_slow:.4f} at 3 m/s')
print(f'11b. swerve feasibility OK: extra lateral demand {a_fast:.1f} m/s^2 at '
      f'11 m/s and {a_slow:.1f} m/s^2 at 3 m/s (limit {A_LAT_LIMIT:.0f}); '
      f'blend stretches with speed')

# 11c. Pass-side arithmetic, the piece both stacks share verbatim.
a, c = choose_pass(0.0, 0.95, 0.9)          # object dead centre: either side
assert abs(abs(a) - 0.9) < 1e-9 and abs(c - 0.9) < 1e-9, (a, c)
a, c = choose_pass(0.4, 0.95, 0.9)          # object left of the line: go right
assert a < 0 and abs(c - 0.9) < 1e-9, (a, c)
a, c = choose_pass(-0.4, 0.95, 0.9)         # object right of the line: go left
assert a > 0 and abs(c - 0.9) < 1e-9, (a, c)
a, c = choose_pass(0.0, 0.3, 0.9)           # no room: best effort, short gap
assert abs(abs(a) - 0.3) < 1e-9 and abs(c - 0.3) < 1e-9, (a, c)
print('11c. pass-side choice OK: nearer side chosen, clearance met when the '
      'corridor allows, shortfall reported honestly when it does not')

# ------------------------------------------------ 12. blocked corridor
# Two objects 0.8 m either side of the line: no gap wide enough to pass.
bx1, by1 = centerline_at(0.10, 0.40)
bx2, by2 = centerline_at(0.10, -0.40)
blocker = make_planner(
    _map=map_from(track, extra=[
        (bx1, by1, ConeDetection2D.COLOR_UNKNOWN, ConeMapEntry.SIDE_UNKNOWN),
        (bx2, by2, ConeDetection2D.COLOR_UNKNOWN, ConeMapEntry.SIDE_UNKNOWN)]),
    _pose=(20.0, 0.0, math.pi / 2.0))
for cycle in range(8):     # confirm frames for the flags, then for BLOCKED
    blocked_path = blocker._compute_path()
    assert blocked_path is not None, 'planner stopped publishing when blocked'
cap_v, cap_reason, cap_detail = blocker._last_cap
assert cap_reason == SpeedLimit.REASON_BLOCKED, (cap_reason, cap_detail)
assert cap_v == 0.0, cap_v
assert len(blocked_path.points) >= 10, 'path must stay valid so nothing EBSes'
print(f'12. blocked corridor OK: cap {cap_v:.1f} m/s, still publishing '
      f'{len(blocked_path.points)} path points — "{cap_detail}"')

print('\nALL ALGORITHM TESTS PASSED')
