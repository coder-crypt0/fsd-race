# ARCHITECTURE — FSD Autonomous Stack

*How the code implements FSD_System_Interface_Specification.md. The spec
defines WHAT the interfaces are; this document explains HOW and WHY.
Last updated: 2026-07-07.*

## 1. Bird's-eye view

Two parallel implementations of the same eight-block pipeline, sharing one
message package and one topic contract:

```
                 ┌─────────────────────────────────────────────────────┐
                 │                    fsd_msgs                         │
                 │      (the locked contract — 12 message types)       │
                 └───────────────▲──────────────────▲──────────────────┘
                                 │                  │
        ┌────────────────────────┴───┐   ┌──────────┴────────────────────┐
        │        fsd_cpp (C++)       │   │      fsd_stack (Python)       │
        │  PRODUCTION: Jetson + FSDS │   │  REFERENCE: algorithms, sim,  │
        │  CUDA perception, 50 Hz    │   │  dashboard, CAN bridge,       │
        │  control loops             │   │  quick experiments            │
        └────────────────────────────┘   └───────────────────────────────┘
```

Rule: the two stacks stay **behaviorally equivalent per block**. Algorithm
fixes land in both or go through a spec change.

## 2. The pipeline (identical in both stacks)

```
 cameras ──▶ [1+2 PERCEPTION] ──cones (base_link)──▶ [4 MAPPING] ──map──▶ [5 PLANNING]
                                        ▲                  ▲                   │path
 IMU+wheels ──▶ [3 STATE ESTIMATION] ───┴── odom ──────────┴───────┬───────────▼
                                                                   │      [6 CONTROL]
                                                                   │           │cmd @50Hz
             [8 SAFETY SUPERVISOR] ◀── heartbeats from ALL nodes   │           ▼
                      │ebs_trigger                            [7 VEHICLE I/F]
                      └───────────────────────────────────────▶ (STM32 / FSDS adapter)
```

| # | Block | C++ node | Python node | Rate |
|---|---|---|---|---|
| 1+2 | Perception (detection + stereo ranging) | `stereo_cone_node` | `cone_detection` + `cone_localization` | ≥30 Hz |
| 3 | State estimation | `state_estimation_node` | `state_estimation` | 50 Hz |
| 4 | Cone mapping | `cone_mapping_node` | `cone_mapping` | 10 Hz out |
| 5 | Path planning | `path_planning_node` | `path_planning` | 10 Hz |
| 6 | Motion control | `motion_control_node` | `motion_control` | fixed 50 Hz |
| 7 | Vehicle interface | `fsds_adapter_node` (sim) / STM32 firmware (car) | `can_bridge` (car) | 50 Hz |
| 8 | Safety supervisor | `safety_supervisor_node` | `safety_supervisor` | 20 Hz checks |

Coordinate frames: `odom` (world-fixed, zeroed at AS Ready, continuous) and
`base_link` (rear-axle center, x fwd / y left / z up). Perception outputs
base_link; mapping and everything downstream works in odom.

## 3. Perception design (Blocks 1+2)

**GPU/CPU split principle:** the GPU does per-pixel work, the CPU does
per-cone work.

```
left BGR frame                          right BGR frame
   │  cudaMemcpy H→D                       │  cudaMemcpy H→D
   ▼                                       ▼
[CUDA: BGR→HSV→3 masks + gray, one pass]  [CUDA: BGR→gray]   src/cuda/segmentation.cu
   │  masks D→H          └────gray stays on device────┘
   ▼
[CPU: morphology open → contours → bbox filters]        (per-cone, negligible)
   ▼
[CUDA: batched SAD disparity — one block per detection,
 128 threads over disparities, shared-mem two-best
 reduction for the ratio test; only bbox centers up,
 N results down]
   │      └─ ratio test fails → mono pinhole from known cone height (σ×3)
   │      └─ any CUDA failure → identical CPU SAD fallback per bbox
   ▼
Cone3DArray in base_link (camera offset applied)
```

Why not full-frame SGBM: we only need range at ~20 bbox centroids, not a
dense depth map — ROI matching is ~1000× less work. Why not YOLO in the C++
sim path: FSDS cones are color-clean; HSV+contours is sufficient and keeps
the sim loop dependency-free. On the real car, YOLO (TensorRT engine) slots
into the Python `cone_detection` node or a future TensorRT C++ node — the
`/perception/cones` contract doesn't change either way.

CUDA is optional at build time: CMake `check_language(CUDA)` compiles the
kernel when a toolchain exists (JetPack on Orin, `CMAKE_CUDA_ARCHITECTURES`
defaults to 87); otherwise the node builds with `segment_cpu()` (identical
thresholds via cv::inRange). Runtime CUDA errors also fall back per-frame.

## 4. Mapping (Block 4) — the data association core

```
per detection (base_link) ──TF using pose INTERPOLATED at detection stamp──▶ odom
  1. query confirmed map (spatial hash grid, 2 m cells) within SEARCH_RADIUS
     └─ best under GATE → scalar-Kalman position update, vote color/side, done
  2. else query tentative buffer
     └─ hit → update; obs_count ≥ N_CONFIRM(3) → promote with persistent id
     └─ miss → new tentative entry
  3. GC: tentative entries older than 1.5 s with < N_CONFIRM obs are dropped
```

Invariants (violating these = bug, not tuning):
- **Color is never an association gate** — spatial distance only; color
  resolved by majority vote (YOLO/HSV flip yellow↔orange under bad light).
- **Confirmed landmarks are never deleted mid-run** — occlusion ≠ absence.
- C++ implementation is a **slot map**: one append-only landmark vector
  (stable indices — entries never erased, they flip state TENTATIVE →
  CONFIRMED or → DEAD), spatial hash grids with O(1) incremental insert
  and lazy deletion (queries filter by state; stale/duplicate entries are
  harmless), cell-crossing re-insert when a Kalman update moves a landmark
  across a 2 m cell border, and a 10 s compaction timer. This removes the
  index-invalidation hazard class by construction (see MEMORY.md for the
  bug that motivated it).

## 5. Planning (Block 5)

```
cone window (25 m ahead / 5 m behind, vehicle frame)
  → Delaunay triangulation          C++: own Bowyer-Watson  |  Py: scipy.spatial
  → keep edges joining OPPOSITE sides (blue↔yellow; side field when color unknown)
  → length gate 1.5–6.0 m → midpoints → dedupe (<0.3 m)
  → greedy forward chain from nearest midpoint (no reversals >~101°)
  → spline                          C++: Catmull-Rom        |  Py: cubic B-spline
  → resample @0.5 m with analytic heading + curvature
```

Fallback ladder (normative): fresh path → OK; compute fails ≤1 s → republish
last valid + DEGRADED; >1 s → publish **empty** path + ERROR heartbeat, which
the control staleness policy and supervisor turn into a controlled stop.

The C++ Bowyer-Watson is O(n²) incremental with a super-triangle at 2000×
the point-cloud span — verified against the empty-circumcircle definition
and Euler's identity T = 2n−2−h (see `fsd_cpp/test/`).

## 6. Control (Block 6)

Longitudinal plan on every new path (10 Hz):
```
v_i = min(v_max, sqrt(a_lat_max / |κ_i|))       lateral grip limit
backward pass: v_i ≤ sqrt(v_{i+1}² + 2·a_brake·ds)   can we brake in time?
forward pass:  v_i ≤ sqrt(v_{i-1}² + 2·a_accel·ds)   can we reach it?
```
Actuation every 20 ms (fixed timer, never event-driven):
- Pure Pursuit: lookahead L_d = clamp(0.8·v, 2, 8) m; δ = atan(2L·sinα / L_d)
- PI on speed error → torque (≥0) XOR brake (deadband at −0.3 m/s) — never both
- Staleness: path >0.5 s → hold path, decay v_max at 2 m/s² (DEGRADED);
  path >2 s or odom >0.2 s → emergency_stop=true (ERROR)
- Torque is zeroed unless AS state == DRIVING (fed by adapter/bridge)

## 7. Safety architecture — three independent stop paths

```
PATH 1  HARDWARE   RES wireless stop → shutdown circuit. Zero software.
PATH 2  FIRMWARE   STM32: no valid CAN cmd (CRC8+rolling counter) in 100 ms
                   → zero torque; +100 ms → EBS. Catches a dead/insane Jetson.
PATH 3  SOFTWARE   Supervisor: heartbeat watchdog (all nodes ≥5 Hz, 500 ms
                   timeout), NaN/pose-jump/steering-limit/moving-blind checks.
                   Publishes keepalive FALSE at 10 Hz — its own death is
                   detectable downstream. Trigger latches until restart.
```
No two paths share a failure mode. The FSDS adapter honors the same
`/safety/ebs_trigger` latch (full brake) so path 3 is exercised in sim.

## 8. Simulation & test infrastructure

- **FSDS** (`fsd_cpp/launch/fsds.launch.py` + `fsds/settings.json`): stereo
  pair 640×480 @90° HFOV, 12 cm baseline (fx=fy=320 — set from geometry, not
  calibration). Adapter converts VehicleCmd→fs_msgs/ControlCommand
  (throttle=torque/max, steering normalized with a `steering_sign` switch),
  GSS→wheel speeds, GO signal→AS_DRIVING.
- **Kinematic sim** (`fsd_stack sim.launch.py`): elliptical track, bicycle
  model, publishes ideal `/perception/cones` — full closed loop with zero
  external installs. Honors command timeout and EBS like the firmware.
- **Dashboard** (`:8321`): stdlib HTTP + canvas; consumes only contract
  topics, so it works against sim, FSDS, bag replay, and the car unchanged.
- **Bag platform**: `tools/fsd_bag.py` / dashboard REC. Replay stages:
  `raw` (sensors in, full recompute), `cones` (perception out, downstream
  recompute), `all` (verbatim playback).
- **Standalone tests** (no ROS needed): `fsd_stack/test/test_algorithms.py`
  (imports real node modules with rclpy stubbed);
  `fsd_cpp/test/run_tests.sh` (extracts algorithm blocks verbatim from the
  shipped .cpp files, compiles with g++, asserts Delaunay/spline/association
  correctness).

## 9. Threading & performance rules

- One process per block in development; composition is a measured
  optimization, not a default.
- Any node with >1 input uses a MultiThreadedExecutor + per-subscription
  callback groups (Python) / keeps callbacks non-blocking (C++ single
  thread is fine at current rates).
- Images: BEST_EFFORT KeepLast(1). Control/safety: RELIABLE. Never block a
  ROS executor thread on GPU sync.
- Profile before porting or optimizing: `ros2_tracing` + `tegrastats` on the
  Orin. The Python stack exists precisely so hot-loop porting is a decision
  backed by numbers.

## 10. Extension points (designed-in, not speculative)

| Future change | Where it lands | What stays fixed |
|---|---|---|
| ZED2i camera | replace Blocks 1+2 internals | `/perception/cones` contract |
| YOLO/TensorRT in C++ | new detector inside `stereo_cone_node` | same |
| robot_localization EKF | swap Block 3 | `/odometry/filtered` |
| EKF-SLAM / loop closure | inside Block 4 | `/mapping/track` |
| MPC controller | inside Block 6 | `/control/cmd` |
| CAN FD | Block 7 both sides | message semantics |
