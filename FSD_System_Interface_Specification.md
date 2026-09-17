# System specifications and interfaces

**Revision 2 · 16 September 2026 · implemented FSDS monocular profile**

Supersedes the July design baseline. Historical hardware choices, deadlines and
“locked/production” claims are not current verification evidence. Source of truth:
[settings.json](fsd_ws/fsds/settings.json),
[fsds_params.yaml](fsd_ws/src/fsd_cpp/config/fsds_params.yaml),
[launch](fsd_ws/src/fsd_cpp/launch/fsds.launch.py), and
[message schemas](fsd_ws/src/fsd_msgs/msg).

## 1. Scope and requirements

| Requirement | Current implementation / evidence |
|---|---|
| Exactly one forward RGB camera | Configured and verified in the live ROS graph |
| No LiDAR | No configured LiDAR, point-cloud topics or autonomy point-cloud subscriptions |
| No stereo dependence | Mono-only mode; right-image subscriber is not created |
| No preloaded track / reference-pose driving | Online cone map; reference consumers restricted to diagnostics |
| Start without intervention | FSDS auto-GO enabled; no operator GO click |
| Complete the first unseen lap | Demonstrated twice; repeatability remains incomplete |
| Fast, optimal first lap | Not demonstrated; local first-lap ceiling 2.5 m/s |
| General obstacle avoidance | Not implemented; cone-like objects/blockage logic only |
| Jetson deployment | Target design, not verified hardware support |
| Formula Bharat qualification | Not established by these software tests |

An exact competition edition, official rulebook revision, permitted IMU/encoder
interfaces, scoring criteria and submission requirements still need a
rule-to-evidence review. This document does not assert competition eligibility.

## 2. Sensor and calibration specification

| Property | FSDS value |
|---|---|
| Vehicle / camera names | `FSCar` / `cam_left` (one forward camera) |
| Image type | Scene RGB, FSDS ImageType 0; processed through BGR/OpenCV |
| Resolution / horizontal FOV | 424 × 320 / 70° |
| Mount x, y, z | +1.0 m forward, 0 m lateral, +0.8 m up |
| Mount pitch, roll, yaw | 0°, 0°, 0° |
| Intrinsics | fx = fy = 302.8 px, cx = 212 px, cy = 160 px |
| Effective ground height / horizon | 0.8 m / row 160 |
| Depth acceptance | >0.5 m and ≤18 m, estimated sigma ≤4 m |
| Planner local depth sigma | ≤1.5 m |
| Image subscriber QoS | Best effort, KeepLast(1) |
| IMU | Enabled; angular velocity plus valid orientation |
| Wheel signal | FSDS rotation-angle increments, front-wheel estimate |
| GSS | Enabled in settings, not consumed by autonomy |
| LiDAR / second camera / depth camera | Not configured |

The mount follows FSDS Z-up conventions; negative Z previously placed the
camera underground. These values are simulator calibration, not physical Pi
camera measurements. A real mount above the seat requires measured height,
pitch, vibration, field-of-view, distortion and body-occlusion validation.
References: [FSDS frames](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.0.1/coordinate-frames/),
[FSDS cameras](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/camera/).

HSV uses OpenCV H = 0–179, S/V = 0–255:

| Class | Hue | Minimum saturation | Minimum value |
|---|---:|---:|---:|
| Blue | 100–130 | 85 | 100 |
| Yellow | 20–38 | 25 | 90 |
| Orange | 5–18 | 80 | 80 |

Other filters: 3 px closing kernel, minimum final colored area 5 px, aspect ratio
0.55–3.5, ground-inferred height 0.12–0.65 m and width 0.06–0.65 m. These tuned
gates are not validated across arbitrary lighting/camera conditions.
CPU/CUDA threshold constants are shared in `hsv_thresholds.hpp`.

## 3. Frames, units and timestamps

- `base_link`: local vehicle convention, x forward, y left, z up. Simulator
  origin is the runtime reference; a physical rear-axle/CG transform is not yet calibrated.
- `odom`: startup-relative planar frame, x/y zero and yaw relative to initial IMU
  heading. It drifts; it is neither GPS nor an absolute corrected track frame.
- Pixel coordinates: x right, y down. Camera-axis conversion to vehicle axes is
  performed in perception using configured extrinsics.
- Linear quantities are metres, seconds, m/s; angular quantities radians/rad/s.
- Perception arrays retain source image stamps. Mapping/planning query a
  300-entry interpolated pose buffer; queries outside it clamp to an endpoint,
  so this is not full timestamp-consistency enforcement.
- Paths carry generation stamps; controller checks their stamp separately
  from publisher reception freshness.
- Wheel angular speed is derived in sensor timestamp time; runtime timers use
  wall-clock scheduling. `use_sim_time` is not enabled by the default launch.
  Physics-time and wall-time velocities are not interchangeable under load.

No real camera–IMU synchronization accuracy or end-to-end latency percentile has
been measured.

## 4. ROS interface inventory

Types without a namespace below are from `fsd_msgs/msg`.
Rates are requested publication rates unless explicitly described as observed.

| Topic | Type | Producer → consumers | Rate / QoS |
|---|---|---|---|
| `/fsds/cam_left/image_color` | sensor_msgs/Image | Camera bridge → perception | Camera-driven; subscriber best effort, depth 1 |
| `/imu` | sensor_msgs/Imu | FSDS bridge → estimator | Source-driven; estimator reliable, depth 10 |
| `/wheel_states` | fs_msgs/WheelStates | Bridge → adapter | Source-driven; subscriber best effort, depth 5 |
| `/wheel_speeds` | WheelSpeeds | Adapter → estimator | Accepted encoder samples; reliable, depth 10 |
| `/perception/cone_detections` | ConeDetection2DArray | Perception → diagnostics | Image-driven; reliable, depth 5 |
| `/perception/cones` | Cone3DArray | Perception → mapper/planner | Image-driven; reliable, depth 5 |
| `/odometry/filtered` | nav_msgs/Odometry | Estimator → mapper/planner/controller/safety/dashboard | 50 Hz; reliable, depth 10 |
| `/mapping/track` | ConeMap | Mapper → planner/safety/dashboard | 10 Hz; reliable, depth 5 |
| `/mapping/status` | TrackStatus | Mapper → planner/evaluator | 10 Hz; reliable, depth 5 |
| `/localization/correction` | PoseCorrection | Optional mapper → estimator | Disabled in profile; reliable |
| `/planning/path` | PathPointArray | Planner → controller/dashboard | 10 Hz; reliable, depth 5 |
| `/planning/speed_limit` | SpeedLimit | Planner → controller | 10 Hz; reliable, depth 5 |
| `/control/cmd` | VehicleCmd | Controller → adapter/safety/dashboard | 50 Hz; reliable, depth 1 |
| `/control_command` | fs_msgs/ControlCommand | Adapter → FSDS bridge | On control callback; reliable, depth 1 |
| `/vehicle/status` | VehicleStatus | Adapter → controller/dashboard | 50 Hz; reliable, depth 10 |
| `/safety/heartbeat` | Heartbeat | C++ nodes → supervisor/dashboard | 10 Hz per emitter; reliable, depth 10 |
| `/safety/ebs_trigger` | std_msgs/Bool | Supervisor → adapter/dashboard | 10 Hz plus trigger; reliable, depth 10 |
| `/signal/go`, `/signal/finished` | fs_msgs/GoSignal, FinishedSignal | Simulator → adapter | Event-driven |
| `/testing_only/*` | External bridge types | Simulator → diagnostic tools only | Not autonomy inputs |

Durability is default volatile; this is not a latched DDS transient-local safety
channel. Source bridge QoS must be compatible with these subscribers.

### Message semantics

| Contract | Important fields and interpretation |
|---|---|
| ConeDetection2D | color, bbox center/size pixels, confidence, source; current source HSV, confidence fixed 0.8 |
| Cone3D | Per-frame temporary id, color, x/y/z metres, depth_sigma; not a persistent landmark id |
| ConeMapEntry | Persistent id, voted color, planar odom x/y, side, observation_count |
| PathPoint | odom x/y, heading, signed curvature, track_width |
| SpeedLimit | v_max_mps and reason: EXPLORE=0, RACE=1, OBSTACLE=2, BLOCKED=3 |
| TrackStatus | mode EXPLORE=0 / KNOWN=1, inferred laps, crossing flag, lap distance, track length, map/localization counters |
| VehicleCmd | Tire-angle convention +left, torque request, normalized brake [0,1], emergency_stop |
| VehicleStatus | AS state and feedback fields; FSDS echoes steering command, other hardware fields are not measurements |
| WheelSpeeds | fl/fr/rl/rr in rad/s; radius conversion belongs to estimator |
| Heartbeat | node_id, OK=0 / DEGRADED=1 / ERROR=2, explanatory message |
| PoseCorrection | dx/dy/dyaw, inliers, RMS, valid; unused in the current profile |

ROS cone colors: BLUE=0, YELLOW=1, ORANGE_SMALL=2, ORANGE_BIG=3, UNKNOWN=255.
Map sides: UNKNOWN=0, LEFT=1, RIGHT=2. The direct FSDS referee RPC uses a
different color enum; evaluation explicitly translates it.

Vehicle AS values: OFF=0, READY=1, DRIVING=2, FINISHED=3, EMERGENCY=4.
Auto-GO bypasses waiting for GO in this simulation profile. EBS remains latched
until restart. No one-lap automatic finish mission is implemented: runs can
continue into a following lap until stopped, evaluation expires or a finish
signal is received.

## 5. Main control and mapping parameters

| Group | Current FSDS values |
|---|---|
| Estimator | 50 Hz, radius 0.18 m, front wheels + IMU orientation, map correction off |
| Mapper | 3 sightings to confirm; 1.5 s tentative timeout; gate 0.5–1.5 m, sigma scale 3; publish 10 Hz |
| Lap knowledge | Minimum distance 20 m; start corridor 6 m; KNOWN after one estimated crossing |
| Local planner | 25 m ahead / 5 m behind; 0.5 m samples; default width 3.5 m |
| Local observation cache | Fresh observation window 0.35 s; cache 3 s; merge radius 0.75 m |
| Racing line | Enabled experimentally; 60 m ahead window; 1 m margin; 400 optimization iterations |
| Speed ceilings | First lap 2.5 m/s; valid known-track path 10 m/s; controller absolute 12 m/s |
| Controller | Wheelbase 1.55 m; ±0.35 rad; lookahead 0.8 × speed, clamped 2–8 m |
| Longitudinal | Kp 2.0, Ki 0.8; torque scaling 30 Nm; cap ramp 2 m/s² |
| Dynamics assumptions | Lateral/braking 6 m/s²; acceleration 3 m/s² |
| Missing speed-limit message | After 1 s, fallback ceiling 5 m/s; path/odometry checks still apply |
| Adapter | Steering sign −1; normalized torque/steering/braking; auto_go true |

Values are tuning assumptions, not verified tire/motor limits. The 5 m/s
missing-cap fallback can exceed the first-lap cap and needs a safety review.
Inactive stereo baseline/disparity fields in YAML are not evidence of stereo use.
Full timeout behavior and actuator-watchdog gaps: [architecture §8](ARCHITECTURE.md#8-safety-behavior-and-gaps).

## 6. Platform and hardware requirements

Verified software platform: x86-64 Ubuntu 22.04.5 in Docker, ROS 2 Humble,
Python 3.10.12, OpenCV 4.5.4, C++17, CPU perception; Windows FSDS rendering.

Intended embedded platform: NVIDIA Jetson Orin Nano with one Pi camera, IMU,
wheel encoders and an independently validated vehicle interface. The exact
Jetson SKU/RAM, Pi sensor model, lens, CSI carrier/driver, steering ratio,
motor/inverter, brake actuator, battery, tire properties and physical vehicle
geometry have **not** been measured/confirmed for this release. Old references
to specific motors, BNO055 or other BOM items are design history, not a locked
procurement specification.

[Jetson deployment and compatibility](docs/jetson-deployment.md) defines the
remaining integration work. The Windows simulator is not intended to execute
on the Jetson.

## 7. Evidence and change control

Change this document when sensor policy, frames, topic semantics or defaults
change. Re-run built-node tests and fresh FSDS trials after behavior changes.
No documentation statement should turn a parameter, synthetic test, marketing
specification or single successful lap into a safety/performance guarantee.

See [validation](docs/validation.md), [performance](docs/performance.md) and
[release gates](docs/roadmap.md).
