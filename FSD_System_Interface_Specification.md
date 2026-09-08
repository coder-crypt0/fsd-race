# FSD System Interface Specification
### Formula Bharat Driverless Cup 2027 — EV → Driverless Transition
**Version:** 1.0 | **Date:** 2026-07-06 | **Status:** Baseline — locked for parallel development
**Target:** Working closed-loop prototype by **Dec 15, 2026**

---

## 0. How to Use This Document

This is the **single source of truth** for inter-block interfaces. Each block has one owner. You may change *anything inside* your block without asking anyone. You may **not** change your block's input/output topics, message types, rates, or frames without a version bump to this document agreed by all three owners.

**Ownership map:**

| Owner | Blocks |
|---|---|
| Perception | 1 — Cone Detection, 2 — Cone Localization |
| Mapping & Planning | 3 — State Estimation, 4 — Cone Mapping, 5 — Path Planning |
| Controls & Actuation | 6 — Motion Planning & Control, 7 — Vehicle Interface (STM32), 8 — Safety Supervisor |

**Platform baseline:** ROS 2 Humble, Ubuntu 22.04.5 LTS, C++ (hot loops) / Python (tooling & prototyping), Jetson Orin Nano (or dev PC), custom STM32 ECU, CAN 2.0B (CAN FD capable).

---

## 1. Full System Interface Diagram

```
                        ┌─────────────────────┐   ┌──────────────────────┐
   SENSORS              │ Left cam (OV5647)   │   │ Right cam (OV5647)   │
                        │ /camera/left/       │   │ /camera/right/       │
                        │   image_raw @30Hz   │   │   image_raw @30Hz    │
                        └─────────┬───────────┘   └──────────┬───────────┘
                                  │                          │
                                  ▼                          │
                    ┌─────────────────────────┐              │
   BLOCK 1          │   CONE DETECTION        │              │
   (Perception)     │   YOLO INT8 TensorRT    │              │
                    │   + HSV fallback        │              │
                    └─────────┬───────────────┘              │
                              │ /perception/cone_detections  │
                              │ ConeDetection2D[] @≥30Hz     │
                              ▼                              ▼
                    ┌────────────────────────────────────────┐
   BLOCK 2          │   CONE LOCALIZATION (STEREO DEPTH)     │
   (Perception)     │   rectify → disparity @ bbox → 3D      │
                    └─────────┬──────────────────────────────┘
                              │ /perception/cones
                              │ Cone3D[] @≥30Hz (base_link)
                              │
   ┌──────────────┐           │
   │ BNO055 IMU   │──┐        │
   │ @100Hz       │  │        │
   ├──────────────┤  ▼        │
   │ Wheel enc.   │ ┌───────────────────┐
   │ @≥50Hz       ├─│ BLOCK 3           │
   ├──────────────┤ │ STATE ESTIMATION  │
   │ GPS A7672S   ├─│ EKF fusion        │
   │ @5-10Hz opt. │ └────────┬──────────┘
   └──────────────┘          │ /odometry/filtered
                             │ nav_msgs/Odometry @≥50Hz
                ┌────────────┼────────────────────────────┐
                │            ▼                            │
                │  ┌──────────────────────┐               │
   BLOCK 4      │  │  CONE MAPPING        │◄── /perception/cones
   (Map & Plan) │  │  data assoc + KF     │               │
                │  │  tentative→confirmed │               │
                │  └────────┬─────────────┘               │
                │           │ /mapping/track              │
                │           │ ConeMap[] @≥10Hz (odom)     │
                │           ▼                             │
                │  ┌──────────────────────┐               │
   BLOCK 5      │  │  PATH PLANNING       │               │
   (Map & Plan) │  │  Delaunay → midpts   │               │
                │  │  → spline            │               │
                │  └────────┬─────────────┘               │
                │           │ /planning/path              │
                │           │ PathPoint[] @≥10Hz          │
                │           ▼                             ▼
                │  ┌────────────────────────────────────────┐
   BLOCK 6      │  │  MOTION PLANNING & CONTROL             │
   (Controls)   └─▶│  velocity profile + Pure Pursuit       │
                   │  + longitudinal PI                     │
                   └────────┬───────────────────────────────┘
                            │ /control/cmd
                            │ VehicleCmd @≥50Hz
                            ▼
                   ┌────────────────────────┐    ┌─────────────────────┐
   BLOCK 7         │  VEHICLE INTERFACE     │    │ BLOCK 8             │
   (STM32 ECU)     │  (STM32 BRIDGE)        │◄───│ SAFETY SUPERVISOR   │
                   │  CAN → Bamocar D3      │    │ heartbeat watchdog  │
                   │  BLDC steering loop    │    │ /safety/ebs_trigger │
                   │  EBS solenoid          │    └─────────▲───────────┘
                   │  cmd-timeout backstop  │              │ /safety/heartbeat
                   └───────┬────────────────┘              │ from ALL nodes @≥5Hz
                           │                               │
              ┌────────────┼──────────────┐     ┌──────────┴──────────┐
              ▼            ▼              ▼     │ HARDWARE LAYER      │
        Bamocar D3    Steering BLDC   EBS pneu. │ RES (wireless stop) │
        (torque CAN)  (pos. loop)     solenoid  │ ASMS, ASSI, SDC     │
                                                │ — bypass ALL SW —   │
                                                └─────────────────────┘
```

**Coordinate frames** (REP-103/REP-105 compliant):
- `base_link` — vehicle body, origin at rear-axle center, x forward, y left, z up
- `odom` — world-fixed, zeroed at AS Ready transition, continuous (no jumps)
- `camera_left` / `camera_right` — static TF from `base_link`, published by `robot_state_publisher` from URDF

---

## 2. Shared Message Package — `fsd_msgs`

Create this package **first**, before any node code. All custom messages live here; nobody defines messages inside their own package.

```
fsd_msgs/
├── msg/
│   ├── ConeDetection2D.msg
│   ├── ConeDetection2DArray.msg
│   ├── Cone3D.msg
│   ├── Cone3DArray.msg
│   ├── ConeMapEntry.msg
│   ├── ConeMap.msg
│   ├── PathPoint.msg
│   ├── PathPointArray.msg
│   ├── VehicleCmd.msg
│   ├── WheelSpeeds.msg
│   ├── Heartbeat.msg
│   └── VehicleStatus.msg
├── CMakeLists.txt
└── package.xml
```

### 2.1 Message Definitions

**ConeDetection2D.msg**
```
uint8 COLOR_BLUE=0
uint8 COLOR_YELLOW=1
uint8 COLOR_ORANGE_SMALL=2
uint8 COLOR_ORANGE_BIG=3
uint8 COLOR_UNKNOWN=255

uint8   color
float32 confidence     # 0.0 - 1.0
float32 cx             # bbox center x, pixels
float32 cy             # bbox center y, pixels
float32 width          # bbox width, pixels
float32 height         # bbox height, pixels
uint8   source         # 0=YOLO, 1=HSV fallback
```

**ConeDetection2DArray.msg**
```
std_msgs/Header header          # stamp = camera frame timestamp, frame_id = "camera_left"
ConeDetection2D[] detections
```

**Cone3D.msg**
```
uint32  id             # per-frame temporary id, NOT persistent
uint8   color          # same enum as ConeDetection2D
float32 confidence
float32 x              # meters, base_link, forward
float32 y              # meters, base_link, left
float32 z              # meters, base_link, up (~0)
float32 depth_sigma    # 1-sigma range uncertainty, meters
```

**Cone3DArray.msg**
```
std_msgs/Header header          # stamp = source image timestamp, frame_id = "base_link"
Cone3D[] cones
```

**ConeMapEntry.msg**
```
uint8 SIDE_UNKNOWN=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

uint32  id                 # persistent global id, never reused
uint8   color              # majority vote result
float32 x                  # meters, odom frame
float32 y                  # meters, odom frame
uint8   side
uint16  observation_count
```

**ConeMap.msg**
```
std_msgs/Header header          # frame_id = "odom"
ConeMapEntry[] cones
```

**PathPoint.msg**
```
float32 x              # meters, odom frame
float32 y              # meters, odom frame
float32 heading        # rad, odom frame
float32 curvature      # 1/m, signed (+left)
float32 track_width    # meters, local corridor width
```

**PathPointArray.msg**
```
std_msgs/Header header          # frame_id = "odom"
PathPoint[] points
```

**VehicleCmd.msg**
```
std_msgs/Header header
float32 steering_angle     # rad at the TIRE, +left / -right
float32 torque_request     # Nm at motor shaft, forwarded to Bamocar
float32 brake_cmd          # 0.0 - 1.0 normalized brake demand
bool    emergency_stop     # true = force immediate EBS, overrides all above
```

**WheelSpeeds.msg**
```
std_msgs/Header header
float32 fl     # rad/s (convert to m/s in state estimation, not here)
float32 fr
float32 rl
float32 rr
```

**Heartbeat.msg**
```
uint8 STATUS_OK=0
uint8 STATUS_DEGRADED=1
uint8 STATUS_ERROR=2

std_msgs/Header header
string node_id
uint8  status
string message          # human-readable, empty when OK
```

**VehicleStatus.msg** (STM32 → Jetson feedback, Block 7 output)
```
std_msgs/Header header
float32 actual_steering_angle   # rad at tire, from BLDC encoder
float32 motor_rpm               # from Bamocar
float32 motor_temp_c
float32 inverter_temp_c
float32 ebs_pressure_bar        # pneumatic line pressure
uint8   as_state                # see AS state machine, Section 9
uint16  fault_flags             # bitfield, see Block 7 spec
```

### 2.2 QoS Profiles (mandatory)

| Topic class | Reliability | History | Depth |
|---|---|---|---|
| Camera images | BEST_EFFORT | KEEP_LAST | 1 |
| Perception/mapping/path outputs | RELIABLE | KEEP_LAST | 5 |
| `/odometry/filtered` | RELIABLE | KEEP_LAST | 10 |
| `/control/cmd` | RELIABLE | KEEP_LAST | 1 |
| `/safety/heartbeat`, `/safety/ebs_trigger` | RELIABLE | KEEP_LAST | 10 |

Rationale: a stale camera frame is worthless (drop it); a lost control or safety message is dangerous (retry it).

---

## 3. BLOCK 1 — CONE DETECTION

**Owner:** Perception | **Platform:** Jetson Orin Nano (GPU) | **Language:** Python OK for v1, C++ if fps drops

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/camera/left/image_raw` | `sensor_msgs/Image` | 30 Hz |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/perception/cone_detections` | `fsd_msgs/ConeDetection2DArray` | ≥30 Hz |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
- Run YOLO (YOLOv8n / YOLO11n, INT8 TensorRT engine) on left image
- Output 2D bboxes with cone color + confidence; publish with the **source image timestamp** (never `now()`)
- Run HSV thresholding in parallel as cross-check; on strong YOLO/HSV disagreement publish `STATUS_DEGRADED` heartbeat
- Publish empty array (not nothing) when zero cones detected — silence means dead node

### INTERNAL PIPELINE (owner's freedom)
```
image → letterbox resize → TensorRT infer → NMS → confidence filter (>0.4)
      → color from class head → publish
parallel: image → HSV mask (blue/yellow/orange) → contour filter → compare counts
```

### ACCEPTANCE CRITERIA
- ≥30 fps sustained on Orin Nano with GPU util visible in `tegrastats`
- ≥90% recall on FSOCO validation split, ≥85% on own track footage at ≤10 m
- End-to-end latency (image stamp → publish) ≤ 40 ms

### TRAINING NOTES
- Base dataset: FSOCO (community FS cone dataset) + own footage; augment for Indian daylight/shadow conditions at Kari
- Export path: PyTorch → ONNX → TensorRT INT8 with calibration set from own footage

---

## 4. BLOCK 2 — CONE LOCALIZATION (STEREO DEPTH)

**Owner:** Perception | **Platform:** Jetson Orin Nano

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/camera/right/image_raw` | `sensor_msgs/Image` | 30 Hz |
| `/perception/cone_detections` | `fsd_msgs/ConeDetection2DArray` | ≥30 Hz |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/perception/cones` | `fsd_msgs/Cone3DArray` | ≥30 Hz |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
- Rectify stereo pair using calibrated intrinsics/extrinsics (calibrate with ≥40 checkerboard poses; reprojection error < 0.5 px)
- Compute disparity **only at bbox regions** (SGBM on ROI), not the full frame — saves ~70% compute
- Triangulate bbox centroid → 3D in camera frame → TF to `base_link`
- **Monocular fallback:** if stereo match fails for a bbox, estimate range from known cone height (325 mm standard / 505 mm big orange) via pinhole model; set `depth_sigma` 3× larger
- Drop detections beyond 12 m or with `depth_sigma` > 1.0 m
- Sync left/right by timestamp (`message_filters::ApproximateTime`, 10 ms slop)

### ACCEPTANCE CRITERIA
- Range error ≤ 5% at 5 m, ≤ 10% at 10 m (validate with tape measure + parked cones)
- Combined Block 1+2 latency ≤ 60 ms

### NOTE ON ZED2i UPGRADE PATH
If/when the ZED2i arrives, Blocks 1+2 collapse: ZED SDK gives rectified images + depth map directly. Keep the `/perception/cones` output contract identical so downstream sees zero difference. Design Block 2 as a thin node so it's cheap to swap.

---

## 5. BLOCK 3 — STATE ESTIMATION

**Owner:** Mapping & Planning | **Platform:** Jetson Orin Nano | **Language:** C++ (use `robot_localization` package as v1)

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` (BNO055) | 100 Hz |
| `/wheel_speeds` | `fsd_msgs/WheelSpeeds` | ≥50 Hz |
| `/gps/fix` (optional) | `sensor_msgs/NavSatFix` | 5–10 Hz |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | ≥50 Hz |
| TF `odom → base_link` | tf2 | ≥50 Hz |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
- EKF fusing: IMU yaw rate + accel (short horizon), wheel-speed odometry (velocity), GPS (optional, low weight, logging insurance)
- Wheel odometry model: rear-axle mean speed + bicycle-model yaw from steering feedback (`/vehicle/status.actual_steering_angle`)
- Re-zero `odom` frame at AS Ready transition
- Publish honest covariance — Block 4's Mahalanobis gate depends on it
- **Do not trust BNO055 heading over a full lap** — it drifts; treat absolute yaw as low-confidence, yaw *rate* as good

### ACCEPTANCE CRITERIA
- Position drift < 2% of distance traveled over one 200 m lap (validate against tape-measured straight)
- Velocity estimate within ±0.2 m/s of encoder ground truth at constant speed
- No discontinuities > 5 cm between consecutive poses

### V1 SHORTCUT
Use `robot_localization`'s `ekf_node` with a YAML config before writing a custom EKF. Custom filter only if it demonstrably underperforms.

---

## 6. BLOCK 4 — CONE MAPPING

**Owner:** Mapping & Planning | **Platform:** Jetson Orin Nano | **Language:** C++

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/perception/cones` | `fsd_msgs/Cone3DArray` | ≥30 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | ≥50 Hz |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/mapping/track` | `fsd_msgs/ConeMap` | ≥10 Hz |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
- Transform detections `base_link → odom` using pose **interpolated to the detection timestamp** (tf2 buffer lookup, not latest pose)
- Data association per the algorithm below; maintain tentative → confirmed lifecycle
- Kalman-update confirmed landmark positions on re-detection
- Resolve color by majority vote across observations
- Classify left/right side using vehicle heading history relative to the landmark
- Publish only **confirmed** cones

### DATA ASSOCIATION ALGORITHM (normative)

Internal state:
```
confirmed_map : list<ConeMapEntry + covariance + color_votes + last_seen>   # published
tentative     : list<{x, y, cov, color_votes[], observation_count, first_seen}>  # private
```

Per incoming `Cone3DArray`:
```
for each detection d:
    d_odom = TF(d, base_link → odom, at d.header.stamp)

    # 1. Match against confirmed map
    candidates = confirmed_map.query_radius(d_odom, SEARCH_RADIUS)
    best = argmin over candidates of mahalanobis(d_odom, c.pos, c.cov + d.cov)
           subject to distance < GATE_THRESHOLD
    if best exists:
        best.pos, best.cov = kalman_update(best, d_odom)
        best.observation_count += 1
        best.color_votes.push(d.color)
        best.last_seen = now()
        continue

    # 2. Match against tentative buffer
    t = tentative.query_radius(d_odom, SEARCH_RADIUS)
    if t exists:
        t.pos = fuse(t.pos, d_odom); t.observation_count += 1
        t.color_votes.push(d.color)
        if t.observation_count >= N_CONFIRM:
            promote(t → confirmed_map, new persistent id)
    else:
        tentative.add(d_odom, count=1, first_seen=now())

# 3. Garbage-collect tentative only
for t in tentative:
    if now() - t.first_seen > TENTATIVE_TIMEOUT and t.observation_count < N_CONFIRM:
        discard(t)
```

Parameters (initial values — tune on track data):

| Param | Value | Note |
|---|---|---|
| `SEARCH_RADIUS` | 1.0–1.5 m | cone spacing ≥3 m gives margin |
| `GATE_THRESHOLD` | Euclidean 0.5 m during bring-up → Mahalanobis 3σ once EKF covariance trusted | |
| `N_CONFIRM` | 3 (lower to 2 if real cones appear too slowly) | |
| `TENTATIVE_TIMEOUT` | 1.5 s | ~45 confirmation chances at 30 Hz |

**Rules that must not be violated:**
1. **Never hard-gate on color** — spatial distance is the sole association gate; color is resolved by vote
2. **Confirmed cones are never deleted mid-run** — occlusion is not disappearance
3. Use a spatial index (uniform grid or k-d tree rebuilt at 10 Hz) — O(n²) scans will not survive a 200-cone track at 30 Hz

### ACCEPTANCE CRITERIA
- Zero duplicate landmarks for the same physical cone across a full recorded lap (bag replay test)
- ≤1 false landmark per lap from noise
- Map update latency ≤ 20 ms per frame at 150 mapped cones

---

## 7. BLOCK 5 — PATH PLANNING

**Owner:** Mapping & Planning | **Platform:** Jetson Orin Nano | **Language:** C++ (CGAL or custom Delaunay via `delaunator-cpp`)

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/mapping/track` | `fsd_msgs/ConeMap` | ≥10 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | ≥50 Hz (for local-window selection) |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/planning/path` | `fsd_msgs/PathPointArray` | ≥10 Hz |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
- Select cones within a forward window (~25 m ahead, 5 m behind current pose)
- Delaunay triangulation over selected cones
- Keep edges connecting **opposite-side** cones (blue↔yellow, or side field when color unknown); reject edges > 6 m or < 1.5 m
- Midpoints of kept edges → centerline candidates → order by projection along heading → fit cubic spline
- Sample spline at 0.5 m spacing; compute heading, curvature, local track width per point
- Publish minimum 10 m of path; if track ambiguity (missing cones), extend straight along last heading at reduced-confidence and flag `STATUS_DEGRADED`

### FALLBACK BEHAVIOR (normative)
| Condition | Action |
|---|---|
| < 2 cone pairs visible | Publish last valid path, heartbeat DEGRADED |
| No valid path for > 1.0 s | Publish empty path, heartbeat ERROR → supervisor decides (Block 8) |

### ACCEPTANCE CRITERIA
- Centerline stays ≥ 0.75 m from every cone on recorded reference lap
- Curvature profile continuous (no steps > 0.1 m⁻¹ between adjacent points)
- Compute time ≤ 15 ms per cycle at 150 cones

---

## 8. BLOCK 6 — MOTION PLANNING & CONTROL

**Owner:** Controls & Actuation | **Platform:** Jetson Orin Nano | **Language:** C++ (hard requirement — this is the hot loop)

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/planning/path` | `fsd_msgs/PathPointArray` | ≥10 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | ≥50 Hz |
| `/vehicle/status` | `fsd_msgs/VehicleStatus` | ≥50 Hz |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/control/cmd` | `fsd_msgs/VehicleCmd` | **fixed 50 Hz timer**, never event-driven |
| `/safety/heartbeat` | `fsd_msgs/Heartbeat` | ≥5 Hz |

### RESPONSIBILITIES
**Velocity profile (longitudinal plan):**
```
v_curve(s)  = sqrt(a_lat_max / |κ(s)|)          # lateral accel limit
v_target(s) = min(v_curve, v_max_mission)
then backward pass:  v[i] = min(v[i], sqrt(v[i+1]² + 2·a_brake_max·ds))   # braking feasibility
then forward pass:   v[i] = min(v[i], sqrt(v[i-1]² + 2·a_accel_max·ds))   # traction feasibility
```
Initial limits: `a_lat_max = 4 m/s²`, `a_brake_max = 5 m/s²`, `a_accel_max = 3 m/s²`, `v_max_mission = 5 m/s` for first runs. Raise only after clean laps.

**Lateral control — Pure Pursuit (v1):**
```
lookahead L_d = clamp(k_v · v, L_min, L_max)     # k_v ≈ 0.5–1.0 s, L_min 2 m, L_max 8 m
steering_angle = atan2(2 · wheelbase · sin(α), L_d)
```
Stanley as alternate if Pure Pursuit corner-cuts badly; MPC is a **post-December stretch goal**, not v1.

**Longitudinal control:** PI on speed error → `torque_request` (positive) or `brake_cmd` (negative demand). Never command torque and brake simultaneously; enforce a deadband.

**Staleness policy (normative):**
| Condition | Action |
|---|---|
| `/planning/path` age > 500 ms | Continue on held path, decay `v_target` at 2 m/s², heartbeat DEGRADED |
| `/planning/path` age > 2 s OR `/odometry/filtered` age > 200 ms | `emergency_stop = true`, heartbeat ERROR |
| Startup / any reset | Output zero torque, zero steering rate demand until first valid path + odom |

### ACCEPTANCE CRITERIA
- 50 Hz output jitter < 5 ms (measure with `ros2 topic hz`)
- Lateral error ≤ 0.5 m RMS at 5 m/s on recorded reference lap (simulation first, then track)
- Simulation validation in EUFS sim or simple kinematic Python sim **before** any hardware run

---

## 9. BLOCK 7 — VEHICLE INTERFACE (STM32 BRIDGE)

**Owner:** Controls & Actuation | **Platform:** custom STM32 ECU | **Language:** C (bare-metal or FreeRTOS)

### INPUT
| Channel | Content | Rate |
|---|---|---|
| CAN from Jetson | `VehicleCmd` serialized (see CAN map below) | ≥50 Hz |
| Steering BLDC encoder | position feedback | 1 kHz loop |
| Wheel speed sensors | pulse counting | continuous |
| SDC / ASMS / RES lines | discrete inputs | interrupt + polled |

### OUTPUT
| Channel | Content | Rate |
|---|---|---|
| CAN to Bamocar D3 | torque request (Bamocar register 0x90 protocol) | 50–100 Hz |
| Steering BLDC driver | position setpoint, closed loop on ECU | 1 kHz |
| EBS solenoid driver | valve state | event + 100 Hz refresh |
| CAN to Jetson | `VehicleStatus` + `WheelSpeeds` | ≥50 Hz |
| ASSI lights, brake light | per AS state | continuous |

### CAN MESSAGE MAP (Jetson ↔ STM32, CAN 2.0B, 500 kbps)
| CAN ID | Direction | Content | DLC | Rate |
|---|---|---|---|---|
| 0x100 | Jetson→STM | steering_angle (i16, mrad), torque (i16, 0.1 Nm), brake (u8, 0–200=0–1.0), flags (u8: bit0=e-stop) | 6 | 50 Hz |
| 0x101 | Jetson→STM | rolling counter (u8) + CRC8 of 0x100 payload | 2 | 50 Hz |
| 0x200 | STM→Jetson | actual steering (i16 mrad), motor rpm (i16), as_state (u8), fault_flags (u16) | 7 | 50 Hz |
| 0x201 | STM→Jetson | wheel speeds fl/fr/rl/rr (4× u16, 0.01 rad/s) | 8 | 50 Hz |
| 0x210 | STM→Jetson | ebs_pressure (u16, 0.01 bar), motor_temp (i8), inverter_temp (i8), lv_voltage (u16, 0.01 V) | 6 | 10 Hz |

### AS STATE MACHINE (STM32 is the authority)
```
AS_OFF ──ASMS on + SDC closed + mission selected──▶ AS_READY
AS_READY ──RES "go" signal──▶ AS_DRIVING
AS_DRIVING ──mission complete + standstill──▶ AS_FINISHED
AS_DRIVING ──any EBS trigger──▶ AS_EMERGENCY
AS_READY/AS_FINISHED ──fault──▶ AS_EMERGENCY
AS_EMERGENCY: EBS fired, latched until manual reset, ASSI blue flash + buzzer
```
ASSI: yellow continuous = AS_READY, yellow flash = AS_DRIVING, blue = AS_EMERGENCY/AS_FINISHED per rulebook. **Re-verify exact patterns against the FB2027 Driverless document when it publishes.**

### RESPONSIBILITIES
- Translate 0x100 into Bamocar torque frames; clamp torque/steering to configured limits **in the ECU** (never trust the Jetson's numbers blindly)
- Close 1 kHz steering position loop; rate-limit steering slew (start: 90°/s at tire)
- Validate rolling counter + CRC on every 0x100/0x101 pair; two consecutive failures = treat as timeout
- **Command timeout:** no valid 0x100 within **100 ms** → zero torque, hold steering, engage EBS after further 100 ms
- Own the AS state machine; refuse torque unless AS_DRIVING
- Fire EBS on: e-stop flag, timeout, SDC open, RES stop, `/safety/ebs_trigger` relayed flag, LV undervoltage

### ACCEPTANCE CRITERIA (bench, before vehicle)
- Steering step response: 10° tire step settles < 200 ms, overshoot < 10%, on a loaded bench rig
- EBS: solenoid fire → pressure at caliper < 100 ms on bench; full vehicle reaction (SDC open → deceleration onset) < 200 ms; avg decel > 10 m/s² (typical FS requirement — confirm FB2027 numbers)
- Pull-the-cable test: yank Jetson CAN mid-command → safe state within 200 ms, every time, 20/20 trials

---

## 10. BLOCK 8 — SAFETY SUPERVISOR

**Owner:** Controls & Actuation | **Platform:** Jetson (software layer) — STM32 timeout is the independent hardware backstop

### INPUT
| Topic | Type | Rate |
|---|---|---|
| `/safety/heartbeat` (all nodes) | `fsd_msgs/Heartbeat` | ≥5 Hz per node |
| `/odometry/filtered` | `nav_msgs/Odometry` | ≥50 Hz |
| `/perception/cones`, `/mapping/track`, `/planning/path`, `/control/cmd` | (monitored for plausibility) | passive |

### OUTPUT
| Topic | Type | Rate |
|---|---|---|
| `/safety/ebs_trigger` | `std_msgs/Bool` | event + 10 Hz keepalive (`false` = healthy) |
| `/safety/system_state` | `fsd_msgs/Heartbeat` | 10 Hz (aggregate status for logging/ASSI) |

### RESPONSIBILITIES
- Registry of required nodes: `cone_detection`, `cone_localization`, `state_estimation`, `cone_mapping`, `path_planning`, `motion_control`. Missing heartbeat > **500 ms** from any → EBS
- Any node reporting `STATUS_ERROR` → EBS
- Plausibility checks:
  - NaN/inf in pose, velocity, or commands → EBS
  - Speed > 0 for > 3 s with zero confirmed cones in map → EBS (car has left the track or perception is blind)
  - `|steering_angle|` command > physical max → EBS
  - Position jump > 1 m between consecutive odom updates → EBS
- The keepalive design is deliberate: STM32 also watches for `/safety/ebs_trigger` keepalive presence via the relay flag — **a dead supervisor is itself a trigger condition**

### THE THREE INDEPENDENT STOP PATHS (all must exist, none may share a failure mode)
1. **Hardware:** RES wireless stop + SDC — opens shutdown circuit directly, zero software involved
2. **Firmware:** STM32 command-timeout + CRC/counter validation — catches dead/insane Jetson
3. **Software:** this supervisor — catches live-but-wrong software states

---

## 11. Safety Requirements Summary (sign-off checklist)

| # | Requirement | Verified by | Status |
|---|---|---|---|
| S1 | RES opens SDC with no software in the path | wiring inspection + live test | ☐ |
| S2 | EBS reaction < 200 ms, avg decel > 10 m/s² (confirm FB2027 numbers when published) | instrumented vehicle test | ☐ |
| S3 | STM32 cmd timeout 100 ms → safe state | bench pull-the-cable, 20/20 | ☐ |
| S4 | Every ROS node publishes heartbeat ≥5 Hz | supervisor log audit | ☐ |
| S5 | Supervisor death itself triggers EBS | kill -9 test | ☐ |
| S6 | All nodes default to no-motion on startup/restart | restart-under-load test | ☐ |
| S7 | Torque refused outside AS_DRIVING | state machine bench test | ☐ |
| S8 | EBS energy source passive (pneumatic reservoir holds pressure with LV dead) | LV-kill test | ☐ |
| S9 | CRC + rolling counter on control CAN frames | fault-injection test | ☐ |
| S10 | Steering slew rate limited in ECU | bench measurement | ☐ |

**Testing discipline (non-negotiable order):**
1. Bench: each actuator isolated (steering loop, EBS solenoid, Bamocar torque via CAN)
2. Vehicle on stands: full chain, wheels off ground
3. Walking-pace closed loop, open area, human with RES within range, trained spotters
4. Speed increases only after 3 consecutive clean runs at the previous speed
5. Every dynamic test: RES check + EBS check **before** the run, logged

---

## 12. Development & Integration Plan

| Month | Perception | Mapping & Planning | Controls & Actuation | Joint |
|---|---|---|---|---|
| **Jul** | Camera driver, calibration rig, FSOCO training starts | `robot_localization` config, sim env setup | STM32 CAN skeleton, Bamocar comms on bench | `fsd_msgs` locked, measure wheelbase/track/steering ratio, bag-record rig |
| **Aug** | Blocks 1+2 MVP on recorded bags | Block 4 on synthetic + bag data | Steering bench rig, EBS solenoid bench, Block 7 timeout logic | First full-chain bag replay |
| **Sep** | Latency optimization, TensorRT INT8 | Block 5 working on bags, Block 3 tuned on rolling-cart data | Block 6 in simulation, Block 8 framework, ASSI/ASMS/RES wired | Static full-stack on vehicle (stands) |
| **Oct** | Live on-vehicle validation | Live map building at walking pace | First closed-loop walking-pace runs, EBS vehicle timing test | S1–S10 checklist pass |
| **Nov** | Robustness (lighting, occlusion) | Loop-closure investigation (stretch) | Speed buildup 3→8 m/s, tuning | Repeated full-mission runs, failure-injection days |
| **Dec 1–15** | freeze | freeze | freeze | Validation, documentation, buffer |

**Rule: bag files are the integration currency.** Every block must run against recorded bags before touching the vehicle. Record everything, always (`ros2 bag record -a` on every power-up).

---

## 13. Known Gaps & Open Items

| Item | Blocks affected | Action | Deadline |
|---|---|---|---|
| Wheelbase / track width unknown | 3, 5, 6 | measure the car | this week |
| Steering ratio + max angle unknown | 6, 7 | measure + document | this week |
| LV power budget unknown | all HW | electrical team audit | July |
| FB2027 Driverless rulebook not yet published | 7, 8, 9, safety numbers | watch formulabharat.com/downloads; re-verify S2, ASSI patterns, RES spec | on release |
| FB2027 registration status | everything | confirm Driverless Cup entry with organizers | **immediately** |
| ZED2i procurement | 1, 2 | budget decision; contract already ZED-proof | Sept latest |
| Loop closure / pose drift over laps | 4 | v2 item, not for Dec 15 | post-Dec |
| MPC controller | 6 | stretch goal after Pure Pursuit works | post-Dec |

---

## 14. Threading & Compute Guidelines (all owners)

- **One process per block** during development (debuggability) → composable nodes with intra-process comms as an optimization later, only if profiling demands it
- Use `MultiThreadedExecutor` + separate `CallbackGroup` per subscription in any node with >1 input — the default single-threaded executor will silently serialize your camera callback behind your timer
- Blocks 1–2: GPU work in CUDA streams; never block the ROS executor thread on `cudaDeviceSynchronize`
- Block 6's 50 Hz timer gets its own `MutuallyExclusiveCallbackGroup` so path updates can never delay a control tick
- Pin big-message topics (images) to BEST_EFFORT + `KEEP_LAST(1)`; consider `image_transport` compressed only for logging, never in the perception path
- Profile with `ros2_tracing` + `tegrastats` before optimizing anything

---

*Document owner: all three sub-team leads jointly. Any interface change requires a version bump and sign-off from every owner whose block touches the changed topic.*
