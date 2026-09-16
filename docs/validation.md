# Validation record

## Camera mount correction

FSDS uses a Z-up sensor mount convention. The original `Z: -0.8` camera
configuration placed the camera below the track surface. Captured RGB images
showed cones above the horizon against empty background. After changing the
mount to `Z: 0.8`, a stationary capture shows asphalt below the horizon and the
starting corridor with blue, yellow, and orange cones.

The default camera remains one forward-facing 424 x 320 stream, 70 degree
horizontal field of view. Intrinsics are fx = fy = 302.8, cx = 212, cy = 160.
The horizon filter uses the bounding-box bottom. Previous driving observations
with the underground camera are invalid evidence for perception quality.

Bounded local capture (run inside the ROS environment):

```bash
python3 tools/capture_diagnostics.py --output artifacts/capture --seconds 10
```

This saves at most ten PNGs and associated telemetry. It publishes no messages.
Artifacts are ignored by Git. Simulator reference odometry, when available, is
diagnostic data only and is not a planner input.

Reference: [FSDS coordinate frames](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.0.1/coordinate-frames/).

## Acceptance still required

Race-speed behavior, other tracks, dynamic-obstacle trials, sloped ground and
Jetson hardware benchmarks remain unverified. Passing helper algorithm tests
alone does not demonstrate end-to-end reliability. Current lap evidence follows.

## Recovery and odometry regression checks

The runtime regression script launches the actual compiled C++ nodes in ROS
domain 91, separated from the simulator. Verified cases:

- Blue-left and yellow-right recovery remain correct after crossing a boundary.
- A live planner publishing no corridor requests braking without latching EBS.
- A new valid corridor resumes propulsion automatically.
- An unresponsive planner still triggers emergency braking.
- Front-wheel odometry excludes driven rear-wheel spin; available IMU orientation
  determines heading relative to startup without integrating the same rate twice.

```bash
python3 tools/test_runtime.py
```

The C++ build passed with these changes. The preceding 90-second FSDS evaluation
recorded 12 down-or-out cones and no complete lap despite no EBS: it is a failed
run. During a wheelspin event the rear-wheel speed estimate reached 12.49 m/s
while reference forward speed was 0.58 m/s. The updated FSDS profile uses front
wheel speed and IMU orientation and disables map-driven pose correction pending
revalidation. The hardware defaults retain gyro integration and rear encoders;
the real drivetrain and IMU capabilities must determine those choices.

The controller now checks path timestamps and remaining path length. Missing
geometry requests a controlled stop; stale publishers remain watchdog failures.
First-lap planning no longer falls back to the entire accumulated map when local
observations disappear. `evaluate_fsds.py` records reference pose and referee
statistics exclusively for evaluation, never as autonomy inputs.

## 16 September 2026: perception and encoder-displacement correction

The previous 100-second run had no complete lap, eight referee cone hits and
38.35 m maximum position error. Integrating reference speed over wall time gave
100.62 m while reference position travelled only 50.50 m: simulation physics
was running slower than wall time. Front-wheel encoder increments measured
47.65 m at radius 0.18 m. The FSDS adapter now uses angle differences with
rollover handling and short smoothing, instead of integrating physics-time RPM.
Hardware wheel/IMU inputs are unchanged.

The blue mask previously joined cones to bluish road pixels, then rejected the
whole connected region. CPU and CUDA also used different thresholds. Shared
thresholds now separate that background and retain pale yellow. In the FSDS
profile, sky is removed before contour extraction, aligned stripe fragments
are merged, and calibrated ground-foot projection plus physical size gates
reject implausible candidates. Known-height ranging remains the uncalibrated
hardware default; the FSDS flat-ground model is not validated for slopes or
large pitch/roll. The level mount's effective ground height is 0.8 m, checked
against visible start cones; the absolute camera RPC z is not the height to use.

The controller's startup speed ramp now starts at zero and resets on corridor
loss; the previous initialization bypassed it and immediately requested full
throttle. The FSDS longitudinal gain is reduced to avoid throttle/brake cycling.
The runner no longer starts two ROS CLI subscriptions every status update.

First complete evaluation (`20260916_124717`, 220 seconds, fresh spawn):

- FSDS referee: one completed lap, **151.362 s**, **zero down-or-out cones**.
- Reference distance: 541.60 m including the partial following lap.
- Zero controller or supervisor EBS samples; no empty paths after startup.
- Minimum sampled vehicle-body clearance: 0.305 m; zero outside-corridor samples.
- Maximum accumulated position error: 5.241 m; global map drift remains.
- Perception: 11.83 Hz; control and odometry: approximately 50 Hz on this PC.

Second fresh-spawn evaluation (`20260916_125234`, 190 seconds) completed a
157.341-second lap with zero referee cone hits, zero EBS samples, no sampled
body departures and 0.340 m minimum body clearance. Maximum position error was
3.544 m. Perception averaged 10.19 Hz; control and odometry stayed near 50 Hz.
There was a brief recoverable corridor dropout near the start/finish region;
the car resumed without manual intervention. These two runs used no preloaded
track geometry and no reference-pose input to autonomy.

Clearance is evaluated at 5 Hz against straight segments connecting the
reference blue/yellow boundaries, with a 1.8 x 1.0 m vehicle box. This is useful
evidence, not continuous-time collision proof. Reference vehicle dimensions:
[FSDS vehicle model](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.0.1/vehicle_model/).

The built-node regression suite passed startup ramping, recovery, watchdog,
encoder rollover/physics-time independence, sky/asphalt rejection, pale yellow
detection and a captured FSDS start frame. Standalone C++ algorithm and mapping
tests passed. CUDA thresholds share source constants, but CUDA execution and
Jetson timings were not tested.

```bash
python3 tools/test_runtime.py --frame artifacts/calibration/camera_00.png
python3 tools/summarize_fsds.py demo_logs/latest/evaluation.json \
  --reference artifacts/reference.json
```

The optional captured-frame check requires your local diagnostic frame; it is
not bundled in Git. `capture_reference.py` requires Python msgpack and reads only
referee geometry and camera calibration for offline comparison, never controls.
