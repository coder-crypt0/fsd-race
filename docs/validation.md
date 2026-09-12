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

Repeated collision-free laps, measured corridor clearance, robust recovery,
race-speed behavior, and Jetson hardware benchmarks remain unverified. Passing
the helper algorithm tests alone does not demonstrate any of those outcomes.

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
