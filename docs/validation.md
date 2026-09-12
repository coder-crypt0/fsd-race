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
