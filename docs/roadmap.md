# Completion roadmap and release gates

**16 September 2026.** “Implemented,” “demonstrated” and “validated” are distinct.
Documentation does not convert a prototype into a competition-ready system.

## Current milestone

The repository has a runnable monocular FSDS stack, repeatable launch/cleanup,
diagnostics, source tests and built-node regressions. Two clean laps demonstrate
that the main perception-to-control loop can work. Failed repeats show that the
reliability milestone is still open.

## Prioritized work

| Priority | Remaining work | Evidence required to close |
|---|---|---|
| P0 | Intermittent IMU/wheel gaps and EBS | Time-aligned bridge/sensor/callback traces; controlled host-load comparison; repeated clean runs without relaxing watchdogs |
| P0 | Independent actuator-side command/supervisor timeout | Failure-injection tests for dead controller, dead supervisor and broken bridge; bounded braking response |
| P0 | Global odometry/map drift and duplicates | Reference-aligned map/pose errors over repeated laps; validated associations/corrections and rejection tests |
| P1 | First-lap speed and turn reliability | Incremental speed sweep after P0 closure; no cone hits/departures; stopping margin and latency evidence |
| P1 | Perception generalization | Annotated frames split by run/conditions; class precision/recall, false positives, depth errors by range |
| P1 | Known-track optimization | Verified closed corridor, bounded curvature/clearance, repeatable faster clean laps; no “optimal” claim without objective comparison |
| P1 | Jetson/Pi camera integration | Native ARM build, calibrated one-camera input, thermal/resource/latency benchmarks and HIL tests |
| P1 | Physical vehicle interface | Completed board-specific firmware and independent safety commissioning; not simulator auto-GO |
| P2 | Reproducible provisioning | Pinned Dockerfile/dependency manifest, clean-machine setup and clean-clone verification |
| P2 | Operator tooling | Dashboard port forwarding, local-only/authenticated access, separated replay domain, bounded recording controls |
| P2 | Competition handoff | Exact rulebook edition and rule-to-evidence matrix, accepted submission format, reviewed demo/video |

## Proposed engineering acceptance gates

These are project gates, not official Formula Bharat requirements.

### Monocular demo baseline

- One image input; no LiDAR, stereo, reference pose or preloaded track in autonomy.
- Ten consecutive fresh-spawn TrainingMap runs with one referee-confirmed lap,
  zero down-or-out cones, no EBS, no manual intervention and no sampled body departure.
- Preserve all pass/fail run IDs and the first failure trace; do not count only successes.
- Repeat on an additional track and under controlled input/rendering load.
- Clear startup, stop and failure behavior; no stale geometry presented as fresh.

Ten clean runs are an engineering checkpoint, not proof of zero future failure.

### Accuracy and speed

Use held-out labeled images and reference geometry. Report per-color detection
precision/recall and range error versus distance; map completeness, duplicates
and association error; pose RMSE/max/drift per distance; actual lateral tracking
error and boundary clearance; lap time, minimum stopping margin and latency tails.
Set numeric acceptance bounds from vehicle geometry and safety margins, then
test them. Existing average topic rates are insufficient.

A one-shot unseen lap must optimize online within the visible horizon; the
current post-lap known-track mode cannot satisfy globally optimal first-lap
claims. A one-lap mission should also explicitly finish and stop, which the
default demo currently does not do automatically.

### Embedded and physical release

Pass native Jetson builds, CPU/CUDA comparisons, camera timestamp/calibration
tests, thermal endurance and network-loss HIL. Commission physical steering,
braking, torque limits, independently enforced timeout and emergency-stop
hardware before any dynamic vehicle test. Confirm exact regulations separately.

## Out of scope of this handoff

No learned detector/model training, LiDAR, second camera, new hardware purchase,
competition approval, real-car test or guarantee of winning/zero mistakes was
completed. No fabricated percentage is used where measurements are missing.
