# fsd-race

Single-camera autonomous racing research for Formula Student / Formula Bharat,
built around **ROS 2, C++17 and the Formula Student Driverless Simulator (FSDS)**.

The system detects colored cones, estimates motion from IMU and wheel encoders,
builds an online cone map, plans a local corridor, and controls the simulated car.
It starts autonomously from a fresh spawn without a preloaded track.

## Project status

**Engineering demonstrator — clean laps demonstrated, repeatability not yet achieved.**
Reviewed **16 September 2026**, against driving implementation `79502e9`.
This is not a race-ready or real-vehicle-certified release.

| Capability | Implemented / observed status |
|---|---|
| One forward RGB camera | Active; 424 × 320, 70° horizontal FOV |
| LiDAR / stereo / neural detector | None active; no YOLO weights or inference runtime |
| Cone perception | HSV masks, geometry rejection, monocular ground-foot ranging |
| Online mapping | Confirmed landmarks and color-based boundaries; accumulated drift remains |
| Unseen-track driving | Local corridor planning; two clean FSDS TrainingMap laps recorded |
| Recovery | Brief corridor loss can stop and resume automatically; EBS stays latched |
| Obstacle handling | Cone-like objects and blockage logic; general/dynamic objects unvalidated |
| Known-track racing line | Implemented, experimental; full-speed clean laps not demonstrated |
| Jetson Orin Nano | Deployment target; hardware build, timing and vehicle tests outstanding |

Three dedicated fresh-spawn lap trials produced **two clean laps: 151.362 s and
157.341 s**, each with **zero referee cone hits**. The third stopped on stale
IMU/wheel data before finishing. A later resource-profiling run reproduced that
stop. These are individual trials, not a statistically established reliability
rate. See the [validation record](docs/validation.md).

The first-lap limit is **2.5 m/s in the controller's time base**. The 10 m/s
known-track setting is an unvalidated ceiling, not achieved race performance.
An unknown track cannot be globally optimized before it is observed; the first
lap uses online, local planning.

## Sensor policy: one camera, no LiDAR

The supported demo uses only `cam_left`—a historical name for the single
centered, forward-facing RGB camera. The live detector has **one image
subscription**, no right-image subscription, and no point-cloud input.

IMU and wheel encoders support motion estimation; “single camera” does not mean
vision-only odometry. FSDS settings still enable GSS, but autonomy does **not**
consume GSS, GPS, referee geometry or reference odometry. Read-only evaluation
tools consume `/testing_only/*` for scoring, not driving. Optional stereo and
multi-camera code remains disabled. [Sensor specification →](FSD_System_Interface_Specification.md)

## Run the Windows demo

The convenience launcher uses an existing workstation installation:

- FSDS at `%USERPROFILE%\FSDS\FSDS.exe`.
- Kali WSL with Docker and the local `fsd-test:latest` image.
- Built FSDS ROS 2 bridge at `/root/FSDS_repo/ros2/install`.
- Ext4 workspace at `/root/fsd_ws`, synchronized by the launcher.

External dependencies are not bundled or automatically provisioned.

```powershell
.\run.ps1                               # synchronize, build, launch
.\run-nobuild.ps1                       # reuse the last compiled binaries
.\run-nobuild.ps1 -EvaluationSeconds 190  # bounded evaluation, then stop
```

The car starts automatically in **FSDS TrainingMap**. The dashboard opens at
[localhost:8321](http://localhost:8321). Ctrl+C stops the demo; `-KeepSimulator`
keeps the simulator open on exit. Use `run.ps1` after C++ changes.

The launcher deliberately restarts existing FSDS processes for a fresh spawn.
Logs and evaluation JSON are copied to `fsd_ws/demo_logs`; timestamped originals
remain in WSL. Exit code zero means the process ran successfully—not that a
clean lap passed. [Setup and troubleshooting →](docs/operations.md)

## How it works

```text
One RGB camera → cone detection + monocular ranging ─┬→ local corridor planner
                                                     └→ persistent cone map
IMU + wheel encoders → estimated pose ────────────────→ map / planner / control
Planner → path + speed ceiling → Pure Pursuit + PI → FSDS adapter → vehicle
Node health + command/pose checks → safety supervisor → latched emergency brake
```

The dashboard visualizes the estimated map, path, orientation, trail, health
and controls. Its map is not simulator truth, and its lap display is not the
official referee result. [Detailed architecture →](ARCHITECTURE.md)

## Measured accuracy and resource usage

| Measurement | Observed result / scope |
|---|---|
| Clean-run maximum accumulated position error | 5.241 m and 3.544 m over the complete recording windows |
| Minimum sampled body-to-boundary clearance | 0.305 m and 0.340 m; 5 Hz reference-polyline comparison |
| Perception publication rate in clean runs | 11.83 Hz and 10.19 Hz |
| Control / estimated odometry publication | Approximately 50 Hz in clean runs |
| Cone precision, recall, mAP | Not measured on an annotated dataset |
| PC host CPU, 30-s post-stop snapshot | 45.65% mean across 22 logical CPUs; includes other applications |
| FSDS rendering GPU, same profiling run | 89.71% mean for the FSDS process's 3D engine |
| Seven C++ nodes, post-stop snapshot | 46.39% of **one CPU core** combined, not 46.39% of the PC |
| Autonomy CUDA usage | CPU-only build; CUDA not exercised |
| Jetson CPU/GPU/RAM/power/latency | Not measured |

Host: Intel Core Ultra 7 155H / Intel Arc laptop, approximately 32 GB RAM.
Resource samples were collected **after a stale-sensor emergency stop**, with
rendering and nodes still running. They are not moving-lap or worst-case
benchmarks. [Per-process measurements and methods →](docs/performance.md)

## Documentation

| Document | Contents |
|---|---|
| [Architecture](ARCHITECTURE.md) | Data flow, algorithms, mission states and safety limitations |
| [Specifications and interfaces](FSD_System_Interface_Specification.md) | Sensors, calibration, frames, ROS contracts, rates, configuration |
| [Operations](docs/operations.md) | Setup, demo, logs, captures, tests, storage and troubleshooting |
| [Validation](docs/validation.md) | Successful and failed trials, accuracy definitions, acceptance checks |
| [Performance](docs/performance.md) | CPU/GPU/RAM measurements and reproducible profiling |
| [Jetson deployment](docs/jetson-deployment.md) | Orin Nano versus original Nano, camera integration, bring-up gates |
| [Completion roadmap](docs/roadmap.md) | Remaining work and release acceptance criteria |
| [Workspace guide](fsd_ws/README.md) | Packages, build commands, runtime/reference distinction |

## Verification

Standalone C++ checks, without ROS:

```bash
bash fsd_ws/src/fsd_cpp/test/run_tests.sh
```

Built-node regressions after building and sourcing the ROS workspace:

```bash
cd fsd_ws
source install/setup.bash
python3 tools/test_runtime.py
```

Evaluate a recorded run; optional reference geometry strengthens the check:

```bash
python3 tools/summarize_fsds.py demo_logs/latest/evaluation.json \
  --reference artifacts/reference.json --require-clean-lap
```

CI runs source/synthetic checks; it does not launch FSDS, benchmark a Jetson or
certify a vehicle. [Test coverage →](docs/validation.md)

## Repository layout and scope

```text
fsd_ws/src/fsd_cpp/    C++ runtime, optional CUDA, FSDS adapter, configuration
fsd_ws/src/fsd_msgs/   ROS message contracts
fsd_ws/src/fsd_stack/  Live dashboard + legacy Python reference nodes
fsd_ws/fsds/          Single-camera FSDS settings
fsd_ws/tools/         Launch, capture, evaluation, regression and profiling
fsd_ws/firmware/      Incomplete embedded bridge prototype
docs/                Engineering handoff and evidence summaries
```

Build products, private context, recordings and model weights are ignored.
Earlier architecture/specification versions remain in Git history; the kickoff
questionnaire is historical background, not a hardware BOM.

External dependencies retain their licenses. Package manifests declare MIT;
the repository currently lacks a standalone LICENSE file. Resolve ownership and
attribution before redistribution. No competition compliance or qualification
approval is claimed.
