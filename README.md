# fsd-race

A camera-based Formula Student driverless research stack using ROS 2, C++17,
optional CUDA perception, and the Formula Student Driverless Simulator (FSDS).
The deployment target is NVIDIA Jetson Orin Nano; hardware performance has not
yet been measured on that device.

## Development status

This is an experimental simulator project. Two fresh-spawn FSDS TrainingMap runs
completed laps in 151.36 and 157.34 seconds with zero referee cone hits on
16 September 2026. Repeatability testing is in progress; race-speed operation and other tracks are
not validated. See the [validation record](docs/validation.md) for measured limits.
The current default uses one forward camera with HSV cone detection and
calibrated flat-ground monocular ranging in FSDS. No trained YOLO model is bundled
or enabled. Real-camera height, pitch and intrinsics require separate calibration.

## Architecture

Camera images feed cone detection and ranging. Wheel speed and IMU measurements
feed state estimation. Cone mapping, local path planning, motion control, and a
safety supervisor communicate through the messages in `fsd_msgs`.

- `fsd_ws/src/fsd_cpp`: C++ runtime, optional CUDA kernels, FSDS adapter.
- `fsd_ws/src/fsd_msgs`: ROS message definitions.
- `fsd_ws/src/fsd_stack`: Python reference implementation and telemetry dashboard.
- `fsd_ws/fsds/settings.json`: single-camera simulator configuration.
- `fsd_ws/tools`: demo launcher and record/replay utilities.
- `fsd_ws/firmware`: embedded bridge prototype; not validated for vehicle use.

## Windows FSDS demo

The current convenience launcher expects an existing local installation:

- FSDS at `%USERPROFILE%\FSDS\FSDS.exe`.
- Kali WSL with Docker and the local `fsd-test:latest` image.
- A built FSDS ROS 2 bridge at `/root/FSDS_repo/ros2/install` in WSL.
- The project workspace is synchronized to `/root/fsd_ws` by the launcher.

These dependencies are not bundled, and the launcher does not provision a fresh
machine. See [workspace instructions](fsd_ws/README.md) for ROS dependencies and
manual build commands, and the [official FSDS documentation](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/)
for simulator installation.

From PowerShell in this directory:

```powershell
.\run.ps1          # synchronize, build, launch
.\run-nobuild.ps1  # synchronize configuration and launch existing binaries
.\run-nobuild.ps1 -EvaluationSeconds 190  # record a bounded run and stop cleanly
```

The demo starts the simulated vehicle automatically. The dashboard is available
at `http://localhost:8321`. Ctrl+C stops the demo. Text logs are copied to
`fsd_ws/demo_logs`; recordings, build products, and model weights are excluded
from version control. Run `run.ps1` after changing C++ sources.
Bounded evaluations save `fsd_ws/demo_logs/evaluation.json`, including referee
lap times, cone hits, path geometry, vehicle state and emergency-brake status.
The optional evaluator reads simulator reference data; autonomy never consumes it.

## Algorithm checks

With Bash and a C++17 compiler:

```bash
bash fsd_ws/src/fsd_cpp/test/run_tests.sh
```

These tests extract algorithm helpers from runtime sources and execute synthetic
cases. They do not yet cover the complete ROS planner or controller lifecycle.

## Runtime regression check

After building the workspace in the ROS environment, run the isolated runtime
regression suite to exercise the compiled C++ nodes' recovery, controlled-stop,
planner-watchdog, and odometry behavior:

```bash
cd fsd_ws
python3 tools/test_runtime.py
```

Build and source the workspace first (`colcon build --symlink-install` followed
by `source install/setup.bash`); the check launches the installed C++ node
binaries. It uses ROS domain 91 and does not require FSDS. See the
[validation record](docs/validation.md) for its exact coverage and current
end-to-end limitations.

## Current engineering priorities

1. Record camera frames, detections, planned paths, and simulator reference pose
   to explain the first corridor departure or path failure.
2. Validate camera geometry, timestamp alignment, cone associations, and short
   path behavior before increasing the configured speed ceiling.
3. Evaluate cone-trained neural detectors on captured FSDS frames; verify class
   mappings, preprocessing, inference latency, and model licensing.
4. Benchmark the complete stack on the target Jetson and validate repeated laps.

The simulator, ROS dependencies, and any future model weights remain separate
projects with their own licenses. Historical design documents describe intended
behavior and must not be interpreted as verified implementation or certification.
