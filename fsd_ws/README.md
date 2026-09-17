# ROS workspace

The supported demo runs **fsd_cpp with one forward camera in FSDS**.
No LiDAR is configured. This is an engineering demonstrator, not a commissioned
real-car system. Start with the [root README](../README.md).

## Packages

| Directory | Role |
|---|---|
| `src/fsd_msgs` | Runtime and diagnostic message contracts |
| `src/fsd_cpp` | Seven C++ processes, optional CUDA, FSDS configuration |
| `src/fsd_stack` | Current dashboard; older reference driving nodes/simulator |
| `src/fs_msgs` | External FSDS ROS 2 messages, ignored by this repository |
| `fsds` | Single-camera simulator settings |
| `tools` | Launch, tests, bounded capture/evaluation, profiling |
| `firmware/stm32_bridge` | Prototype with incomplete board/HAL integration |

C++ and Python driving behavior has diverged. The Python simulator,
`vehicle.launch.py` and `tools/docker_test.sh` are legacy development paths,
not substitutes for FSDS validation. Do not launch them on an actuated vehicle.

## Build in an existing ROS environment

Baseline: Ubuntu 22.04 / ROS 2 Humble, C++17, CMake, colcon, OpenCV core/imgproc,
cv_bridge, Python 3, NumPy, SciPy and Python OpenCV.

After installing ROS using its official instructions:

```bash
source /opt/ros/humble/setup.bash
sudo apt install build-essential cmake python3-colcon-common-extensions \
  libopencv-dev ros-humble-cv-bridge python3-numpy python3-scipy python3-opencv
cd fsd_ws
# Only if src/fs_msgs does not already exist:
git clone --branch ros2 https://github.com/FS-Driverless/fs_msgs.git src/fs_msgs
git -C src/fs_msgs checkout 4146e5b4889fb92332c9ce5ee42a8081649cbe72
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The ROS 1 default branch of fs_msgs is not interchangeable. This builds the
workspace, not the external FSDS bridge or a fresh workstation environment.
CMake enables CUDA if nvcc is found, default architecture 87 (Orin). CPU
perception is the tested path; CUDA remains unvalidated. Never copy x86
build/install directories to ARM64. [Jetson guide](../docs/jetson-deployment.md).

## FSDS runtime

Start FSDS with this repository's settings and its ROS 2 bridge. Then:

```bash
source /opt/ros/humble/setup.bash
source /path/to/FSDS/ros2/install/setup.bash
source install/setup.bash
ros2 launch fsd_cpp fsds.launch.py
```

Replace the bridge path with the real installation. The configured Windows
launcher mounts it at `/fsds/ros2/install` inside Docker.
Image: `/fsds/cam_left/image_color`; IMU: `/imu`.
Use `left_image:=<observed-topic>` to remap the one forward image if necessary.
Auto-GO is enabled for simulation.

On the configured Windows workstation, use the root `run.ps1` or
`run-nobuild.ps1`. [Operations and prerequisites](../docs/operations.md).

## Checks and diagnostics

```bash
bash src/fsd_cpp/test/run_tests.sh
python3 tools/test_runtime.py
python3 tools/capture_diagnostics.py --output artifacts/capture --seconds 10
python3 tools/evaluate_fsds.py --output artifacts/evaluation.json --seconds 190
python3 tools/summarize_fsds.py artifacts/evaluation.json --require-clean-lap
```

Runtime tests require built/sourced ROS packages and use isolated domain 91.
Capture/evaluation require live FSDS. Reference comparison and captured-frame
checks require local artifacts, not bundled files. Keep recordings bounded.

[Architecture](../ARCHITECTURE.md) ·
[Contracts](../FSD_System_Interface_Specification.md) ·
[Validation](../docs/validation.md) · [Performance](../docs/performance.md)
