# FSD Workspace — Formula Bharat Driverless Cup 2027

Complete autonomous stack per **FSD_System_Interface_Specification.md** (one
folder up), in two implementations sharing the same locked interfaces:

- **`fsd_cpp`** — C++ production stack for the Jetson Orin. GPU (CUDA) color
  segmentation in perception, self-contained Delaunay + Catmull-Rom planning,
  50 Hz control. Runs against **FSDS** (stereo cameras only, no lidar) via
  the included adapter, and later against the real car.
- **`fsd_stack`** — Python reference stack + built-in kinematic simulator +
  the **mission-control dashboard** and bag run platform. Great for algorithm
  work and quick experiments; the C++ stack is what goes on the car.

```
fsd_ws/
├── src/
│   ├── fsd_msgs/                  # interface messages — the locked contract
│   ├── fsd_cpp/                   # C++/CUDA stack + FSDS adapter
│   │   ├── src/stereo_cone_node.cpp      # Blocks 1+2 (GPU segmentation,
│   │   │                                 #   stereo SAD ranging, mono fallback)
│   │   ├── src/cuda/segmentation.cu      # the CUDA kernel
│   │   ├── src/state_estimation_node.cpp # Block 3
│   │   ├── src/cone_mapping_node.cpp     # Block 4 (data association)
│   │   ├── src/path_planning_node.cpp    # Block 5 (Bowyer-Watson + Catmull-Rom)
│   │   ├── src/motion_control_node.cpp   # Block 6 (Pure Pursuit, 50 Hz)
│   │   ├── src/safety_supervisor_node.cpp# Block 8
│   │   └── src/fsds_adapter_node.cpp     # FSDS glue (replaces Block 7 in sim)
│   └── fsd_stack/                 # Python stack, sim, dashboard
│       └── fsd_stack/dashboard_node.py   # FSD MISSION CONTROL (port 8321)
├── fsds/settings.json             # FSDS stereo-camera config (copy to sim)
├── tools/fsd_bag.py               # run record/replay CLI
└── firmware/stm32_bridge/main.c   # Block 7 ECU firmware (real car)
```

## Build (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install python3-scipy python3-opencv ros-humble-cv-bridge
cd fsd_ws
git clone -b ros2 https://github.com/FS-Driverless/fs_msgs.git src/fs_msgs   # FSDS control messages (ros2 branch — master is ROS1/catkin!)
colcon build --symlink-install
source install/setup.bash
```

CUDA is detected automatically (JetPack on the Orin ships it). On a dev PC
with an NVIDIA GPU: `colcon build --cmake-args -DCMAKE_CUDA_ARCHITECTURES=native`.
Without CUDA everything still builds and runs on the CPU fallback.

## Run against FSDS (Formula Student Driverless Simulator)

1. Install FSDS per https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/
2. Copy the stereo camera config:
   `cp fsds/settings.json ~/Formula-Student-Driverless-Simulator/settings.json`
3. Start the simulator, then its ROS2 bridge.
4. `ros2 topic list` — confirm the camera topic names, then:

```bash
ros2 launch fsd_cpp fsds.launch.py
# if the bridge names cameras differently:
ros2 launch fsd_cpp fsds.launch.py left_image:=/fsds/<actual_left_topic> right_image:=/fsds/<actual_right_topic>
```

5. Open **http://localhost:8321** — the dashboard comes up with the stack.
6. Send the GO signal from the FSDS operator page (or set `auto_go: true`
   in `src/fsd_cpp/config/fsds_params.yaml` for unattended testing).

Two calibration switches live in `fsds_params.yaml`, both explained inline:
`steering_sign` (flip if the car mirrors its steering) and `auto_go`.

## Mission-control dashboard

`ros2 run fsd_stack dashboard` (included in every launch file) →
http://localhost:8321. Dark-theme single page, zero external dependencies:

- live track map: mapped cones, planned path, car pose + trail, 5 m grid
- speed gauge, lateral-g dot, steering/torque/brake bars
- lap counter with current/last/best times (auto start-line detection)
- node health tiles driven by /safety/heartbeat (stale = red)
- per-topic Hz meters, telemetry table, timestamped event log
- EBS banner that goes loud the instant the trigger latches
- **Run platform** panel: record the current session, browse stored runs,
  replay one back into the stack at chosen rate/stage

## Bag run platform

Runs live in `~/fsd_runs/<timestamp>_<name>/`. From the CLI:

```bash
python3 tools/fsd_bag.py record --name skidpad_v2    # Ctrl-C to stop
python3 tools/fsd_bag.py list
python3 tools/fsd_bag.py play 20261012_143000_skidpad_v2 --stage raw
```

Replay stages: `raw` feeds recorded camera/IMU/wheel topics so the **full
stack recomputes** (regression-test a perception change against an old run);
`cones` starts from recorded perception output; `all` replays everything
verbatim for pure visualization. Launch the stack without sim/FSDS/car
first — the bag is the input source.

## Run the Python closed loop (no simulator install needed)

```bash
ros2 launch fsd_stack sim.launch.py
ros2 run fsd_stack dashboard   # separate terminal
```

## Full verification in Docker (no ROS install required)

`tools/docker_test.sh` builds all four packages against `ros:humble`, runs
both test suites, then launches the actual closed loop headless and asserts
the car drives (≥45 Hz control, car reaches speed, cones mapped, path
produced). Verified passing 2026-07-07 (50.9 Hz, 4.3 m/s, 39.7 m covered):

```bash
docker run --rm -v "$PWD":/ws ros:humble bash /ws/tools/docker_test.sh
```

## Offline algorithm tests (no ROS required)

```bash
python3 src/fsd_stack/test/test_algorithms.py
```
Verifies data association (zero duplicate landmarks), planning geometry
(cone clearance, curvature), velocity-profile feasibility, and closed-loop
Pure Pursuit tracking — importing the real node modules with ROS stubbed.
The C++ Delaunay/Catmull-Rom core has an equivalent standalone test
(structure + empty-circumcircle property + Euler identity).

## Real vehicle

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch fsd_stack vehicle.launch.py   # Python stack + CAN bridge
```
Prerequisites: camera driver, BNO055 driver, STM32 flashed with
`firmware/stm32_bridge` (fill the `TODO(board)` HAL stubs), measured values
replacing every `!!` placeholder in `src/fsd_stack/config/params.yaml`.

## Before ANY dynamic test — non-negotiable (spec §11)

1. S1–S10 safety checklist items relevant to the test passed.
2. RES check + EBS check before every run, human with RES in range.
3. `fsd_bag.py record` (or dashboard REC) on every power-up — bags are the
   integration currency; every block must work on bags before the car.

## Rules for contributors

- Interfaces (topics, message fields, rates, frames) are LOCKED. Changing
  one requires a spec version bump signed off by all three owners.
- Every node publishes `/safety/heartbeat` ≥ 5 Hz or the supervisor EBSes
  the car. That is a feature.
- The Python and C++ stacks must stay behaviorally equivalent per block;
  fix algorithms in both or ship the difference through the spec.
