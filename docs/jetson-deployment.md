# Jetson compatibility and deployment plan

**16 September 2026 · proposed bring-up, not verified on hardware**

## 1. Which “Nano”?

| Platform | Assessment for this repository |
|---|---|
| Jetson Orin Nano 8 GB / developer kit | Intended target; plausible source-build path, but no hardware build or timing evidence |
| Jetson Orin Nano 4 GB | Same architecture family; memory/thermal headroom must be measured; not tested |
| Original Jetson Nano 2 GB / 4 GB | Not supported as-is; older BSP/CUDA/Ubuntu stack requires a separate port |
| This Intel laptop | FSDS and CPU runtime actually exercised; cannot predict embedded timing |

Orin Nano 8 GB uses an Ampere GPU and 8 GB LPDDR5; it is not the original
Maxwell-era Nano. Hardware capability figures are not this application's FPS.
[NVIDIA platform overview](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/leveraging-ros-2-and-hil-in-isaac-sim/02-nvidia-jetson-platform-overview.html).

The original Nano's JetPack 4.6 family uses an Ubuntu 18.04-based BSP and CUDA
10.2. This project targets ROS 2 Humble/Ubuntu 22.04 and defaults CUDA architecture
to Orin's 87, so installation/build compatibility cannot be assumed.
[NVIDIA JetPack 4.6](https://developer.nvidia.com/embedded/jetpack-sdk-46).

## 2. Version-pinned bring-up target

A **candidate** environment is JetPack 6.2.1 / Jetson Linux 36.4.4 with Ubuntu
22.04, CUDA 12.6 and ROS 2 Humble ARM64. This aligns the OS generation with the
tested CPU workspace, but is not a tested deployment.
[NVIDIA release](https://developer.nvidia.com/embedded/jetpack-sdk-621),
[ROS target platforms](https://www.ros.org/reps/rep-2000.html).

This is deliberately version-pinned, not a claim that JetPack 6 is the latest.
The current NVIDIA quick-start guide references JetPack 7.2.1 and a different
installation flow. A newer image requires checking its BSP, ROS, camera and
CUDA compatibility rather than applying old instructions blindly.
[NVIDIA quick start](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html).

Do not flash or upgrade an existing board without preserving its current setup
and confirming its exact module/carrier/firmware. No board was flashed here.

## 3. What can run where

```text
PC: FSDS rendering + simulator sensors
              ↓ network ROS/bridge inputs
Jetson: one-camera perception → estimation → mapping → planning → control
              ↓
Simulation adapter during HIL; validated ECU interface only during later hardware work
```

FSDS Windows/Unreal rendering is not part of the intended Jetson workload.
The local amd64 Docker image and compiled binaries cannot be copied to ARM64.
Build from source on the target. The C++ package currently depends on fs_msgs
because it builds the simulator adapter together with the other nodes.

Use the [workspace build guide](../fsd_ws/README.md) after installing matching
ROS packages on the board. Start with non-actuating tests. CMake automatically
enables CUDA when nvcc is found; that is a different, currently unvalidated
perception path. For a deliberate CPU-only comparison on a CUDA-equipped board,
configure a separate build with `-DCMAKE_CUDA_COMPILER=NOTFOUND` and verify the
CMake output says CPU-only. Do not reuse a CUDA-configured build cache blindly.

## 4. One Pi camera

The exact Pi sensor module, lens and interface were not specified. “Pi Camera”
does not establish driver compatibility. Do not assume OV5647, IMX219 and IMX477
are interchangeable.

For Raspberry Pi Camera Module v2, NVIDIA documents a 15-pin to 22-pin conversion
cable for the Orin Nano developer kit. Confirm the carrier connector and the
selected JetPack's driver support before purchasing/connecting.
[NVIDIA camera guidance](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/howto.html).

Required integration:

1. Bring up exactly one sensor with its supported V4L2/Argus/GStreamer driver.
2. Publish a timestamped ROS Image stream; confirm encoding and QoS.
3. Calibrate lens intrinsics/distortion at the actual output resolution.
4. Measure mount height, pitch, yaw and camera-to-vehicle transform.
5. Validate near-field cone visibility and body occlusion above the seat.
6. Set exposure/white balance appropriate to cone colors; test blur and vibration.
7. Recalibrate flat-ground ranging and validate pitch/roll/slopes before driving.

The current detector reads calibration parameters, not CameraInfo updates.
There is no complete Pi-camera ROS driver/launch integration shipped here.
The single-camera policy must remain enforced through this work.

## 5. Remaining non-camera integration

The FSDS adapter is not a physical vehicle interface. Real wheel encoders need
timestamp/radius/direction calibration; an IMU needs frame alignment, bias and
orientation-validity checks. Select front/rear wheel sources according to the
actual drivetrain, not the simulator's defaults.

The STM32 bridge contains incomplete board/HAL code. Steering/braking feedback,
actuator saturation, torque scaling, independently enforced command timeout,
physical emergency stop and CAN behavior need bench and hardware-in-loop tests.
The old Python vehicle launch is not a validated C++ deployment launch.

Do not enable auto-GO on the vehicle. Perform non-actuating sensor tests first,
then independently reviewed actuator/safety commissioning in controlled conditions.

## 6. Profiling the real target

Record exact SKU/RAM, JetPack/BSP, ROS/OpenCV/CUDA versions, power mode, cooling,
supply and camera settings. Read-only monitoring commands:

```bash
sudo /usr/sbin/nvpmodel -q
sudo tegrastats --interval 1000
python3 tools/profile_resources.py --seconds 30 --output artifacts/jetson-processes.json
```

Stop tegrastats after the planned window. NVIDIA identifies it as the primary
tool for CPU/GPU/memory/temperature/power monitoring on Jetson, rather than
assuming desktop nvidia-smi support.
[NVIDIA monitoring guide](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/howto.html).

Measure per-node CPU/RSS, GPU utilization, memory pressure, power, temperatures,
throttling, camera drops, worst sensor gaps and camera-to-command p50/p95/p99.
Compare CPU/CUDA outputs and timing on the same images. Repeat with and without
dashboard/evaluator and with sustained thermal load.

The existing low PC CPU snapshot is not sufficient to promise 30 FPS or
race-speed operation. Release gates: [roadmap](roadmap.md).

## 7. Decision

Proceed with **Orin Nano source bring-up**, not an original-Nano drop-in promise.
No additional cameras or LiDAR are required by the implemented FSDS architecture,
but reliable physical monocular operation still needs calibration and validation.
The board's actual performance and safe speed remain unmeasured.
