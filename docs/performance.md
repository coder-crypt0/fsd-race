# Performance and resource measurements

**Measured 16 September 2026. PC results only; Jetson not benchmarked.**

## 1. Measurement environment

| Component | Observed |
|---|---|
| Host CPU | Intel Core Ultra 7 155H, 16 physical / 22 logical cores |
| Host RAM | 33,685,856,256 bytes reported usable, approximately 31.37 GiB |
| Host GPU | Intel Arc integrated graphics; driver 32.0.101.6314 |
| Simulator | FSDS TrainingMap, window 960 × 540 |
| Sensor image | One RGB camera, 424 × 320, 70° |
| Runtime | Docker Linux/amd64, Ubuntu 22.04.5, ROS 2 Humble |
| Python / OpenCV | 3.10.12 / 4.5.4 |
| Perception backend | CPU-only; no nvcc in runtime environment |
| Workload | FSDS bridge, seven C++ nodes, dashboard and bounded evaluator |
| Host isolation | Normal interactive desktop; background applications remained open |

Profiling run: `20260916_212359`, 100 s evaluation.
It stopped on stale IMU/wheel data at approximately 32 s, with no cone hits.
The measurements below were taken **after EBS, while the stationary scene,
ROS nodes and evaluator continued running**. They are not steady driving or
worst-case benchmarks and cannot establish a race-speed compute budget.

## 2. Windows host and FSDS rendering

Thirty 1-second performance-counter samples; window ended around
15:55:53 UTC, after the stop. The Windows and Linux windows were separate,
not synchronized.

| Metric | Mean | p95 | Maximum |
|---|---:|---:|---:|
| Whole-host CPU, % of all 22 logical CPUs | 45.652% | 53.836% | 57.269% |
| FSDS Blocks CPU, % of whole host | 3.006% | 5.493% | 5.606% |
| FSDS 3D-engine utilization | 89.709% | 94.495% | 96.607% |
| FSDS private working set | 1343.799 MiB | 1344.645 MiB | 1344.672 MiB |
| Whole-host RAM used | 25.882 GiB | 26.038 GiB | 26.184 GiB |

GPU values are summed Windows 3D-engine counters attributed to the FSDS process.
They are not CUDA usage, all-application GPU usage or a portable compute-load
percentage. Integrated graphics shares system memory. GPU memory, temperature,
power draw and throttling were not measured.

High renderer utilization is observed; attributing every stale-input event to
GPU saturation would require time-aligned latency traces and controlled load
experiments. Average CPU headroom does not rule out callback stalls.

## 3. Linux per-process CPU and resident memory

Thirty approximately 1-second `/proc` deltas ending 15:55:19 UTC, entirely after
the stop. CPU convention: **100% = one logical CPU core**; values are not
normalized to the 22-core host. RSS includes shared-library pages.

| Process | Mean CPU | p95 CPU | Mean RSS |
|---|---:|---:|---:|
| Monocular perception (`stereo_cone_node`) | 4.483% | 5.978% | 68.837 MiB |
| State estimation | 10.260% | 11.952% | 22.859 MiB |
| Cone mapping | 2.989% | 3.987% | 23.375 MiB |
| Path planning | 3.553% | 3.991% | 23.031 MiB |
| Motion control | 7.770% | 8.964% | 22.344 MiB |
| Safety supervisor | 7.139% | 8.954% | 22.172 MiB |
| FSDS adapter | 10.194% | 11.950% | 22.688 MiB |
| FSDS main bridge | 43.466% | 47.838% | 32.593 MiB |
| FSDS camera bridge | 0.996% | 1.991% | 69.098 MiB |
| Dashboard server | 29.454% | 36.875% | 60.102 MiB |
| Read-only evaluator | 72.218% | 81.580% | 63.802 MiB |

The seven C++ processes total **46.388% of one core** on average in this
post-stop window, approximately 2.11% of 22 equal logical CPUs arithmetically.
This is not a prediction for heterogeneous ARM cores. The evaluator/dashboard
are significant diagnostic overhead and should be separated from vehicle runtime
benchmarks. Summing RSS overcounts shared memory.

One independent `docker stats` snapshot during the same run reported 210.75%
CPU (2.11 core-equivalents) and 348.3 MiB container memory. It is an instantaneous,
differently defined measure—not the sum of the RSS table and not a 30-s average.

## 4. Rates and latency

| Run | Perception output | Control | Estimated odometry |
|---|---:|---:|---:|
| Clean run 1 | 11.83 Hz | 49.93 Hz | 49.90 Hz |
| Clean run 2 | 10.19 Hz | 49.91 Hz | 49.89 Hz |
| Failed repeat 3 | 6.94 Hz | 49.91 Hz | 49.87 Hz |
| Later profiling run | 3.37 Hz | 49.65 Hz | 49.56 Hz |

Rates are received message counts divided by whole evaluation duration.
They include startup/stopped periods and are not detector inference-time
benchmarks. A 50 Hz average can coexist with a >0.3 s gap that trips a watchdog.

No measured camera-to-command p95/p99 latency, worst-case scheduling jitter,
hardware FPS or thermal endurance result is available. Do not infer those
from topic rate or marketing TOPS. Raw FSDS physical speed and estimated
wall-time speed also differ under slow simulation; see [validation](validation.md).

## 5. Reproduce the measurements

While one FSDS demo is live, run from the repository root:

```powershell
.\Measure-FSDS.ps1 -Seconds 30
```

This writes `artifacts/windows-resources.json`, using local performance counters.
It fails if exactly one Blocks process is not running. Unavailable GPU counters
are reported as null, never zero. English counter names are assumed.

Inside the live Docker environment:

```bash
docker exec fsd-demo python3 /ws/tools/profile_resources.py \
  --seconds 30 --output /ws/artifacts/linux-resources.json
docker stats fsd-demo --no-stream
```

The Linux tool uses no ROS/dependencies and samples only named runtime processes.
It guards PID reuse, records CPU convention, and bounds duration to 60 s.
Raw files are local/ignored; these tables retain the measured summary in Git.
Record movement/EBS state, camera rate, background load and exact sample times
with every future comparison. Latest script timestamps make that correlation
explicit; the first snapshot above predates per-sample timestamp fields.

## 6. Jetson performance conclusion

**No Jetson utilization or FPS claim can currently be made.** Orin Nano is the
intended target; the original Nano needs a separate software port. Run the
autonomy stack on Jetson, with FSDS rendering on the PC, and measure the actual
camera/bridge workload before selecting power mode or speed.

CPU-only operation is already available. CUDA segmentation is optional and
untested here; no TensorRT, YOLO, neural inference, DLA or LiDAR workload is
active. See [hardware deployment](jetson-deployment.md) for the profiling and
integration checklist.
