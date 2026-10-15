# Measurement and reproduction protocol

[Research index](README.md)

## Commands

```powershell
.\run.ps1 -EvaluationSeconds 330 -NoBrowser
# Subsequent runs, unchanged C++ binaries:
.\run-nobuild.ps1 -EvaluationSeconds 330 -NoBrowser
# Opt-in callback timings and resource sampling (adds observer load):
.\run-nobuild.ps1 -EvaluationSeconds 330 -Research -NoBrowser
```

After sourcing ROS, external bridge and project overlays:

```bash
bash /ws/src/fsd_cpp/test/run_tests.sh
python3 /ws/tools/test_runtime.py --timing --frame /ws/artifacts/calibration/camera_00.png
python3 /ws/tools/test_map_lifecycle.py
python3 /ws/tools/summarize_fsds.py /ws/demo_logs/latest/evaluation.json --require-clean-lap
python3 /ws/tools/plot_map_learning.py /ws/demo_logs/latest/evaluation.json \
  --output /ws/artifacts/map-learning.png
```

The optional captured-frame fixture is local and ignored; omit `--frame` if it
is unavailable. ROS tests use domain 91 and never inject into the demo domain.
`run-nobuild` synchronizes source but does not compile modified C++ or messages.

## Definitions and interpretation

| Quantity | Measurement | Limitation |
|---|---|---|
| Lap time / cone hits | FSDS referee on `/testing_only/extra_info` | Not inferred mapper laps; simulator timing differs from host wall time |
| Map freeze | First `TrackStatus.map_frozen`; stable IDs/coordinates afterwards | Count stability alone does not establish correspondence accuracy |
| Race operation | SpeedLimit reason RACE plus actual speed and clean referee lap | A 6 m/s ceiling does not mean 6 m/s was achieved |
| Localization | Unique inliers, correspondence ratio, fit RMS, reference pose error offline | Small residual can coexist with a wrong association |
| Sensor delivery | Maximum IMU/wheel arrival gap at evaluator | Observer arrival is not estimator callback timing; message age also matters |
| Startup | Windows phase NDJSON/JSON and Linux `startup.ndjson` | Separate simulator spawn, RPC readiness, camera verification, ROS discovery and first motion |
| Callback duration | Opt-in monotonic callback scopes | Not end-to-end image-to-actuation response time |
| CPU | Process CPU time delta / wall time | Linux 100% is one core; Windows host percentage is normalized across logical CPUs |
| GPU | Windows FSDS PID 3D-engine counters | Rendering load, not CUDA inference; Intel Arc cannot validate Jetson CUDA |
| RAM | Process RSS/private working set and host available memory | Shared pages and WSL/Docker VM accounting can overlap |

Ordinary evaluation records sampled geometry at about 5 Hz, snapshots at about
1 Hz, and the final map. Research mode adds map snapshots, per-arrival gaps and
callback samples. The recorder never publishes a driving command. Raw JSON,
build outputs, model weights and bags remain ignored to conserve storage.
Keep a compact hashed summary and selected figures for published evidence.

Acceptance for map reuse requires a fresh-spawn first lap, map freeze, unchanged
landmark IDs/coordinates, fresh correspondences, a referee-confirmed faster
subsequent lap, zero cone hits and no EBS. Also inspect reference corridor
clearance; zero cone hits alone can miss an off-track excursion. Preserve failed
trials and restart from spawn rather than clearing an EBS latch mid-run.
