# Setup and operations

Reviewed 16 September 2026. The supported demonstration is **FSDS**, not the
legacy Python kinematic simulation.

## 1. Existing workstation requirements

The root PowerShell launcher is currently workstation-specific:

| Component | Expected location / requirement |
|---|---|
| Simulator | `%USERPROFILE%\FSDS\FSDS.exe`, TrainingMap available |
| WSL distribution | `kali-linux`, root access, Docker daemon |
| Runtime image | Existing `fsd-test:latest`, Ubuntu 22.04 / ROS 2 Humble dependencies |
| WSL source workspace | `/root/fsd_ws` |
| External FSDS bridge | `/root/FSDS_repo/ros2/install` |
| Container mounts | Workspace → `/ws`; external bridge repo → `/fsds` read-only |
| Network | WSL-to-Windows RPC port 41451; dashboard port 8321 |
| Disk | Space for existing simulator/image/build plus bounded logs; no automated cache deletion |

The image and prebuilt bridge are local dependencies, not downloadable releases
of this repository. There is no checked-in Dockerfile that recreates
`fsd-test:latest`. A clean machine needs simulator installation, ROS dependencies
and bridge compilation first. The [workspace guide](../fsd_ws/README.md) covers
the autonomy build; use the
[official FSDS installation/bridge docs](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/ros-bridge/)
for external components. Do not copy x86 binaries to Jetson.

Recorded external revisions:

- FSDS repository: `ba7b1bdac76be3895b864ca917dfc844b344c762`.
- fs_msgs ROS 2: `4146e5b4889fb92332c9ce5ee42a8081649cbe72`.
- Local image ID: `ea6380ca72355efc60c8f3c43913c7881cfc91bc87497b81628c71693a4f0c0e`,
  Linux/amd64. An image ID is provenance, not a public registry pull address.
- WSL kernel: `6.6.87.2-microsoft-standard-WSL2`.

## 2. Launch and stop

From PowerShell at the repository root:

```powershell
.\run.ps1
.\run-nobuild.ps1
.\run-nobuild.ps1 -EvaluationSeconds 190
.\run-nobuild.ps1 -KeepSimulator
```

Choose one command, not concurrent instances. All launches:

1. Copy the single-camera settings to both the FSDS binary folder and the
   fixed user configuration directory.
2. Restart existing Blocks/FSDS processes to obtain a fresh spawn.
3. Wait for RPC, synchronize source/config/tools into ext4, preserve build/install.
4. Build unless SkipBuild is requested and an install exists.
5. Start the bridge and require an actual image, not merely a registered topic.
6. Start C++ autonomy and dashboard; auto-GO moves the simulated car.
7. On bounded completion or exit, clean up the owned stack/container and copy logs.

The dashboard opens at [localhost:8321](http://localhost:8321). Use one dashboard
tab when measuring performance; multiple polling tabs add load. The
`DashboardPort` launcher option is not currently forwarded to the ROS dashboard
node; use the default 8321 until that wiring is corrected.

Ctrl+C stops the stack and normally the simulator. `-KeepSimulator` retains
FSDS only, not ROS. An EBS is latched: restart from a fresh spawn rather than
clearing it mid-run. Do not run these scripts on hardware.

## 3. Logs and evaluation

WSL: `/root/fsd_ws/demo_logs/YYYYMMDD_HHMMSS/` with a `latest` symlink.
Windows: `fsd_ws/demo_logs/` contains copied latest files and may be overwritten
by the next run.

| File | Meaning |
|---|---|
| `bridge.log` | RPC, camera and simulator bridge failures |
| `stack.log` | Runtime startup, planner diagnostics, first EBS reason |
| `build.log` | Build output, possibly empty in no-build runs |
| `evaluation.json` | Only for bounded evaluation; samples, referee result, rates |
| `evaluation.log` | Evaluator progress/summary |

Always use the exact timestamped directory when comparing trials. Do not mix an
old Windows evaluation file with a newer ordinary run that did not produce one.

A completed launcher process can contain a failed lap. In the sourced ROS
environment, from `/ws` or `fsd_ws`:

```bash
python3 tools/summarize_fsds.py demo_logs/latest/evaluation.json --require-clean-lap
# Stronger check if reference geometry was captured for this same track/spawn:
python3 tools/summarize_fsds.py demo_logs/latest/evaluation.json \
  --reference artifacts/reference.json --require-clean-lap
```

The second command checks sampled body clearance as well as referee lap/cone
counts and EBS. Reference geometry never goes into the driving stack.

## 4. Bounded diagnostics

Run alongside an existing demo, inside its ROS environment:

```bash
python3 tools/capture_diagnostics.py --output artifacts/capture --seconds 10
python3 tools/evaluate_fsds.py --output artifacts/manual-evaluation.json --seconds 190
```

The image tool limits capture to 60 s and at most ten PNGs. Evaluation is capped
at 900 s, samples state near 5 Hz and detailed paths/detections near 1 Hz.

Offline reference capture requires Python `msgpack` and a reachable FSDS RPC
server; it publishes no controls:

```bash
python3 tools/capture_reference.py --host <observed-FSDS-host-IP> \
  --output artifacts/reference.json
```

Do not assume a remembered WSL host address. The runner obtains the default
gateway on each launch. A reference captured on another track is not valid for
clearance evaluation. Optional plots use matplotlib:

```bash
python3 tools/plot_fsds_evaluation.py --help
```

The dashboard has recording/replay controls. Raw image bags grow rapidly;
recording is not automatic in the ordinary demo. Never replay onto the live
FSDS or real-vehicle ROS domain. The bounded tools are preferred for disk-limited
debugging.

## 5. Verification commands

Without ROS:

```bash
bash fsd_ws/src/fsd_cpp/test/run_tests.sh
python3 -m compileall -q fsd_ws/src/fsd_stack fsd_ws/tools
bash -n fsd_ws/tools/run_fsds_demo.sh
```

Built-node regressions in the configured WSL container:

```bash
docker run --rm --net=host --ulimit core=0 \
  -v /root/fsd_ws:/ws -v /root/FSDS_repo:/fsds:ro \
  fsd-test:latest bash -lc '
    source /opt/ros/humble/setup.bash
    source /fsds/ros2/install/setup.bash
    source /ws/install/setup.bash
    python3 /ws/tools/test_runtime.py'
```

Tests explicitly use domain 91; ensure it is reserved for test traffic.
An optional `--frame /ws/artifacts/calibration/camera_00.png` checks a locally
captured stationary start frame; that image is not bundled.

## 6. Troubleshooting

| Symptom | Inspect / action |
|---|---|
| Launcher exit 141 before bridge startup | Use updated runner; old early-exit awk caused SIGPIPE with pipefail |
| Exit 1 | Read first actual failure in bridge/stack/build log, not just PowerShell's wrapper exception |
| RPC not available | Verify FSDS started, port 41451, Windows/WSL connectivity and firewall |
| Camera topic exists but no image arrives | Inspect camera RPC errors and both settings-file locations |
| Camera underground | Keep FSDS mount Z positive; verify source frame before calibration changes |
| No yellow / sky as blue | Check recorded image, exposure, calibration and HSV/geometry gates; not only map output |
| Stopped, DEGRADED, waiting for corridor | Planner has no valid local geometry; can resume when it returns |
| AS EMERGENCY / EBS | Find first EBS reason; restart only after diagnosis |
| “IMU or wheel data stale” | Reproduced blocker; inspect bridge/input timing and host load; do not loosen watchdog to hide it |
| Old behavior after source edit | Run `run.ps1`; no-build retains existing C++ binaries |
| Dashboard lap differs from referee | Dashboard uses an estimated crossing; evaluator/referee is authoritative |
| Slow frame rate | Separate simulator rendering, bridge transport, perception and diagnostic overhead |

The adapter's independent watchdog and dashboard port/network limitations are
documented in [architecture](../ARCHITECTURE.md) and [roadmap](roadmap.md).

## 7. Storage and access

Use `Get-PSDrive C` on Windows and `du -sh` on explicitly selected WSL project
directories to inspect growth. During the 16 September profiling run approximately
26 GB remained on C:. This is a historical snapshot, not current free space or
a minimum system requirement.

Current WSL build/install are approximately 52 MB / 3.9 MB; timestamped demo logs
approximately 19 MB. Docker images, simulator assets and WSL virtual disks are
separate and can be much larger. Deleting Linux files does not necessarily shrink
the Windows VHDX immediately.

Do not run broad Docker prune, recursive home/workspace deletion or remove the
known-working image to gain space. Archive identified old runs before removal.
No models, videos or new runtime images were downloaded for this documentation.

The dashboard binds `0.0.0.0` and has no authentication; recording/replay routes
can start processes. Do not expose it or the ROS/RPC interfaces to the Internet.
Restrict them to a trusted development network.
