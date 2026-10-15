# Current test results and limitations

Publication review: **16 October 2026**. FSDS trials below were recorded on
15 October. This is an experimental implementation checkpoint, not a validated
race release.

## Latest recorded FSDS trial

Local run ID: `20261015_153155`, bounded duration 330 s, fresh TrainingMap spawn.
One RGB camera, IMU and wheel encoders; no LiDAR, stereo or reference geometry
in driving inputs.

| Measurement | Observed result |
|---|---|
| Referee lap time | 172.086746 s, one lap recorded |
| Referee down-or-out count at end | 1; this run fails the clean-lap gate |
| Map freeze | About 176.48 s into the evaluation |
| Frozen confirmed landmarks | 255 |
| Landmarks receiving triangulation refinement | 173, cumulative counter |
| Full-map race route | Rejected; planner retained local exploration mode |
| Maximum position error versus reference | 9.133 m |
| Median heading error versus reference | 0.002 degrees |
| Maximum observer IMU / wheel arrival gaps | 0.484658 s / 0.507861 s |
| Emergency stop | Stale IMU/wheel input, after map freeze |
| Faster clean second lap | Not demonstrated |

The zero-cone-hit historical September runs remain separate evidence in
[validation](../validation.md). Neither those runs nor successful synthetic
fixtures validate the new map-reuse behavior at racing speed.

Offline triangle-adjacency inspection of this frozen map found 246 blue/yellow
boundary entries and 232 candidate midpoints forming one open component. The
current runtime still uses beam-search midpoint ordering: adjacency-based
ordering has **not** been implemented. The diagnostic plot is generated with
`analyze_track_graph.py`; SciPy and Matplotlib are offline tool dependencies,
not new autonomy runtime dependencies.

The lower-rate bridge did not eliminate the sensor stalls. Average rates of
about 100 Hz IMU and 50 Hz wheels coexist with half-second gaps; averages are
not a freshness guarantee. Safety thresholds have not been relaxed.

## Configuration versus tested evidence

The recorded trial used `localize_while_mapping: false`, with IMU heading
locked. The pending experimental profile changes that setting to `true` for
translation-only corrections during learning. The launcher also adds lower
rendering quality and a 30 FPS cap to investigate simulator load. Neither of
those last two configuration changes has a subsequent FSDS result yet.

Unreal console-command launch syntax follows the
[official command-line reference](https://dev.epicgames.com/documentation/en-us/unreal-engine/unreal-engine-command-line-arguments-reference).
This establishes command syntax, not that reduced rendering resolves the stall.

The earlier unrestricted-yaw trial recorded a zero-hit referee lap but
26.68 degrees median heading error, failed estimated loop closure, and later
EBS. That prompted heading locking. An intervening locked-heading trial also
stopped on stale sensors before lap completion. Failed trials are not excluded
from the reliability assessment.

## Verification at this checkpoint

- Standalone C++ suite rerun successfully on 16 October: Delaunay structure,
  spline geometry, slot-map association, bearing triangulation, unique color
  correspondence, heading-locked registration and periodic speed constraints.
- Python compilation and PowerShell launcher parsing pass.
- Release ROS build and isolated built-node lifecycle/runtime regressions
  passed during the 15 October implementation session. The lifecycle fixture
  verifies frozen IDs/coordinates, whole-map planning on a synthetic circle,
  and race-cap removal after correspondence loss; it is not an FSDS lap test.
- No Jetson hardware, CUDA equivalence, physical vehicle or competition
  qualification test has been performed.

Remaining release blockers are a valid route from a learned map, reliable
correspondence through subsequent laps, sensor-delivery tail latency, and a
repeatable faster clean second lap. The implementation does not establish a
globally minimum achievable lap time.
