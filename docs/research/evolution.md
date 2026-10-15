# Evolution and experimental comparisons

[Research index](README.md)

| Earlier approach | Observed weakness | Current implementation / evidence boundary |
|---|---|---|
| Multiple oblique cameras | Added calibration/association complexity; no controlled advantage demonstrated | Supported runtime is one forward RGB camera |
| Cone box-height ranging | Striped/truncated boxes bias range | Ground-foot ranging on calibrated flat track, with uncertainty gates |
| Broad blue masks / restrictive yellow masks | Sky/road false positives and pale-yellow misses | Pre-morphology horizon masking and revised color/shape gates; targeted frame tests pass |
| Raw FSDS RPM integrated against wall time | Simulation/host clock mismatch stretches odometry | Wrapped wheel-angle differences over timestamps; front wheels avoid driven-wheel spin |
| Temporary missing corridor treated as fatal | Stops did not recover | Fresh empty path requests braking; fresh geometry may resume; true freshness failures still latch EBS |
| Unrestricted map-relative yaw feedback | October trial: 26.68° median heading error and missed estimated lap closure | Keep FSDS IMU heading fixed; translation-only learning-stage feedback is an unvalidated follow-up experiment |
| Proximity-only map association | Cross-color/repeated assignments and duplicates | Same-color, unique per-image matches; bounded temporal bearing refinement |
| Mapping continues after first lap | Same cones can be recreated as odometry drifts | Immutable post-lap map; detections used for correspondence instead |
| Global planner given local window / 40-point walk | Cannot reliably construct whole lap | Full-map input and whole-lap walk; isolated built-node circle regression passes |
| Minimum-curvature line accepted as faster | Curvature proxy is not a time objective | Compare periodic speed-model time against centerline; retain baseline if no candidate improves it |
| Upstream high-frequency RPC profile | Repeated stale IMU/wheel emergency stops | Lower-rate project-owned bridge profile under evaluation; watchdog limits unchanged |

These changes are not a controlled ablation study. Several layers changed
together, and host/rendering load varies. We can demonstrate particular bug
mechanisms and regression outcomes, not assign a percentage lap-time gain to
each patch. No YOLO-versus-classical accuracy benchmark has been performed.

Historical September clean laps (151.362 s, 157.341 s) are retained in the
[validation record](../validation.md). October failed runs are retained too;
see [current results](results.md). A passing synthetic test cannot replace a
fresh-spawn referee-confirmed simulator run.
