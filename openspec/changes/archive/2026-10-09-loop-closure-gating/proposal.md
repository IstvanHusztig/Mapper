# Proposal

## Why

The user saw a ghost wall (a doubled wall) in `raw data/44_ao3_s0`, the first full run with `imu_always_on`. A revisit check measured it: for every 5 s stretch of the final cloud, the offset against the map recorded at least 40 s earlier (point-to-plane on 0.25 m voxels; job script `ghost.py`).
- `44_ao3_s0`: 51 cm at t = 275 s, the end of the recording back in the start room. Loop closure tried only one partner for that last segment: the earlier segment with the nearest centre (segment 3). The registration failed (30° correction) and was rejected, so the segment was never corrected. The other runs matched it to segment 1 or 2.
- `44_ao3_s1` and `s2`: 58 and 62 cm at t = 240 s. A false loop was accepted: segment 49 against 6 with a 3.05 / 3.21 m correction, while its neighbours 47, 48 and 52 agree on 1.6–2.0 m. The pose graph trusts the loop edges (0.05 m) more than the odometry between segments (0.1 m), so one false edge bends the trajectory by decimetres.
- The shipped settings on the corrected 44 reach 112 cm (12 stretches over 10 cm), and the always-on runs reach 20–60 cm. So the remaining ghost walls come from loop closure, not from the odometry.

This is re-check item 4.6 of `synchronized-retune` (loop-closure gating). Caves are the target use case: one false or missed loop in a long passage leaves a doubled wall.

## What Changes

- **More than one partner per segment:** when the registration against the nearest earlier segment is rejected, the next nearest is tried, up to `loop_closure_candidates` (3).
- **False loops dropped:** after the pose graph is solved, the loop edge with the largest disagreement between the solution and its measurement is dropped if that disagreement exceeds `loop_closure_max_residual_m` (0.15 m) or `loop_closure_max_residual_deg` (2°), and the graph is solved again, until all edges agree. The log names each dropped edge and lists the final residuals.
- New settings in `[loop_closure]`: `loop_closure_candidates`, `loop_closure_max_residual_m`, `loop_closure_max_residual_deg`.
- **Judged** like `imu-always-on`: three runs of the corrected 44 with the IMU shifted by +0, +1 and +2 ms (copies `44_s0..s2`) plus 45f, by the revisit offsets, the loops and their corrections, and the user's look at the cloud.

## Capabilities

### New Capabilities

- `loop-closure`: which revisits become loop edges, and when an accepted edge is dropped again.

### Modified Capabilities

None.

## Impact

- `src/application/loop_closure.{h,cpp}`, `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`, the golden TOML files, `tests/loop_closure_test.cpp`.
- Loop closure takes longer when registrations are rejected (up to three per segment instead of one); the pose graph has only about 58 nodes, so solving it again costs milliseconds.
- New baselines together with `imu-always-on` (both change the numbers of the full 44; 44_short is too short for loop closure); docs: the loop closure section and table in `docs/index.html` and `docs/hu/index.html`.
- Release only: the change cannot depend on the build configuration.
