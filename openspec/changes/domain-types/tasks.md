# Tasks

Every code commit: both presets build `dMapper` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Window type

- [x] 1.1 Add `app/window.h` (design decision 1: `GyroBiasState`, `Window`, `ImuTrajectory`) and add it to `SOURCES`. Verify both presets build.
- [x] 1.2 Switch the app from `WorkerData` to `Window` with the member renames (design decision 2, member-access patterns only): `poses`, `motion_model_poses`, `timestamps`, `imu_angles`, `gyro_bias.mean`, `gyro_bias.covariance`.
  - Check every `.gyro_bias` hit by hand.
  - `compute_step_2`'s carried bias becomes one `GyroBiasState` (design decision 3).
  - Remove `#include <structures.h>` uses that only served `WorkerData`, where nothing else needs them.
  - Update the affected comments.

  Verify:
  - a grep for `WorkerData|intermediate_trajectory_timestamps|intermediate_trajectory_motion_model|imu_om_fi_ka|gyro_bias_cov` in `app/` finds only `optimize_lidar_odometry`'s parameter names (left for 5b);
  - `.intermediate_trajectory` occurs nowhere as a member access.

  Run the step checks. Commit.

## 2. Variable names

- [x] 2.1 Rename `worker_data` → `windows` (including `OdometryPipeline::windows`, the GUI, `ResultSink::processing_results`' parameter), `worker_data_concatenated` → `chunks`, `Trajectory` → `ImuTrajectory`, and `OdometryPipeline::trajectory` → `imu_trajectory` (design decision 2). Update the comments.

  Verify that a grep for `worker_data|\bTrajectory\b` in `app/` finds nothing, apart from `TrajectoryRow`. Run the step checks. Commit.

## 3. Integration

- [ ] 3.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json` (expected, since no arithmetic changed), or else that `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes. Report which of the two.
- [ ] 3.2 GUI check, **done by the user**: one `44_short` run (Start → 1 → `lidar-vertical.toml`, Start → 2 → `44_short`); the preview and trajectory look as before. I verify that the result is IDENTICAL with the manifest.
- [x] 3.3 Docs:
  - `CLAUDE.md`: Pipeline (`windows`, `Window` in `app/window.h`), IMU rotation prior and gyro bias (`GyroBiasState`), and the note that `WorkerData` only remains for `core/src/session.cpp`;
  - `docs/ARCHITECTURE.md`: roadmap step 5 split into 5a (done) and 5b (option structs for `optimize_lidar_odometry`); §4, the `WorkerData` item is solved.

  Commit.
- [x] 3.4 Run `openspec validate domain-types --strict` and confirm it passes.
