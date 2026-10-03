# Proposal

## Why

`app/lidar_odometry_utils_optimizers.cpp` is 2,922 lines. It contains six unrelated parts:
- the least-squares helpers;
- the robust-mode optimizers (`optimize_sf2`, `optimize_rigid_sf`);
- the scan-to-map optimizer (`optimize_lidar_odometry`);
- the IMU kinematics helpers;
- the window loop (`compute_step_2`);
- the two consistency passes.

The IMU helpers are pure math, but they sit in an anonymous namespace, so no test can reach them. This is roadmap step 6 of `docs/ARCHITECTURE.md`. The user split it into three changes on 2026-10-03:
- **6a (this change):** split the file and add the first tests on synthetic data;
- **6b:** break `compute_step_2`'s window loop into named steps;
- **6c:** a synthetic scan-to-map test of `optimize_lidar_odometry`.

## What Changes

- **The file is split into one file per service, as pure moves.** No function body changes; only includes, `static`/anonymous-namespace linkage and declarations change.
  - `app/normal_equations.{h,cpp}`: `Blocks`, `SumBlocks`, `pose_band_to_sparse`.
  - `app/imu_kinematics.{h,cpp}`: `exp_so3`, `rotation_of`, `relative_rotation`, `remove_gyro_bias_from_rotations`, `world_acceleration`, `observed_acceleration_offset`, `is_window_stationary`.
  - `app/scan_to_map.{h,cpp}`: `optimize_lidar_odometry`.
  - `app/robust_lidar_odometry.{h,cpp}`: `optimize_sf2` and `optimize_rigid_sf`, the optimizers of `use_robust_and_accurate_lidar_odometry`.
  - `app/consistency.{h,cpp}`: `Consistency`, `Consistency2`.
  - `app/run_odometry.{h,cpp}`: `compute_step_2`, which is what remains of the old file. It is renamed with `git mv`, so its history is kept. The function keeps its name until 6b.
  - `MotionModelRotationOptions`, `MotionModelTranslationImuOptions` and `get_motion_model_rotation_options` move from `lidar_odometry_utils.h` and the optimizer file to `app/optimizer_options.{h,cpp}`, next to the other option structs.
  - The double-precision `DEG_TO_RAD`/`RAD_TO_DEG` of the old file move to `app/angle_units.h`.
- **Declarations:** they move from `app/lidar_odometry_utils.h` to the new headers, and callers include what they use.
- **First tests on synthetic data** (doctest, no recorded data):
  - `tests/imu_kinematics_test.cpp`;
  - `tests/normal_equations_test.cpp`;
  - a `get_motion_model_rotation_options` case in `tests/optimizer_options_test.cpp`.
- **Full 44 baseline refresh, before any code change.** `max-prior-velocity` changed `settings/lidar-vertical.toml` and was not checked on the full 44 (user decision). This change runs master on the full 44 first, so that the refactor is compared against a known reference.
- **Docs:** `CLAUDE.md`, `docs/ARCHITECTURE.md` (step 6 split into 6a/6b/6c, 6a done), `docs/PIPELINE.md` and the `where` lines of `docs/index.html` follow the new file names. `CLAUDE.md` also loses a stale mention of `optimize_icp`, a function that does not exist in the app.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. This is a pure refactor: no spec-level behaviour changes, so the change sets `skip_specs: true`.

## Impact

- **Code:** `app/lidar_odometry_utils_optimizers.cpp` (removed by the split), `app/lidar_odometry_utils.h` and `app/optimizer_options.{h,cpp}`, 12 new files in `app/`, plus the callers `app/odometry_pipeline.cpp`, `app/lidar_odometry.cpp` and the GUI if needed, and `CMakeLists.txt`.
- **Tests:** two new test files. The tests compile `app/imu_kinematics.cpp` and `app/normal_equations.cpp` directly; they still don't link `core`.
- **Results:**
  - `44_short` must stay bit-identical on every commit.
  - The full 44 must be identical with the refreshed manifest, or within `--tolerance 1e-6` of the refreshed baseline folder.
- **Baselines:** `raw data/44_baseline` and `raw data/44_short_baseline` are replaced by runs of current master. `tools/baseline/44.manifest.json` is rewritten only if the limit of `max-prior-velocity` changed the 44 result.
