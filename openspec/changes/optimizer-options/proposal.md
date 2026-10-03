# Proposal

## Why

Roadmap step 5b (step 5 was split by the user on 2026-10-02; 5a, the domain types, is done). `optimize_lidar_odometry` (`app/lidar_odometry_utils_optimizers.cpp`), the core of the odometry, takes **35 positional parameters**:
- 14 sigmas as separate doubles;
- 4 ablation flags;
- the two NDT grids and maps;
- distances, the multithread flag;
- five fields of one window passed one by one (`poses`, `motion_model_poses`, `timestamps`, gyro bias mean and covariance);
- the velocity prior, the acceleration-offset state, the logger, and an output pointer.

Its only caller, `compute_step_2`, passes them in a 37-line call. Mixing up two neighbouring doubles would compile and silently change the results.

Step 6 (splitting the optimizer into domain services and unit-testing them on synthetic data) needs a signature a test can fill in by name. The optimizer helpers `optimize_sf2` and `optimize_rigid_sf` still use the old HDMapping parameter names (`intermediate_trajectory`, `intermediate_trajectory_motion_model`).

## What Changes

- **Option structs** in a new `app/optimizer_options.{h,cpp}` (included by `app/lidar_odometry_utils.h`), each with a builder from `OdometryConfig`:
  - `ScanToMapOptions` (per run): the indoor and outdoor NDT grid parameters, `multithread`, `min_distance`, `min_distance_outdoor_grid`, `max_distance`, and the four ablation flags (`use_planarity`, `use_norm`, `use_hierarchical_rgd`, `use_view_point_and_normal_vectors`). Builder: `get_scan_to_map_options(config)`.
  - `MotionModelOptions` (per window): `correction` (`TaitBryanPose`), `x/y/z_sigma_m`, `om/fi/ka_sigma_deg`, `fix_origin_x/y/z_sigma_m`, `fix_origin_om/fi/ka_sigma_deg`, `acc_sigma_mps2`. Builder: `get_motion_model_options(config)`; `compute_step_2` overrides the position sigmas per window, as today.
  - `VelocityPrior`: `use`, `velocity`.
- **New signature, 15 parameters:**
  ```cpp
  optimize_lidar_odometry(points, Window &window, buckets_indoor, buckets_outdoor,
                          const ScanToMapOptions &, const MotionModelOptions &,
                          const MotionModelRotationOptions &, const MotionModelTranslationImuOptions &,
                          const VelocityPrior &, double lm_factor, double &delta,
                          Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &, Eigen::Matrix3d *info)
  ```
  The window's poses, motion-model poses, timestamps and gyro bias are read and written through `window`. The existing `MotionModelRotationOptions` and `MotionModelTranslationImuOptions` stay as they are.
- **Inside the function, only names change.** Each old parameter name becomes its new place, for example `intermediate_trajectory` → `window.poses` and `lidar_odometry_motion_model_x_1_sigma_m` → `motion_model.x_sigma_m`. The renames are done with `tools/rename_identifiers.py`, confined to the function's lines. No expression is reordered.
- **`optimize_sf2` and `optimize_rigid_sf`:** their parameters become `trajectory` / `motion_model_trajectory`. They can't become `poses`, which is already a local there.
- **`tools/rename_identifiers.py` gets two small features:**
  - `--lines A-B` limits the renames to a line range of one file (here, one function);
  - `-r` may rename to a dotted name (`window.poses`).
- **Unchanged:** the arithmetic, the outputs (`44_short` bit-identical; 44 bit-identical against the refreshed baseline, or within `--tolerance 1e-6` if `/fp:fast` moves the last bits), the settings and the CLI/GUI.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- None. This is a pure refactor with no behaviour change, so the change sets `skip_specs: true`.

## Impact

- **Code:**
  - new `app/optimizer_options.{h,cpp}` (the structs and builders);
  - `app/lidar_odometry_utils.h` (the include, new declaration);
  - `app/lidar_odometry_utils_optimizers.cpp` (`optimize_lidar_odometry`, `optimize_sf2`, `optimize_rigid_sf`, the call in `compute_step_2`);
  - `tools/rename_identifiers.py` and `tools/README.md`.
- **Tests:** `tests/` gets a test that the builders copy every config field to the right member, the guard against swapped sigmas. It compiles only `app/optimizer_options.cpp`.
- **Docs:** `CLAUDE.md` (the option structs), `docs/ARCHITECTURE.md` (5b done; §4 "about 40 positional arguments" solved).
- **Not touched:** `core/`, `Consistency`/`Consistency2` (step 6), the outputs.
- **Validation:** `44_short` on every commit, the full 44 at the end. The GUI isn't affected, so no GUI check is needed.
