# Design

## Context

- **`optimize_lidar_odometry`** (`app/lidar_odometry_utils_optimizers.cpp`, around line 606) takes 35 parameters (listed in proposal.md). Its single caller is `compute_step_2` (around line 1870), inside the per-iteration loop. That call passes:
  - `params.*` directly for most settings;
  - the per-window `sigma_x_m`/`sigma_y_m`/`sigma_z_m`, which come from `params.lidar_odometry_motion_model_*_1_sigma_m` and are overridden with `imu_degenerate_translation_sigma_m` in degenerate windows;
  - `windows[i].poses`, `.motion_model_poses`, `.timestamps`, `.gyro_bias.mean`, `.gyro_bias.covariance`;
  - the per-window `use_prev_velocity` and `prev_velocity`, `window_rotation_options`, `translation_imu_options`, `acc_offset`/`acc_offset_cov`, `lm_factor` (changes per iteration), and `delta` (output).
- **Existing option structs:** `MotionModelRotationOptions` (+ `get_motion_model_rotation_options(params)`) and `MotionModelTranslationImuOptions` (filled in `compute_step_2`).
- **Locals that block the obvious names:**
  - `optimize_lidar_odometry` and `optimize_sf2` each have a local `std::vector<TaitBryanPose> poses`;
  - `optimize_rigid_sf`, `optimize_sf2` and `optimize_lidar_odometry` use `intermediate_trajectory`/`intermediate_trajectory_motion_model` as parameter names.
- **Baseline:** since the refresh (2026-10-03), the 44 manifest matches the current code bit for bit.

## Goals / Non-Goals

**Goals:**
- `optimize_lidar_odometry` with named option structs, the window passed as a `Window`, and about 15 parameters.
- No old HDMapping parameter names left in the optimizer functions' signatures.
- A unit test that guards the config-to-struct mapping.
- Identical results (`44_short` bit for bit; 44 bit for bit or within 1e-6).

**Non-Goals:**
- Splitting the optimizer file or changing any computation (step 6).
- `Consistency`, `Consistency2`, `optimize_icp` and the other optimizers' parameter lists.
- Moving `NDT::GridParameters` or `TaitBryanPose` into the domain.

## Decisions

1. **New file `app/optimizer_options.{h,cpp}`** (application/domain boundary; depends on `OdometryConfig`, Eigen, `NDT::GridParameters`, `TaitBryanPose`):
   ```cpp
   // scan-to-map settings of the LiDAR odometry, fixed for a run
   struct ScanToMapOptions {
       NDT::GridParameters grid_indoor, grid_outdoor;
       bool multithread = false;
       double min_distance = 0.0, min_distance_outdoor_grid = 0.0, max_distance = 0.0;
       bool use_planarity = false, use_norm = false, use_hierarchical_rgd = false, use_view_point_and_normal_vectors = false;
   };
   // motion model of a window: correction of the IMU frame and the sigmas of the consecutive-node and fix-origin priors
   struct MotionModelOptions {
       TaitBryanPose correction;
       double x_sigma_m, y_sigma_m, z_sigma_m, om_sigma_deg, fi_sigma_deg, ka_sigma_deg;
       double fix_origin_x_sigma_m, fix_origin_y_sigma_m, fix_origin_z_sigma_m;
       double fix_origin_om_sigma_deg, fix_origin_fi_sigma_deg, fix_origin_ka_sigma_deg;
       double acc_sigma_mps2;
   };
   // the previous window's final velocity as a prior for this one
   struct VelocityPrior { bool use = false; Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); };

   ScanToMapOptions get_scan_to_map_options(const OdometryConfig &config);
   MotionModelOptions get_motion_model_options(const OdometryConfig &config);
   ```
   - Scalars map one-to-one to today's parameters, not to `Eigen::Vector3d` sigmas, so each renamed use reads the same double as before, with no new indexing expressions.
   - The grids are **copied** into the struct, built once per run in `compute_step_2`, instead of being passed by reference to `params`. The values are identical.
   - `compute_step_2` builds `MotionModelOptions` once per run and sets `x/y/z_sigma_m` per window, as the local `sigma_*_m` do today. `get_motion_model_rotation_options` stays where it is.
2. **Signature:**
   ```cpp
   void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                                NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                                const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                                const MotionModelRotationOptions &rotation_options,
                                const MotionModelTranslationImuOptions &translation_imu_options,
                                const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                                Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                                Eigen::Matrix3d *lidar_translation_information = nullptr);
   ```
   The window's gyro bias is updated in `window.gyro_bias` (only with `estimate_bias`, as today).
3. **The body is changed only by renames,** with `tools/rename_identifiers.py --lines <function>`:
   - `intermediate_trajectory` → `window.poses`, `intermediate_trajectory_motion_model` → `window.motion_model_poses`, `timestamps` → `window.timestamps`, `gyro_bias` → `window.gyro_bias.mean`, `gyro_bias_cov` → `window.gyro_bias.covariance`;
   - `rgd_params_indoor` → `scan_to_map.grid_indoor`, and likewise for outdoor, `multithread`, the distances and the ablation flags;
   - `motion_model_correction` → `motion_model.correction`, and each `lidar_odometry_motion_model_*` → `motion_model.*`, `acc_sigma_mps2` → `motion_model.acc_sigma_mps2`;
   - `use_prev_velocity` → `velocity_prior.use`, `prev_velocity` → `velocity_prior.velocity`.

   The local `std::vector<TaitBryanPose> poses` doesn't collide: the window's poses are always reached as `window.poses`.
4. **`optimize_sf2` and `optimize_rigid_sf`:** parameters `intermediate_trajectory` → `trajectory`, `intermediate_trajectory_motion_model` → `motion_model_trajectory`, each with `--lines` of that function. Neither function uses `trajectory` yet; the tool's dry run confirms it before the change.
5. **Tool extensions** (`tools/rename_identifiers.py`):
   - `--lines A-B` (only with a single file) limits renames to that line range, counted before the rename;
   - `-r` accepts a dotted NEW (`window.poses`).

   The tool's `--help` and `tools/README.md` document both. Checked on a sample file with two functions sharing a name, before the tool is used on the optimizer.
6. **Test** (`tests/optimizer_options_test.cpp`): set every mapped `OdometryConfig` field to a distinct value, call both builders, and check each struct member, so a swapped sigma or flag fails. It compiles `app/optimizer_options.cpp` into the tests.

## Risks / Trade-offs

- **[A rename hits the wrong scope]:** for example `timestamps`, a common name. → `--lines` restricts it to the one function. The dry-run hit list is reviewed before every real run, and the compiler catches type mismatches (`window.timestamps` vs a local).
- **[`/fp:fast` code generation]:** the function's arguments now come from struct members, which can change register allocation and FMA contraction. → `44_short` must be identical. The full 44 is checked against the refreshed manifest; if it isn't bit-identical, the 1e-6 tolerance applies, as decided.
- **[Copying the grid parameters]:** a small struct copied once per run. No measurable cost.
- **[Default arguments]:** the `lidar_translation_information` default stays. No other defaults are added, so a missing option is a compile error.

## Migration Plan

Commits:
1. tool extensions (Python only);
2. `optimizer_options.{h,cpp}` + test;
3. the new signature, call site and body renames;
4. `optimize_sf2`/`optimize_rigid_sf` parameter renames;
5. the full 44 run and the docs.

Each code commit is built in both presets with `ctest`, and checked on `44_short`. Rollback: revert.
