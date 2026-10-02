# Design

## Context

- **`struct WorkerData`** (`core/include/structures.h`, about line 398):
  - three cache paths (`std::filesystem::path`);
  - `intermediate_trajectory`, `intermediate_trajectory_motion_model` (`std::vector<Eigen::Affine3d>`);
  - `intermediate_trajectory_timestamps` (`std::vector<std::pair<double, double>>`, timestamp and unix timestamp);
  - `imu_om_fi_ka`, `imu_acc` (`std::vector<Eigen::Vector3d>`);
  - `gyro_bias` (`Eigen::Vector3d`), `gyro_bias_cov` (`Eigen::Matrix3d`);
  - `bool show`.

  `core/src/session.cpp` (`fill_session_from_worker_data`) still uses it; the app doesn't call that function.
- **Use counts in `app/`:** `WorkerData` 23, `worker_data` about 200, `intermediate_trajectory` 158, `intermediate_trajectory_timestamps` 36, `intermediate_trajectory_motion_model` 28, `imu_om_fi_ka` 12, `.gyro_bias`/`.gyro_bias_cov` 12 (only on window objects, in `lidar_odometry_utils_optimizers.cpp` and `folder_result_sink.cpp`).
  - Several of these names also exist as **plain identifiers**, which must not be renamed. For example, `optimize_lidar_odometry`'s parameters are called `intermediate_trajectory`, `intermediate_trajectory_motion_model` and `gyro_bias`; they belong to step 5b.
- **`Trajectory`** (`app/lidar_odometry.h`): the AHRS orientation track, filled by `calculate_trajectory`, read by `compute_step_1`, and stored in `OdometryPipeline::trajectory`.

## Goals / Non-Goals

**Goals:**
- `Window`, `GyroBiasState` and `ImuTrajectory` in a domain header that depends only on Eigen and the standard library.
- No app code uses `WorkerData`.
- Identical results; ideally bit-identical even on 44, since no arithmetic moves.

**Non-Goals:**
- `optimize_lidar_odometry`'s parameters and option structs (5b), and splitting the optimizer file (step 6).
- Moving `Point3Di`, `Imu` or `NDT` types into the domain (later, as the layers form in step 7).
- Removing `WorkerData` from `core` (`session.cpp` uses it; `core` changes only when a step needs it).

## Decisions

1. **`app/window.h`:**
   ```cpp
   // per-window gyro bias estimate [rad/s] and its covariance [(rad/s)^2], body frame
   struct GyroBiasState { Eigen::Vector3d mean = Eigen::Vector3d::Zero(); Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(); };

   // one optimization window: consecutive trajectory nodes (one per IMU sample) and what the odometry estimates for them
   struct Window {
       std::vector<Eigen::Affine3d> poses;              // optimized node poses (world)
       std::vector<Eigen::Affine3d> motion_model_poses; // the motion-model prior of each node
       std::vector<std::pair<double, double>> timestamps; // (timestamp [s], unix timestamp [s]) per node
       std::vector<Eigen::Vector3d> imu_angles;         // IMU orientation (om, fi, ka) [rad] per node
       std::vector<Eigen::Vector3d> imu_acc;            // raw accelerometer [g], body frame (empty if unknown)
       GyroBiasState gyro_bias;
   };

   // AHRS orientation track: IMU timestamp -> (pose matrix, unix timestamp)
   using ImuTrajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;
   ```
   The defaults match `WorkerData`'s (zero bias and covariance, empty vectors), and nothing relied on the removed members. Field order follows `WorkerData`'s, minus the removed ones.
2. **Renames by a token-aware script.** It edits code only, skipping comments and strings, and only member accesses (preceded by `.` or `->`):
   - `.intermediate_trajectory_motion_model` → `.motion_model_poses`;
   - `.intermediate_trajectory_timestamps` → `.timestamps`;
   - `.intermediate_trajectory` → `.poses`;
   - `.imu_om_fi_ka` → `.imu_angles`;
   - `.gyro_bias_cov` → `.gyro_bias.covariance`;
   - `.gyro_bias` → `.gyro_bias.mean`, but only where the owner is a window object. The script reports every hit, and I check them by hand, since only 12 exist.

   Plain identifiers:
   - `WorkerData` → `Window`;
   - `worker_data` → `windows`;
   - `worker_data_concatenated` → `chunks` (`save_result`'s output chunks);
   - `Trajectory` → `ImuTrajectory` (excluding `TrajectoryRow`);
   - the pipeline member `trajectory` → `imu_trajectory`.

   Each rename is checked by the compiler. A name the script misses, or renames wrongly, doesn't compile, or shows up in the `44_short` check.
3. **`compute_step_2`'s carried bias** (`carried_bias`, `carried_bias_cov`) becomes one `GyroBiasState carried_bias`, assigned from and to `window.gyro_bias`. Its arithmetic accesses `.mean`/`.covariance` the same way as before.
4. **Comments** that say "worker data" or `intermediate_trajectory` are updated in the same commit, so the names stay consistent in the code.
5. **Order of commits:** first `window.h` and the type switch (`WorkerData` → `Window`, member renames), then the variable renames (`worker_data` → `windows`, `trajectory` → `imu_trajectory`). Each is built and checked on its own.

## Risks / Trade-offs

- **[The script renames a parameter or local that only shares the name]** → member-only patterns for field names, and a review of every rename hit in `optimize_lidar_odometry`'s signature and body: its parameters keep their names until 5b.
- **[`/fp:fast` code generation]:** no expression changes. Struct layout changes (removed members) could in principle change register use. → `44_short` must be identical; the full 44 is checked against the manifest first, then with the 1e-6 tolerance.
- **[Merge conflicts with later work]:** this touches most app files. It's done in one go, with no parallel work on the app.
- **[`core` still has `WorkerData`]:** a reader may wonder which type to use. → `CLAUDE.md` says the app uses `Window`, and `WorkerData` exists only for `core/src/session.cpp`.

## Migration Plan

Two commits as in decision 5, each built in both presets with `ctest`, and checked on `44_short`. Then the full 44, the user's quick GUI run of `44_short`, and the docs. Rollback: revert.
