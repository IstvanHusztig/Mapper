# Proposal

## Why

Roadmap step 5 in `docs/ARCHITECTURE.md` introduces the domain model. The user split it into two changes (2026-10-02). This one, **5a**, introduces the domain types; **5b** will give `optimize_lidar_odometry` option structs.

Today the central data type of a run is `WorkerData`, from `core/include/structures.h`:
- Its names come from HDMapping's code: `intermediate_trajectory`, `intermediate_trajectory_motion_model`, `intermediate_trajectory_timestamps`, `imu_om_fi_ka`.
- It still carries three cache-path fields, unused since step 4a, and a GUI `show` flag, unused since the GUI cleanup.
- The gyro bias is two loose members, `gyro_bias` and `gyro_bias_cov`.
- The AHRS orientation track is a bare alias, `Trajectory = std::map<...>`. The name suggests the optimized trajectory, which is something else.

Step 6 (splitting the optimizer into domain services, with unit tests on synthetic data) needs types whose names say what they hold, and that don't depend on `core`'s structures.

## What Changes

- **New domain header** `app/window.h`:
  - `struct GyroBiasState { Eigen::Vector3d mean; Eigen::Matrix3d covariance; }`, the per-window gyro bias estimate.
  - `struct Window`: one optimization window of the run:
    - `poses` (was `intermediate_trajectory`);
    - `motion_model_poses` (was `intermediate_trajectory_motion_model`);
    - `timestamps` (was `intermediate_trajectory_timestamps`);
    - `imu_angles` (was `imu_om_fi_ka`);
    - `imu_acc`;
    - `gyro_bias` (a `GyroBiasState`; was `gyro_bias` + `gyro_bias_cov`).

    There are no cache paths and no `show` flag.
  - `using ImuTrajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>`: the AHRS orientation track per IMU timestamp (was `Trajectory`).
- **All app code uses `Window`:**
  - `std::vector<WorkerData> worker_data` becomes `std::vector<Window> windows`, including `OdometryPipeline::windows`, which the GUI reads;
  - `OdometryPipeline::trajectory` becomes `imu_trajectory`;
  - local copies of the gyro bias in `compute_step_2` (`carried_bias`, `carried_bias_cov`) become one `GyroBiasState`.
- **The renames are mechanical:** a token-aware script renames member accesses and variables, and the compiler checks every use. No expression, evaluation order or arithmetic changes.
- **`WorkerData` stays in `core`**, whose `session.cpp` still uses it. The app no longer includes or uses it.
- **Unchanged:** every output file (`44_short` bit-identical; 44 expected bit-identical too, otherwise within `--tolerance 1e-6`), file and setting names, the CLI and GUI.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- None. This is a pure refactor with no behaviour change, so the change sets `skip_specs: true`.

## Impact

- **Code:**
  - new `app/window.h`;
  - changed `app/lidar_odometry.{h,cpp}`, `app/lidar_odometry_utils.h`, `app/lidar_odometry_utils_optimizers.cpp`, `app/loop_closure.{h,cpp}`, `app/odometry_pipeline.{h,cpp}`, `app/output_ports.h`, `app/folder_result_sink.{h,cpp}`, `app/lidar_odometry_gui.cpp`, `app/input_ports.h` (the `Imu` alias stays there).
- **Docs:** `CLAUDE.md` (Pipeline, IMU rotation prior and gyro bias sections), `docs/ARCHITECTURE.md` (roadmap: step 5 split into 5a done and 5b; §4 `WorkerData` problem solved).
- **Not touched:** `core/` (including `WorkerData` and `session.cpp`), the optimizer's parameters (5b), the outputs.
- **Validation:** `44_short` on every commit, the full `44` at the end. There's no GUI behaviour change, so the user's GUI check is a quick run of `44_short`.
