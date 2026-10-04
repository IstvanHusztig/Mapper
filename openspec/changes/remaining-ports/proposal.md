# Proposal

## Why

Roadmap step 9. After step 7b the layer libraries enforce the dependency rule, with two listed exceptions: `lo_application` sees Fusion (the AHRS, and `FusionVector` inside the `Imu` port type) and links TBB (`parallel_for` in `load_data`). `docs/ARCHITECTURE.md` plans an `OrientationEstimator` and a `ParallelExecutor` port for them. With both behind ports, the application layer has no third-party dependency left besides the math kernel, and the IMU orientation step can be tested with a fake estimator.

## What Changes

- **IMU samples become a domain type.** `Imu` (today `std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>>`) becomes `std::vector<ImuSample>`. `ImuSample` (`src/domain/imu_sample.h`) holds the timestamp pair, the gyroscope and the accelerometer as `Eigen::Vector3f`: float, as `FusionVector` is, so every value stays bit-identical. The CSV reader fills it, and the infrastructure no longer needs Fusion for it.
- **`OrientationEstimator` port** (`src/application/service_ports.h`): reset with an axes convention and a gain, set the gain, update with gyroscope, accelerometer and time step, and read the orientation. `calculate_trajectory` keeps its logic (gyro bias from the first stationary samples, the gain dropped while the acceleration is away from 1 g, the time-step rules) and talks to the port. The adapter `FusionOrientationEstimator` (`src/infrastructure`) wraps `FusionAhrs`, with the same calls in the same order.
- **`ParallelExecutor` port** (same header): run a body for every index of `0..n-1`. `load_data`'s two `tbb::parallel_for` loops use it. The adapter `TbbExecutor` (`src/infrastructure`) calls `tbb::parallel_for`. The tests get a serial executor.
- **`RunServices`** (`orientation`, `parallel`) is passed to `OdometryPipeline` next to `RunInputs`. `make_run_services()` (`src/infrastructure`) builds the Fusion and TBB adapters, and the CLI and the GUI call it.
- **CMake:** `lo_application` loses the Fusion include directory and the TBB link; `lo_infrastructure` gets them. A Fusion or TBB include in the application then fails to compile (checked once, as in 7b). The test target drops its Fusion include directory.
- **Tests:** `calculate_trajectory` with a recording fake estimator (bias removal, gain switching, time steps, orientation into the trajectory), and the serial executor.
- **No behaviour change.** Results, outputs and settings stay the same. The precise build must stay bit-identical; the normal build IDENTICAL or within 1e-6.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None: a pure refactor (`skip_specs`). No main spec names Fusion, TBB or the IMU tuple. The `run-progress` scenario "Parallel loading" is about the loading counter, which doesn't change.

## Impact

- **Code:** `src/application/input_ports.h`, `lidar_odometry.{h,cpp}`, `odometry_pipeline.{h,cpp}`; new `src/domain/imu_sample.h`, `src/application/service_ports.h`, `src/infrastructure/{fusion_orientation_estimator,tbb_executor,run_services}.{h,cpp}`; `recording_folder_source.{h,cpp}`; `src/presentation/cli/cli.cpp` and `lidar_odometry_gui.cpp` (one call each); `CMakeLists.txt`, `tests/CMakeLists.txt`, a new test file.
- **Docs:** `CLAUDE.md` (Layers without the exceptions, pipeline step 2, the ports), `docs/ARCHITECTURE.md` (step 9 done).
- **Not in this change:**
  - the `std::execution::par_unseq` loops in `scan_to_map`, `robust_lidar_odometry` and `consistency`. They are standard library, not TBB, and run only with `useMultithread`, which is off because their accumulation isn't deterministic. Putting them behind the port would mean restructuring that accumulation, a behaviour change of its own;
  - renaming the `fusionConvention*` settings (TOML keys stay);
  - the GUI globals (step 10) and splitting `core` (step 11).
