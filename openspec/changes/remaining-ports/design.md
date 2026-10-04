# Design

## Context

Where Fusion and TBB are used today (2026-10-04):
- **`src/application/input_ports.h`** includes `<Fusion.h>` because `using Imu = std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>>` is part of `RecordingSource::read_imu`. `FusionVector` holds three `float`s.
- **`src/infrastructure/recording_folder_source.cpp`** (`load_imu`) fills those tuples from the CSV: `double` values assigned to the `float` axes.
- **`calculate_trajectory`** (`src/application/lidar_odometry.cpp`) is the only AHRS user:
  - `FusionAhrsInitialise`, then `ahrs.settings.convention` from the first set flag of `fusionConventionNwu/Enu/Ned` (none set: Fusion's default, NWU), and `ahrs.settings.gain = ahrs_gain`;
  - with `use_remove_imu_bias_from_first_stationary_scan`: the gyro bias is the per-axis median of the first 1000 samples, or 0 when their variance exceeds 5;
  - per sample: `gain` is set to 0 while |acc| is outside 0.9–1.1 g and back to `ahrs_gain` otherwise; the first update uses dt = 1/200 s, later ones the timestamp difference, or 0.002 s when it isn't positive; the quaternion becomes the trajectory node's rotation;
  - with `debugMsg`, every 100th sample logs `FusionQuaternionToEuler`.
- **`compute_step_1`** reads each node's raw accelerometer from the same tuples.
- **TBB:** only `load_data`, with two `tbb::parallel_for` loops (IMU chunks, LAZ chunks). Each index writes its own slot, and the merge after them is sequential.
- **`std::execution::par_unseq`** in `scan_to_map`, `robust_lidar_odometry` and `consistency` runs only with `useMultithread` (off). It is the standard library, not TBB.
- **CMake:** `lo_application` gets the Fusion include directory `PUBLIC`, links `Fusion` `PUBLIC` and TBB (`PLATFORM_MISCELLANEOUS_LIBS`) `PRIVATE`. The test target adds the Fusion include directory itself.

## Goals / Non-Goals

**Goals:**
- `lo_application` includes and links neither Fusion nor TBB; a Fusion or TBB include there fails to compile.
- The orientation step is testable with a fake estimator, and loading with a serial executor.
- Results unchanged (precise build bit-identical).

**Non-Goals:**
- The `par_unseq` loops (see Context). Routing them through `ParallelExecutor` would mean reworking their shared accumulation; that is a behaviour change for its own proposal.
- Changing the AHRS itself, its settings or the TOML keys `fusionConvention*`.
- GUI globals (step 10), splitting `core` (step 11).

## Decisions

1. **`ImuSample`, a domain value type** (`src/domain/imu_sample.h`):
   ```cpp
   struct ImuSample {
       std::pair<double, double> timestamps; // the IMU timestamp and the unix timestamp [s]
       Eigen::Vector3f gyro;                 // as in the CSV
       Eigen::Vector3f acc;                  // [g]
   };
   using Imu = std::vector<ImuSample>;
   ```
   - **Float, on purpose.** `FusionVector` is float, and the CSV values are narrowed to float on reading today. `Eigen::Vector3f` keeps exactly those values, so every later expression sees the same numbers.
   - The struct has three members in the old tuple order, so the structured bindings (`const auto& [timestamp_pair, gyr, acc]`) stay; only `.axis.x` becomes `.x()`, and `std::get<0>` / `std::get<2>` become `.timestamps` / `.acc`.
   - `Imu` moves from `input_ports.h` to `imu_sample.h`; `input_ports.h` includes it.

   Alternative: `Eigen::Vector3d`. Rejected: the narrowing to float would move from reading to the AHRS call, and the bias median and the accelerometer per node would change in the last bits.

2. **The `OrientationEstimator` port** (`src/application/service_ports.h`):
   ```cpp
   enum class AxesConvention { Nwu, Enu, Ned };
   class OrientationEstimator {
   public:
       virtual ~OrientationEstimator() = default;
       virtual void reset(AxesConvention convention, float gain) = 0;  // a fresh filter
       virtual void set_gain(float gain) = 0;
       virtual void update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s) = 0;
       virtual Eigen::Quaternionf orientation() const = 0;             // w, x, y, z
   };
   ```
   - **The logic stays in `calculate_trajectory`:** the bias estimate, the gain switching, the time-step rules, the conversion to the trajectory. Those are the algorithm. The adapter only wraps the filter, so a fake estimator in a test sees exactly the calls the real one gets.
   - **`float` in the port** where Fusion takes `float` (gain, vectors, dt). The use case keeps today's expressions and conversions, character for character, including `static_cast<float>(gyr.x() * RAD_TO_DEG) - bias_gyr_x` (a double), narrowed to float when the vector is built, as the `FusionVector` initializer does today.
   - **`reset`** maps the flags as today: the first set of Nwu / Enu / Ned, else `Nwu` (Fusion's default). The adapter calls `FusionAhrsInitialise` and then writes `settings.convention` and `settings.gain` directly, as today; `set_gain` writes `settings.gain`.
   - **The debug Euler line:** `calculate_trajectory` computes roll, pitch and yaw from the port's quaternion with a file-local copy of `FusionQuaternionToEuler`'s float formula, so the log line stays the same. Fusion's `FusionAsin` clamps its argument; the copy does too.

   Alternative: the whole `calculate_trajectory` as the adapter. Rejected: then the bias and gain rules, which are ours, would sit in infrastructure, untestable without Fusion.

3. **The `ParallelExecutor` port** (same header):
   ```cpp
   class ParallelExecutor {
   public:
       virtual ~ParallelExecutor() = default;
       // runs body(i) for every i in [0, n), possibly in parallel; returns when all are done
       virtual void for_each_index(std::size_t n, const std::function<void(std::size_t)> &body) = 0;
   };
   ```
   - `TbbExecutor` calls `tbb::parallel_for(std::size_t(0), n, body)`.
   - The first loop in `load_data` uses `blocked_range` today. Its body touches only `imuChunks[i]`, so one index per call gives the same chunks.
   - Results don't depend on the schedule (one slot per index, sequential merge). Only the order of log lines between chunks can differ, as it does today.

4. **`RunServices`**, beside `RunInputs`:
   ```cpp
   struct RunServices {
       std::unique_ptr<OrientationEstimator> orientation;
       std::unique_ptr<ParallelExecutor> parallel;
       bool complete() const;
   };
   ```
   - `OdometryPipeline` takes it after `RunInputs` (default empty), and `load()` fails without it, as it does without inputs.
   - `make_run_services()` (`src/infrastructure/run_services.{h,cpp}`) builds the Fusion and TBB adapters.
   - The CLI and the GUI pass `make_run_services()` where they pass `make_folder_inputs(...)` today. The GUI's default-constructed pipeline stays empty.

   Alternative: put both into `RunInputs`. Rejected: they are not inputs of the run, and `make_folder_inputs` is about a folder.

5. **CMake.**
   - `lo_application`: no Fusion include directory, links only `lo_domain` (plus `PLATFORM_MISCELLANEOUS_LIBS` only if something besides TBB needs it, found out in the task).
   - `lo_infrastructure`: Fusion include directory and link, and TBB, all `PRIVATE`.
   - The test target drops the Fusion include directory.
   - **The check:** temporarily add `#include <Fusion.h>` and `#include <tbb/tbb.h>` to `run_odometry.cpp`. Both must fail to compile; then revert.

6. **Tests** (`tests/orientation_test.cpp`, serial executor in `tests/serial_executor.h`):
   - a recording fake `OrientationEstimator` (it records every call and returns a fixed rotation) on a hand-made `Imu`;
   - checks: one `reset` with the configured convention and gain; the gyro passed in degrees with the median bias removed (and no bias when the first 1000 samples move too much); the gain 0 while |acc| is outside 0.9–1.1 g; dt = 1/200 s first, then the timestamp difference, and 0.002 s for a repeated timestamp; one trajectory node per sample with the returned rotation;
   - `load_data` isn't unit-tested here (it needs a `RecordingSource`); the serial executor is checked to run every index once in order.

7. **Commit order**, each one green:
   1. `ImuSample` replaces the tuple (Fusion still called directly);
   2. the `OrientationEstimator` port, its adapter, `RunServices` and the wiring;
   3. the `ParallelExecutor` port and its adapter;
   4. CMake without the exceptions, with the check;
   5. the tests;
   6. the full 44, then the docs.

8. **Checks**, as in 7b: every code commit with the step checks (both presets build `dMapper` and `lidar_odometry_tests`, `ctest`, precise `44_short` IDENTICAL with the precise manifest from master, fast `44_short` IDENTICAL or within 1e-6), and the full 44 once (IDENTICAL with `tools/baseline/44.manifest.json` or within 1e-6 of `raw data/44_baseline`).

## Risks / Trade-offs

- **[A float/double conversion drifts]** The AHRS input is float, the bias is double, the trajectory double; one changed cast moves every result. → Decision 1 keeps float storage, decision 2 keeps each expression as written, and the precise `44_short` must stay IDENTICAL; any difference stops the task.
- **[Virtual calls change what `/fp:fast` and `/GL` inline]** → The precise build decides; the fast build only has to be within 1e-6.
- **[The progress counter while loading]** It is driven by the `RecordingSource`, not by the loop, so it is unchanged. → Checked once in the GUI or the CLI log.
- **[Something else in `lo_application` needs TBB]** e.g. through `core`'s headers. → The build fails at task 4; it is recorded and reported, not worked around.
