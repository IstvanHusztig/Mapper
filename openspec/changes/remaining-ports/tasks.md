# Tasks

**Checks on every code commit** (design decision 8), run by the scratchpad's `step_check.sh`:
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- the precise build's `44_short` is IDENTICAL with the precise manifest from master (made again first if master's source changed since the last one);
- the fast build's `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline` (report the largest difference).

Close the GUI before building. A precise `44_short` difference stops the task: find the changed conversion (design risk 1), don't move on.

## 1. Ports

- [x] 1.1 `ImuSample` (decision 1): add `src/domain/imu_sample.h` with `ImuSample` and `Imu`; `input_ports.h` uses it instead of the Fusion tuple. Update `load_imu` / `read_imu` (`recording_folder_source`), `calculate_trajectory` and `compute_step_1` (`.x()`, `.timestamps`, `.acc`); Fusion is still called directly in `calculate_trajectory`, now from `Eigen::Vector3f` values. Verify: the step checks; `recording_folder_source.{h,cpp}` and `input_ports.h` no longer include `<Fusion.h>`. Commit. Result: `ImuSample` holds `Eigen::Vector3f` gyro and acc; the reader assigns the CSV doubles to them as it did to the `FusionVector` axes. `input_ports.h` and `recording_folder_source.{h,cpp}` no longer include `<Fusion.h>`. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files).
- [x] 1.2 `OrientationEstimator` (decisions 2 and 4): add `src/application/service_ports.h` with `AxesConvention`, `OrientationEstimator` and `RunServices`; `FusionOrientationEstimator` and `make_run_services()` in `src/infrastructure`. `calculate_trajectory` takes an `OrientationEstimator&` instead of calling Fusion, with each expression kept as written; the debug Euler line uses a file-local copy of Fusion's formula. `OdometryPipeline` takes `RunServices`; the CLI and the GUI pass `make_run_services()`. Verify: the step checks; `grep -rn "Fusion" src/application` finds only comments. Commit. (The GUI check moved to 1.3: both tasks change `load()`, so one GUI run after 1.3 covers them.) Result: `calculate_trajectory` calls `reset`, `set_gain`, `update` and `orientation` where it called Fusion; each gyro value is built as before (a float times `RAD_TO_DEG`, minus the double bias, narrowed to float), and dt and the gain are narrowed to float at the call as before. The only "Fusion" in `src/application` is the comment on the debug Euler copy. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files).
- [ ] 1.3 `ParallelExecutor` (decision 3): add it to `service_ports.h` and `RunServices`, with `TbbExecutor` in `src/infrastructure`; `load_data` takes a `ParallelExecutor&` instead of `tbb::parallel_for`. Verify: the step checks; `grep -rn "tbb" src/application` finds nothing; the loading counter in the CLI log looks as before; the GUI loads and processes `44_short` once (by the user), and its result matches the CLI's (`compare_baseline.py`). Commit. Result so far: `load_data` runs both loops through `ParallelExecutor::for_each_index` (the IMU loop one index per call instead of TBB's ranges; each index still fills only its own slot). `TbbExecutor` calls `tbb::parallel_for`, and `lo_infrastructure` links TBB. No `tbb` left in `src/application`. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files); the loading lines of the CLI log are as before. **Open: the GUI run.**

## 2. Enforcement and tests

- [ ] 2.1 CMake (decision 5): `lo_application` without the Fusion include directory, the `Fusion` link and TBB; `lo_infrastructure` with them, `PRIVATE`; the test target without the Fusion include directory; the comments above the libraries updated. Then the check: temporarily add `#include <Fusion.h>` and `#include <tbb/tbb.h>` to `src/application/run_odometry.cpp`; both must fail to compile (record the error text); revert. Verify: the step checks after the revert. Commit.
- [ ] 2.2 Tests (decision 6): `tests/serial_executor.h` and `tests/orientation_test.cpp` with the recording fake estimator and the listed checks. Verify: both presets build, `ctest` passes (report the new cases' run time in Debug). Commit.
- [ ] 2.3 Run the full 44 once (background or subagent). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline` (report the largest difference). Record the result here.

## 3. Docs

- [ ] 3.1 Update:
  - `CLAUDE.md`: Layers (`lo_application` without exceptions; `lo_infrastructure` has Fusion and TBB), pipeline step 2 (`calculate_trajectory` through `OrientationEstimator`), the ports (`service_ports.h`, `RunServices`, `make_run_services`), Testing (the new test file);
  - `docs/ARCHITECTURE.md`: step 9 done, and §5's library line for `lo_application`; the `par_unseq` loops named as what remains of the parallelism decision.

  Verify: `grep -rn -i "fusion\|tbb" CLAUDE.md docs/ARCHITECTURE.md` shows only the new wording and history, and `openspec validate remaining-ports --strict` passes. Commit.
