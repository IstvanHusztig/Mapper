# Tasks

**Checks on every code commit** (design decision 7):
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- the precise build's `44_short` is IDENTICAL with the precise manifest from master;
- the fast build's `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline` (report the largest difference).

The scratchpad script `step_check.sh` runs all of them. Close the GUI before building. Every diff may hold only what design decision 4 allows.

## 1. NDT value types

- [x] 1.1 Add `core/include/ndt_types.h` with `NdtGridParameters`, `NdtBucket`, `NdtBucketCoef` and `NdtBucket2`, moved verbatim out of class `NDT` (decision 2), with `using` aliases left in `NDT`. Rename the app's and tests' uses (`NDT::GridParameters` → `NdtGridParameters`, and so on). In `decimate`, replace `PointCloud::PointBucketIndexPair` with a file-local struct of the same layout (decision 3). Verify: the step checks; `grep -rn "NDT::\|PointCloud::" app tests` finds nothing. Commit. Result: both presets build, `ctest` passes, precise and fast `44_short` IDENTICAL (11 files).

## 2. The split

- [x] 2.1 Create `app/ndt_map.h`, `app/odometry_config.h` and `app/odometry_state.h` with the moved declarations (decision 1). Rename `app/lidar_odometry_utils.cpp` to `app/ndt_map.cpp` with `git mv`, and update `CMakeLists.txt` and `tests/CMakeLists.txt`. `lidar_odometry_utils.h` keeps only its third-party includes and includes of the three new headers. Verify: the step checks; `git diff -M` shows the blocks as moves. Commit. Result: both presets build, `ctest` passes, precise and fast `44_short` IDENTICAL (11 files).
- [x] 2.2 Delete `app/lidar_odometry_utils.h`. Give each file that no longer compiles the includes it uses, and an `fs` alias where it uses `fs::` (decision 4). Verify: the step checks; `grep -rn "lidar_odometry_utils" app tests CMakeLists.txt` finds nothing; no app header includes `laszip`, `nlohmann`, `ndt.h`, `session.h` or `freeglut` unless it declares something that needs it (list any exceptions here with the reason). Commit. Result: both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files).
  - **The one exception:** `input_ports.h` still includes `Fusion.h`, because the `Imu` port type uses Fusion (roadmap step 9).
  - **`lidar_odometry.h`** no longer includes `session.h`, `export_laz.h`, `Version.hpp`, `<iomanip>` or `<sstream>`, since none of its declarations needs them; they moved into `lidar_odometry.cpp`.
  - **Headers that got their own includes:** `folder_result_sink.h` (`run_reporting.h`, for `Logger`) and `scan_to_map.h` (`ndt_map.h`, `window.h`, `run_reporting.h`).

## 3. Include check and full run

- [x] 3.1 Write the `/showIncludes` script (decision 6) in the scratchpad and run it on `scan_to_map.cpp`, `normal_equations.cpp`, `imu_kinematics.cpp`, `window_steps.cpp`, `ndt_map.cpp`, `optimizer_options.cpp` and `robust_lidar_odometry.cpp`. Verify: none of them includes LASzip, nlohmann json, GL/freeglut, `session.h`, `point_cloud.h` or Fusion, directly or transitively. Record each file's result here. A file that still does is reported with the include chain, and implementation pauses. Result: all 7 files pass:

  | File | Includes |
  |---|---|
  | `scan_to_map.cpp` | 525 |
  | `normal_equations.cpp` | 459 |
  | `imu_kinematics.cpp` | 493 |
  | `window_steps.cpp` | 462 |
  | `ndt_map.cpp` | 494 |
  | `optimizer_options.cpp` | 497 |
  | `robust_lidar_odometry.cpp` | 516 |

  The script was checked against two files that must fail, and it flagged both. Both chains are notes for 7b:
  - `run_odometry.cpp`: `run_odometry.h` → `input_ports.h` → `Fusion.h`, and its own `export_laz.h` → `session.h` → `point_cloud.h`;
  - `lidar_odometry.cpp`: the same `Fusion` chain, and `session.h` → `point_cloud.h`.
- [ ] 3.2 Run the full 44 once (Release, `settings/lidar-vertical.toml`, in the background or through a subagent). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline` (report the largest difference; the params file there is older). Record the result here.

## 4. Docs

- [x] 4.1 Update:
  - `CLAUDE.md`: the file names (`app/lidar_odometry_utils.*` → `ndt_map`, `odometry_config.h`, `odometry_state.h`) in the architecture section and the tests' source list;
  - `docs/ARCHITECTURE.md`: step 7 split into 7a (this change, done) and 7b (folders and libraries), with the finding that `ndt.h` pulls in GL;
  - any other doc that names `lidar_odometry_utils` (`grep -rn lidar_odometry_utils docs tools CLAUDE.md`).

  Verify: `grep -rn "lidar_odometry_utils" --include=*.md --include=*.html .` finds nothing outside `openspec/changes/archive`, and `openspec validate split-utils-header --strict` passes. Commit.

  Result: updated `CLAUDE.md` (tests' source list, `update_rgd`'s files, the Settings-and-state headers), `docs/ARCHITECTURE.md` (step 7 → 7a done and 7b), `docs/degeneracy-information.html` and `docs/index.html`. The grep still finds four mentions in `ARCHITECTURE.md`, and all of them are history: the ✅ problem list, the 6c entry (now marked "now `ndt_map.cpp`"), the 7a entry itself, and `lidar_odometry_utils_optimizers.cpp`, a different file deleted in step 6a. `openspec validate --strict` passes.
