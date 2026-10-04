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
- [ ] 2.2 Delete `app/lidar_odometry_utils.h`. Give each file that no longer compiles the includes it uses, and an `fs` alias where it uses `fs::` (decision 4). Verify: the step checks; `grep -rn "lidar_odometry_utils" app tests CMakeLists.txt` finds nothing; no app header includes `laszip`, `nlohmann`, `ndt.h`, `session.h` or `freeglut` unless it declares something that needs it (list any exceptions here with the reason). Commit.

## 3. Include check and full run

- [ ] 3.1 Write the `/showIncludes` script (decision 6) in the scratchpad and run it on `scan_to_map.cpp`, `normal_equations.cpp`, `imu_kinematics.cpp`, `window_steps.cpp`, `ndt_map.cpp`, `optimizer_options.cpp` and `robust_lidar_odometry.cpp`. Verify: none of them includes LASzip, nlohmann json, GL/freeglut, `session.h`, `point_cloud.h` or Fusion, directly or transitively. Record each file's result here. A file that still does is reported with the include chain, and implementation pauses.
- [ ] 3.2 Run the full 44 once (Release, `settings/lidar-vertical.toml`, in the background or through a subagent). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline` (report the largest difference; the params file there is older). Record the result here.

## 4. Docs

- [ ] 4.1 Update:
  - `CLAUDE.md`: the file names (`app/lidar_odometry_utils.*` → `ndt_map`, `odometry_config.h`, `odometry_state.h`) in the architecture section and the tests' source list;
  - `docs/ARCHITECTURE.md`: step 7 split into 7a (this change, done) and 7b (folders and libraries), with the finding that `ndt.h` pulls in GL;
  - any other doc that names `lidar_odometry_utils` (`grep -rn lidar_odometry_utils docs tools CLAUDE.md`).

  Verify: `grep -rn "lidar_odometry_utils" --include=*.md --include=*.html .` finds nothing outside `openspec/changes/archive`, and `openspec validate split-utils-header --strict` passes. Commit.
