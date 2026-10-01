# Tasks

## 1. Compare tool

- [ ] 1.1 Write `tools/compare_baseline.py` (standard library only): folder-vs-folder and folder-vs-manifest modes, the normalization list from design.md decision 1, one line per differing, missing or extra file, exit codes 0/1/2; verify it by comparing `raw data/44/lio_result_0` with a copy of itself (exit 0)
- [ ] 1.2 Check the negative cases on scratch copies: change one digit in a `trajectory_lio_*.csv` (reports that file, exit 1), delete one `scan_lio_N.laz` (reports it missing, exit 1), change only `elapsed time seconds`, the file-name timestamps and the output folder location (exit 0)
- [ ] 1.3 Add `--write-manifest` (dataset, params path, commit, `HD_CPU_OPTIMIZATION`, date, `{file: sha256}`); verify a manifest written from a folder makes that folder compare identical in manifest mode

## 2. Recording cut tool and the short dataset

- [ ] 2.1 Confirm the units of LAZ `gps_time` and of the IMU `timestamp` column against `load_point_cloud` / `load_imu` in `app/lidar_odometry_utils.cpp`, and record them in design.md decision 6
- [ ] 2.2 Write `tools/cut_recording/cut_recording.cpp` and its CMake target `cut_recording` (links LASzip only): `cut_recording <in_folder> <out_folder> <start_s> <duration_s>`, copies the LAZ header, points in range and the IMU header and lines in range, refuses an output folder with `.laz`/`.las`/`.csv`; verify both presets build it
- [ ] 2.3 Cut `raw data/44` from 0 to 30 s into `raw data/44_short`; verify the IMU file is the header plus a contiguous block of the input lines (byte compare), and that the first and last kept points' coordinates and GPS time match the input (spot check with a small `--verify` mode of the tool, or by reading both with LASzip)
- [ ] 2.4 Run the Release CLI on `44_short`; verify it completes and record the run time (target: about 3–4 min; if it is much longer, shorten the cut and note the duration in design.md)

## 3. Determinism check

- [ ] 3.1 Check in `app/lidar_odometry.cpp` whether the CLI reads `working_directory_cache` or only writes it; if it reads it, delete the input's `cache/` before each run. Record the finding in design.md
- [ ] 3.2 Run the Release CLI twice on `44_short` (into `44_short_det_a` and `44_short_det_b`) and compare them; verify identical (exit 0). If they differ, stop and report the first differing file to the user before going on
- [ ] 3.3 Run the Release CLI twice on `raw data/44` (into `44_det_a` and `44_det_b`, in the background) and compare them; verify identical (exit 0). If they differ, stop: find the first differing file and window, report it to the user, and land any fix in its own commit (checked with `eval_odometry.py` against `lio_result_0`) before repeating 3.2–3.3

## 4. Golden baselines

- [ ] 4.1 Rename `44_short_det_a` to `44_short_baseline` and `44_det_a` to `44_baseline`; write `tools/baseline/44_short.manifest.json` and `tools/baseline/44.manifest.json`; verify each `_det_b` folder and each baseline folder compares identical with its manifest (exit 0)
- [ ] 4.2 Delete both `_det_b` folders; document the cut tool, the two datasets, the workflow ("short cut on every commit, full 44 once per change") and whether `cache/` must be deleted first in `tools/README.md`; verify the documented commands run as written
- [ ] 4.3 Commit the compare tool, the cut tool, both manifests and the README in one commit

## 5. Unit-test harness

- [ ] 5.1 Vendor doctest's single header and LICENSE under `3rdparty/doctest/`; verify the header contains `DOCTEST_VERSION_STR`
- [ ] 5.2 Add `tests/CMakeLists.txt` (`lidar_odometry_tests`, `add_test`) and `enable_testing()` + `add_subdirectory(tests)` in the root `CMakeLists.txt` without touching `CORE_SOURCE_FILES`; verify both presets configure and build the target
- [ ] 5.3 Write the smoke test (`tests/main.cpp` + `tests/smoke_test.cpp`: TOML round trip of the default `LidarOdometryParams` in a temp dir, or the Eigen-only fallback from design.md decision 5); verify `ctest --test-dir out/build/x64-Release` and `ctest --test-dir out/build/x64-Debug` pass, and that a deliberately broken assertion makes ctest fail
- [ ] 5.4 Update `CLAUDE.md`: remove "There are no tests", add the commands to build and run the tests, the cut tool and the baseline check; mark step 0 as done in `docs/ARCHITECTURE.md` and add the short-cut rule there

## 6. Integration

- [ ] 6.1 Run `openspec validate golden-baseline-test-harness --strict` and confirm it passes
