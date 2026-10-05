# Tasks

**Checks on every code commit:**
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- precise `44_short` is IDENTICAL with the precise reference;
- fast `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or else within `--tolerance 1e-6` of `raw data/44_short_baseline` (design decision 5).

Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes: a forward slash changes `working_directory_preview` in the results JSON.

## 1. Deterministic accumulation

- [x] 1.1 Add the multithread equality tests to `tests/scan_to_map_test.cpp` (design decision 5.1), on the synthetic room and corridor, with the points in scene order and shuffled:
  - one iteration and a 30-iteration optimization with `multithread = true` and `false` give exactly equal poses, gyro bias and translation information;
  - two multithreaded runs are equal.

  Expect the multithreaded cases to fail before 1.2; mark them `doctest::should_fail` with the cause, as the test rules say. Verify: build, `ctest`. Commit.

  Result: two cases in `tests/scan_to_map_test.cpp` (`run_window`, `identical`). Without the fix, all 10 checks failed in each of 3 runs, two multithreaded runs against each other included. Both cases are marked `should_fail`; Release build and `ctest` pass (the Debug build is checked with 1.2).
- [x] 1.2 In `optimize_lidar_odometry`, group the points by node in their original order and run the indoor, then the outdoor pass per node, with `std::execution::par` or sequentially (decisions 1–3). Remove the mutexes and `<mutex>`. Verify:
  - the 1.1 tests pass, and their `should_fail` markers go;
  - the step checks with `useMultithread = false`;
  - precise and fast `44_short` with a scratchpad copy of `lidar-vertical.toml` with `useMultithread = true` (and `log_stage_timing = true`): IDENTICAL apart from the params TOML, and IDENTICAL with the `false` run of the same build.

  If precise isn't identical, pause. Report the speed-up of the NDT phases and of the odometry. Commit.

  Result:
  - The points are grouped by node with a counting sort (`node_begin`, `points_by_node`) and run through `for_each_node`, with `std::execution::par` or sequentially; the mutexes are gone.
  - The equality tests pass in 5 of 5 runs, and their `should_fail` markers are removed.
  - Step checks with the setting off: both presets and `ctest` pass, and precise and fast `44_short` are IDENTICAL (11 files).
  - Setting on: precise and fast `44_short` differ from the manifests only in the params TOML, and they are IDENTICAL with the setting-off runs of the same build apart from it.
  - Speed (fast `44_short`, against the `stage-timing` profile): NDT indoor 6.27 → 1.76 ms per iteration (3.6×, including the grouping), outdoor 5.06 → 0.85 ms (6×); odometry 166.0 → 108.8 s (−34 %); whole run 200 → 144 s (−28 %).
- [ ] 1.3 `robust_lidar_odometry.cpp`: remove the stale `// ToDo fix for this case` and the unused `<mutex>` include. Verify: build (robust mode is off in `lidar-vertical.toml`, so it is only build-checked). Commit.

## 2. New default

- [ ] 2.1 Set `useMultithread = true` in `OdometryConfig` (and update its comment), `settings/default_settings.toml` and `settings/lidar-vertical.toml` (decision 4). Regenerate the golden TOML files with `LO_UPDATE_GOLDEN=1` in their own commit. Verify:
  - the step checks;
  - the `44_short` params TOML differs only in `useMultithread = true`;
  - update its hash in the 44 and 44_short manifests and the precise reference in the same commit.

  `44_short` (fast and precise) is then IDENTICAL. Commit.

## 3. Full run, GUI, docs

- [ ] 3.1 Run the full 44 in the background with the new default, `log_stage_timing` on through a scratchpad copy of the settings. Verify: IDENTICAL with `tools/baseline/44.manifest.json` apart from the params TOML, or within `--tolerance 1e-6` of `raw data/44_baseline`. Report the profile against `stage-timing`: odometry 1578.5 s, whole run 1990 s, NDT loops 737 s. If neither holds, pause.
- [ ] 3.2 GUI check by the user: load `44_short` in the GUI with `settings/lidar-vertical.toml`. Its `lio_result_<n>` must be IDENTICAL with the manifest. The user reports whether the window stays responsive during the run, and its time. No CLI job may run at the same time.
- [ ] 3.3 Docs:
  - `docs/PERFORMANCE.md`: a new fix entry (problem, fix, measured gain on `44_short` and 44, how it was checked) and its summary row; the `useMultithread` row of "Settings that trade speed" (default `true`, no cost to reproducibility); the analysis section's planned order (this step done).
  - `docs/index.html`: the `useMultithread` row and the "Results differ between runs" troubleshooting row.
  - `docs/ARCHITECTURE.md` decisions: the parallelism entry, and the bit-identical refactor rule, which no longer needs multithreading off.
  - `CLAUDE.md`: only if something it says changes.

  Verify: the numbers match the 1.2 and 3.1 reports. Commit.
