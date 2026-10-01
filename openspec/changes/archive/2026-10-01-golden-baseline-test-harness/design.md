# Design

## Context

See proposal.md for the motivation. Observed facts that shape the approach:

- **Output of one run of 44** (`raw data/44/lio_result_0`, about 224 MB): 14 `scan_lio_N.laz`,
  `scan_lio_merged.laz` (83 MB), 14 `trajectory_lio_N.csv`, `session.mjs`,
  `session_poses.mrp`, `session_ini_poses.mri`, `point_sizes_per_chunk.json`,
  `HDMapping_params_<ver>_<YYYY-MM-DD_HH-MM>.toml` and `HDMapping_results_<ver>_<datetime>.json`.
  A run takes about 30 min in Release.
- **Run-dependent content:**
  - the date-time in the two `HDMapping_*` file names;
  - in `session.mjs`: `elapsed time seconds` and absolute paths of the output folder;
  - in the results JSON: `processing_info.processing_date`, `elapsed_time_seconds`, `build_date`,
    and `directory_info.*` paths;
  - in the params TOML: `build_date` and `[paths]` (the preview folder inside the input folder).
- **Parallelism:**
  - The `std::execution::par_unseq` reductions in `lidar_odometry_utils_optimizers.cpp` are gated
    by `useMultithread`, which defaults to `false` (its comment says parallel summation is not
    deterministic).
  - The two `tbb::parallel_for` loops in `load_data` (`app/lidar_odometry.cpp`) write each file
    into its own slot, so their result doesn't depend on thread scheduling.
  - So a run is *expected* to be deterministic, but this has never been checked.
- **Input-folder side effects:** the app writes `cache/` and `preview/` into the input folder and
  picks `lio_result_N` from `get_next_result_path`. The CLI takes an explicit output folder, so
  the baseline and compare runs use explicit folders.
- **Build:** `CMakeLists.txt` defines one executable. The `core` library is built with
  `WITH_GUI=1`. There is no `enable_testing()` and there are no tests.

## Goals / Non-Goals

**Goals:**
- Prove or establish determinism on 44, then freeze a baseline that later steps check against in
  one command.
- A test target that later steps can fill with real unit tests.

**Non-Goals:**
- No unit tests of the algorithms yet: they need the domain extraction from roadmap steps 1–6.
- No CI, no automated 30-minute run on every build.
- No change to output formats. In particular, the timestamps in file names are not removed here.
  They are normalized by the tool instead (a change to output is a separate decision).

## Decisions

1. **Normalize, then hash.** The tool reads each file and normalizes the known run-dependent
   fields. JSON and TOML are edited textually by key: the value is replaced with a placeholder,
   and paths are rewritten so that the output prefix becomes `<OUT>` and the input prefix becomes
   `<IN>`. The tool then hashes the result. The two `HDMapping_*` files are matched by the
   pattern `HDMapping_params_*.toml` and `HDMapping_results_*.json`, not by exact name.
   Every other file is hashed as raw bytes.
   *Alternative:* parse the files and compare numbers with a tolerance. Rejected: the
   architecture decision is bit-identical for refactors, and parsing would hide float drift.
2. **The manifest is the source of truth in git; the baseline folder is optional.**
   `tools/baseline/44.manifest.json` holds the dataset, config path, commit, date, and
   `{file: sha256}` for the normalized files (about 34 entries, a few KB). The full baseline folder
   (`raw data/44_baseline`) is kept next to the data so a mismatch can be diffed in detail.
   The tool accepts either a folder or a manifest as the reference.
   *Alternative:* Git LFS for the outputs. Rejected: 224 MB per baseline, and the input data isn't
   in git either.
3. **Tool interface:**
   ```
   python tools/compare_baseline.py <new_out> <baseline_dir | manifest.json>
   python tools/compare_baseline.py --write-manifest <out_dir> <manifest.json> --dataset 44 --params settings/lidar-vertical.toml
   ```
   Pure Python 3 standard library, like the existing tools. Exit code 0 means identical, 1 means
   different, 2 means a usage or input error.
4. **Determinism first, baseline second, short cut first.** The check runs on `44_short` first
   (fast feedback), then on the full 44. Two runs go to `44_det_a` and `44_det_b` and are
   compared with the tool (folder vs folder). Only if they match does `44_det_a` become
   `44_baseline` and its manifest get written. If they don't match, the cause is investigated per
   differing file (the first differing chunk points to the window). Any fix must keep the
   single-run result valid (check it with `eval_odometry.py`) and goes in its own commit before
   the baseline.
5. **doctest, vendored as the single header** `3rdparty/doctest/doctest/doctest.h` plus its
   LICENSE. `tests/CMakeLists.txt` adds the `lidar_odometry_tests` executable with
   `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` in `tests/main.cpp`. The root `CMakeLists.txt` gets
   `enable_testing()` and `add_subdirectory(tests)`, and the test is registered with
   `add_test`.
   The smoke test uses only header-level code that has no link-order traps: Eigen and
   `app/toml_io.h` round-trip a default `LidarOdometryParams` through TOML in a temp dir. If
   pulling `lidar_odometry_utils.h` into the test drags in GUI or link problems, the fallback is
   an Eigen/TBB-only smoke test. The test target must not touch the `CORE_SOURCE_FILES` link
   order.
6. **The short cut is made by a C++ tool on the vendored LASzip.** 44 is one recording
   (`lidar0001.laz` 87 MB, `imu0001.csv` with 249k lines over 280 s), so a subset of files isn't
   possible.
   - `tools/cut_recording/cut_recording.cpp` becomes a separate CMake target `cut_recording`
     that links only LASzip, not `core` or the app.
   - It copies the input LAZ header (point format, scale, offset) and writes only the points in
     range, so the stored integer coordinates are unchanged. It copies the IMU header and the
     lines in range verbatim.
   - The range is measured from the first IMU sample. Units, confirmed in
     `app/lidar_odometry_utils.cpp`: `load_imu` divides the IMU `timestamp` column (ns) by 1e9,
     and `load_point_cloud` uses the LAZ `gps_time` as-is, in seconds on the same clock. On 44,
     0–30 s keeps 26,307 IMU lines and 1,888,527 of 16,953,548 points (11 % of the points for
     10.7 % of the time).
   - The default cut is 0–30 s: at about 6.8× real time that should process in about 3–4 min,
     and it includes the stationary start that the IMU bias removal needs.

   *Alternatives:* CloudCompare's CLI (rewrites headers and may re-quantize), and Python
   (no LAZ support without extra packages). Both rejected.
   The short cut doesn't contain a loop, so it doesn't exercise loop closure. That's why every
   change still ends with one full-44 run.
7. **Runs use the CLI with explicit output folders outside the input folder**, so
   `get_next_result_path` numbering never interferes, and `useMultithread` stays at its default.

## Risks / Trade-offs

- [The run is not deterministic (uninitialized memory, `unordered_map` iteration feeding the
  optimizer, TBB inside `core`)] → Step 1 exists to find out. If the cause is deep, the fallback
  is to report it to the user and decide together (fix it, or accept a documented tolerance). The
  architecture doc's rule is not relaxed silently.
- [Each full run takes about 30 min] → Intermediate commits are checked on `44_short`
  (about 3–4 min). The full 44 runs once per change.
- [The short cut hides a regression in loop closure or in late windows] → The full-44 run at the
  end of each change catches it. The short cut is a fast filter, not a replacement.
- [Adding the `tests` target changes the app binary] → It is a separate target with no shared
  sources or flags, so there is no separate re-check. The next change's run on the short cut
  catches it.
- [Normalizing by key could hide a real change in a normalized field] → Only the listed keys are
  normalized. The list lives in one place in the tool and is printed with `--verbose`.
- [A `cache/` left in the input folder from an older build could change the result] → Checked:
  `compute_step_1` (`app/lidar_odometry.cpp`) rewrites every cache file of the run before step 2,
  loop closure, consistency or `save_result` read it, so stale files are never used and need no
  deleting. Two runs **on the same input folder at the same time** would overwrite each other's
  cache, so runs on one dataset are always sequential.
- [Compiler or flag changes break bit-identity without any logic change] → The manifest records
  the commit and `HD_CPU_OPTIMIZATION`. A toolchain change sets a new baseline in its own commit.

## Findings during implementation

- Determinism confirmed: two Release runs each on `44_short` (200 s) and on `44` (1,929 s and
  1,902 s) are identical on all files (11 and 35).
- **The GUI and CLI paths differ.** `run_loop_closure` is called only from the CLI branch of
  `main()` (`app/lidar_odometry_gui.cpp`). The GUI runs step 2 and, only when its button is
  pressed, consistency, so it never runs loop closure.
  - The GUI result `44/lio_result_0` (made 2026-10-01 12:30) matches the CLI baseline in the
    params, the results JSON (including `gyro_bias_per_window`) and `session.mjs`, so step 2 is
    identical.
  - Its trajectories and LAZ files differ by up to 0.42 m, because the CLI applies 20 loop
    closures.
  - The baselines are CLI runs. Closing this gap is exactly roadmap step 2 ("one pipeline, two
    front ends"). It is not fixed here because it changes the GUI's numbers.
- Re-running `cmake --preset` rewrites the C++20 module-scan files and makes Ninja rebuild every
  object. Build with `cmake --build --preset ...` alone (it reconfigures when needed).

## Migration Plan

Nothing to migrate. Rollback means deleting `tools/compare_baseline.py`, `tools/baseline/`,
`tests/`, `tools/cut_recording/`, `3rdparty/doctest/` and the CMake lines that add them.
