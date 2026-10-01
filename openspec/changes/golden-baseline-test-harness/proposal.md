# Proposal

## Why

`docs/ARCHITECTURE.md` plans a step-by-step refactor toward Clean Architecture, and every pure
refactor step must reproduce the output of `raw data/44` bit for bit. Today there is no stored
baseline, no tool to compare two runs, no proof that two identical runs give identical output, and
no unit-test target. This is step 0 of the roadmap: without it no later step can show that it
changed nothing.

## What Changes

- **Determinism check:** run the Release CLI twice on `raw data/44` with
  `settings/lidar-vertical.toml` and compare the results. If they differ, find and fix the cause
  before anything else, without changing the result of a single run beyond that.
- **Golden baseline:** one baseline run of 44, stored **outside git** (it is about 224 MB of LAZ).
  The repo stores a small manifest: the dataset, config, commit and a SHA-256 hash of every
  normalized output file.
- **Compare tool** `tools/compare_baseline.py` (pure Python 3, no extra packages):
  - compares a new output folder against a baseline folder or against the manifest;
  - ignores only the fields that change from run to run (timestamps in file names, elapsed time,
    processing date, build date, output paths);
  - lists every file that differs and exits non-zero when anything differs.
- **Short reference cut `44_short`:** the first 30 s of `raw data/44`. 44 is a single recording
  (one LAZ, one IMU CSV, 280 s, about 30 min to process), so the cut is by time. A new
  `cut_recording` tool, built as a CMake target on the vendored LASzip, copies the LAZ points and
  IMU lines inside a time range unchanged. The cut has its own manifest
  (`tools/baseline/44_short.manifest.json`) and should process in about 3–4 min.
  - The short cut checks every intermediate commit.
  - The full 44 runs once at the end of each change, because only the full recording exercises
    loop closure.
- **Unit-test harness:** vendor doctest under `3rdparty/doctest`, add a `tests` CMake target (an
  executable `lidar_odometry_tests`, built by both presets and registered with CTest), and add a
  first smoke test that runs without data files.
- **Docs:** `tools/README.md` explains the baseline workflow. `CLAUDE.md` loses the line "there are
  no tests" and gains the test and baseline commands.
- No numerical behaviour change and no change to the app's output format.

## Capabilities

### New Capabilities
- `regression-baseline`: reproducible output of the pipeline for a fixed input and config, and a
  tool that verifies a new run against the stored golden baseline (bit-identical apart from the
  listed run-dependent fields).
- `recording-cut`: cutting a time range out of a recording (LAZ + IMU CSV) without altering the
  data that is kept, to make short reference datasets.
- `unit-tests`: a test executable that builds with both presets and runs without data files.

### Modified Capabilities
- None.

## Impact

- New: `tools/compare_baseline.py`, `tools/baseline/44.manifest.json`,
  `tools/baseline/44_short.manifest.json`, `tools/cut_recording/` (CMake target
  `cut_recording`), `3rdparty/doctest/`, `tests/` (CMake target and smoke test).
- Changed: root `CMakeLists.txt` (adds `tests`, `cut_recording`, `enable_testing()`), `tools/README.md`,
  `CLAUDE.md`.
- App code changes only if the determinism check finds a nondeterminism to fix.
- The baseline output lives next to the data
  (`C:\Users\User\Downloads\dLidar\raw data\`: `44_baseline`, `44_short`, `44_short_baseline`),
  not in the repo.
