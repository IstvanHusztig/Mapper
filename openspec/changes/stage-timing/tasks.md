# Tasks

**Checks on every code commit:**
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`. If the `/fp:fast` build moves last bits, design decision 6 applies: the precise `44_short` stays bit-identical with the precise reference, and the fast build stays within 1e-6.

Close the GUI before building. Never run two jobs on the same input folder at once.

## 1. Setting

- [x] 1.1 Add the setting and its golden files.
  - Add `bool log_stage_timing = false;` to `OdometryConfig` (`src/domain/odometry_config.h`), next to `log_iteration_convergence`.
  - Add its `POINTERS` entry and its place in the `lidar_odometry_debug_info` category in `src/infrastructure/toml_io.h`.
  - Add `log_stage_timing = false` to `settings/default_settings.toml` and `settings/lidar-vertical.toml`.
  - Commit the pending `docs/PERFORMANCE.md` "Analysis (2026-10-05)" section with this task.
  - Regenerate the golden TOML test files with `LO_UPDATE_GOLDEN=1` in their own commit.

  Verify: the step checks. The `44_short` output's params TOML gains only this key; update its hash in the 44 and 44_short manifests and the precise reference manifest in the same commit. `44_short` is then IDENTICAL. Commit.

  Result: both presets built, `ctest` passed. Precise and fast `44_short` differed only in the params TOML, which gained the line `log_stage_timing = false`. Its hash `3bee6e2c` → `5c242431` is updated in the 44 and 44_short manifests and the precise reference; both runs are then IDENTICAL (11 files).

## 2. Timers

- [x] 2.1 Add `src/domain/phase_timing.h` with `PhaseTime`, `ScanToMapTiming` and `PhaseTimer` (design decision 2). Add a test in `tests/` showing that:
  - a `PhaseTimer` on a null target does nothing;
  - on a target, it adds one call and a non-negative time;
  - two timers on the same target add up.

  Verify: build, `ctest`. Commit.

  Result: `tests/phase_timing_test.cpp`, three cases; both presets build and `ctest` passes.
- [x] 2.2 Add the parameter `ScanToMapTiming *timing = nullptr` to `optimize_lidar_odometry`, and time its eight phases (decision 3) without reordering any statement. Verify:
  - the step checks with the setting off;
  - the precise `44_short` is bit-identical;
  - the synthetic scan-to-map tests still pass unchanged.

  Commit.

  Result: eight `PhaseTimer`s, started and stopped around the existing statements; no statement moved. Checked together with 2.3 and 2.4: both presets and `ctest` pass, and with the setting off, precise and fast `44_short` are IDENTICAL (11 files), the fast build too.
- [x] 2.3 In `run_odometry.cpp`, add `OdometryTiming` to `WindowLoopCarry`. When the setting is on:
  - `run_lm_iterations` passes the `ScanToMapTiming`;
  - the window loop times `point_load`, `lm_iterations`, `add_to_map`, `preview` and `window_total`;
  - `finish_run` logs the breakdown (decision 4).

  Verify:
  - the step checks with the setting off;
  - `44_short` with the setting on (a scratchpad copy of `lidar-vertical.toml`) matches the manifest, apart from the params TOML, and prints the breakdown;
  - the LM phases add up to `lm_iterations` within about 1 %.

  Commit.

  Result: with the setting on, precise and fast `44_short` differ from the manifests only in the params TOML (`log_stage_timing = true`). The breakdown is printed. The total is measured with `steady_clock`, like the phases; a first version used the `system_clock` total of the "elapsed time" line, which was 3 s shorter than the phases on one run, giving a negative `other`.

  Deviation: the eight named LM phases add up to 93 % of `lm_iterations`, not 99 %. The rest (11.9 s of 165.5 s on `44_short`) is logged as `lm other`. It is the loop in `run_lm_iterations` plus the end of each `optimize_lidar_odometry` call, after the `update` timer has stopped, where the call's matrices (the dense 37 MB normal matrix among them) are freed. That is read from the code; it isn't timed separately.
- [x] 2.4 Add the per-step lines in `odometry_pipeline.cpp` (decision 5). Verify: the `44_short` run with the setting on prints one line per step, and the steps add up to the CLI's total elapsed time within about 1 %. Results are as in 2.3. Commit.

  Result: one line per step (`load_data` 1.0 s, `calculate_trajectory` 0.0 s, `compute_step_1` 0.4 s, `odometry` 166.0 s, `loop_closure` 0.2 s, `consistency` 30.8 s, `save` 1.8 s). Up to the save, they add up to 199.4 s against the 198.5 s "Processing time" (+0.5 %). Overhead: 44_short took 200 s off and 200 s on (fast), 206 s off and 203 s on (precise), which is within run-to-run noise.

## 3. Measurement and docs

- [ ] 3.1 Run `44_short` with the setting off, then with it on, and report:
  - the profile;
  - the overhead (on vs off).

  Then run the full 44 in the background with the setting on. Verify: the output matches `tools/baseline/44.manifest.json`, apart from the params TOML. Report the profile and the odometry time against the last 44 run (1562–1567 s). If the result isn't identical, pause.
- [ ] 3.2 Docs:
  - `docs/PERFORMANCE.md`: the measured profile (44 and `44_short`), what it confirms or contradicts in the 2026-10-05 analysis (especially the share of the dense-matrix setup), and the "Planned order" updated from it;
  - `docs/index.html`: the setting's row in the debug table, next to `log_iteration_convergence`;
  - `CLAUDE.md`: only if something it says changes.

  Verify: the numbers in `PERFORMANCE.md` match the 3.1 report. Commit.
