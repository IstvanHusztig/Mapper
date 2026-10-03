# Tasks

**Checks on every code commit:**
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`.

If the `/fp:fast` build moves last bits, the precise-build rule of `window-loop-steps` applies instead (design decision 6). Close the GUI before building.

## 1. Setting and helper

- [x] 1.1 Add `bool log_iteration_convergence = false;` to `OdometryConfig` (`app/lidar_odometry_utils.h`), the `POINTERS` entry and the `lidar_odometry_debug_info` category in `app/toml_io.h`. Add `log_iteration_convergence = false` to `app/default_settings.toml` and `settings/lidar-vertical.toml`. Regenerate the golden TOML test files with `LO_UPDATE_GOLDEN=1`, in their own commit. Verify: the step checks; the params TOML of the `44_short` output gains only this key, so update the manifests' params hash in the same commit, as `max-prior-velocity` did. Commit. Result: `44_short` differed only in the params file. Its hash `15d6c9f0` → `d61da98f` is updated in the 44, 44_short and precise manifests; `44_short` is then IDENTICAL (11 files). The baseline folders still hold the old params file, so a `--tolerance` comparison against them will flag that one file.
- [x] 1.2 Add `IterationConvergence` and `convergence_of(poses_after, deltas)` to `app/window_steps.{h,cpp}` (design decision 3). Tests in `tests/window_steps_test.cpp`:
  - poses that settle at a known iteration give that N_final;
  - deltas that cross 1e-2 at a known iteration give that N_fast;
  - the error at N_fast is the largest node difference from the final poses;
  - a short (cut) sequence is measured against its last entry.

  Verify: build, `ctest`. Commit.

## 2. Logging in the window loop

- [x] 2.1 In `run_lm_iterations`, when the setting is on, record the poses after each iteration and the deltas, and fill `WindowOptimization::convergence` with `convergence_of` (decision 2). Add the step `log_iteration_convergence`, which prints the per-window line after `log_window_result`, and collect the values in `WindowLoopCarry`. Verify: the step checks with the setting off; then `44_short` with the setting on (a scratchpad copy of `lidar-vertical.toml`) is IDENTICAL with the manifest, and its log has 70 `convergence window` lines. Commit. Result: setting off IDENTICAL (11 files). Setting on: 10 of 11 files identical; only the params TOML differs, because it records `log_iteration_convergence = true`. 70 lines.
- [x] 2.2 The end-of-run summary in `finish_run` (decision 4), only with the setting on. Verify: the `44_short` run with the setting on prints it; its numbers match a hand count from the 70 lines. Commit. Result (`44_short`): settled after median 52, p90 74, max 83. Delta below 1e-2 after median 19, p90 43 (in all 70 windows). The error at that point is median 0.0126 mm, p90 0.051 mm, max 0.174 mm. The hand count over the 70 lines gives the same values. Setting off: IDENTICAL. Committed together with 2.1.

## 3. Tool, measurement, docs

- [x] 3.1 Add `tools/convergence_summary.py` (decision 5) and its section in `tools/README.md`. Verify: on the 2.2 `44_short` log it prints both histograms and the ten largest errors, consistent with the summary line. Commit.
- [ ] 3.2 Run the full 44 in the background with the setting on (decision 7). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, and 690 `convergence window` lines. Report the distribution and the odometry time against the last 44 run (1567 s). If the result isn't identical, pause.
- [ ] 3.3 Docs:
  - `docs/PERFORMANCE.md`: the 44 distribution next to the synthetic numbers, and what it suggests for `nr_iter` and `convergence_delta_threshold`, with no setting changed;
  - `docs/index.html`: the setting's row in the debug table;
  - `CLAUDE.md`, only if something it says changes.

  Verify: the numbers in `PERFORMANCE.md` match the 3.2 report. Commit.
