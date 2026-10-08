# Tasks

**Checks on every code commit:** the Release build of `dMapper` and `lidar_odometry_tests` passes and `ctest` passes; for task 1 `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json` (and the full 44 once, below). For tasks 2 and 3 the numbers are allowed to move by rounding: `44_short` is compared within `--tolerance 1e-6` of `raw data/44_short_baseline` and the movement is reported; the baselines are replaced once, in task 4.

Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes.

## 1. Measure

- [ ] 1.1 Add per-phase timers to `Consistency` (the phases listed in the proposal), summed over the passes of a run and logged by `run_consistency` after the last pass when `log_stage_timing` is on, as `timing consistency <phase> N s`. Reuse `PhaseTime` and `PhaseTimer` from `src/domain/phase_timing.h`; the totals travel in a small struct passed down like `ScanToMapTiming` (default `nullptr`). Verify: `ctest`; `44_short` IDENTICAL with and without the setting. Run the full 44 with the setting on (scratchpad copy of the settings) and report the profile of the 10 passes. If the profile contradicts the guess in the proposal, update the proposal and the plan of tasks 2 and 3 before going on. Commit.

## 2. Direct accumulation

- [ ] 2.1 In `Consistency`, replace the triplet lists and the sparse products by direct accumulation (proposal, task 2): NDT term per node in parallel in point order into a `BandedNormalMatrix` and a dense right-hand side, smoothness and fix-origin terms added to the same matrix, then `to_sparse()` and the existing `SimplicialCholesky`. Remove the mutex and the hard-coded `multithread` flag of this function. Tests: `tests/consistency_test.cpp` still passes with its recorded numbers (the cases that run `run_consistency` on the synthetic loop recording, including the `should_fail` ones, which must keep failing by the same amounts within rounding); a new case compares the new normal equations with a direct dense evaluation of `Σ JᵗPJ` and `Σ JᵗPδ` on a small synthetic point set, bit for bit not required but within 1e-9 relative. Verify: `ctest`; `44_short` within `--tolerance 1e-6` of the current baseline (report the largest difference); report the timing phases and the consistency time. Commit.

## 3. Points between passes, chunks in parallel (only if task 1 or 2.1 shows them to matter)

- [ ] 3.1 Decide from the measurement: if loading the points, the chunk preparation or `update_rgd` is at least about 15 % of the consistency time after 2.1, implement (a) the points loaded once per run, and (b) the chunks built in parallel, their partial sums merged in chunk order; otherwise mark this task skipped with the numbers. Verify as in 2.1, and the result must not depend on the number of threads (run twice, bit-identical with each other). Commit.

## 4. Full run, baselines, docs

- [ ] 4.1 Full 44 in the background with the shipped settings. Verify: within `--tolerance 1e-6` of `raw data/44_baseline`, `eval_odometry.py` metrics against the old baseline unchanged to the printed precision, and a repeat of `44_short` bit-identical with itself. Then refresh the baselines from the final code (old folders renamed with a date suffix, both manifests rewritten). Report the whole-run and consistency times. Commit.
- [ ] 4.2 Docs: `docs/PERFORMANCE.md` (a fix entry with the table and the checks, the plan, the run times), `docs/performance.html` and `docs/hu/performance.html` (the stage table, the "what is left" table, the chart, the card numbers), `CLAUDE.md` and `tools/README.md` only if run times they state change. Verify: the numbers match the 1.1, 2.1, 3.1 and 4.1 reports. Commit.
