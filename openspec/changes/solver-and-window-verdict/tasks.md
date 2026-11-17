# Tasks

**Step checks, on every commit:**
- the Release build;
- the unit tests;
- the characterization guard, byte-identical with the reference recorded at `b234b6a`;
- the loop guard (task 2.1), byte-identical with its reference;
- the replays of the 44_short capture (20–40 s, 21 windows) and the Livox 3 capture (100–130 s, 84 windows), every
  window identical;
- 44_short against its manifest.

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the code. Result: plan section "Stage 4: solver
  and window verdict", 2026-11-17; ten review findings worked in.
- [x] 1.2 The user's decisions on the seven points at the end of the plan section. Verify: the plan's heading reads
  "Decisions (user, <date>)" with what was decided. Result: 2026-11-17, all seven as recommended.

## 2. Guards

- [x] 2.1 Record the characterization reference from `b234b6a`, twice. Add the loop guard to the tests (skipped;
  `--no-skip`): the synthetic loop recording through `optimize_windows` with `lm_step_check` off and on, a low
  iteration limit and a convergence threshold that ends some windows early; every window's poses, velocity, bias,
  iteration count and rejected steps as hexadecimal doubles. Verify: two recordings are identical; the file shows
  rejected steps and windows that ended early (change the fixture until it does); step checks pass. Result: 4ed2f46. Characterization reference recorded twice from b234b6a's code, identical with stage 3's. The loop
  guard (tests/window_capture_test.cpp) runs 4 variants; 111 and 121 steps rejected, 1 and 9 windows ending early;
  two recordings identical. WindowOutcome gained the rejected-step count (not in the capture file).

## 3. The solver unit (pure refactors)

- [x] 3.1 `src/domain/lm_solver.{h,cpp}` with the problem interface and `lm_iteration` (with a step, and without one);
  `src/domain/window_problem.{h,cpp}`; `optimize_lidar_odometry` builds a `WindowProblem` and calls `lm_iteration`.
  Tests on a small problem: one iteration with and without a step, the covariance handed over from the undamped
  matrix. Verify: step checks identical; the new tests pass. Result: adcaf1c. lm_solver.{h,cpp} with the problem interface, lm_iteration and (already) lm_solve; the damped
  solve and the covariance moved from window_solver; next_lm_factor from scan_to_map. tests/lm_solver_test.cpp
  (4 cases, also covering lm_solve). All checks identical.
- [x] 3.2 `lm_solve` (the loop, the stop cause, the callbacks for the time limit, the per-iteration record and the
  debug line); `run_lm_iterations` builds the problem and options and calls it; the solver's result replaces
  `WindowOptimization`. Tests on the small problem: the damping schedule, a refused step, convergence, the iteration
  limit, a forced last iteration, the time callback, the step check taking a step back, no state saved without it.
  Verify: step checks identical (the loop guard above all); the new tests pass. Result: d4714bf. WindowOptimization is LmResult plus the cost and the convergence record; all checks identical,
  the loop guard included.
- [x] 3.3 The scan-to-map tests call `lm_solve` instead of their own copy of the loop (`run_iterations`). Verify: every
  number those tests print or check is unchanged; if one changes, keep the copy and report the difference. Result: b1a15f5; every number the scan-to-map tests print is unchanged.

## 4. The window loop in four steps (pure refactor)

- [x] 4.1 `prepare_window`, `solve_window`, `report_window`, `finish_window`, in today's order, with the whole-window
  timer in the loop. Verify: step checks identical; `tools/function_sizes.py` lists no function of
  `run_odometry.cpp` above 80 lines. Result: 89fcc6d. LoopContext and WindowWork carry what the steps share; no function of run_odometry.cpp is above
  80 lines; all checks identical.

## 5. The window verdict

- [x] 5.1 `src/domain/window_verdict.{h,cpp}`: the evidence of a window, `judge_window` (readings and the four flags
  with the agreed limits; `NOT MEASURED` for a missing reading), the run summary (counts, shares, first and worst,
  coverage, the run's triggers, the readings' median / 95th percentile / largest with linear interpolation, the chi²
  statistics over the weak windows and over all). Tests on made-up evidence, as in the spec. Verify: the tests pass;
  step checks identical. Result: fc23ac4; 4 test cases pass; unit tests and both guards identical (nothing calls it yet).
- [x] 5.2 The verdict in the loop: the translation information in every run; the evidence built in `report_window`;
  the summary in `WindowLoopCarry` (not captured), logged at the end of the odometry, of a replay and of a cancelled
  run; setting `log_window_verdict` (`OdometryConfig`, `TomlIO::POINTERS` and the debug section's `CATEGORIES` entry,
  the three shipped settings files); the `window verdict <n>:` line; the columns in `window_features.csv`. Test: the
  synthetic recording gives the same poses with the setting on and off. Golden TOMLs in their own commit. Verify:
  step checks identical (44_short: only the params TOML differs); the stage 1 captures replay as "same settings";
  the odometry time of 44_short before and after written into this file. Result: 92102af, goldens 13dff5c. All checks identical; 44_short differs only in the params TOML. Odometry time
  of 44_short 109.5 s before and 115.4 s after (+5 %, within the spread of identical runs today: 109.5 / 110.1 s; to be
  confirmed on the full runs). On 44_short: bias high in 68 of 69 windows (the L2's gyro), velocity chi2 TRIGGER
  (weak windows median 189).
- [x] 5.3 Cross-check with `tools/run_health.py` on 44_short run with `log_window_diagnostics` and
  `log_window_verdict`: the bend, speed and bias counts and the weak windows' chi² statistics, with the windows that
  differ by the log's printed precision or on purpose listed. Verify: no unexplained difference; the comparison
  written into this file. Result: on 44_short the bent (0), bent severely (0), fast (0) and bias-high (any) results equal run_health.py's;
  no bend within 0.0005 deg of a limit. The chi2 populations differ by window 1 only (no accelerometer term: the log
  prints chi2 0 and run_health.py counts it; NOT MEASURED in the verdict): 26 weak windows, median 189.04 / p95 793.35,
  against 27, median 182.20 / p95 779.43.

## 6. Readings, docs, manifests

- [ ] 6.1 One full run each of 44, Livox 3 and Livox 4 with `log_window_verdict` and `log_window_terms` (sequential,
  in the background). A table for the user: per recording the flags' counts and the run's triggers, each reading's
  median / 95th percentile / largest, and for Livox 4 the flagged windows over time against its known bad stretches
  (100–125 s, 200–280 s). No limit is proposed. Verify: the table is shown to the user; their decision on limits is
  recorded in the plan.
- [x] 6.2 Docs: `CLAUDE.md` (solver, window problem, the four steps, the verdict and its setting), `docs/index.html`
  (the setting; the map of documents if a page is added), `tools/README.md` (the new feature columns), the plan's
  progress. Verify: the text matches the code. Result: 82c9208 (the guide's map gets no new page in this stage).
- [x] 6.3 Manifests (44, 44_short, Livox 3): the params TOML entry, in their own commit. Verify: the stage end runs
  compare IDENTICAL. Result: 4896eeb; 44 35 files, Livox 3 35, 44_short 11, all IDENTICAL.

## 7. End of stage

- [x] 7.1 Full 44 and Livox 3 against their manifests with the shipped settings. Verify: identical except the params
  TOML before task 6.3, IDENTICAL after it. Result: 44 (odometry 1214 s) and Livox 3 (697 s) and 44_short differ only in
  the params TOML before 6.3; IDENTICAL after it. The readings runs (with both logs on) differ only in the params TOML
  and the two extra CSV files.
- [x] 7.2 Debug build and `ctest`. Verify: pass. Result: Debug build exit 0, ctest 100 % passed (589 s).
