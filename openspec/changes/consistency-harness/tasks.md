# Tasks

Who works: the verification region builds the generator, the truth tests and the tool (it never edits `src/`); the
noise-budget and solver regions make the two small diagnostics changes; the orchestrator owns `owners.toml`, the
merges, the runs and the summaries. Implementation on Opus / medium; reading results on Fable / high (the user is
told before each switch). At most two editing agents at once. Every number goes to the user without a verdict, after
the conclusion reviewer.

## 1. Groundwork (orchestrator)

- [x] 1.1 Branch `consistency-harness` made from `honest-inertial-carry` (ba0ec25); research merged and reviewed
  (`research.md`). Verify: `git log -1` on the branch; the review's corrections are in the file.
- [ ] 1.2 `docs/regions/owners.toml`: the new test files (inertial synthesis, in-memory recording source, truth
  statistics, truth test cases) under verification. Verify: `python tools/region_owners.py` passes.
- [ ] 1.3 Read (only) the recorder's source for the Mid-360S mount's axis map and write the lever arm in the
  recording's axes, with the source file and line, into `experiments/consistency/evidence/lever_arm.md`. Verify: the
  note exists and names its source; no settings file is changed.

## 2. The record and the readings (noise-budget, then solver; results unchanged)

- [ ] 2.1 Noise-budget: the handed-on state (end velocity, gravity vector and unit factor, offset covariance,
  velocity-offset covariance, gyro bias, the last node's pose as solved and as handed on) and the two features
  (mean rotation rate, world-frame acceleration over the first 30 ms) in the per-window record; new columns of
  `window_features.csv` after the existing ones. Verify: a unit test on a synthetic two-window run (the record
  equals what the second window's prior is built from); on both step-check captures `window_terms.csv` is the same
  bits and the old columns of `window_features.csv` have the same values; 44_short bit-identical.
- [ ] 2.2 Solver: the window problem keeps the start velocity's covariance block and its block with the offset
  (restored when a step is taken back); the verdict computes the normalised moves of D9 (start velocity, or start
  velocity and offset jointly; gyro bias) with the `NOT MEASURED` rules; summary lines with median, 95th percentile
  and coverage. The velocity chi2 untouched. Verify: known-answer case K5 (normalised move averages 1, the old
  statistic 0.5); verdict tests (reading present, absent, denominator below 0.5); the new blocks against a dense
  inverse; step checks ALL HOLD.
- [ ] 2.3 Orchestrator: both branches checked (ownership, step checks in the worktree) and merged; the step checks'
  table references re-recorded in their own commit; knowledge files (noise-budget, solver) and the two paragraphs of
  `CLAUDE.md` (per-term table, window verdict) say what was added. Verify: `tools/step_checks.py` on the main tree
  ALL HOLD with the new references.

## 3. The synthetic recording (verification)

- [ ] 3.1 Motion and truth: analytic, twice differentiable trajectories from segments (rest, speeding up, walking
  with the measured Livox 3 dynamics, turn, slowing down), with pose, velocity, acceleration and body rate at any
  instant; IMU samples from them (gyro and accelerometer as interval means, white noise, bias and offset random
  walks, seed). Verify: the strap-down test (noise-free samples reproduce the truth within the stated tolerance) and
  the noise-statistics test pass.
- [ ] 3.2 Scenes, routes and points: R1 (loop with pillars), R2 (out and back through a plain corridor with a blind
  middle), R3 (rest, walk, rest); timed points on the faces; the named deviations of D2 as single switches. Verify:
  points lie on the scene; each switch changes only its own quantity (a test per switch on the generated samples).
- [ ] 3.3 The in-memory recording source and the runner: a synthetic recording through `OdometryPipeline::load` and
  `run_odometry` with a keeping record sink; the truth aligned at the first node. Verify: the run completes without
  recorded data; in R2 the middle windows have an information ratio below the projection's lower threshold and the
  others above it; in R1 no window is blind.
- [ ] 3.4 Documentation of the generator: the Testing section of `CLAUDE.md` and the hand-back for
  `docs/regions/verification.md` (how to make a recording, the model world's substitutions and their reasons).
  Verify: every substitution of D3 is listed with its reason in the test file and in the hand-back.

## 4. Tier A and the known-answer cases (verification)

- [ ] 4.1 A1 (rest) and A2 (constant velocity): the floor, and the tolerances derived from it written in the test.
  Verify: both pass.
- [ ] 4.2 A3-A6 (constant acceleration, constant turn, walking, and the same with `handoff_tail_nodes = 0`): the
  honest outcome per case, marked as expected to fail with its finding where today's code deviates; for A3 a second
  case asserting the predicted sizes. Verify: `ctest` passes; the hand-back holds a table of each case's measured
  deviations next to the audit's predictions.
- [ ] 4.3 The hand-off against one long window (random walks zero): end state and covariance of two windows in
  sequence against the same nodes as one window. Verify: the case passes or is marked with its cause; both numbers
  in the hand-back.
- [ ] 4.4 Known-answer cases K1 (displaced first pose), K3 (prior over the gap), K4 (re-propagated tail), K6 (rest
  test with a tilted filter), K7 (fractional projection step), K9 (cell test and rotation). Verify: each passes or
  is marked as expected to fail with its finding's identifier; measured against predicted in the hand-back.
- [ ] 4.5 Orchestrator: the hand-back checked (ownership, step checks, conclusion reviewer on "which predictions
  held"), merged; the table to the user without a verdict. Verify: the table is in
  `experiments/consistency/tier_a.md` and the user has seen it.

## 5. Tier B (verification)

- [ ] 5.1 The truth statistics: normalised error per window and state, sums over runs, chi-square intervals, window
  classes from the route; tested on a linear chain with a textbook Kalman filter. Verify: the honest chain lies
  inside the interval, the same chain with a doubled measurement weight outside.
- [ ] 5.2 The small set in `ctest` (6 runs of a shortened R2): the blind-window check with both switches on, and
  positive control 1 (both switches off must fail). Verify: both behave as required; all truth cases together take
  at most one minute in Release (the time is reported); the loop-level cases are skipped in Debug.
- [ ] 5.3 The full set on demand (40 runs, R2 and R1): the table per class (mean normalised error per degree of
  freedom, share of windows inside their interval) for velocity, offset, both jointly and gyro bias; positive
  control 2 (noise three times stronger and weaker); K2 (exit windows) and K8 (the LiDAR term's claim against its
  scatter); the classes known to fall short marked as expected to fail with their finding and factor. Verify: one
  documented command runs it; the table and both controls are in the hand-back.
- [ ] 5.4 Orchestrator: hand-back checked and merged; the class table to the user without a verdict. Verify: the
  table is in `experiments/consistency/tier_b.md` and the user has seen it.

## 6. The tool (verification)

- [ ] 6.1 `tools/honesty_report.py` with tests in `tests/test_verification_tools.py` and a section in
  `tools/README.md`: the signature table of D8 from one or more `window_features.csv`, side by side, also as CSV.
  Verify: the tests pass; on `experiments/stage8/carry/livox3_efs_jc` it gives the audit's figures (move median
  23.3 mm/s, lag-1 autocorrelation -0.35, the quarters by rotation rate) within rounding.

## 7. Tier C and the evidence on recordings

- [ ] 7.1 Verification: tier C on demand: the model world and each deviation of D2 at its fitted size, one run of R1
  with walking each, written outside the repository; the readings of D8's last paragraph. Verify: one signature
  table with a row per world, next to the real rows, in `experiments/consistency/tier_c/`.
- [ ] 7.2 Orchestrator: the tool on the existing runs with tables (Livox 3: base, layer 1, layers 1+2, the gyro-noise
  run; the Livox 4 candidates; the 44_short table of the step checks). Verify: `experiments/consistency/evidence/
  existing_runs.md` holds the table.
- [ ] 7.3 Orchestrator: replays with `handoff_tail_nodes = 0` of the Livox 3 guard capture and the stage 8 Livox 4
  captures, through `tools/run_replay.py`. Verify: the tool's table before and after, in `evidence/tail_off.md`.
- [ ] 7.4 Orchestrator: one full Livox 3 run with the lever arm of 1.3 (its settings file in the run's folder, not
  in `settings/`), `verify-run` on its log, the tool on its table. Verify: the run folder holds settings, log,
  result and health table; the lever-arm regression before and after is in `evidence/lever_arm.md`.
- [ ] 7.5 Orchestrator: `experiments/consistency/summary.md`: tiers A to C, predictions against measurements, the
  evidence on recordings, what is not concluded; the conclusion reviewer; the numbers to the user without a
  verdict. Verify: the reviewer's corrections are applied and the user has seen the summary.

## 8. Decision and close

- [ ] 8.1 The user's decision: which findings become changes and in what order, and what happens to the layers of
  `honest-inertial-carry`. Verify: the decision is written into this file.
- [ ] 8.2 Knowledge files (verification, inertial, noise-budget, solver, lidar-matching) with the findings and
  experiments; `docs/code-review-tracker.md` entries for the new findings (F1, F2a, F2b, F3, F6, the cell test);
  the English guide where it describes the tests and the per-window readings. Verify: `python
  tools/region_owners.py` passes; every new finding has a tracker entry with its status.
- [ ] 8.3 Final check of "no result changes": `tools/step_checks.py` and `tools/end_runs.py` (full 44, Livox 3,
  44_short against their manifests). Verify: ALL HOLD and all identical.
