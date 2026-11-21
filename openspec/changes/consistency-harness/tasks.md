# Tasks

Who works: the verification region builds the exact control, the generator, the fixtures, the loop measurement and
the tool (it never edits `src/`); the noise-budget and solver regions make the two diagnostics changes; the
orchestrator owns `owners.toml`, `tests/CMakeLists.txt`, the shared loop steps, the documentation, the merges, the
replays and the summaries. Implementation on Opus / medium; reading results on Fable / high (the user is told before
each switch). At most two editing agents at once. Every number goes to the user without a verdict, after the
conclusion reviewer. **In every group, the CMake and ownership entries land in the same commit as the new tracked
files they name;** the orchestrator adds them when it integrates a hand-back.

## 1. Groundwork (orchestrator)

- [x] 1.1 Branch `consistency-harness` made from `honest-inertial-carry` (ba0ec25); research merged and reviewed
  (`research.md`). Verify: `git log` on the branch; the review's corrections are in the file.
- [x] 1.2 Two independent design reviews (`review-codex.md`, `review-codex-2.md`, both "not ready"); their claims
  about the code checked; proposal, design, specs and tasks revised after each. Verify: the design's last section
  maps every item of both reviews to its resolution; `openspec validate consistency-harness --strict` passes.

## 2. The exact control: testing the instruments (verification)

- [x] 2.1 A linear-Gaussian chain written in the test (increments, absolute measurements, a prior): solved as one
  batch and by exact sequential elimination of the same factors; the normalised-error sum and its chi-square
  quantiles, checked on inputs with known values; a function that gives the smallest number of runs meeting a power
  target for a prespecified variance multiplier and its reciprocal, with the tail probabilities corrected for the
  number of primary comparisons. Verify: batch and sequential agree to rounding in mean and covariance; the honest
  chain lies inside its interval; with the reported covariance multiplied by 1.5 and by its reciprocal, on a state
  that the scaled quantity dominates and over the number of runs the function gives, the test rejects low and high.
- [x] 2.2 Orchestrator: integrate (CMake, ownership, in the commit that adds the files). Verify: `python
  tools/region_owners.py` and `ctest` pass. (2026-11-20: c345927, merged d763f7f; batch vs sequential 2e-15; honest
  chain 2 of 200 seeds rejected at 1 %; M = 96 runs of 2 dof for 1.5, measured rejection 88.5 % / 93 %; q averages
  1.002, the old form 0.501; 5 cases 0.5 s; step checks ALL HOLD on c345927.)

## 3. The record and the readings (noise-budget, solver, orchestrator; results unchanged)

- [x] 3.1 Noise-budget: the record's new parts and their CSV columns after the existing ones. On the window's own
  row, taken right after it was prepared: its first pose and the prior means and covariances it was prepared with.
  Taken after the hand-off: the carry handed on (velocity, gravity vector and the unit factor before the fold,
  offset covariance, cross covariance, gyro bias, each with validity), the last node's pose and time as solved and
  as handed on, the mean rotation rate from the attitude increments and the start acceleration, with their
  definitions. Full covariance matrices in memory, means and sigmas in the CSV. Verify: a unit test of the new parts
  on made-up loop values (every field, the absent markers); the CSV writer's test shows the old columns unchanged
  in name, order and value.
- [x] 3.2 Solver: the window problem keeps the start velocity's covariance block and its blocks with the offset,
  with a validity that follows a step taken back, and the size of the last accepted step for the tested states; a
  pure function for the normalised move (Cholesky whitening, the eigenvalue check, every `NOT MEASURED` condition of
  D10, including the last-step tolerance); the verdict's readings (start velocity always, joint in joint mode, gyro
  bias), summary lines with largest, median, 95th percentile and coverage, readings on flagged lines. The velocity
  chi2 and its reporting untouched. Verify: K5 on the pure function (averages 1; the old form 0.5); on a window
  problem with a non-zero last step and the step check off, a finite reading on an informative window, with the
  separate and with the joint prior; a failed factorisation, a step taken back and a clamped bias each give `NOT
  MEASURED`; the new blocks against a dense inverse.
- [x] 3.3 Orchestrator: the loop's prepare, report and finish steps fill the record's new parts (3.1) and write it
  once per window at the end of the hand-off (a cancelled hand-off writes it with the handed-on part absent); the
  snapshot of 3.2 is taken before the window problem goes out of scope. Then the gate of D12: both step-check
  tables' old-column prefixes compared token by token with their references and every other reference checked
  unchanged; the external references recorded again from the integrated commit, with the provenance (commit, date,
  what was verified) written to `experiments/consistency/references.md`; `tools/step_checks.py` ALL HOLD. Knowledge
  files (noise-budget, solver) and the two paragraphs of `CLAUDE.md` (per-term table, window verdict). Verify: the
  provenance note exists and names the token comparison's result; step checks ALL HOLD on the main tree; 44_short
  matches its manifest.

## 4. The synthetic recording (verification)

- [x] 4.1 Motion, truth and IMU samples: the segments of D3; gyro values as means over the interval ending at a
  node, with the first update's 5 ms step; accelerometer values for the interval starting at a node; noise, a bias
  and an offset that are constant inside a window and step between windows; seeds. Verify: the independent
  strap-down test and the noise-statistics test pass; the bounds of the discretisation terms (the body force
  rotated at the left end, the position quadrature, a mean body rate integrated as one rotation) are derived in the
  test file and hold.
- [x] 4.2 Scenes, routes, points and the source: R1, R2 and R3 of D8 with an explicit emitted range; points
  generated at their node's pose and stamped strictly inside its interval; a stationary seed prefix and one IMU
  sample beyond the last tested window; a recording source that applies the production source's gyro calibration,
  zero-time rejection, raw XY filter and lever-arm translation; windows cut by the real `calculate_trajectory` and
  `compute_step_1`. Verify: every eligible tagged hit is in its window's original point set at its node, also at
  window boundaries; the decimated sets cover every node; more than 1 000 assigned original points per slice; the
  first window at rest; the tested motion after the filter's ramp; the expected window count and timestamps.
- [x] 4.3 Documentation: the Testing section of `CLAUDE.md` and the hand-back for `docs/regions/verification.md`:
  how to make a recording, and the table of D4 with, per claim, its kind and the check that is planned for it in
  group 5. Verify: every row of D4 names a planned check; the table is completed with the checks' results in 5.7.

## 5. Fixtures on the production code: measure, report, then pin (verification)

- [x] 5.1 F-a, the terms whose claim can be made true, at the truth, noise-free: one production span or block at a
  time, the signed residual recovered as D5 says, the measured motion-model poses kept; rest (A1, active states
  only), constant acceleration (A3), turning in place and a turn with centripetal acceleration (A4a, A4b), walking
  (A5: the terms' quadrature error, the map error and the total, nothing attributed); the checks behind D4 (the
  rotation edges' variance behind the filter's ramp and angle conversion; the gap covariance's algebra apart from
  its mean). Verify: residuals within their derived bounds, or the excess reported per term and motion.
- [x] 5.2 F-b, the accelerometer terms' noise: signed whitened residuals at the truth with the true rotations,
  gravity, offset and unit factor, over model-world recordings, in the sample-aligned fixture and for the Livox
  timing. Verify: the sample supports' disjointness is asserted; unit variance and no covariance between terms
  with disjoint supports; for the Livox timing the measured variance and covariance per term kind (triple, start,
  end) reported.
- [x] 5.3 F-c, a window's claimed covariance against its scatter: priors drawn from their claimed covariance,
  model-world inertial data, the LiDAR absent (an explicitly empty point set) or a corridor with exact cell
  statistics whose along-axis translation information the test verifies to be near zero; every active factor listed
  with its kind; the error vectors of D5 with their three controls. Verify: the controls pass; where only claims
  that were made true are active, the summed normalised error per state and axis with its interval and the
  detectable deviation, sized by 2.1; elsewhere the scatter beside the claim, descriptively.
- [x] 5.4 F-d and the unit-level cases: K3 (prepared prior mean over the gap), K4 (the tail alone, and the next first
  pose as actually predicted), A2 (noise-free constant velocity through the loop steps: is it taken as rest, and
  what follows), K6 (rest test with a tilted filter pitch), K7 (fractional projection's own step), K9 (cell test
  and rotation), K10 (origin fix weight in radians). Verify: each reports its measurement beside the audit's
  hypothesis and names its oracle, if it has one.
- [x] 5.5 F-e, chains through `process_windows`: the loop's setup from a valid stationary window, a start index
  above zero with window 0 and the predecessor history, indexed intermediate points, a consistent carried state, the
  grids seeded from true points. The blind stretch with both switches off and on; K2 (exit with a boundary error
  injected in the predecessor's frame); K1 (the map displaced along travel, paired with the zero-shift control);
  K11 (the boundary attitude turned by 0.05 deg, paired); constant acceleration with `handoff_tail_nodes` 15 and 0.
  Verify: each fixture's validity cases pass (the injected error is present after the window was prepared;
  blindness verified; matches to the designated structure; no rebuild); the paired measurements are in the
  hand-back.
- [x] 5.6 K8, the LiDAR term's claim: a scalar rigid shift, independent scan draws against a fixed true map and with
  an earlier noisy map redrawn per trial, fine grid alone and both grids, a flat wall along travel. Verify: scatter
  over claimed sigma per variant, with duplication, map noise and plane pull separated, and the noise assumptions
  stated; reported descriptively.
- [x] 5.7 Orchestrator: the hand-backs checked (ownership, step checks, the conclusion reviewer on "measured against
  hypothesis" and on which discrepancies are reproduced, isolated and have an oracle); identifiers assigned to new
  findings; the table of D4 completed with the checks' results; `experiments/consistency/fixtures.md`; the table to
  the user without a verdict. Verify: the user has seen the table; every discrepancy to be marked has an identifier.
- [x] 5.8 Verification: for each discrepancy that 5.7 lists with an oracle, a case asserting the honest outcome
  marked as expected to fail with its identifier and cause, beside the passing validity cases. Verify: `ctest`
  passes; every marker carries an identifier from `research.md` or from 5.7.

## 6. The loop on synthetic recordings: an integration measurement (verification, orchestrator)

- [x] 6.1 The runner through the front end and `optimize_windows` with a keeping sink and `log_window_terms` on; the
  preconditions of D8 computed per run with their predeclared tolerances, the outbound landmark cells designated by
  key and the return matches computed by the test; the short smoke run in `ctest`. Verify: the smoke run completes,
  its preconditions hold, its declared deterministic fields repeat and its unavailable readings are counted; all
  new cases in `ctest` take at most one minute in Release (the time is reported, and a run that reached the
  solver's wall-clock limit does not count); loop-level cases are skipped in Debug.
- [x] 6.1a Verification (added 2026-11-21 after the loop-lag diagnosis, `experiments/consistency/loop/analysis-claude.md`
  and `analysis-codex.md`; tests only): the loop fixture made honest before 6.2. (1) One sensor model: the world's IMU
  errors (gyro white noise, gyro bias walk, accelerometer offset walk, accelerometer density) built from the run's own
  `OdometryConfig`, their equality a precondition, the old default world kept only as a labelled mismatch case. (2)
  Observability classes: each window's along-travel share of the LiDAR information computed from a truth-placed map
  against the projection's thresholds (removed, partly kept, kept whole), reported beside the truth classes; scenes
  given along-travel structure where a route is meant to be seen. (3) Controls per route: noise-free IMU, and a pair
  with `ahrs_gain` 0, beside the production setting (gain stays 0.05 in the main set). (4) R2's blind stretch and exit
  reworked until its preconditions hold in the noise-free control. Verify: in the matched world R3 ends within its
  seeds' spread; R1, R2, R3 preconditions hold in the noise-free control; smoke case still at most one minute.
- [x] 6.2 On demand: R1, R2 and R3 with the candidate settings, both switches off and on, and a second set with the
  shipped Livox file; per class of window the errors, normalised errors and readings; comparisons from the seeds
  valid under both settings, with the exclusions per setting; outputs outside the repository. Verify: one
  documented command; runs with a failed precondition are listed and left out; the class tables are in
  `experiments/consistency/loop/`.
- [x] 6.3 Orchestrator: the tables checked with the conclusion reviewer and shown to the user without a verdict,
  worded as measurements. Verify: `experiments/consistency/loop.md`; the user has seen it.

## 7. The tool and the replays

- [x] 7.1 Verification: `tools/honesty_report.py` with tests in `tests/test_verification_tools.py` and a section in
  `tools/README.md`: the table of D11 from the new columns, each figure with its mask and definition, "unavailable"
  for a missing input. Verify: the tests pass, including one with an old table (every figure that needs a new
  column is unavailable) and one synthetic table with known figures.
- [x] 7.2 Orchestrator: replays of the existing captures with the new build and `log_window_terms` (Livox 3, the
  stage 8 Livox 4 captures, 44_short) through `tools/run_replay.py`; the tool on their tables; a comparison with
  the audit's figures that names each difference in definition. Verify: `experiments/consistency/replays.md` holds
  the tables, each labelled as a forced replay conditional on its captured map and carried state, with the windows
  that stopped at the time limit taken from the original run's log.

- [x] 7.3 Orchestrator (added 2026-11-20 at the user's request: Livox 4 is the recording that matters most): one full
  Livox 4 run from this branch with the stage 8 candidate settings and `log_window_terms`, `log_window_diagnostics`,
  `log_window_verdict` on, run alone (no step checks or replays beside it); `verify-run` on its log; the tool on its
  `window_features.csv`: the readings per class of window and per stretch (the corridors at 126-183 s, the exits, the
  turns), and how often the rest test fires while moving (A2). Results are identical to the same settings without the
  logs (this change changes no result); the run is the "before" against which the fixes of 8.2 are measured on
  Livox 4 (base and repeats). Verify: the run folder (settings, log, result, health table, readings table) under
  `experiments/consistency/livox4/`, and the user has seen the table.

## 8. Summary, decision and close

- [x] 8.1 Orchestrator: `experiments/consistency/summary.md`: the instruments, the fixtures (measured against
  hypothesis, what was pinned), the loop measurement, the replays, what is not concluded; the conclusion reviewer;
  the numbers to the user without a verdict. Verify: the reviewer's corrections are applied and the user has seen
  the summary.
- [x] 8.2 The user's decision: which findings become changes and in what order, which follow-ups of the design are
  taken up, and what happens to the layers of `honest-inertial-carry`. Every fix that follows is judged first on Livox 4
  (full runs, base and repeats, against the "before" of 7.3), then Livox 3 and 44. Verify: the decision is written into this
  file.
  **The user's decision (2026-11-21)**, after `experiments/consistency/summary.md` and two independent
  recommendations (`recommendation-claude.md`, `summary-codex.md`, compared in `comparison-claude-codex.md`). Each
  step is its own OpenSpec change (or, for a measurement, its own task list), judged first on Livox 4 (full runs,
  base and repeats, against the reference run of 7.3), then Livox 3 and 44, with the user's look at the maps:
  1. Look first, no code: windows 372 to 374 of the Livox 4 reference run (a start-velocity move of 1.1 m/s against
     a prior sigma of 5 mm/s), and the lever-arm replay.
  2. The two `honest-inertial-carry` switches on Livox 4: each alone and both together against the reference run;
     then Livox 3 and 44. Both switches stay in the code, off by default, until this is read.
  3. F2a: the start velocity prior advanced over the gap (marker K3).
  4. F2b: the tail re-propagation under acceleration, on its own (marker K4).
  5. The rest test (F3, marker K6): stops and slow walking labelled independently first, then the fix.
  6. The fixed first pose (the boundary state): design first, then the change.
  7. F9 (marker K9), then C16 (marker K7), each alone.
  Alongside, tests and tools only: the check register of test limits and markers, one source for the verdict
  limits, R2's exit nomination, the blindness check of `loop_chains_test.cpp` along the true axis, a raised
  iteration cap on selected synthetic cases, the normalised start-velocity move reported beside the velocity
  chi-square, the four old failing tool tests.
  Follow-ups of the design: the lever-arm replay now (step 1); the time-offset run and the window-length
  sensitivity with step 6; the deviation sweep and the raw-data regressions when a fix needs them.
  Noted, not re-run: the synthetic loop of 6.2 used `ahrs_gain` 0.05, the Livox 4 reference run 0.
- [ ] 8.3 Knowledge files (verification, inertial, noise-budget, solver, lidar-matching) with the findings and
  experiments; `docs/code-review-tracker.md` entries completed for the identifiers of 5.7, with their status; the
  English guide where it describes the tests and the per-window readings. Verify: `python tools/region_owners.py`
  passes; every marked case's identifier has a tracker entry.
- [ ] 8.4 Final check of "no result changes": `tools/step_checks.py` and `tools/end_runs.py` (full 44, Livox 3,
  44_short against their manifests). Verify: ALL HOLD; every comparison identical, with any that depends on the
  time limit flagged.
