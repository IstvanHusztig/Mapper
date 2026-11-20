# Tasks

Who works: the verification region builds the exact control, the generator, the fixtures, the loop measurement and
the tool (it never edits `src/`); the noise-budget and solver regions make the two diagnostics changes; the
orchestrator owns `owners.toml`, `tests/CMakeLists.txt`, the shared loop steps, the documentation, the merges, the
replays and the summaries. Implementation on Opus / medium; reading results on Fable / high (the user is told before
each switch). At most two editing agents at once. Every number goes to the user without a verdict, after the
conclusion reviewer. Ownership entries land in the same commit as the files they name.

## 1. Groundwork (orchestrator)

- [x] 1.1 Branch `consistency-harness` made from `honest-inertial-carry` (ba0ec25); research merged and reviewed
  (`research.md`). Verify: `git log` on the branch; the review's corrections are in the file.
- [x] 1.2 Independent design review (`review-codex.md`, "not ready", 9 blocking items); its claims about the code
  checked; proposal, design, specs and tasks revised. Verify: the design's last section maps every review item to
  its resolution; `openspec validate consistency-harness --strict` passes.

## 2. The exact control: testing the instruments (verification)

- [ ] 2.1 A linear-Gaussian chain written in the test (increments, absolute measurements, a prior): solved as one
  batch and by exact sequential elimination of the same factors; the normalised-error sum with its chi-square
  interval; a function that gives the number of runs needed to detect a stated variance ratio. Verify: batch and
  sequential agree to rounding in mean and covariance; the honest chain lies inside its interval; with the reported
  covariance scaled down and up, on a state that the scaled quantity dominates, the test rejects high and low.
- [ ] 2.2 Orchestrator: the new test files in `tests/CMakeLists.txt` and under verification in
  `docs/regions/owners.toml`, in the commit that adds them. Verify: `python tools/region_owners.py` and `ctest`
  pass.

## 3. The record and the readings (noise-budget, solver, orchestrator; results unchanged)

- [ ] 3.1 Noise-budget: the record's new part and its CSV columns after the existing ones: the carry handed on
  (velocity, gravity vector and the unit factor before the fold, offset covariance, cross covariance, gyro bias,
  each with validity), the last node's pose and time as solved and as handed on, the priors the window was prepared
  with, the next prepared first pose, the mean rotation rate and the start acceleration with their definitions.
  Verify: a unit test of the record's new part on made-up loop values (every field, the absent markers); the CSV
  writer's test shows the old columns unchanged in name, order and value.
- [ ] 3.2 Solver: the window problem keeps the start velocity's covariance block and its blocks with the offset,
  with a validity that follows a step taken back; a pure function for the normalised move (Cholesky whitening, the
  eigenvalue check, every `NOT MEASURED` condition of D10); the verdict's readings (start velocity always, joint in
  joint mode, gyro bias), summary lines with largest, median, 95th percentile and coverage, readings on flagged
  lines. The velocity chi2 and its reporting untouched. Verify: K5 on the pure function (averages 1; the old form
  0.5); a test per `NOT MEASURED` condition; the new blocks against a dense inverse and after a rejected step; the
  verdict tests.
- [ ] 3.3 Orchestrator: the loop's report and finish steps complete the record after the hand-off and write it once
  per window (a cancelled hand-off writes it with the handed-on part absent); the snapshot of 3.2 is taken before
  the window problem goes out of scope. Then the gate of D12: old-column tokens of both step-check tables compared
  with their references and every other reference unchanged, the table reference refreshed in its own commit, and
  `tools/step_checks.py` ALL HOLD. Knowledge files (noise-budget, solver) and the two paragraphs of `CLAUDE.md`
  (per-term table, window verdict). Verify: the token comparison's output is kept with the commit message; step
  checks ALL HOLD on the main tree; 44_short matches its manifest.

## 4. The synthetic recording (verification)

- [ ] 4.1 Motion, truth and IMU samples: the segments of D3; gyro values as means over the interval ending at a
  node, accelerometer values for the interval starting at it; noise, a bias and an offset that are constant inside
  a window and step between windows; seeds. Verify: the independent strap-down test and the noise-statistics test
  pass; the bound of the rotation discretisation term is derived in the test file and holds.
- [ ] 4.2 Scenes, routes and points: R1, R2 and R3 of D8 with an explicit emitted range; points generated at their
  node's pose and stamped strictly inside its interval; windows cut by the real `calculate_trajectory` and
  `compute_step_1`. Verify: every generated point is retained with its node index, also at window boundaries; the
  front end's needs of D3 hold (60 001 seed points and the first window at rest, more than 1 000 points per slice
  after decimation, the expected window count and timestamps, the tested motion after the filter's ramp).
- [ ] 4.3 Documentation: the Testing section of `CLAUDE.md` and the hand-back for `docs/regions/verification.md`
  (how to make a recording; the table of D4 as built, each claim with its status). Verify: every row of D4 has a
  test that checks it, named in the hand-back.

## 5. Fixtures on the production code: measure, report, then pin (verification)

- [ ] 5.1 F-a, each term at the truth, noise-free: rest (A1, active states only), constant acceleration (A3),
  turning in place and a turn with centripetal acceleration (A4a, A4b), walking (A5: the terms' quadrature error
  and the map error first); the checks behind D4 (the rotation edges' variance after the filter's ramp and angle
  conversion; the gap growth formulas). Verify: residuals within their derived bounds, or the excess reported per
  term and motion.
- [ ] 5.2 F-b, the terms' noise: whitened residuals at the truth over model-world recordings, in the sample-aligned
  fixture and for the Livox timing. Verify: unit variance and no cross-covariance where the claim is made true;
  for the Livox timing the measured variance and cross-covariance per term kind (triple, start, end) reported.
- [ ] 5.3 F-c, a window's claimed covariance against its scatter: priors drawn from their claimed covariance,
  model-world inertial data, the LiDAR absent or a corridor with exact cell statistics whose along-axis translation
  information the test verifies to be near zero; the error vectors of D5 with their three controls (a sensor scale
  other than 1 g, a rotated start, a cross covariance of known sign). Verify: the controls pass; per state and axis
  the summed normalised error with its interval and the detectable deviation, sized by 2.1.
- [ ] 5.4 F-d and the unit-level cases: K3 (prepared prior mean over the gap), K4 (the tail alone, and the next first
  node), A2 (noise-free constant velocity through the loop steps: is it taken as rest, and what follows), K6 (rest
  test with a tilted filter pitch), K7 (fractional projection's own step), K9 (cell test and rotation), K10 (origin
  fix weight in radians). Verify: each reports its measurement beside the audit's hypothesis.
- [ ] 5.5 F-e, chains through `process_windows` from a known carried state and a map of true points: the blind
  stretch with both switches off and on; K2 (exit with an injected entry error); K1 (the map displaced along
  travel, paired with the zero-shift control); K11 (the boundary attitude turned by 0.05 deg, paired); constant
  acceleration with `handoff_tail_nodes` 15 and 0. Verify: each fixture's validity cases pass (blindness verified,
  matches to mapped structure, no rebuild); the measurements are in the hand-back.
- [ ] 5.6 K8, the LiDAR term's claim: a scalar rigid shift, independent scan draws against a fixed true map and with
  an earlier noisy map redrawn per trial, fine grid alone and both grids, a flat wall along travel. Verify: scatter
  over claimed sigma per variant, with duplication, map noise and plane pull separated, and the noise assumptions
  stated.
- [ ] 5.7 Orchestrator: the hand-backs checked (ownership, step checks, the conclusion reviewer on "measured against
  hypothesis" and on which discrepancies are reproduced and isolated); `experiments/consistency/fixtures.md`; the
  table to the user without a verdict. Verify: the user has seen the table.
- [ ] 5.8 Verification: for each discrepancy that 5.7 found reproduced and isolated, a case asserting the honest
  outcome marked as expected to fail with its finding and cause, beside the passing validity cases. Verify: `ctest`
  passes; every marker names a finding of `research.md` or a new one entered in 8.3.

## 6. The loop on synthetic recordings: an integration measurement (verification, orchestrator)

- [ ] 6.1 The runner through the front end and `optimize_windows` with a keeping sink and `log_window_terms` on; the
  preconditions of D8 computed per run; the short smoke run in `ctest`. Verify: the smoke run completes, its
  preconditions hold and it repeats bit for bit; all new cases in `ctest` take at most one minute in Release (the
  time is reported); loop-level cases are skipped in Debug.
- [ ] 6.2 On demand: R1, R2 and R3 with the candidate settings, both switches off and on from the same seeds, and a
  second set with the shipped Livox file; per class of window the errors, normalised errors and readings; outputs
  outside the repository. Verify: one documented command; runs with a failed precondition are listed and left out;
  the class tables are in `experiments/consistency/loop/`.
- [ ] 6.3 Orchestrator: the tables checked with the conclusion reviewer and shown to the user without a verdict,
  worded as measurements. Verify: `experiments/consistency/loop.md`; the user has seen it.

## 7. The tool and the replays

- [ ] 7.1 Verification: `tools/honesty_report.py` with tests in `tests/test_verification_tools.py` and a section in
  `tools/README.md`: the table of D11 from the new columns, each figure with its mask and definition, "unavailable"
  for a missing input. Verify: the tests pass, including one with an old table (every figure that needs a new
  column is unavailable) and one synthetic table with known figures.
- [ ] 7.2 Orchestrator: replays of the existing captures with the new build and `log_window_terms` (Livox 3, the
  stage 8 Livox 4 captures, 44_short) through `tools/run_replay.py`; the tool on their tables; a comparison with
  the audit's figures that names each difference in definition. Verify: `experiments/consistency/replays.md` holds
  the tables, each labelled as conditional on its captured map and carried state, with time-limited windows
  flagged.

## 8. Summary, decision and close

- [ ] 8.1 Orchestrator: `experiments/consistency/summary.md`: the instruments, the fixtures (measured against
  hypothesis, what was pinned), the loop measurement, the replays, what is not concluded; the conclusion reviewer;
  the numbers to the user without a verdict. Verify: the reviewer's corrections are applied and the user has seen
  the summary.
- [ ] 8.2 The user's decision: which findings become changes and in what order, which follow-ups of the design are
  taken up, and what happens to the layers of `honest-inertial-carry`. Verify: the decision is written into this
  file.
- [ ] 8.3 Knowledge files (verification, inertial, noise-budget, solver, lidar-matching) with the findings and
  experiments; `docs/code-review-tracker.md` entries for the findings this change confirmed, with their status; the
  English guide where it describes the tests and the per-window readings. Verify: `python tools/region_owners.py`
  passes; every marked case's finding has a tracker entry.
- [ ] 8.4 Final check of "no result changes": `tools/step_checks.py` and `tools/end_runs.py` (full 44, Livox 3,
  44_short against their manifests). Verify: ALL HOLD; every comparison identical, with any that depends on the
  time limit flagged.
