# Proposal: consistency-harness

## Why

Two layers of `honest-inertial-carry` did what they were designed to do, and new symptoms appeared behind them. The
audit of 2026-11-20 (`research.md`) points at things the scheme drops or assumes (the first pose of every window held
exact, a LiDAR term that claims more than it knows, a stale hand-off) more than at slips in a term's maths; its
evidence is one recording, candidate settings and one blind stretch, and three quarters of the velocity jump in
ordinary windows is unexplained. Each finding was found by reading, by accident or by a reviewer. Nothing in the
tests or the logs asks the question that would have found them: **is what the estimator claims to know as good as it
says?** No test compares a claimed covariance with an actual error, no test gives the terms and the hand-off data
with a known truth and inertial sensors, and the one statistic that tries (the velocity chi2) divides by the wrong
covariance.

Without that question answered by a test, every further fix is a patch judged on proxies.

## What Changes

No estimator behaviour changes. Result files stay identical under the existing checks; only tests, diagnostics and a
tool are added.

- **Attribution only where the observation model is known.** A failure is called an error of the code or the scheme
  only in a fixture whose residuals, priors, covariances and observable directions were verified independently.
  Everything measured on the full window loop is reported as a measurement, not as a verdict.
- **An exact control.** A small linear-Gaussian problem that is honest by construction tests the measuring
  instruments themselves: the normalised-error test, its direction and power, batch against sequential elimination,
  and the new reading.
- **Synthetic recordings with a known truth and inertial sensors,** in a "model world" where each sensor error is
  drawn as the estimator's weights assume, with the estimator's own time conventions. Cut into windows by the real
  front end.
- **Fixtures on the production code with a known boundary state:**
  - each term's residual at the truth, noise-free and with model-true noise (variance and independence as claimed);
  - one window's claimed covariance against the scatter of its error, with a prior drawn from its claimed
    distribution and the LiDAR's information controlled and verified;
  - the hand-off (the prior over the gap, the re-propagated tail, a displaced boundary pose with a paired control,
    short chains through the real loop steps from a known carried state and a map of true points).
- **Discrepancies are measured first, then pinned.** A discrepancy that is reproduced and isolated in a fixture gets
  a case that states the honest outcome, marked "expected to fail" with its cause, so that a fix must remove the
  marker. Predictions of the audit are tested, not trusted.
- **The full loop on synthetic recordings, as an integration measurement:** with its preconditions checked (the
  stretch is blind along the true axis, the re-observed structure was mapped before, no map rebuild), errors and
  readings per class of window, with the two layer switches off and on from the same seeds.
- **Honesty readings on real runs.** Per window, the move of the start velocity and of the gyro bias from prior to
  result against the covariance that move should have, scaled to an expected value of 1; `NOT MEASURED` wherever
  the covariance behind it is absent, stale or unusable. Added next to the existing velocity chi2, whose formula,
  limits and reporting stay as they are. The per-window record gains the state actually handed to the next window.
- **A report tool** (`tools/honesty_report.py`) that makes one table from the new columns of a real run and of a
  synthetic run under the same definitions, and says "unavailable" where an input is missing.

Not in this change, named so that they are not lost: runs with one real-world deviation at a time (lever arm, time
offset, axis rotation, body-fixed bias, ...) against the recordings' signatures; the Mid-360S lever arm on a
recording; the regressions that need raw IMU data and trajectories; sensitivity to the window length; any fix of a
finding; any change of a limit, a setting file or a baseline; loop closure and consistency. Which of these follow,
and what happens to the layers of `honest-inertial-carry`, is the user's decision on this change's numbers.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: new requirements for synthetic recordings with inertial data and truth, an exact control for the
  statistics, terms against the truth, a window's claimed covariance against its error, the hand-off from a known
  boundary state, the recording of known discrepancies, and the measurement of the loop on synthetic recordings.
- `processing-pipeline`: a new requirement for the handed-on state in the per-window record and for the honesty
  readings.

## Impact

- Tests: `tests/synthetic_recording.{h,cpp}` extended; new files for the inertial synthesis, the recording source,
  the exact control and statistics, and the fixture cases; `tests/CMakeLists.txt`. A fast set in every `ctest`;
  the large Monte Carlo sets and the loop measurements on demand.
- Code (diagnostics only): the per-window record and its CSV (`window_term_sink.h`,
  `run_odometry.cpp::record_window_terms`, `window_term_csv`), the window verdict (`window_verdict.{h,cpp}`,
  `run_odometry.cpp::window_evidence`), the window problem keeping two more blocks of the covariance it already
  computes, and the loop's report and finish steps (the record is completed after the hand-off). No change to
  `Window`, to captures or to the settings hash.
- Outputs: `window_features.csv` gains columns after the existing ones; the verdict summary gains lines. The step
  checks' table reference is refreshed once, after the old columns were shown unchanged.
- Tools: `tools/honesty_report.py`, its tests, `tools/README.md`.
- Regions: verification (generator, fixtures, tool), noise-budget (record and CSV), solver (covariance blocks,
  verdict); the orchestrator wires CMake, the shared loop steps and the documentation.
- Runs: replays only; no full run is needed for this change besides the final check that results are unchanged.
- Depends on the code of `honest-inertial-carry` (both layers, off by default) so that each layer can be measured
  off and on; the branch is made from it.
