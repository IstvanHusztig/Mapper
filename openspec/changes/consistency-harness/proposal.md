# Proposal: consistency-harness

## Why

Two layers of `honest-inertial-carry` did what they were designed to do, and new symptoms appeared behind them. The
audit of 2026-11-20 (`research.md`) found why: the largest errors are not slips in a term's maths but things the
scheme drops or assumes (the first pose of every window held exact, a LiDAR term that claims more than it knows, a
stale hand-off), and three quarters of the velocity jump in ordinary windows is still unexplained. Each of these was
found by reading, by accident or by a reviewer. Nothing in the tests or the logs asks the one question that would
have found them: **is what the estimator claims to know as good as it says?** No test runs the window loop on data
with a known truth and inertial sensors, no test compares a claimed covariance with an actual error, and the one
statistic that tries (the velocity chi2) divides by the wrong covariance.

Without that question answered by a test, every further fix is a patch judged on proxies.

## What Changes

No estimator behaviour changes. Every result file stays bit-identical; only tests, diagnostics and a tool are added.

- **Synthetic recordings with a known truth and inertial sensors.** Raw IMU samples and timed LiDAR points, served
  to the real front end and window loop the way a recording folder is. In the "model world" every sensor error obeys
  the estimator's own noise claims; each deviation of reality from the model (lever arm, time offset, axis rotation,
  body-fixed bias, ...) is one named switch.
- **Truth tests in three tiers**, each answering one question:
  - *A, noise-free:* truth in, truth out. Any deviation is an inconsistency between parts of the code. Deterministic.
  - *B, model-true noise:* does each carried state's claimed uncertainty match its actual error (Monte Carlo)?
    With controls that prove the test can fail.
  - *C, one deviation at a time:* how large is each real-world effect, and which reproduces the signature of the
    unexplained velocity jump on the recordings? An experiment with a report, not a pass/fail test.
- **Known-answer tests for the audit's findings.** One small case per verified finding, stating the honest outcome;
  marked "expected to fail" with its cause while the defect exists, so a fix is forced to remove the marker.
- **Honesty readings on real runs.** Per window, the move of the start velocity (with the offset when carried
  jointly) and of the gyro bias from prior to result, measured against the covariance that move should have, scaled
  to an expected value of 1. Added next to the existing velocity chi2, whose formula and agreed limits stay untouched
  until the user decides. The per-window record also gains the state handed to the next window.
- **A report tool** (`tools/honesty_report.py`) that computes the same signature table (jump size by rotation rate
  and boundary acceleration, autocorrelation, the readings, the regressions for hand-off, lever arm and time offset)
  from a real run's `window_features.csv` and from a synthetic run alike.
- **Evidence on recordings with existing settings** (no code): the tool on the existing runs, a replay with
  `handoff_tail_nodes = 0`, one Livox 3 run with the lever arm set.

Not in this change: any fix of a finding, any change of a limit or a setting file, loop closure and consistency.
Which fixes become changes is the user's decision on this change's numbers.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: new requirements for synthetic recordings with inertial data and truth, noise-free reproduction,
  claimed uncertainty against actual error, the hand-off against one long window, and known-answer cases for audit
  findings.
- `processing-pipeline`: a new requirement for the carried-state record and the honesty readings of a window.

## Impact

- Tests: `tests/synthetic_recording.{h,cpp}` extended; new files for the inertial synthesis, the in-memory recording
  source, the truth statistics and the truth test cases. A fast set runs in every `ctest`; the full Monte Carlo and
  the deviation runs are on demand.
- Code (diagnostics only): the per-window record (`window_term_sink.h`, `run_odometry.cpp::record_window_terms`,
  `window_term_csv`), the window verdict (`window_verdict.{h,cpp}`, `run_odometry.cpp::window_evidence`), and the
  window problem keeping one more block of the covariance it already computes. No change to `Window`, to captures or
  to the settings hash.
- Outputs: `window_features.csv` gains columns (existing columns unchanged); the verdict summary gains lines. The
  step checks' table references are re-recorded once.
- Tools: `tools/honesty_report.py`, its tests, `tools/README.md`.
- Regions: verification (generator, truth tests, tool), noise-budget (record), solver (verdict, covariance block);
  `docs/regions/owners.toml` gains the new files.
- Runs: one full Livox 3 run (about 25 min) and replays; all sequential.
- Depends on the code of `honest-inertial-carry` (both layers, off by default) so that each layer can be tested off
  and on; the branch is made from it.
