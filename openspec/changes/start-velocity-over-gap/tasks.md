# Tasks

## 1. Decide and implement

- [ ] 1.1 Settle the source of the acceleration over the gap (design decision 1); check the unused last-node sample in `accelerometer_term.cpp` and how the window's acceleration is stored (`imu_acc`, world frame, offset)
- [ ] 1.2 Advance the prior mean in `prepare_velocity_state` (the start-velocity prior; the joint-covariance path was removed with layer 2 on 2026-11-25), with the gap guard; at rest and without a carried velocity unchanged
- [ ] 1.3 Remove `doctest::should_fail` from `unit case K3 marker`; add a case for a data gap

## 2. Measure

- [ ] 2.1 Release build and `tools/step_checks.py` in a standing region worktree; report which guards, replays and 44_short move and by how much (no verdict)
- [ ] 2.2 Replays of the Livox 4 turn windows (369 to 378) before and after: start-velocity move, normalised move, claimed sigma
- [ ] 2.3 Re-run the synthetic loop tables (R1, R2, R3) and compare the end-velocity readings
- [ ] 2.4 Full runs (44, 44_short, Livox 3, Livox 4) as a measurement against the old baselines (`--tolerance`, `tools/trajectory_seams.py`), one after another
- [ ] 2.5 If the maps differ, the seven-member sweep on Livox 4 (`verify-robustness`), before and after; overlapping envelopes are inconclusive

## 3. Decide

- [ ] 3.1 Show the numbers and the before/after maps to the user; no verdict before the user's look
- [ ] 3.2 On the user's decision: re-record the manifests and baselines in their own commit, update `docs/regions/inertial.md` (F2a) and `docs/code-review-tracker.md`, archive the change
