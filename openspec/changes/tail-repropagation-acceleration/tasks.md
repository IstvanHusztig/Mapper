# Tasks

## 1. Decide and implement

- [ ] 1.1 Settle the estimator (design decision 1): check what `window.velocity.end` refers to, the shortest kept-node count in the Livox and 44 windows, and the noise of a second difference
- [ ] 1.2 Re-propagate the tail from the velocity at the last kept node with the end acceleration; constant-velocity fallback where inputs are missing
- [ ] 1.3 Remove `doctest::should_fail` from `unit case K4 marker`; add cases for constant velocity, rest and the short-window fallback in `tests/window_steps_test.cpp`

## 2. Measure

- [ ] 2.1 Release build and `tools/step_checks.py` in a standing region worktree on top of `start-velocity-over-gap`; report what moves (no verdict)
- [ ] 2.2 Replays of the Livox 4 windows 369 to 378 and a quiet stretch before and after: next first pose, start-velocity move, the per-term table
- [ ] 2.3 `tools/map_quality.py` on 44's s0/s1/s2 against the noise floor (the tail moves map points); synthetic loop tables
- [ ] 2.4 Full runs (44, 44_short, Livox 3, Livox 4) as a measurement against the old baselines, one after another; the seven-member sweep on Livox 4 if the maps differ

## 3. Decide

- [ ] 3.1 Show numbers and before/after maps to the user; no verdict before their look
- [ ] 3.2 On the user's decision: re-record manifests and baselines in their own commit, update `docs/regions/inertial.md` (F2b) and `docs/code-review-tracker.md`, archive the change
