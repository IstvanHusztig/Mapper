# Tasks

Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes. Run background jobs from a copy of the binaries. Keep the clean reference `raw data/44/lio_result_2` and the stalling `raw data/44/lio_result_3` untouched.

## 1. Reproduce and instrument

- [x] 1.1 Add a seam measure to `tools/eval_odometry.py` (or a small tool next to it): given a run folder and a reference run folder, print the largest position difference and the times where it exceeds 5, 20 and 50 cm, as `trajcmp.py` / `early.py` did in the scratchpad (pure Python, no numpy). Verify: on `lio_result_3` against `lio_result_2` it reports about 1.03 m at t about 210.6 s; on `lio_result_2` against itself 0. Commit.

  Result: a separate small tool, `tools/trajectory_seams.py` (documented in `tools/README.md`), because `eval_odometry.py` needs the IMU data and looks at yaw. `lio_result_3` against `lio_result_2`: max 1.026 m at t = 210.6 s (reference position 2.4 -1.8 -0.1), first above 5 / 20 / 50 cm at 14.2 / 32.5 / 209.8 s; `lio_result_2` against itself 0.
- [x] 1.2 Add per-window diagnostics behind a new setting (`log_window_diagnostics`, default false, in `TomlIO` and the shipped settings, golden TOML files regenerated in their own commit): for each window its start time, the LM iterations used, the last delta, the displacement of its first nodes against the previous velocity times the elapsed time and against the IMU-predicted displacement, the information eigenvalues and whether the accelerometer term was active. Verify: `ctest`; `44_short` IDENTICAL with the setting off. Commit.

  Result: `log_window_diagnostics` (`[lidar_odometry_debug_info]`, default false) logs `window diagnostics N: t, dt, iterations, delta, start, guess move, result move, previous-velocity move, result-guess, info ratio, weak direction, acc term on/off` per window (`run_odometry.cpp`, only read). Golden TOML files regenerated in their own commit. `ctest` passes; `44_short` has identical data files, only the parameter file differs by the new setting line. Already visible in the existing log before this: in both builds every window in 575 to 592 runs all 100 iterations with delta about 1e-6 (so an early stop is not the cause), and the sliding-map reset (acc_distance back to 0) happens at window 585 in the fast run and 589 in the precise run.
- [ ] 1.3 Cut a repeatable test case around the stall (`cut_recording`, for example from t = 180 s for 60 s as `44_stall`) and run it with the current Release build and with a `/fp:fast` build (a scratchpad build tree, not a preset). If the stall or the same deviation between the builds shows up, use it for the following tasks; if not, use the full 44 with the diagnostics and look at windows 570 to 600. Report which.

## 2. Diagnose

- [ ] 2.1 From the diagnostics of the clean and the stalling run, report windows 575 to 590: iterations, delta, displacement against prediction, information ratio, accelerometer term state. Decide which suspect it is: an early stop of the iterations along a weak direction, a wrong local minimum, or the missing motion prior before the gate (ratio below 0.3) opens. Write the finding into this file and, if it changes the plan of task 3, into the proposal before going on.

## 3. Fix

- [ ] 3.1 Implement the smallest fix the diagnosis supports (candidates in the proposal), with its setting(s) in `TomlIO`, the shipped settings and `docs`. Tests on synthetic windows: a corridor-like window whose weak direction carries a motion that the LiDAR cannot see must not stall (position error along the corridor within a given bound), and well-constrained windows give the same result as before to the printed precision. If the specified motion-model behaviour changes, add the delta spec. Verify: `ctest`; report the window diagnostics of windows 575 to 590 with the fix. Commit.

## 4. Acceptance, baselines, docs

- [ ] 4.1 Full 44 with the shipped settings from the Release build, and the same code as a `/fp:fast` build. Verify: no lag at 210 to 225 s; the two builds within a few centimetres over the whole 44 (report the largest difference); seam measure against `lio_result_2` reported; `eval_odometry.py` metrics not worse than `lio_result_2`'s. The user checks the cloud (the room of the first 10 s) for the ghost wall.
- [ ] 4.2 New baselines (old folders renamed with a date suffix, both manifests rewritten from the final code), `docs/PERFORMANCE.md` (correct the size of the `/fp:precise` effect and describe the fix), `docs/degeneracy-information.html` if the gate changed, `tools/README.md`, memory notes. Commit, archive, push.
