# Proposal

## Why

The full 44 shows a **ghost wall** (a doubled wall) in the room the recording starts in. It is not new in the code but new in the result: `raw data/44/lio_result_2` (GUI run of 2026-10-02, `/fp:fast`) is clean, and every baseline up to `44_baseline_2026-10-08b` (all `/fp:fast`) has the same trajectory as it. The `/fp:precise` runs (`44_baseline`, and the Release test `lio_result_3` on 2026-10-08) differ from it by up to 1.03 m and show the ghost wall. This was found only after `fixed-pattern-precise-fp` replaced `/fp:fast`; the switch was judged by `eval_odometry.py`, which does not see seams, and by `44_short`, which is too short to show it (2 cm there). The statement made then, "centimetres to decimetres over the 44", understated the full 44.

What the two trajectories show (`lio_result_3` against `lio_result_2`, read-only analysis of their `trajectory_lio_*.csv`):

- The first 10 s are identical to under 1 cm; the ghost wall is not in the first scan but in the **second visit of the same room**, about 3.5 min later.
- At **t = 209.3 to 210.0 s** (windows about 578 to 581, position about x 2.4, y -0.6 to -1.4, z -0.3) the precise run stalls along y: it moves about 0.15 m in 0.7 s while the clean run moves 0.9 m/s, then jumps (1.84 m/s at 210.0 s). The lag of 0.8 to 1.0 m stays for 210 to 225 s (83 cm, 52 cm at 220 s) and is gone at 230 s (1.3 cm), after loop closure and consistency. The segment of 210 to 225 s is therefore drawn about 0.8 m beside the first pass of the room.
- The per-window log (`lidar translation information window N`) shows windows 579 to 586 with the weakest direction along y, the information ratio falling from 0.59 to 0.25, in both builds. The accelerometer term (`imu_acceleration_only_when_degenerate`) switches on only from window 584, when the ratio of the previous window is below `imu_degenerate_information_ratio = 0.3`, so it does not cover the stall at windows 578 to 581. After window 585 the two builds diverge in the ratio itself (586: 0.2465 against 0.3204) and the term switches off in the precise build at 587 while the ratio is still 0.32 to 0.39 with a weak y direction (a hard gate hovering around its threshold).

The cause of the stall itself (an early stop of the Levenberg-Marquardt iterations along a weak direction, a wrong local minimum, or the missing motion prior before the gate opens) is **not yet known**. The rounding difference of the two builds only decides which way a fragile window goes; without a fix any later change can bring the ghost wall back or remove it by luck.

## What Changes

- **Reproduce and instrument** (task 1): per-window diagnostics behind a setting (time, iterations used, last delta, the window's displacement against the IMU-predicted and the previous velocity, information eigenvalues, whether the accelerometer term was active), a seam measure for `eval_odometry.py` (the deviation of a run from a reference trajectory, and the largest step difference between two runs), and a short repeatable test case cut from the 44 around the stall if the stall reproduces from a cut; otherwise the full 44 with the diagnostics for windows 570 to 600.
- **Diagnose** (task 2): report what happens in windows 575 to 590 in the clean and in the stalling run, and which of the three suspects (early stop, local minimum, missing prior) it is.
- **Fix** (task 3): the smallest change that removes the stall in both builds and does not depend on the rounding. Candidates, to be chosen from the diagnosis: soften the hard gate of the accelerometer term (a weight that grows with the degeneracy instead of an on/off switch at 0.3, with hysteresis), a velocity continuity prior that is active in every window with a sigma large enough not to hurt well-constrained windows, or a stall check (window displacement against the predicted one) that repeats the window with a stronger prior. If the fix changes the specified motion-model behaviour, a delta for `openspec/specs/lidar-odometry-motion-model` is added and `skip_specs` removed.
- **Acceptance** (task 4), all of: the full 44 has no 0.8 m lag at 210 to 225 s; the trajectory of the `/fp:precise` build is within a few centimetres of a `/fp:fast` build of the same code over the whole 44 (the build no longer decides); `eval_odometry.py` metrics and the new seam measure are no worse than the clean `lio_result_2`; the ghost wall is gone in a visual check by the user of `lio_result_N`; `44_short` and the full 44 get new baselines (the fix changes numbers) with the old ones kept dated; docs updated.

**Not in this change:** the speed work (finished); `Consistency2`; the loop closure; going back to `/fp:fast` (kept as the fallback if the fix fails: it would hide the problem, not solve it).

## Capabilities

### Modified Capabilities

Possibly `lidar-odometry-motion-model` (the gate of the accelerometer term), decided at task 3. For now none (`skip_specs: true`).

## Impact

- **Code:** `src/application/run_odometry.cpp` (window loop, gating, diagnostics), possibly `src/domain/scan_to_map.cpp` and `optimizer_options.h` (a prior), `src/domain/odometry_config.h` and `toml_io.h` (new settings, with the golden TOML files regenerated in their own commit), `tools/eval_odometry.py`; tests on synthetic windows in `tests/scan_to_map_test.cpp` and `tests/window_steps_test.cpp` for the changed behaviour (a corridor-like degenerate window must not stall).
- **Results:** change on purpose; the full 44 and `44_short` get new baselines, judged by the acceptance list above. Old baselines kept as `*_<date>`.
- **Docs:** `docs/PERFORMANCE.md` (the `/fp:precise` section, whose statement about the size of the change is corrected), `docs/degeneracy-information.html` if the gate changes, `tools/README.md`.
- **Debug build:** Release only until a task adds configuration-dependent code (`CLAUDE.md` rule).
