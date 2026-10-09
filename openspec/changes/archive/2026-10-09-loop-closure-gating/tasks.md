# Tasks

**Checks on every code commit:** the Release build of `dMapper` and `lidar_odometry_tests` passes, and `ctest` passes. Release only (no configuration-dependent code). Judged with three runs of the corrected 44 (copies `44_s0..s2`, IMU shifted +0/+1/+2 ms) plus 45f, with `settings/lidar-vertical.toml` (always on). Never two jobs on the same input folder.

## 1. Build

- [x] 1.1 Further partners: collect the qualifying earlier segments, nearest first; on a rejected registration try the next, up to `loop_closure_candidates` (3); stop at the first accepted.
- [x] 1.2 False edges: `loop_edge_residuals`; after the solve drop the worst edge while its residual exceeds `loop_closure_max_residual_m` (0.15) or `_deg` (2.0), solve again; log dropped edges and the final residuals; no edge left → trajectory unchanged.
- [x] 1.3 Settings in `OdometryConfig`, `TomlIO::POINTERS` and `CATEGORIES`; golden TOML files regenerated.
- [x] 1.4 Test (`loop_closure_test.cpp`, short): a square of 8 segments with two exact loops and one 1 m off: the false edge has the largest residual, and without it the others agree within 2 cm.

## 2. Judge

- [x] 2.1 Runs `44_lc_s0..s2` and `45f_lc`: the revisit offsets (`ghost.py`), the loops accepted, dropped and their corrections, the stall detector, `eval_odometry.py`, compared with `44_ao3_s0..s2` and `45f_ao3` (same odometry, old loop closure). Pass: no revisit offset above the old runs' at 240 s and 275 s, no new large one, 45f no worse. The user checks the cloud.

  **Result (2026-10-09):** the false edge 6 -> 49 (3.05 / 3.21 m among neighbours of about 2 m) dropped in both runs where it was accepted (residual 0.34 m); the retry found 2 -> 57 (the end of the recording) in `s0` and `s2`, and 5 -> 50 / 5 -> 51. Loops 23 / 21 / 21, largest correction 1.72 / 1.86 / 1.92 m (before 1.64 / 2.60 / 2.93 m); residuals of the kept edges ≤ 5 cm. Revisit offsets, largest (stretches > 10 cm): `s0` 51 -> 35 cm (the 35 cm is vertical at 155 s, as in most runs; horizontal ≤ 16 cm), `s1` 58 -> 25 cm (5 -> 1), `s2` 62 -> 19 cm (13 -> 4), 45f 27 -> 23 cm (5 -> 2); 45f 21 loops, none dropped. Stall detector: none on 44; on 45f the same nine stretches as in every earlier 45f run (the odometry is unchanged).

## 3. Ship

- [x] 3.1 New baselines together with `imu-always-on` (the two changes are merged in one baseline commit), docs (loop closure section and `[loop_closure]` table in `docs/index.html` and the Hungarian copy), sync the spec, archive, push.
