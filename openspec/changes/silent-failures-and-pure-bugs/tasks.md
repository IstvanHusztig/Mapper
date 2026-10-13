# Tasks

Rules: `docs/code-review-tracker.md` (status per finding); `44_short` against `tools/baseline/44_short.manifest.json` after each bit-identical commit; Release `ctest` always; Debug build + `ctest` for C27 (build flags).

## 1. Bit-identical fixes

- [x] 1.1 C7: `solution_by_row(x, n)` in `normal_equations.{h,cpp}`, used by `optimize_lidar_odometry` and consistency; the size guard counts a real failure only. Unit test: a solution with an exact zero keeps its place. Verify: `ctest`; `44_short` IDENTICAL.
- [x] 1.2 C8: big jump rejects the whole step (no pose, bias, velocity or offset change in that iteration). Unit test on a synthetic window. Verify: `ctest`; `44_short` IDENTICAL (0 big jumps on 44).
  Done 2026-10-13: `solution_by_row` (normal_equations) in the optimizer and consistency, the skip counter now counts a failed solve (`info() != Success`); `stepped_poses` decides the big jump before anything is applied (whole step rejected). Unit tests for both; Release ctest 102/102; 44_short IDENTICAL (11 files).
- [x] 1.3 C27: MSVC warning C4715 as an error (`/we4715`). Verify: Release and Debug build, `ctest` both.
  Done 2026-10-13: `/we4715` on lo_domain, lo_application, lo_infrastructure, lo_gui_view_model, dMapper and the tests (106 compile commands); Release and Debug build and `ctest` pass.
- [x] 1.4 C12: documentation of `point_sizes_per_chunk.json` and the CSV `om/fi/ka` columns (README / output description). Verify: the text exists.
  Done 2026-10-13: README output line (the CSV angle columns and `point_sizes_per_chunk.json`); `docs/index.html` and `docs/PIPELINE.md` already called the angles the raw IMU / AHRS angles.

## 2. Output precision

- [x] 2.1 C11: `.mrp` / `.mri` with `setprecision(17)`. Verify: unit test round-trips a pose at 1234.56789 m; `44_short` data files IDENTICAL, `.mrp` / `.mri` differ only in digits; new `44_short` manifest in its own commit.
  Done 2026-10-13: `setprecision(17)` in `save_poses`; test `FolderResultSink writes the chunk poses with full precision` (round trip at 1234.56789012 m); 44_short: 9 of 11 files IDENTICAL, `session_poses.mrp` / `session_ini_poses.mri` differ only in digits; new 44_short baseline in its own commit. The full 44 manifest also lists the pose files: renewed at the end of this change (C9/C10 change numbers anyway).

## 3. Fixes that change numbers (measured)

- [ ] 3.1 C10 (deferred 2026-10-13): `ndt_cell_index` with `floor`. Unit test: points at −0.01 and +0.01 m fall in different cells. Verify: harness on s0–s5 against the noise floor (change A); new baselines in their own commit if accepted.
  Tried 2026-10-13 and reverted: with `floor`, 6 synthetic tests get worse by a few mm (perfect guess 2.3 mm instead of < 1 mm, gyro bias 0.079 instead of < 0.05). Cause: the synthetic floor lies at exactly z = 0, which truncation put inside the double-high cell around 0 (a lucky case for NDT) and `floor` puts on a cell boundary. So the tests show that NDT degrades when a plane falls on a cell boundary, not that `floor` is wrong. The effect on 44 (floor not at a boundary) has to be measured on s0–s5 before deciding; the synthetic scene would then need planes off the cell boundaries.
- [x] 3.2 C9: loop-closure shift and plausibility about the segment centre / per node; cross-floor limit as a setting `loop_closure_cross_floor_max_shift_m = 1.0` (golden TOML files in their own commit). Unit test: the synthetic loop shifted by +200 m gives the same edges. Verify: harness from the s0–s5 snapshots (pass/fail count of the loop-closure outcome) against the floor.
  Done 2026-10-13 (a457d35): shift = movement of the source segment's centre; plausibility per segment anchor position; the cross-floor limit stays a named constant (`cross_floor_max_shift_m`), not a setting: the settings audit (change `settings-audit`) decides which constants become settings. Unit test at 0 and 300 m. Six-input noise floor (master, final map): façade out −0.40/−3.71/−0.58/−4.40/−1.06/−3.74°, s5 broken already in the odometry (revisit 8°, 7 m). C9 from the same snapshots: s1, s2, s4, s5 façade unchanged, s0 −0.56/−0.99 (was −0.40/−0.73), s3 offset 0.31 m (was 0.11); pass/fail count unchanged, all within the spread. Accepted by the user as a correctness fix (neutral on 44, which stays within ~70 m of the origin).

## 4. Wrap-up

- [ ] 4.1 Tracker rows C7–C12, C27 updated; `master-reviewer` on the commits. Verify: entries exist.
