# Tasks

Rules: `docs/code-review-tracker.md` (status per finding); `44_short` against `tools/baseline/44_short.manifest.json` after each bit-identical commit; Release `ctest` always; Debug build + `ctest` for C27 (build flags).

## 1. Bit-identical fixes

- [x] 1.1 C7: `solution_by_row(x, n)` in `normal_equations.{h,cpp}`, used by `optimize_lidar_odometry` and consistency; the size guard counts a real failure only. Unit test: a solution with an exact zero keeps its place. Verify: `ctest`; `44_short` IDENTICAL.
- [x] 1.2 C8: big jump rejects the whole step (no pose, bias, velocity or offset change in that iteration). Unit test on a synthetic window. Verify: `ctest`; `44_short` IDENTICAL (0 big jumps on 44).
  Done 2026-10-13: `solution_by_row` (normal_equations) in the optimizer and consistency, the skip counter now counts a failed solve (`info() != Success`); `stepped_poses` decides the big jump before anything is applied (whole step rejected). Unit tests for both; Release ctest 102/102; 44_short IDENTICAL (11 files).
- [ ] 1.3 C27: MSVC warning C4715 as an error (`/we4715`). Verify: Release and Debug build, `ctest` both.
- [ ] 1.4 C12: documentation of `point_sizes_per_chunk.json` and the CSV `om/fi/ka` columns (README / output description). Verify: the text exists.

## 2. Output precision

- [ ] 2.1 C11: `.mrp` / `.mri` with `setprecision(17)`. Verify: unit test round-trips a pose at 1234.56789 m; `44_short` data files IDENTICAL, `.mrp` / `.mri` differ only in digits; new `44_short` manifest in its own commit.

## 3. Fixes that change numbers (measured)

- [ ] 3.1 C10: `ndt_cell_index` with `floor`. Unit test: points at −0.01 and +0.01 m fall in different cells. Verify: harness on s0–s5 against the noise floor (change A); new baselines in their own commit if accepted.
- [ ] 3.2 C9: loop-closure shift and plausibility about the segment centre / per node; cross-floor limit as a setting `loop_closure_cross_floor_max_shift_m = 1.0` (golden TOML files in their own commit). Unit test: the synthetic loop shifted by +200 m gives the same edges. Verify: harness from the s0–s5 snapshots (pass/fail count of the loop-closure outcome) against the floor.

## 4. Wrap-up

- [ ] 4.1 Tracker rows C7–C12, C27 updated; `master-reviewer` on the commits. Verify: entries exist.
