# Tasks

**Order matters:** the floating-point switch first (it must change nothing but the compiler mode), then the pattern.

## 1. `/fp:precise` everywhere

- [ ] 1.1 Remove `/fp:fast` from the MSVC Release flags in `CMakeLists.txt` (the AMD and AUTO branches), state `/fp:precise` explicitly, and remove the `/fp:precise` workaround for `normal_equations_test.cpp` from `tests/CMakeLists.txt`. Verify: Release build and `ctest` pass; `44_short` with the shipped settings is IDENTICAL with the precise reference manifest from before (the old `GENERIC` build differs only in `/Oi /Ot /Oy`, which must not matter under `/fp:precise`; if it does, stop and report). Measure the speed against the last `/fp:fast` run of `44_short` (odometry 72.2 s, whole run about 107 s) with `log_stage_timing`. Make the precise output the new `44_short` baseline: `raw data/44_short_baseline` (old folder kept as `44_short_baseline_2026-10-08`) and `tools/baseline/44_short.manifest.json`. Commit.

## 2. Fixed Cholesky pattern

- [ ] 2.1 Re-apply the experiment as production code: written-element tracking in `BandedNormalMatrix` and `to_sparse()` emitting written elements; `PatternCachedCholesky` in `normal_equations.{h,cpp}`; `ScanToMapWorkspace` in `scan_to_map.h`, the `workspace` parameter of `optimize_lidar_odometry`, a per-run workspace in `optimize_windows` (not in `WindowLoopCarry`: the solvers are not copyable) and the analysis counts in the `log_stage_timing` summary. Tests: update the `BandedNormalMatrix` tests to the new meaning (written zeros stay in the pattern; the cancelling case now keeps its explicit zero); `PatternCachedCholesky` keeps the analysis for one pattern, redoes it for another, and equals a fresh object bit for bit; an optimizer window with a workspace equals one without, bit for bit, and analyses only a few times in 30 iterations. Verify: Release build and `ctest`; `44_short` within `--tolerance 1e-6` of the task 1.1 baseline folder (expect about 1e-10, and the analysis counts in the log); report the phases and the speed-up. Commit.

## 3. Full run and baselines

- [ ] 3.1 Full 44 in the background with the shipped settings. Verify: within `--tolerance 1e-6` of `raw data/44_baseline` (expect about 1e-10), and `eval_odometry.py` metrics against the old baseline unchanged. Then refresh the baselines from the final code: `raw data/44_baseline` (old kept as `44_baseline_2026-10-08b`) and `tools/baseline/44.manifest.json`, and the same for `44_short` (folder and manifest) from the final code. Report the profile. Commit.

## 4. Docs

- [ ] 4.1 `docs/PERFORMANCE.md`: a fix entry for the fixed pattern (with the table and the checks), one for `/fp:precise` (the measured cost), the optimization-flags section, the plan, and the Cholesky paragraph of fix 8 (now solved). `docs/ARCHITECTURE.md`: the two `/fp:fast` decisions replaced by "one floating-point mode, exact refactors are bit-identical on the one manifest". `CLAUDE.md`: Build (the `/fp:fast` note), Testing (the precise-build and tolerance bullets). `tools/README.md`: the tolerance explanation and the precise-build section. Verify: no document still tells to build `GENERIC` for exactness or to use `--tolerance 1e-6` for the 44. Commit.
