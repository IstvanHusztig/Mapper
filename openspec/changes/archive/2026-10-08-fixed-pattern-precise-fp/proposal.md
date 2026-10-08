# Proposal

## Why

Two findings of 2026-10-08, decided together by the user the same day:

1. **A fixed Cholesky pattern pays.** The two sparse Cholesky solves of every Levenberg–Marquardt iteration redo their symbolic analysis (ordering and elimination tree), which is 30 % of the odometry. It could not be reused, because the sparsity pattern differed on every call: entries that are exactly zero in one iteration and tiny in the next come and go (about ±100 of 33,000, on every call). Keeping every *written* element in the pattern, zeros included, makes the pattern the same from call to call, so the analysis is done 57 + 58 times in 138,000 solves instead of every time. An experiment (not committed) on the full 44 gave: odometry 689 → 518 s (−25 %), whole run about 1090 → 908 s, results within 1e-10 of the current baseline (trajectories; JSON 2e-12), and the `eval_odometry.py` metrics unchanged to the printed precision. The elimination order changes with the pattern, so the results are not bit-identical; this is a numerical change, small enough to be judged by those numbers.
2. **`/fp:fast` buys nothing worth its cost.** Precise and fast builds ran `44_short` in 107/105 s, 147/147 s, 205/202 s and 211/214 s in the step checks of the last days (0–2 %, within noise). In return `/fp:fast` makes the long run drift in its last bits whenever unrelated code changes, needs a second "precise" build and reference for exact refactor checks, a `--tolerance 1e-6` rule for the full 44, and even reordered the reference sum of a unit test. One floating-point mode everywhere removes all of that.

Both change the numbers a little, so they share one baseline refresh.

## What Changes

- **Fixed superset Cholesky pattern.** `BandedNormalMatrix` records which elements were ever written, and `to_sparse()` emits exactly those (explicit zeros included), instead of the nonzero ones. A `PatternCachedCholesky` keeps the symbolic analysis while the pattern compares equal and re-analyses when it differs. A `ScanToMapWorkspace` with the two solvers (covariance of the extra unknowns, damped solve) lives for one run and is passed to `optimize_lidar_odometry` (default `nullptr`: then every call analyses, as now). With `log_stage_timing` the end-of-odometry summary also prints how many analyses were done.
- **`/fp:precise` everywhere (MSVC).** `/fp:fast` is removed from the Release flags in `CMakeLists.txt` (`/Oi /Ot /Oy` stay), `/fp:precise` is stated explicitly, and the `/fp:precise` workaround for `tests/normal_equations_test.cpp` goes. The `HD_CPU_OPTIMIZATION` option stays.
- **One set of baselines.** `tools/baseline/44_short.manifest.json`, `44.manifest.json` and the baseline folders are refreshed from the final code. The separate precise reference and the `--tolerance 1e-6` rule for the full 44 are no longer needed and leave the docs; exact refactors are again checked against the one manifest, bit for bit.

**Unchanged:** the maths, the settings, the file formats. The results move by at most about 1e-10 relative to the current baseline (to be confirmed on the final code). The `/fp:precise` switch alone must reproduce the existing precise reference exactly.

## Capabilities

### Modified Capabilities

None. A performance and build change with a numerically negligible effect (`skip_specs: true`).

## Impact

- **Code:** `CMakeLists.txt`, `tests/CMakeLists.txt`, `src/domain/normal_equations.{h,cpp}`, `src/domain/scan_to_map.{h,cpp}`, `src/application/run_odometry.cpp`; tests in `tests/normal_equations_test.cpp` and `tests/scan_to_map_test.cpp`.
- **Baselines:** `tools/baseline/44.manifest.json`, `44_short.manifest.json`, `raw data/44_baseline`, `raw data/44_short_baseline` (old folders kept with a date suffix).
- **Docs:** `docs/PERFORMANCE.md` (two fix entries, the optimization-flags section, the settings and plan), `docs/ARCHITECTURE.md` (the two decisions about `/fp:fast`), `CLAUDE.md` (Testing and Build), `tools/README.md` (baseline checks; the precise-build section goes).
- **Debug build:** no new asserts and no configuration-dependent code, so Release only (`CLAUDE.md` rule).
