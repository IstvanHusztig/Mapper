# Performance fixes

This page lists the changes that made dMapper faster or kept it from becoming slower: what was slow, what changed, what it gained, and how the result was checked. It also lists the settings that trade speed for accuracy or reproducibility, and the open performance findings.

Times are for the reference recordings in `raw data/`, with the Release build and `settings/lidar-vertical.toml`:
- `44` is the full recording, 690 windows, about 10 min (30 min before the performance fixes);
- `44_short` is its first 30 s, 70 windows, about 1.5 min (3.5 min before).

## Summary

| Fix | Date | Where | Gain | Results |
|---|---|---|---|---|
| [Sparse band of the normal matrix](#1-sparse-band-of-the-normal-matrix) | 2026-09-30 | window optimizer | about 35 % less time per window | bit-identical |
| [Release build by default (presets)](#2-release-build-by-default) | 2026-10-01 | build | hours → about 30 min on 44 | same code |
| [Release optimization flags](#3-release-optimization-flags) | (inherited) | build | faster arithmetic (not measured here) | last bits differ from a build without `/fp:fast` |
| [GUI frame-rate cap](#4-gui-frame-rate-cap) | 2026-10-02 | GUI | `44_short` in the GUI: 510–570 s → 195 s (CLI 197 s) | GUI = CLI again |
| [Remaining-time estimate from the odometry start](#5-remaining-time-estimate) | 2026-10-01 | GUI | a realistic estimate (no speed gain) | – |
| [Place only the next window](#6-place-only-the-next-window) | 2026-10-03 | window loop | 237,705 → 689 window placements on 44: 6.7 s → 0.02 s (about 0.4 % of the run) | last bits, within 1e-6 |
| [Deterministic multithreading](#7-deterministic-multithreading) | 2026-10-05 | window optimizer | 44: odometry 1578 → 1060 s (−33 %), whole run 1990 → 1467 s (−26 %) | bit-identical |
| [Banded normal matrix](#8-banded-normal-matrix) | 2026-10-08 | window optimizer | 44: odometry 1060 → 689 s (−35 %), whole run 1467 → 1137 s (−22 %) | bit-identical on `44_short`; full 44 within 1e-9 |
| [Fixed Cholesky pattern](#9-fixed-cholesky-pattern) | 2026-10-08 | window optimizer | 44: odometry 689 → 510 s (−26 %), whole run about 1090 → 908 s | within 1e-10 |
| [One floating-point mode (`/fp:precise`)](#10-one-floating-point-mode) | 2026-10-08 | build | no speed cost measured | results change by cm–dm over the 44 (new baselines) |
| [Faster consistency](#11-faster-consistency) | 2026-10-08 | consistency | 44: consistency (10 passes) 363.6 → 44.7 s (−88 %), processing time 888 → 628 s | within 1e-10 (new baselines) |
| [Stored NDT cell inverses](#12-stored-ndt-cell-inverses) | 2026-10-08 | window optimizer, consistency | 44: odometry 517 → 490 s (−5 %), processing time 628 → 601 s | bit-identical |

## 1. Sparse band of the normal matrix

**Problem.** Each window's Levenberg–Marquardt iteration builds a dense normal matrix: 6 unknowns per trajectory node plus 3 gyro-bias unknowns. Every iteration, the whole dense matrix was scanned and turned into a sparse matrix for the Cholesky solver. Most of it is zero, because a node is only connected to its neighbours.

**Fix.** Only the band that can hold values (neighbouring nodes and the bias unknowns) is converted to sparse form. Today this is `pose_band_to_sparse` in `src/domain/normal_equations.cpp`, tested in `tests/normal_equations_test.cpp`.

**Effect.** About 35 % less time per window, with bit-identical results. It also reduced how often a window reached the 10 s `real_time_threshold_seconds` under CPU load. Before this fix, recording b1 needed up to 9 s per window, so the time limit could cut iterations and change results.

Commit `fea1751`.

## 2. Release build by default

**Problem.** Visual Studio's default configuration was an unoptimized Debug build (`/Od`). A Ctrl+F5 run of 44 took hours, and the remaining-time estimate looked absurd. The build looked like Release, but wasn't.

**Fix.** `CMakePresets.json` defines exactly two configurations, `x64-Release` and `x64-Debug` (Ninja + cl, `out/build/<preset>`). Data is always processed with `x64-Release`. Other build trees are not to be added.

**Effect.** 44 runs in about 30 min instead of hours.

**Check first** when a run is slow: the path of the running `dMapper.exe` must contain `x64-Release`.

Commit `9b822c4`.

## 3. Release optimization flags

`-DHD_CPU_OPTIMIZATION=AUTO|INTEL|AMD|ARM|GENERIC` in `CMakeLists.txt` sets the Release flags:
- every MSVC Release build uses `/O2 /GL /fp:precise` with `/LTCG` (whole-program optimization);
- `AMD` (what `AUTO` picks on this machine) adds `/Oi /Ot /Oy`.

Until 2026-10-08 the `AMD` build also used `/fp:fast`; see [fix 10](#10-one-floating-point-mode) for why it is gone.

## 4. GUI frame-rate cap

**Problem.** `display()` ended with `glutPostRedisplay()`, so the GUI redrew the whole trajectory and the ImGui windows hundreds of times per second, competing with the processing thread. A GUI run of `44_short` took 510–570 s against 197 s on the CLI.

This also changed results. A slow window stops at `real_time_threshold_seconds`:
- `lio_result_5` differed from the CLI by 1–2 mm;
- `lio_result_7` differed by 9e-9 m.

**Fix.** `schedule_redraw()` in `src/presentation/gui/lidar_odometry_gui.cpp` redraws on a `glutTimerFunc` timer:
- about 10 frames per second during a run;
- about 60 otherwise, so the camera stays smooth.

**Effect.** The GUI ran `44_short` in 194.9 s (0.99× the CLI), and the result is identical with the baseline.

Change `gui-frame-rate-cap`, commit `c410577`.

The live point-cloud preview added later keeps this budget. It adds at most 4 finished windows per frame, thinned to 0.10 m voxels, and a full 44 GUI run took 1.03× the CLI time. Its memory use is an open finding (below).

## 5. Remaining-time estimate

The GUI's estimate used to extrapolate from the start of the run, including data loading, so it was wrong for most of the odometry. It now extrapolates from the start of the odometry stage. The processing is not faster, but the estimate is right.

Commit `9b822c4`.

## 6. Place only the next window

**Problem.** After each window, `optimize_windows` (`src/application/run_odometry.cpp`) places all later windows again, each after the new end of the one before it. That is quadratic in the number of windows: about 238,000 placements on 44, where 690 are enough. Only the next window's placement is ever read.

**Fix.** Place only window `i + 1`. If a run fails at a window, place the remaining windows once, so the saved trajectory stays continuous.

**Effect.** A benchmark of the placement loops alone (690 windows of 360 nodes, Release flags) gives 6.5–6.8 s for the old loop and 0.018 s for the new one, so the fix saves about 7 s of the roughly 27-minute 44 run. That is smaller than the run-to-run variation, so the whole-run times can't show it:
- `44_short`: 163–164 s with the fix, 165 s for master;
- `44`: 1567 s with the fix, 1651 s for master the same day.

Because the old cost grows with the square of the number of windows, it would have reached about 3 % of a 5,000-window run and 12 % of a 20,000-window run. The fix also removes the rounding error of rebuilding window `j` `j` times. The results changed only in the last bits (44: at most 1e-9), and new baselines were written.

OpenSpec change `chain-next-window-only`. The full explanation, with diagrams and numbers, is in [`window-chaining.html`](window-chaining.html).

## 7. Deterministic multithreading

**Problem.** The NDT point loops of `optimize_lidar_odometry` took 47 % of the odometry and ran on one of the machine's 16 cores. `useMultithread = true` ran them in parallel, but the results changed from run to run. Each point added its terms into its own trajectory node's block under a per-node lock, so the points of a node arrived in thread order, and floating-point sums depend on order. Because of that the setting was off.

**Fix.** The points are grouped by node, keeping their order (a counting sort per call), and the nodes run in parallel (`std::execution::par`). Each node's points are added in their original order, with no lock. A point writes only its own node's block, so every block receives exactly the additions of the serial loop, in the same order. The results are therefore the same bits with or without threads. `useMultithread` was first made `true` by default, then removed as a setting (change `builtin-multithread`, 2026-10-06): the parallel loop is always on. The serial path is the same code with a sequential loop over the nodes. The old lock inside `par_unseq` was formally undefined behaviour, and it is gone.

**Effect** (fast build, with `log_stage_timing`; before = the `stage-timing` profile):

| | `44_short` before | `44_short` after | `44` before | `44` after |
|---|---|---|---|---|
| NDT indoor loop (with the grouping), per iteration | 6.27 ms | 1.76 ms | 6.05 ms | 1.74 ms |
| NDT outdoor loop, per iteration | 5.06 ms | 0.85 ms | 4.63 ms | 0.88 ms |
| odometry | 166.0 s | 108.8 s | 1578.5 s | 1059.6 s (−33 %) |
| whole run | 200 s | 144 s | 1990 s | 1467 s (−26 %) |

The two loops together run 4.1× faster. They are now 17 % of the odometry. The dense normal-matrix setup (4.1 ms per iteration, 27 %) is the largest phase, which is the next change, `banded-normal-matrix`.

**Checks.**
- Unit tests in `tests/scan_to_map_test.cpp`, on the synthetic room and corridor, with points in order and shuffled: multithreaded and single-threaded optimizations give exactly equal poses, bias and translation information, and two multithreaded runs are equal. Before the fix, all ten checks failed on every run.
- `44_short`, precise and fast builds: IDENTICAL with the setting off and on.
- The full 44 with the new default: IDENTICAL with the manifest; only the params TOML records the settings.

OpenSpec change `deterministic-multithread`.

## 8. Banded normal matrix

**Problem.** After fix 7, 46 % of the odometry went on one dense matrix. Each Levenberg–Marquardt iteration allocated and zeroed the window's 2166×2166 normal matrix (37 MB for 360 nodes), filled a narrow band of it, copied that band into a sparse matrix, and freed it again, 100 times per window. Per iteration on 44: 4.09 ms for the setup, about 1.7 ms for the rest (mostly freeing it), 1.30 ms for the copy.

**Fix.** `BandedNormalMatrix` (`src/domain/normal_equations.{h,cpp}`) stores only the band: 17 parameters on each side of the diagonal (more in the few windows where the accelerometer term is active, from the span of its terms), plus the rows and columns of the extra unknowns (gyro bias, acceleration offset). Every element receives exactly the additions, in the same order, that the dense element got, so the sums are the same bits. Its conversion to a sparse matrix gives the same entries and the same pattern as the old band copy, so the Cholesky ordering is unchanged. The band is computed before anything is added, since it depends only on the timestamps. The fix-origin term is added to the band directly instead of through three small sparse products.

**Effect** (fast build, `log_stage_timing`; before = fix 7):

| | `44_short` before | `44_short` after | `44` before | `44` after |
|---|---|---|---|---|
| setup, per iteration | 4.08 ms | 0.12 ms | 4.09 ms | 0.17 ms |
| rest of the iteration (freeing), total | 12.7 s | 0.8 s | 118.2 s | 3.7 s |
| assembly, per iteration | 1.30 ms | 1.00 ms | 1.30 ms | 1.11 ms |
| odometry | 108.8 s | 72.2 s | 1059.6 s | 688.7 s (−35 %) |
| whole run | 144 s | 107 s | 1467 s | 1137 s (−22 %) |

**Checks.**
- `tests/normal_equations_test.cpp` compares the type with a dense matrix and the old band copy, bit for bit and pattern included (random blocks, edges, couplings with 0, 3 and 6 extra unknowns, bands of 17 and 113, cancelling additions, writes outside the band). That test file is compiled with `/fp:precise`: with `/fp:fast` the compiler reordered the test's own dense reference sum and the comparison failed by 1e-12 on 5 % of the elements, although both sides equal a plain sequential sum in isolation.
- `44_short`, precise and fast builds: IDENTICAL with the precise reference and the manifest.
- Full 44, fast build: within 1e-9 of the baseline (trajectories; results JSON 8e-14), not bit-identical. This is the `/fp:fast` effect on the long run when the shape of a function changes; the 44 baselines were refreshed from this run.

OpenSpec change `banded-normal-matrix`.

### Cholesky symbolic reuse: measured, not possible bit-identically

The two sparse Cholesky solves were 44 % of the odometry, and on `44_short` `analyzePattern` (the fill-reducing ordering and the elimination tree) was 21.6 s of the 72.1 s (30 %), the numeric `factorize` only 6.7 s. Reusing the analysis does not work while the result must stay the same bits: the analysis depends only on the sparsity pattern, but the pattern differed on every call. The number of stored entries wobbled by about ±100 of 33,000 between iterations, with about 930 of 2160 columns differing by a few entries (entries that are exactly zero in one iteration and tiny in the next; 6,900 analyses in 6,900 covariance solves, 7,000 in 7,000 damped solves). A cache guarded by a pattern comparison never hit. Fix 9 solves it with a fixed pattern, which is a numerical change.

## 9. Fixed Cholesky pattern

**Problem.** See the end of fix 8: the symbolic analysis of the two Cholesky solves was 30 % of the odometry, and could not be reused because the sparsity pattern changed on every call (exactly-zero entries came and went).

**Fix.** `BandedNormalMatrix` records which elements were written, and `to_sparse()` emits exactly those, explicit zeros included. The pattern then depends on which terms were added and not on whether a sum happens to be exactly zero, so it is the same from one iteration to the next. A `PatternCachedCholesky` keeps the symbolic analysis while the pattern compares equal (and redoes it when it differs, so a changed pattern can never give a wrong result), and a `ScanToMapWorkspace` with the two solvers (covariance of the extra unknowns, damped solve) lives for one run. With `log_stage_timing` the summary prints the analysis counts: on the full 44, 57 + 58 analyses in 68,900 + 69,000 solves (on `44_short` 6 + 7 in 6,900 + 7,000).

The explicit zeros change the fill-reducing ordering, so the solves round differently: this is a numerical change, not a refactor. It is small: on `44_short` the trajectories moved by at most 1e-10 (JSON 4e-12), and the experiment on the full 44 (fast build, against the fast baseline) by at most 1e-10 with the `eval_odometry.py` metrics unchanged to the printed precision.

**Effect** (`44_short`, `/fp:precise`): covariance solve 2.30 → 0.67 ms and damped solve 2.78 → 1.18 ms per iteration; odometry 68.8 → 52.8 s (−23 %), processing time 101.5 → 86.2 s. Full 44: odometry 510 s (689 s before), processing time 888 s.

**Checks.** `tests/normal_equations_test.cpp`: written zeros stay in the pattern, a diagonal block writes all 36 elements, the cache keeps the analysis for one pattern and redoes it for another pattern or size and always equals a fresh object bit for bit. `tests/scan_to_map_test.cpp`: an optimizer window with a workspace equals one without, bit for bit, with at most 3 analyses in 30 iterations.

OpenSpec change `fixed-pattern-precise-fp`.

## 10. One floating-point mode

**Problem.** The `AMD` Release build (what `AUTO` chose here) used `/fp:fast`. It was meant to be faster, but its gain was never measured, and it made results depend on how the compiler happened to order sums: an exact refactor could move the long run by about 1e-9, so exact refactors were checked with a second "precise" build against a second reference, the full 44 had a `--tolerance 1e-6` rule, and a unit test's own reference sum was reordered into failing.

**Measured.** Fast and precise builds ran `44_short` in 107/105 s, 147/147 s, 205/202 s, 211/214 s in the step checks of 2026-10-05 to 08, and with the final change (below) the precise build is the same speed. So `/fp:fast` bought 0–2 %, within run-to-run noise.

**Fix.** `/fp:fast` is gone from `CMakeLists.txt`; MSVC Release uses `/O2 /GL /fp:precise` everywhere, `/Oi /Ot /Oy` stay. The `/fp:precise` workaround for the `BandedNormalMatrix` test is gone. `44_short` built this way is bit-identical with the old precise reference, so the switch changes nothing but the compiler mode.

**What it costs: new numbers.** Precise and fast results are not the same, and the difference is larger than the earlier "millimetres over a run":
- `44_short`: the two baselines differ by up to 2 cm in position;
- the full 44: the calculated route is 215.46 m long against 216.06 m, and some per-window gyro-bias estimates differ by up to 0.19 °/s.

The quality metrics of `tools/eval_odometry.py` are equal or slightly better with precise (jitter 0.068/0.37 → 0.069/0.37 °, window yaw difference 1.37 → 1.33 °, spikes above 3 °: 6 → 4, final yaw −126.4 → −126.0 ° against the gyro's −136.9 → −136.8 °). With no ground truth neither mode can be called closer to the truth; the point of the switch is that the result no longer depends on compiler reordering. The old fast baselines are kept as `raw data/44_baseline_2026-10-08b` and `raw data/44_short_baseline_2026-10-08b`.

**Consequences.** One build and one set of baselines: a refactor of numeric code is checked on `44_short`, and the full 44 once per change, bit for bit against the manifests; the tolerance rule and the precise-build procedure are gone.

OpenSpec change `fixed-pattern-precise-fp`.

## 11. Faster consistency

**Problem.** `Consistency` was 363.6 s of the 888 s full 44 (10 passes). Its profile (`log_stage_timing`): sparse products 156.6 s (43 %), sparse build 50.6 s (14 %), serial point loop 48.4 s (13 %), smoothness term 30.6 s (8 %), solve 24.3 s (7 %), chunk preparation 22.4 s (6 %), loading the points 12.8 s, `update_rgd` 12.7 s.

**Fix.**
- The normal equations are accumulated directly into a `BandedNormalMatrix` (band 17) and a dense right-hand side, with no triplet lists and no sparse products: the point terms of each node in parallel, in point order (a point writes only its own node's block); the smoothness blocks in parallel batches added in order; then `to_sparse()` and the Cholesky.
- A `ConsistencyData` object lives for one run: the points with their nodes are loaded once instead of in every pass, and the Cholesky symbolic analysis is kept (`PatternCachedCholesky`) because the pattern is the same in every pass.
- The chunks of 1M points (each with its own NDT map) are built in parallel, at most 8 at a time, and their partial sums are merged in chunk order, so the result does not depend on the thread timing.

**Effect** (full 44, 10 passes): consistency 363.6 → 87.7 s (direct accumulation) → 44.7 s (shared data, parallel chunks): chunks 18.6 s, assembly 9.2 s, solve 9.2 s, smoothness 4.6 s, loading 1.3 s. `44_short` consistency 38.7 → 13.2 s after the first step. Whole full 44: processing time 888 → 628 s (odometry 517 s, consistency 45 s).

**Checks.** `ctest` passes; the recorded numbers of the consistency tests are unchanged to the printed precision, and a new case runs several chunks twice and requires identical results. The full 44 is within 1e-10 of the previous baseline (`eval_odometry.py` metrics unchanged to the printed precision); a repeat of `44_short` is bit-identical with itself. Because the order of the sums changed the results are not bit-identical with the old baseline, so the baselines were refreshed (old ones kept as `*_2026-10-08c`). Not added: a dedicated unit test of the assembled normal equations (the assembly is not exposed); the end-to-end consistency tests cover it.

OpenSpec change `consistency-speedup`.

## 12. Stored NDT cell inverses

**Problem.** In each of the 69,000 Levenberg–Marquardt iterations of the full 44, every point that hit an NDT cell inverted that cell's 3×3 covariance again (`cov.inverse()` and the two threshold tests), and converted its node's pose to Tait-Bryan form (trigonometry), although both depend only on the cell or the node. A micro-benchmark (serial, 600,000 points against the synthetic room and corridor maps) split the work of one point into: transform 5 %, cell index + `find` 11 %, inverse + thresholds 19 %, pose conversion 12 %, the two generated terms 53 %.

**Fix.** `NdtBucket` stores `cov_inverse` and whether the cell is usable; `update_rgd` refreshes the cells it added or changed once, at its end (`refresh_cell_inverse`), and the loops read them through `ndt_cell_inverse` (a cell that never went through `update_rgd` is inverted on the spot with the same expression). The node's pose is converted once per node instead of once per point, in both NDT loops and in `Consistency`. The values are computed by the same expressions, so the results are the same bits.

**Effect** (`44_short`, 7000 calls each): indoor loop 1.905 → 1.694 ms (−11 %), outdoor loop 0.957 → 0.836 ms (−13 %), odometry 51.9 → 49.8 s. Full 44: indoor 1.613 ms and outdoor 0.810 ms per iteration, odometry 516.7 → 490.5 s, processing time 628 → 601 s (wall-clock 622 s). The gain is smaller than the serial benchmark promised, because the loops already run on 16 threads and are partly memory-bound.

**Not done.** A flat lookup table instead of `unordered_map::find` (task 3.1 of the change): `find` was about 11 % of the serial loop work, so about 4 % of the odometry at best, for a changed container read by several threads.

**Checks.** `tests/ndt_map_test.cpp`: after one and after two `update_rgd` calls (both statistics modes) every cell's stored inverse equals `cov.inverse()` bit for bit and the usable flag matches the thresholds. `tests/scan_to_map_test.cpp`: an optimizer run with the stored inverses equals one inverting on the spot, bit for bit. `ctest` passes; `44_short` IDENTICAL with its manifest; the full 44 differs from its manifest only in the parameter file (the timing setting of the run).

OpenSpec change `ndt-lookup-cache`.

## Settings that trade speed

| Setting | Default | Effect on speed | Cost |
|---|---|---|---|
| `convergence_delta_threshold` | `1e-12` (all `nr_iter` iterations run) | `1e-2` is 3–4× faster | 0.5–1.5 % more occupied 3 cm cells on recordings 42/43, i.e. a slightly less sharp map |
| `real_time_threshold_seconds` | `10.0` | caps a window's time | when a window hits it, results depend on machine speed and load |
| `nr_iter` | 100 | fewer iterations are faster | real windows settle after median 51, at most 91 iterations on 44 (see below) |

The defaults favour reproducible, sharp results over speed. `docs/index.html` has the details of each setting.

### How many iterations a window needs (measured 2026-10-04)

The unit test `scan-to-map: iterations needed` (`tests/scan_to_map_test.cpp`) runs the 100 iterations of a real run on a synthetic window: 50 nodes in a closed room, 60 points per node with ±5 mm noise, and a guess whose heading drifts at 0.5 °/s, as a gyro bias makes it do.

| | Without bias estimation | With bias estimation |
|---|---|---|
| Poses stop changing (within 0.1 mm / 0.001°) after | 4 iterations | 4 iterations |
| `delta` first below `1e-2` (fast mode) after | 3 iterations | 3 iterations |
| Error against the truth at that point | 1.777 mm / 0.0472° | 1.776 mm / 0.0472° |
| Error against the truth after 100 iterations | 1.771 mm / 0.0472° | 1.770 mm / 0.0472° |
| Time per iteration (Release) | 3.1 ms | 3.6 ms |

On this window, 96 of the 100 iterations change nothing, and stopping at the fast-mode threshold costs 0.006 mm.

A synthetic window is small, well-conditioned and has no outliers; a real window has about 360 nodes, more points and harder geometry. So the same two numbers were measured on the reference recordings.

### How many iterations a real window needs (measured 2026-10-04)

The debug setting `log_iteration_convergence` (`[lidar_odometry_debug_info]`, off by default) prints one `convergence window …` line per window and a summary at the end of the run. `tools/convergence_summary.py` turns a saved console log into histograms. The definitions are those of the synthetic test: a window has *settled* after the first iteration from which its poses stay within 0.1 mm and 0.001° of the result after all iterations; *fast stop* is the first iteration with `delta < 1e-2`, and its error is the largest node difference from the final poses at that point. The results are bit-identical with the setting on.

| | Synthetic window | `44_short` (70 windows) | `44` (690 windows) |
|---|---|---|---|
| Settled after (median / p90 / max) | 4 | 52 / 74 / 83 | 51 / 79 / 91 |
| `delta` first below `1e-2` after (median / p90 / max) | 3 | 19 / 43 / – | 27 / 52 / 72 |
| Error at that point against the final poses (median / p90 / max) | 0.006 mm | 0.013 / 0.051 / 0.174 mm | 0.016 / 0.12 / 1.48 mm (window 329) |

Every window reached `delta < 1e-2`, and every window settled before iteration 100. The 44 odometry took 1562 s with the setting on (1567 s for the last run without it), about 23 ms per iteration.

**What this suggests** (no setting was changed):
- **`nr_iter`:** a real window needs about ten times as many iterations as the synthetic one, and the slowest settle at 91 of 100. Part of that is the damping, which grows ×10 every 10 iterations and freezes the poses towards the end. A smaller `nr_iter` would save time only by cutting windows that are still moving, so 100 stays.
- **`convergence_delta_threshold = 1e-2`:** it would stop a window after median 27 instead of 100 iterations, which matches the 3–4× speed-up measured earlier. The poses it leaves behind differ from the full run by a median 0.016 mm and at most 1.5 mm, but those small differences add up along the trajectory; on recordings 42/43 the map was 0.5–1.5 % less sharp. It remains the fast mode, not the default.
- A change to either would be its own change, checked with a full 44 run and `tools/eval_odometry.py`.

With `use_exact_ndt_bucket_statistics = true` (off by default, change `ndt-bucket-statistics`), the median window settled after 29 iterations on 44 instead of 51, while p90 and max stayed at 77 and 94. Its run time couldn't be compared fairly: the switch-off reference was a night run, and on the same morning a switch-off `44_short` run took 206 s against 199 s with the switch on.

## Analysis (2026-10-05) and planned fixes

A code review of the processing path; the measured profile that followed it is [below](#measured-profile-2026-10-05). A window has 360 nodes, i.e. 2166 unknowns, and runs 100 LM iterations of about 23 ms each.

**Why `useMultithread` was not deterministic** (fixed since, see [fix 7](#7-deterministic-multithreading)). Only `optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) is affected. Each point adds its 6×6 / 6×1 terms into its own node's diagonal block under a per-node mutex; with `par_unseq` the points of one node arrive in thread order, so the sums differ in the last bits. (A mutex inside `par_unseq` is also formally undefined behaviour.) `optimize_sf2` and `optimize_rigid_sf` already compute per point in parallel and sum in order (`SumBlocks`), so they are deterministic.

**Planned fix:** run the nodes in parallel and sum each node's points in their original order. That is exactly the serial order of additions per block, so the multithreaded result should be bit-identical with the serial one, and `useMultithread` could then default to `true`.

**Other gaps, by likely impact:**

| # | Where | Gap | Fix (results unchanged) |
|---|---|---|---|
| 1 | `scan_to_map.cpp` | a dense 2166×2166 normal matrix (37 MB) is allocated and zeroed every LM iteration, about 69,000 times on 44; only its band is read | a block-band store with the same `+=` order |
| 2 | `scan_to_map.cpp` | per point and iteration: `cov.inverse()` + threshold test, `pose_tait_bryan_from_affine_matrix`, the planarity eigen-solve | cache per bucket (map is constant during a window) and per pose |
| 3 | `scan_to_map.cpp` | `SimplicialCholesky` rebuilt every iteration though the pattern never changes | `analyzePattern` once, `factorize` per iteration |
| 4 | `ndt_map.h` | NDT maps are node-based `std::unordered_map` (hash + pointer chase per lookup, per point, 2 grids × 100 iterations) | a flat open-addressing map, where nothing depends on iteration order |
| 5 | several | `auto _buckets = buckets;` per window in `optimize_rigid_sf`; `points_to_vector` takes points by value; `load_point_cloud` has no `reserve` and doesn't destroy the laszip handle; `save_result` writes `session_chunk` files and reads them straight back; consistency re-reads every window file each iteration; `ImuTrajectory` is a `std::map` used like a vector | remove copies, reserve, cache, prefetch the next window |

### Measured profile (2026-10-05)

The debug setting `log_stage_timing` (`[lidar_odometry_debug_info]`, off by default; change `stage-timing`) logs the time of each pipeline step and, at the end of the odometry, where the window loop spent it. The results are bit-identical with the setting on. These runs used the Release (`/fp:fast`) build with `settings/lidar-vertical.toml` and nothing else running; the machine has 16 logical cores, and the run uses one of them.

**Pipeline steps:**

| Step | `44_short` (70 windows) | `44` (690 windows) |
|---|---|---|
| `load_data` | 1.0 s | 7.6 s |
| `calculate_trajectory` | 0.0 s | 0.1 s |
| `compute_step_1` | 0.4 s | 4.2 s |
| `odometry` | 166.0 s (83 %) | 1578.5 s (79 %) |
| `loop_closure` | 0.2 s | 53.7 s (3 %) |
| `consistency` | 30.8 s (15 %) | 324.5 s (16 %) |
| `save` | 1.8 s | 21.7 s (1 %) |

**Inside the odometry** (`44`, 69,000 LM iterations of about 22.8 ms; `44_short` has the same shares within 1 %):

| Phase | Time | Share of odometry | Per iteration |
|---|---|---|---|
| NDT point loop, indoor grid | 417.5 s | 26.4 % | 6.05 ms |
| NDT point loop, outdoor grid | 319.2 s | 20.2 % | 4.63 ms |
| setup: allocate and zero the dense normal matrix | 277.4 s | 17.6 % | 4.02 ms |
| damped Cholesky solve | 138.7 s | 8.8 % | 2.01 ms |
| covariance solve (gyro bias, acc. offset) | 117.4 s | 7.4 % | 1.70 ms |
| rest of the iterations (see below) | 112.7 s | 7.1 % | 1.63 ms |
| motion-model, bias, velocity and acc. terms | 102.1 s | 6.5 % | 1.48 ms |
| assembly: fixed-origin triplets, `pose_band_to_sparse` | 84.7 s | 5.4 % | 1.23 ms |
| pose update | 3.4 s | 0.2 % | 0.05 ms |
| window loop outside the iterations (point load, map update, preview) | 5.4 s | 0.3 % | – |

"Rest of the iterations" is the time in `run_lm_iterations` after each call's `update` timer stopped: the end of `optimize_lidar_odometry`, where its matrices are freed (among them the 37 MB normal matrix), and the loop around the calls. That is read from the code; it isn't timed separately.

**What the profile says:**
- **The NDT point loops are 47 % of the odometry (37 % of the whole run).** They are what `useMultithread` runs in parallel, so `deterministic-multithread` comes first. With 16 cores, an ideal speed-up of those loops would save about 690 s of the 1990 s run on 44.
- **The dense normal matrix costs at least 17.6 % of the odometry**, and up to about 25 % with freeing it. That is more than the 2–4 ms estimate expected, and it confirms `banded-normal-matrix` as second. The band store would also shrink `assembly`.
- **The two Cholesky solves are 16 %.** Reusing the symbolic analysis goes with `banded-normal-matrix`.
- **Consistency is 16 % of the whole run**, more than the review assumed. It is serial, rereads every window file in each of its passes, and builds a large sparse triple product. It deserves its own analysis.
- **I/O is negligible in the odometry:** loading points takes 0.7 s and the map update 4.4 s on 44. `io-and-copies` therefore goes last. The save takes 22 s, and its `session_chunk` round trip is part of that.

**Planned order**, updated from the profile; each is its own change, bit-identical against the manifests:
1. `deterministic-multithread` (done, [fix 7](#7-deterministic-multithreading));
2. `banded-normal-matrix` (done, [fix 8](#8-banded-normal-matrix)), and the fixed Cholesky pattern (done, [fix 9](#9-fixed-cholesky-pattern));
3. `ndt-lookup-cache` (done, [fix 12](#12-stored-ndt-cell-inverses); the flat lookup table was measured as not worth it);
4. the consistency analysis (done, [fix 11](#11-faster-consistency));
5. `io-and-copies`.

Result-changing levers come later, each judged with `eval_odometry.py`: a convergence stop rule, and per-pose blocks in `Consistency`.

## Open findings

- **GUI memory with the live preview** (2026-10-02, accepted for now). On the full 44 run, the GUI uses about 3 GB more than the CLI:
  - GUI: 1.3 → 4.3 GB during the odometry, peak 7.8 GB;
  - CLI: 1.1–1.5 GB, peak 4.5 GB.

  The cause is not known yet: the preview's voxel set, or the graphics driver. Next steps when picked up:
  - log the preview's point count;
  - a switch to turn the preview off;
  - a 0.2 m voxel if the 3-million-point cap is reached.
- **Two suspected bugs found in the 2026-10-05 review** (not fixed; each changes results, so each is its own change with an evaluation):
  - `Consistency2` sets `index_end_exclusive = i` (`src/application/consistency.cpp`), which looks like it drops the last point of every cell;
  - `ndt_cell_index` uses `static_cast<int16_t>(p / b)`, which truncates toward zero, so cell 0 is twice as wide on every axis (`src/domain/ndt_map.cpp`).
- **Results depend on machine load through `real_time_threshold_seconds`** (2026-10-02). One slow GUI run differed by 1–2 mm; a repeat was identical. The value was not changed. A higher limit in `settings/lidar-vertical.toml` would make runs reproducible under load, but it is a decision still to be made.
