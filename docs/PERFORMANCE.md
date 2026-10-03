# Performance fixes

This page lists the changes that made dMapper faster or kept it from becoming slower: what was slow, what changed, what it gained, and how the result was checked. It also lists the settings that trade speed for accuracy or reproducibility, and the open performance findings.

Times are for the reference recordings in `raw data/`, with the Release build and `settings/lidar-vertical.toml`:
- `44` is the full recording, 690 windows, about 30 min;
- `44_short` is its first 30 s, 70 windows, about 3.5 min.

## Summary

| Fix | Date | Where | Gain | Results |
|---|---|---|---|---|
| [Sparse band of the normal matrix](#1-sparse-band-of-the-normal-matrix) | 2026-09-30 | window optimizer | about 35 % less time per window | bit-identical |
| [Release build by default (presets)](#2-release-build-by-default) | 2026-10-01 | build | hours → about 30 min on 44 | same code |
| [Release optimization flags](#3-release-optimization-flags) | (inherited) | build | faster arithmetic (not measured here) | last bits differ from a build without `/fp:fast` |
| [GUI frame-rate cap](#4-gui-frame-rate-cap) | 2026-10-02 | GUI | `44_short` in the GUI: 510–570 s → 195 s (CLI 197 s) | GUI = CLI again |
| [Remaining-time estimate from the odometry start](#5-remaining-time-estimate) | 2026-10-01 | GUI | a realistic estimate (no speed gain) | – |
| [Place only the next window](#6-place-only-the-next-window) | 2026-10-03 | window loop | 237,705 → 689 window placements on 44: 6.7 s → 0.02 s (about 0.4 % of the run) | last bits, within 1e-6 |

## 1. Sparse band of the normal matrix

**Problem.** Each window's Levenberg–Marquardt iteration builds a dense normal matrix: 6 unknowns per trajectory node plus 3 gyro-bias unknowns. Every iteration, the whole dense matrix was scanned and turned into a sparse matrix for the Cholesky solver. Most of it is zero, because a node is only connected to its neighbours.

**Fix.** Only the band that can hold values (neighbouring nodes and the bias unknowns) is converted to sparse form. Today this is `pose_band_to_sparse` in `app/normal_equations.cpp`, tested in `tests/normal_equations_test.cpp`.

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
- every MSVC Release build uses `/O2 /GL` with `/LTCG` (whole-program optimization);
- `AMD` (what `AUTO` picks on this machine) adds `/Oi /Ot /Oy /fp:fast`.

`/fp:fast` lets the compiler reorder and fuse floating-point operations. That makes the code faster, but its last bits depend on how the code is shaped. A refactor that is exact can move the 44 results by about 1e-9 m.

This is why refactors are checked:
- with a **precise build** (`GENERIC`, no `/fp:fast`) that must stay bit-identical;
- with the normal build within `--tolerance 1e-6`.

The two builds differ from each other by millimetres over a run, so their results are never compared with each other. See `tools/README.md` and the Decisions in `docs/ARCHITECTURE.md`.

The speed gain of `/fp:fast` over `GENERIC` has not been measured.

## 4. GUI frame-rate cap

**Problem.** `display()` ended with `glutPostRedisplay()`, so the GUI redrew the whole trajectory and the ImGui windows hundreds of times per second, competing with the processing thread. A GUI run of `44_short` took 510–570 s against 197 s on the CLI.

This also changed results. A slow window stops at `real_time_threshold_seconds`:
- `lio_result_5` differed from the CLI by 1–2 mm;
- `lio_result_7` differed by 9e-9 m.

**Fix.** `schedule_redraw()` in `app/lidar_odometry_gui.cpp` redraws on a `glutTimerFunc` timer:
- about 10 frames per second during a run;
- about 60 otherwise, so the camera stays smooth.

**Effect.** The GUI ran `44_short` in 194.9 s (0.99× the CLI), and the result is identical with the baseline.

Change `gui-frame-rate-cap`, commit `c410577`.

The live point-cloud preview added later keeps this budget. It adds at most 4 finished windows per frame, thinned to 0.10 m voxels, and a full 44 GUI run took 1.03× the CLI time. Its memory use is an open finding (below).

## 5. Remaining-time estimate

The GUI's estimate used to extrapolate from the start of the run, including data loading, so it was wrong for most of the odometry. It now extrapolates from the start of the odometry stage. The processing is not faster, but the estimate is right.

Commit `9b822c4`.

## 6. Place only the next window

**Problem.** After each window, `optimize_windows` (`app/run_odometry.cpp`) places all later windows again, each after the new end of the one before it. That is quadratic in the number of windows: about 238,000 placements on 44, where 690 are enough. Only the next window's placement is ever read.

**Fix.** Place only window `i + 1`. If a run fails at a window, place the remaining windows once, so the saved trajectory stays continuous.

**Effect.** A benchmark of the placement loops alone (690 windows of 360 nodes, Release flags) gives 6.5–6.8 s for the old loop and 0.018 s for the new one, so the fix saves about 7 s of the roughly 27-minute 44 run. That is smaller than the run-to-run variation, so the whole-run times can't show it:
- `44_short`: 163–164 s with the fix, 165 s for master;
- `44`: 1567 s with the fix, 1651 s for master the same day.

Because the old cost grows with the square of the number of windows, it would have reached about 3 % of a 5,000-window run and 12 % of a 20,000-window run. The fix also removes the rounding error of rebuilding window `j` `j` times. The results changed only in the last bits (44: at most 1e-9), and new baselines were written.

OpenSpec change `chain-next-window-only`. The full explanation, with diagrams and numbers, is in [`window-chaining.html`](window-chaining.html).

## Settings that trade speed

| Setting | Default | Effect on speed | Cost |
|---|---|---|---|
| `convergence_delta_threshold` | `1e-12` (all `nr_iter` iterations run) | `1e-2` is 3–4× faster | 0.5–1.5 % more occupied 3 cm cells on recordings 42/43, i.e. a slightly less sharp map |
| `useMultithread` | `false` | `true` is about 40 % faster per window | the parallel float sum makes results differ from run to run |
| `real_time_threshold_seconds` | `10.0` | caps a window's time | when a window hits it, results depend on machine speed and load |
| `nr_iter` | 100 | fewer iterations are faster | less converged windows (but see below: a synthetic window converges in 4) |

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

**Caveat.** A synthetic window is small, well-conditioned and has no outliers; a real window has about 360 nodes, more points and harder geometry, so it may need more iterations. This is evidence that `nr_iter = 100` is generous, not proof. The next step would be to log the iteration count per window on `44_short` and `44`. Any change to `nr_iter` or `convergence_delta_threshold` would be its own change, checked with a full 44 run and `tools/eval_odometry.py`.

## Open findings

- **GUI memory with the live preview** (2026-10-02, accepted for now). On the full 44 run, the GUI uses about 3 GB more than the CLI:
  - GUI: 1.3 → 4.3 GB during the odometry, peak 7.8 GB;
  - CLI: 1.1–1.5 GB, peak 4.5 GB.

  The cause is not known yet: the preview's voxel set, or the graphics driver. Next steps when picked up:
  - log the preview's point count;
  - a switch to turn the preview off;
  - a 0.2 m voxel if the 3-million-point cap is reached.
- **Results depend on machine load through `real_time_threshold_seconds`** (2026-10-02). One slow GUI run differed by 1–2 mm; a repeat was identical. The value was not changed. A higher limit in `settings/lidar-vertical.toml` would make runs reproducible under load, but it is a decision still to be made.
