# Proposal

## Why

The synthetic scan-to-map test (`scan-to-map-test`, 2026-10-04) found that a window settles after **4 of its 100** Levenberg–Marquardt iterations. If real windows behaved like that, cutting iterations would be the largest speed gain left: the odometry spends almost all of its ~27 minutes on 44 in these iterations.

But a synthetic window is easier than a real one, and the earlier fast-mode measurement points the other way: `convergence_delta_threshold = 1e-2` was 3–4× faster on recordings 42/43 but left the map 0.5–1.5 % less sharp. So it isn't known how many iterations real windows need, and the log can't tell today. `finished at iteration 100/100` is printed for every window, because the default threshold of 1e-12 is never reached.

This change measures it on real data before anyone touches `nr_iter`. The user asked for it on 2026-10-04 as the follow-up to the 6c measurement.

## What Changes

- **New debug setting `log_iteration_convergence`** (bool, default `false`) in `[lidar_odometry_debug_info]`, loadable and savable through the TOML table like every other setting.
- **When it is on, each window's iterations are observed, not changed.** After every iteration, the loop keeps a copy of the window's poses and the iteration's `delta`. After the window, it logs one line:
  ```
  convergence window 123: settled after 7/100 iterations (0.1 mm / 0.001 deg), delta < 1e-2 after 5, error then 0.03 mm / 0.0004 deg
  ```
  - **settled after N:** the first iteration after which the poses stay within 0.1 mm and 0.001° of the final result;
  - **delta below 1e-2 after M:** where fast mode would have stopped;
  - **error then:** how far the poses at M are from the final result.
- **At the end of the run,** one summary line: the median, 90th percentile and largest settled-after count over all windows, and the same for the fast-mode stop and its error.
- **A short tool, `tools/convergence_summary.py`,** reads a run's console log and prints the distribution: a histogram of N, and the windows with the largest error at M. It is documented in `tools/README.md`.
- **A measurement on 44 with the setting on.** The distribution goes into `docs/PERFORMANCE.md`, next to the synthetic numbers, with what it suggests for `nr_iter` and `convergence_delta_threshold`. No setting is changed here.

**Unchanged:** with the setting off, nothing runs and `dMapper` gives bit-identical results. With it on, only console lines are added: the setting only reads the poses, so the results stay bit-identical too, which is checked.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. This is a diagnostic log and a tool; no processing behaviour changes (`skip_specs: true`).

## Impact

- **Code:**
  - `app/lidar_odometry_utils.h`: the new setting;
  - `app/toml_io.h`: the `POINTERS` entry and the `lidar_odometry_debug_info` category;
  - `app/run_odometry.cpp`: `run_lm_iterations` records the poses and deltas, the window loop logs the line, `finish_run` logs the summary;
  - the golden TOML test files, regenerated in their own commit.
- **New:** `tools/convergence_summary.py`.
- **Docs:** `docs/index.html` (the setting's row), `tools/README.md`, `docs/PERFORMANCE.md` (the measured distribution).
- **Results:** bit-identical, with the setting off and on. `44_short` is checked both ways. The full 44 runs once with the setting on, which is the measurement itself, and must also be IDENTICAL with its manifest.
- **Run time with the setting on:** one copy of a window's poses per iteration (360 nodes × 128 bytes, 100 times per window), negligible against the optimization. Measured anyway.
