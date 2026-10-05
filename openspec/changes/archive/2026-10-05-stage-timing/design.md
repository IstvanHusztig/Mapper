# Design

## Context

- `OdometryPipeline::load` (`src/application/odometry_pipeline.cpp`) runs `load_data`, `calculate_trajectory` and `compute_step_1` one after another. `run_odometry` runs `optimize_windows`, then loop closure and consistency, and `run_all` ends with `save_result`.
- `optimize_windows` (`src/application/run_odometry.cpp`) loops over the windows:
  - `point_store.load` reads the window's points;
  - `run_lm_iterations` calls `optimize_lidar_odometry` up to 100 times;
  - then logging, `send_preview` and `add_window_to_map` (`update_rgd`, plus the sliding-map rebuild every 5 m);
  - `finish_run` logs the total at the end.
- `optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) has these sequential phases:
  1. allocate and zero the dense `AtPAndt`/`AtPBndt`;
  2. the indoor NDT point loop;
  3. the outdoor NDT point loop;
  4. the motion-model, bias, velocity-prior and acceleration terms;
  5. the triplets, the fixed-origin product and `pose_band_to_sparse`;
  6. the covariance solve of the extra unknowns (when there are any);
  7. the damping and the `SimplicialCholesky` solve;
  8. the pose and bias update.
- The domain may use only Eigen, manif, observation_equations and the standard library. `<chrono>` is standard.
- `log_iteration_convergence` is the template for a debug setting that only observes.

## Goals / Non-Goals

**Goals:**
- Wall-clock totals per pipeline step, and per phase of the odometry, summed over a whole run.
- Bit-identical results with the setting on or off.
- Numbers that decide the order of the planned performance changes.

**Non-Goals:**
- Any speed-up. This change only measures.
- Per-window timing lines. The existing `finished at iteration …` line already gives each window's time.
- A sampling profiler or ETW integration. A profiler run of Visual Studio can be done alongside if a phase needs a closer look.

## Decisions

1. **The setting.** `log_stage_timing` goes in `[lidar_odometry_debug_info]`, next to `log_iteration_convergence`, and is added to `POINTERS` and `CATEGORIES`. The golden TOML files are regenerated in their own commit. Alternative: always log. Rejected, because it adds about 20 lines to every run.

2. **Domain structs in a new header, `src/domain/phase_timing.h`** (standard library only):
   ```cpp
   struct PhaseTime { double seconds = 0.0; long long calls = 0; };
   struct ScanToMapTiming { PhaseTime setup, ndt_indoor, ndt_outdoor, motion_terms, assembly, covariance_solve, solve, update; };
   // adds the time from construction to destruction to *t, if t is not null
   class PhaseTimer { ... std::chrono::steady_clock ... };
   ```
   `PhaseTimer` with a null target does nothing, so the code under timing reads the same with the setting off.

3. **`optimize_lidar_odometry` gets a last parameter, `ScanToMapTiming *timing = nullptr`.**
   - Each of the eight phases is wrapped in a `PhaseTimer` block, or a timer is started and stopped around the existing statements where adding a scope would move declarations.
   - The existing statements are not reordered, and no variable changes type or lifetime in a way that changes the arithmetic.
   - The test caller in `tests/scan_to_map_test.cpp` keeps the default.
   - A pointer parameter rather than a field in `ScanToMapOptions`, because the options are `const` settings and the timing is output.

4. **The window loop** (`run_odometry.cpp`) keeps a file-local `OdometryTiming` in `WindowLoopCarry` with:
   - the `ScanToMapTiming`, passed by `run_lm_iterations` when the setting is on;
   - `PhaseTime` for `point_load`, `lm_iterations` (the whole `run_lm_iterations` call), `add_to_map`, `preview` and `window_total`.

   `finish_run` logs the breakdown when the setting is on: one line per phase, with seconds, the share of the odometry time, calls and ms per call. Then one line for `other`, the odometry time minus the named phases. Example:
   ```
   timing odometry 1562.0 s over 690 windows
   timing   lm ndt_indoor   812.4 s  52.0 %  69000 calls  11.77 ms
   timing   lm setup        141.0 s   9.0 %  69000 calls   2.04 ms
   ...
   ```

5. **Pipeline steps** (`odometry_pipeline.cpp`): when the setting is on, each step is timed in `load`, `run_odometry` and `run_all` with a `steady_clock` pair, and one `timing <step> N s` line is logged after it. The GUI uses the same pipeline, so the lines show up in its console too.

6. **Bit-identity.**
   - The timers read the clock and add to doubles that never enter the computation.
   - `/fp:fast` can still move last bits when `optimize_lidar_odometry`'s shape changes, so the precise-build rule applies: the precise `44_short` must be bit-identical with the precise reference, with the setting off and on; the fast build must be IDENTICAL with the manifest, or else within 1e-6.
   - The params TOML is the only file that may differ: it gains the key, and it records `true` when the setting is on.

7. **Measurement.**
   - `44_short` and then the full 44 with the setting on, from a scratchpad copy of `lidar-vertical.toml` with only this key changed. The 44 output must match the manifest, apart from the params file.
   - The overhead is checked against the last setting-off run of the same day.
   - The profile goes into `docs/PERFORMANCE.md`, and its "Planned order" is reordered by the measured shares.

## Risks / Trade-offs

- **Timer overhead in the point loop:** none, because the loops are timed as a whole, never per point.
- **Wall-clock noise.** Machine load changes the numbers, so the measurement runs with nothing else running (no GUI, no second job). The shares matter more than the absolute times.
- **Phases outside `optimize_lidar_odometry` that the plan suspects** (I/O in consistency, `save_result`'s chunk round trip) are covered only by their step's total. A finer breakdown there waits until a planned change needs it.
