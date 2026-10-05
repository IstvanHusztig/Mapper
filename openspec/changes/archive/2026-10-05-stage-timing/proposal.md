# Proposal

## Why

The 2026-10-05 performance review (`docs/PERFORMANCE.md`, "Analysis (2026-10-05) and planned fixes") lists five planned speed-ups, but their costs are only estimates read from the code. The biggest suspect is a dense 2166×2166 normal matrix (37 MB) that is allocated and zeroed in every LM iteration. Today the log shows only the total time and each window's time, so it can't show where the time goes.

This change measures it first. The resulting "before" profile on `44_short` and `44` decides the order of the later changes (`deterministic-multithread`, `banded-normal-matrix`, `ndt-lookup-cache`, `io-and-copies`) and is the reference their gains are measured against. The user chose to measure before changing anything (2026-10-05).

## What Changes

- **New debug setting `log_stage_timing`** (bool, default `false`) in `[lidar_odometry_debug_info]`, loadable and savable through the TOML table like `log_iteration_convergence`.
- **When it is on, wall-clock times are logged, and nothing else changes:**
  - **After each pipeline step, one line:** `timing load_data 12.3 s`. The steps are `load_data`, `calculate_trajectory`, `compute_step_1`, `odometry`, `loop_closure`, `consistency` and `save`; a step that doesn't run (loop closure or consistency off) gets no line.
  - **At the end of the odometry, a breakdown summed over all windows:**
    - inside `optimize_lidar_odometry`: setup (allocating and zeroing the dense normal matrix), the NDT point loop (indoor and outdoor separately), the motion-model and prior terms, the assembly (triplets and `pose_band_to_sparse`), the covariance solve of the extra unknowns, the damped solve, and the pose update;
    - in the window loop: loading the window's points from the `PointStore`, `add_window_to_map` (`update_rgd`, including the sliding-map rebuild), the preview output, and the rest.
  - Each line gives seconds and the share of the odometry time; the LM phases also give the number of calls and ms per call.
- **Measurements:** `44_short` and the full 44 with the setting on. The profile goes into `docs/PERFORMANCE.md`, and the planned order of the later changes is updated from it.

**Unchanged:** the timers only read the clock. With the setting off or on, `dMapper` gives bit-identical results, apart from the params TOML, which records the setting.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. This is a diagnostic log; no processing behaviour changes (`skip_specs: true`).

## Impact

- **Code:**
  - `src/domain/odometry_config.h`: the new setting.
  - `src/infrastructure/toml_io.h`: the `POINTERS` entry and the `lidar_odometry_debug_info` category.
  - `settings/default_settings.toml`, `settings/lidar-vertical.toml`: the new key, set to `false`.
  - The golden TOML files, regenerated in their own commit.
  - New domain header `src/domain/phase_timing.h`: the timing structs and a scope timer, standard library only.
  - `src/domain/scan_to_map.{h,cpp}`: an optional `ScanToMapTiming *` parameter, `nullptr` by default.
  - `src/application/run_odometry.cpp`: the window-loop phases and the end-of-odometry breakdown.
  - `src/application/odometry_pipeline.cpp`: the per-step lines.
- **Manifests:** adding a key changes the params TOML of every run, so the params hash in `tools/baseline/44.manifest.json`, `44_short.manifest.json` and the precise reference manifest (kept outside the repo) is updated in the same commit, as `log_iteration_convergence` did.
- **Docs:** `docs/index.html` (the setting's row in the debug table) and `docs/PERFORMANCE.md` (the measured profile). The commit also carries the pending "Analysis (2026-10-05)" section of `docs/PERFORMANCE.md`.
- **Run time with the setting on:** about ten `steady_clock` reads per LM iteration (about 69,000 iterations on 44), which is negligible. Measured anyway against a run with the setting off.
