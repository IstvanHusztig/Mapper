# Proposal

## Why

`LidarOdometryParams` (`app/lidar_odometry_utils.h`) is both the settings loaded from TOML and the
pipeline's scratchpad. `load_data`, `compute_step_1` and `compute_step_2` write the NDT maps, the
initial pose and points, the run's paths and the trajectory length into it, and it also carries a
GUI colour. So every pipeline function needs a mutable `params&`, nobody can tell which fields a
function changes, and a settings object can't be shared or reused between runs. This is step 1 of
the `docs/ARCHITECTURE.md` roadmap. Steps 2 (`OdometryPipeline`) and 5 (domain model) build on a
read-only config.

## What Changes

- **Rename and split** `LidarOdometryParams` into:
  - **`OdometryConfig`**: every setting, the fields listed in `TomlIO::POINTERS` plus the NDT grid
    parameters and `motion_model_correction`. Pipeline functions take it as `const
    OdometryConfig&`. Only the GUI's settings widgets and TOML loading write to it.
  - **`OdometryState`**: what a run produces and consumes. That is `m_g`, `initial_points`,
    `reference_points`, `reference_buckets`, `buckets_indoor`, `buckets_outdoor`,
    `consecutive_distance`, `total_length_of_calculated_trajectory`, and the run paths
    `working_directory_preview` and `working_directory_cache`. Functions that change it take it
    explicitly.
  - **`GuiPreferences`**: `clear_color` (as plain floats, so no ImGui type in shared headers).
- **TOML I/O takes the three parts** (`OdometryConfig`, `RunPaths` for `[paths]`, `GuiPreferences`),
  so the file format, the section and key order, and the values written to the output
  `HDMapping_params_*.toml` stay exactly the same.
- Read-only helpers get `const` parameters where they never write (`update_rgd`,
  `align_to_reference` and the optimizer grid-parameter arguments).
- Unit tests:
  - a golden-file test that saving the default settings and `settings/lidar-vertical.toml`
    produces byte-identical TOML before and after;
  - the existing TOML round-trip test, moved to `OdometryConfig`.
- `CLAUDE.md` and `docs/ARCHITECTURE.md` are updated (new type names; step 1 done).
- **No behaviour change.** The CLI output must stay bit-identical to
  `tools/baseline/44_short.manifest.json` after every commit and to
  `tools/baseline/44.manifest.json` at the end. The GUI behaves exactly as before (it still has no
  loop closure, which is roadmap step 2).

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- None. This is a pure refactor (`skip_specs: true`): no observable behaviour, output or file
  format changes.

## Impact

- Code: `app/lidar_odometry_utils.h` (types), `app/lidar_odometry.{h,cpp}`,
  `app/lidar_odometry_utils_optimizers.cpp`, `app/lidar_odometry_utils.{h,cpp}`
  (`update_rgd`), `app/loop_closure.{h,cpp}`, `app/toml_io.{h,cpp}`,
  `app/lidar_odometry_gui.cpp` (the `params` global becomes `config` + `state` + `gui_prefs`),
  `tests/`.
- No change to `core/`, `CORE_SOURCE_FILES`, the TOML format, the CLI arguments or the outputs.
- About 400 `params.` references are touched mechanically. The compiler finds every one that
  moves to `OdometryState`, because those fields no longer exist on `OdometryConfig`.
