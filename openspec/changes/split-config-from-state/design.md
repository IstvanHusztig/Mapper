# Design

## Context

See proposal.md for the motivation. What the code does today:

- **Runtime writes into `params`** (outside TOML loading):
  - `load_data` → `working_directory_preview`, `working_directory_cache`;
  - `compute_step_1` → `initial_points`, `m_g`;
  - `compute_step_2` → `buckets_indoor`, `buckets_outdoor` (also through `update_rgd`),
    `consecutive_distance`, `total_length_of_calculated_trajectory`;
  - `load_reference_point_clouds` / `filter_reference_buckets` → `reference_points`,
    `reference_buckets`, `buckets_indoor`;
  - in the GUI, `align_to_reference` → `m_g`.

  Nothing else is written by the pipeline.
- **Read by output writers:**
  - `save_result` writes `total_length_of_calculated_trajectory` into `session.mjs`;
  - `save_processing_results_json` writes the trajectory length, `consecutive_distance`, the
    paths, and `m_g` (as `transformation_matrix`);
  - `save_parameters_toml` writes every TOML field, including `[paths]` and `[misc] clear_color`.
- **Fields that only round-trip:**
  - `current_output_dir`: nothing ever sets it, but it is in `[paths]` and the results JSON;
  - `clear_color`: a GUI colour in `[misc]`, written by the CLI too, since it's built with
    `WITH_GUI=1`.
- **Grid parameters:** `update_rgd` and `update_rgd_spherical_coordinates` only read them, and
  `in_out_params_*` are never written outside TOML loading. `number_of_buckets` (in the results
  JSON) is never computed by the app.
- **Who edits settings:** the GUI, through ~190 `params.` references (ImGui widgets bound to
  fields). `motion_model_correction` is a user setting (GUI input), not state.
- **Defaults:** `main()` calls `set_lidar_odometry_default_params`, which loads
  `default_settings.toml` from the current directory, then the CLI's TOML. For
  `settings/lidar-vertical.toml` the 6 keys it lacks are not in `default_settings.toml` either, so
  the baseline does not depend on the current directory.

## Goals / Non-Goals

**Goals:**
- `OdometryConfig` is passed `const&` to every pipeline function: `load_data`,
  `calculate_trajectory` callers, `compute_step_1`, `compute_step_2`, `run_loop_closure`,
  `run_consistency`/`Consistency`/`Consistency2`, and the `save_*` functions.
- Every runtime write goes through an explicit `OdometryState&` parameter.
- TOML bytes and all outputs unchanged.

**Non-Goals:**
- Grouping `OdometryConfig` into per-section sub-structs, which would touch every `params.x`
  read. It belongs with the domain model (step 5).
- Removing the GUI globals or the GUI/CLI difference (step 2).
- Changing `WorkerData`, `core/`, or the optimizer signatures beyond `const` (step 5).
- Making `set_lidar_odometry_default_params` independent of the current directory. That is a
  behaviour change; it is noted in Risks for step 2.

## Decisions

1. **Three types replace `LidarOdometryParams`.** They live in `app/lidar_odometry_utils.h` for
   now; folders come in step 7.
   - `struct OdometryConfig`: all current setting fields, with the same names, defaults and
     comments, minus the state fields and minus `clear_color`/`current_output_dir`. The names are
     kept so the ~400 reads change only in their prefix.
   - `struct RunPaths { std::string current_output_dir, working_directory_preview,
     working_directory_cache; }`
   - `struct OdometryState { Eigen::Affine3d m_g; initial_points; reference_points;
     reference_buckets; buckets_indoor; buckets_outdoor; consecutive_distance;
     total_length_of_calculated_trajectory; RunPaths paths; }`
   - `struct GuiPreferences { float clear_color[4]; }`. Plain floats keep `#include <imgui.h>` and
     `#if WITH_GUI` out of the shared header. The GUI converts to and from `ImVec4` where it uses
     it.

   *Alternative:* keep `LidarOdometryParams` as an alias during migration. Rejected: the compiler
   errors on the removed fields are the safety net that finds every state access.
2. **`SettingsDocument` for TOML.** `struct SettingsDocument { OdometryConfig config; RunPaths
   paths; GuiPreferences gui; }`. `TomlIO::LoadParametersFromTomlFile(path, SettingsDocument&)`
   and `SaveParametersToTomlFile(path, const SettingsDocument&)` keep the same table-driven code.
   - `POINTERS` becomes member pointers into `OdometryConfig`, with a small extra table for the
     two `RunPaths` keys.
   - `CATEGORIES` and the write order are untouched, so the written file is byte-identical.
   - `save_parameters_toml(config, state.paths, gui_prefs, ...)` builds the document from the
     run's actual paths, as it does today.
3. **Who owns what.**
   - **CLI branch of `main()`:** loads a `SettingsDocument` and builds an `OdometryState` whose
     `paths` start from the document's `[paths]` (`current_output_dir` keeps its loaded value,
     exactly as today). It passes `doc.config` as const, the state as mutable, and `doc.gui` to
     the TOML writer.
   - **GUI:** the global `params` becomes three globals: `config`, `state` and `gui_prefs`.
     Widgets bind to `config.x` / `gui_prefs`. Removing globals is step 2.
4. **`const` where already true:** `update_rgd`, `update_rgd_spherical_coordinates`,
   `align_to_reference`, `optimize_lidar_odometry` and `optimize_sf*` get `const
   NDT::GridParameters&` (they only read it). If the compiler shows one really writes, that
   parameter stays non-const and the finding is recorded here.
5. **Order of work, each step building, testing and passing `44_short`:**
   - (a) a golden TOML test on the current code;
   - (b) introduce `OdometryState` and move the state fields;
   - (c) `RunPaths`/`GuiPreferences`/`SettingsDocument` and the TOML I/O;
   - (d) rename to `OdometryConfig`, `const` everywhere;
   - (e) the full 44 and docs.

   Small steps keep a failed bit-identity check easy to bisect.
6. **Golden TOML test.** Before any refactor, the test saves:
   - `LidarOdometryParams{}` with a fixed `build_date`;
   - the result of loading `settings/lidar-vertical.toml` (with a fixed `build_date`).

   It stores both outputs under `tests/data/` as golden files. After every later step the same
   test must produce identical bytes. This guards the TOML format in seconds, independent of the
   44 runs. The test finds `settings/` through a compile definition holding the repo path.

## Risks / Trade-offs

- [The mechanical edit changes an evaluation order or a copy and thereby the numbers (for
  example, `auto pp = params.initial_points` becoming a reference)] → Moved fields keep their
  types. Copies stay copies, so no `auto&` "optimisations". Every step is checked on `44_short`
  and the end on the full 44.
- [The GUI breaks in a way the CLI baseline can't see, since the GUI isn't covered by the
  baselines] → After step (d), a manual GUI smoke check: open 44_short, run Compute all, save the
  result, and compare the step-2 outputs (results JSON, `session.mjs`) with
  `44_short_baseline`. They must match, as they did before. Trajectories differ only because the
  GUI lacks loop closure (step 2).
- [`set_lidar_odometry_default_params` depends on the current directory] → Unchanged here. The
  baseline commands run from the repo root, and the current config is unaffected. Recorded for
  step 2, where the pipeline gets one explicit way to load its config.
- [A large mechanical diff is hard to review] → One commit per step (a)–(e). Renames happen in a
  commit of their own (d), with no logic change.

## Findings during implementation

- **Uninitialized NDT grid parameters** (`NDT::GridParameters` has no default member values). A
  non-global `LidarOdometryParams` wrote garbage bounding boxes to TOML, which made the
  default-params golden test fail at random.
  - The app's global `params` was zero-initialized statically, and `lidar-vertical.toml` doesn't
    set the bounding boxes either, so the app always ran with zeros.
  - Fixed by value-initializing (`{}`) in its own commit (1845a9d). `44_short` is unchanged, and
    the new types keep the `{}`.
- **The test target needs `WITH_GUI=1`** to save the same fields as the app (`clear_color`). With
  it, the core headers pull in freeglut, so the tests use its include path and
  `FREEGLUT_LIB_PRAGMAS=0`. This goes away once `clear_color` leaves the shared header (task 3.1)
  and once `core` headers stop depending on GUI (later roadmap steps).
- **No `SettingsDocument` (deviation from decision 2).** `TomlIO` load and save take the three
  parts directly, `(LidarOdometryParams/OdometryConfig&, RunPaths&, GuiPreferences&)`, with
  `const&` for saving.
  - The GUI and CLI keep them in separate places (`params`, `state.paths`, `gui_prefs`), so a
    wrapper struct would only add copies back and forth.
  - Everything else is as designed: `[paths]` goes through `RUN_PATH_POINTERS`, `[misc]
    clear_color` through `GuiPreferences`, and `CATEGORIES` is unchanged. The golden TOML test is
    byte-identical.
- **The test target no longer needs `WITH_GUI`** or any GUI headers, now that `clear_color` is
  plain floats and `#if WITH_GUI` is gone from the TOML layer and `lidar_odometry_utils.h`. The
  golden TOML (including `clear_color`) is still byte-identical, which shows the TOML layer is
  GUI-free.
- **`const` everywhere it was planned, with no exceptions.** None of `load_data`,
  `compute_step_1`/`2`, `run_consistency`, `Consistency`/`Consistency2`, the reference-cloud
  functions, `save_result`, `update_rgd`, `update_rgd_spherical_coordinates`,
  `align_to_reference`, `optimize_lidar_odometry` or `optimize_sf2` writes to the config or the
  grid parameters (the compiler accepted every `const`). Only TOML loading,
  `HandleMissingVersion` and `set_lidar_odometry_default_params` write to an `OdometryConfig`.
