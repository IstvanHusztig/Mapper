# Tasks

Every code commit: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Start menu, active parameter file, background colour

- [x] 1.1 Add the Start menu (design decision 1) first in the menu bar: "1. Load parameters…" (today's Load parameters code), "2. Load data and process…" (`openData`, Ctrl+O label), a separator and the disabled "Parameters: <file name>" line. Remove the "Load & process scanning" button. Verify both presets build.
- [x] 1.2 Add `active_parameters` (design decision 2). It is set by `set_lidar_odometry_default_params` when `default_settings.toml` loads, and by Load parameters on success. The progress window shows the file name, with the full path as a tooltip, captured when the run starts. Verify both presets build.
- [x] 1.3 Add `apply_background()` (design decision 3), called after the start-up defaults and after Load parameters. Remove View → Colors. Verify both presets build. Run the step checks. Commit.

## 2. Presets and Parameters menus

- [x] 2.1 Remove the Presets menu and the Parameters menu (Load parameters already lives in Start; Save parameters goes). The CLI branch passes `false` for debug. Remove `lastPar`, and `full_debug_messages` once it has no reader.

  Verify that a grep for `BeginMenu("Presets")|BeginMenu("Parameters")` finds nothing. Run the step checks. Commit.

## 3. Settings window and display

- [x] 3.1 Remove `settings_gui()` entirely, with View → Settings and `is_settings_gui` (design decision 4). Removed with it:
  - the scan range tools (index from/to, Select all/Unselect, the range buttons, Export selected scans, Save RESSO file, the stretch gizmo, the per-window show/hide list);
  - `simple_gui`, every parameter field, "Set example", NWU/ENU/NED, "Load data", "Compute all", "Save result", "Save all point clouds to single las/laz file", "Save trajectory to ASCII", the reference clouds section, and the initial-transformation gizmo and "Align to reference".

  Verify both presets build.
- [x] 3.2 Display and view menu:
  - remove the drawing of the initial points, reference points/buckets, trajectory-as-axes and the from/to markers;
  - remove both gizmo branches and the ImGuizmo frame code in `display()`;
  - remove the View items "Points size", "Show initial points" and "Show trajectory as axes";
  - remove the `session` object;
  - keep the trajectory drawing and the compass/ruler.

  Verify both presets build. Run the step checks. Commit.

## 4. Dead code

- [x] 4.1 Remove the globals whose last reader is gone (design Context list), `compute_all_seconds`, `save_results(info=true)` paths, `step1()`, and `step_1_done`/`step_2_done`/`calculations_failed` if unused. Remove `load_reference_point_clouds`, `filter_reference_buckets` and `align_to_reference` with their declarations. Keep `OdometryState::reference_*` and the `compute_step_2` branch (Non-Goals).

  Verify:
  - a grep for these names finds nothing;
  - a grep for `params\.[a-z_]+ *=|&params\.` in `app/lidar_odometry_gui.cpp` finds nothing (no widget writes the config).

  Run the step checks. Commit.

## 5. Integration

- [x] 5.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json`, or `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes. Result: within `--tolerance 1e-6`, max difference 1e-9 (30 of 35 files differ in the last bits, as in steps 3–4).
- [x] 5.2 GUI check, **done by the user**:
  1. start the Release GUI; Start shows "Parameters: default_settings.toml";
  2. Start → 1. Load parameters → `settings/lidar-vertical.toml`; the line now names it;
  3. Start → 2. Load data and process → `44_short`; the progress window names the file; say which `lio_result_*` it saved to;
  4. the menu bar shows only Start, View (Show trajectory, Show compass/ruler, Lock Z), Camera and Info;
  5. confirm that Presets, Parameters, the Settings window (settings fields, georeferencing, scan range tools), "Points size", "Show initial points", "Show trajectory as axes" and the colour picker are gone.

  Then I verify that the result is IDENTICAL with the `44_short` manifest.

  Result: the first run, `lio_result_5`, differed by about 1–2 mm with identical settings. It took 567 s, against 197 s for the CLI, so some windows likely hit `real_time_threshold_seconds = 10`. That is an existing load dependence, not this change. A repeat with nothing else running, `lio_result_6`, is IDENTICAL with the manifest (11 files).
- [x] 5.3 Docs:
  - `CLAUDE.md`, Run → GUI: the Start menu, settings only via TOML, the active file, no georeferencing or editing tools. Architecture: the GUI globals list.
  - `docs/ARCHITECTURE.md` §4: the GUI changes `OdometryConfig` only through `ConfigRepository`; georeferencing is a future separate module.

  Commit.
- [x] 5.4 Run `openspec validate gui-cleanup --strict` and confirm it passes.
