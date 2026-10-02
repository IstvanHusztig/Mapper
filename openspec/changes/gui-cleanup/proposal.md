# Proposal

## Why

The GUI inherited from HDMapping offers four ways to set processing parameters: Presets, Parameters-menu toggles, a "power user" Settings window with dozens of fields, and loading a TOML. It also has a step-by-step advanced flow next to the one-click flow, and exports that duplicate files every result already contains. Results depend on which widgets were touched, and the CLI can't reproduce that. It also hides bugs: while the Settings window is open it overwrites the TOML's NWU/ENU/NED convention with NWU every frame.

The user decided (2026-10-02):
- **Settings come only from a loaded TOML file.**
- **The app concentrates on mapping.** Georeferencing will be a separate post-processing module applied to result folders.
- **The GUI is a simple Start workflow:** load parameters, then load data and process.

## What Changes

- **New Start menu** in workflow order:
  1. "Load parameters…" picks a `.toml`.
  2. "Load data and process…" (Ctrl+O) is today's one-click flow.

  The menu and the progress window show which parameter file is active (the start-up `default_settings.toml` or the last loaded file, or "built-in defaults").
- **BREAKING** (GUI only): removed:
  - the **Presets** menu and the **Parameters** menu (all toggles, Set real time performance, Full processing messages, Save calibration validation file, Ablation study, Saving results with index pose, Save parameters). Load parameters moves to Start;
  - the **"Load & process scanning"** button (replaced by the Start menu);
  - from the Settings window:
    - every settings field (motion model correction, AHRS gain, NDT grids, sigmas, robust/rigid options, NWU/ENU/NED);
    - the advanced flow ("Load data", "Compute all", "Save result");
    - "Save all point clouds to single las/laz file" (duplicates `scan_lio_merged.laz`);
    - "Save trajectory to ASCII" (duplicates `trajectory_lio_*.csv`);
    - the reference point clouds, the initial-placement gizmo and "Align to reference" (georeferencing);
  - from the View menu: "Show initial points", "Show trajectory as axes" and the Background colour picker.
- **Kept:**
  - the scan range tools (index from/to, Select all/Unselect, Export selected scans, Save RESSO file, the stretch gizmo) in a window renamed **Tools** (View → Tools), available once a run has finished;
  - View: Points size, Show trajectory, Show compass/ruler, Lock Z;
  - the Info button and the progress window with Pause/Resume.
- **The background colour stays a setting**, as `[misc] clear_color` in the TOML. It is loaded and applied at start-up and on Load parameters, and saved with each result. There is no TOML format change, so the baselines and golden files stay valid.
- **Debug messages** ("Full processing messages") are no longer switchable in the GUI; runs use the normal level. A TOML key can be added later if needed.
- **Unchanged:** the CLI, the processing sequence, and every output file. A GUI run of `44_short` with `lidar-vertical.toml` stays identical to the baseline.

## Capabilities

### New Capabilities
- `gui-workflow`: what the GUI offers:
  - the Start menu order;
  - settings only from loaded TOML files, with the active file shown;
  - no georeferencing in this app;
  - the remaining tools and view options.

### Modified Capabilities
- `processing-pipeline`:
  - "GUI and CLI results are identical" no longer mentions the removed "Compute all" (the GUI path is "Load data and process").
  - "No separate consistency action in the GUI" is reworded for the new menu (its scenario referred to "After Compute all").

## Impact

- **Code:**
  - `app/lidar_odometry_gui.cpp` (most of the change: about 700 lines of widgets and menus removed, the Start menu and Tools window added, unused globals deleted);
  - `app/lidar_odometry.{h,cpp}` and `app/lidar_odometry_utils_optimizers.cpp`: `filter_reference_buckets`, `load_reference_point_clouds` and `align_to_reference` lose their only caller and are removed. `OdometryState::reference_points`/`reference_buckets` stay, because `compute_step_2` reads `reference_points` (always empty now); they go with the domain model in step 5.
- **Docs:** `CLAUDE.md` (Run/GUI section), `docs/ARCHITECTURE.md` (§4: the GUI writes `OdometryConfig` only through `ConfigRepository`).
- **Not touched:**
  - `core/` (`bg_color` and `cor_window` live there);
  - the TOML format;
  - the pipeline, ports and adapters.
- **Validation:**
  - `44_short` CLI check on every commit;
  - unit tests;
  - the user checks the GUI: the Start menu order and the active-file display; a run identical to the `44_short` baseline; the Tools window working after a run; and the removed items gone.
