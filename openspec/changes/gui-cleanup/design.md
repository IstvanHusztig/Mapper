# Design

## Context

See proposal.md for the motivation and the decisions. Current state of `app/lidar_odometry_gui.cpp` (about 1,960 lines):

- **Main menu bar** (drawn while no run is going):
  - **Presets:** 4 items that call `set_lidar_odometry_default_params` and then overwrite fields in code; `lastPar`.
  - **Parameters:** toggles on `params`, Set real time performance, `full_debug_messages`, Save calibration validation file, Ablation study, Saving results with index pose, Load parameters, Save parameters.
  - **"Load & process scanning"** button (`openData`, also Ctrl+O).
  - **View:** Points size, Show initial points, Show trajectory, Show trajectory as axes, Show compass/ruler, Lock Z, Colors → Background (`gui_prefs.clear_color` → `bg_color`; `bg_color` is set only while this menu is open), Settings (`is_settings_gui`).
  - **Info** button.
- **`settings_gui()`** ("Settings" window, lines ~408–1075). In order:
  - `simple_gui`;
  - the motion-model-correction fields and "Set example";
  - more parameter fields under `!simple_gui`;
  - the NWU/ENU/NED radio buttons. They are written to `params.fusionConvention*` every frame from a `static int` that starts at 0, so opening the window forces NWU (a bug);
  - "Load data", "Compute all", "Save result", "Save all point clouds to single las/laz file";
  - robust/rigid options;
  - "Save trajectory to ASCII";
  - reference point clouds: Load, Show points/buckets, Filter reference buckets;
  - the initial-transformation gizmo and "Align to reference" (shown when `pipeline.state.initial_points` is non-empty);
  - the scan range tools (shown when `pipeline.worker_data` is non-empty): index from/to and the ±1/±10 buttons, Select all/Unselect, Export selected scans, Save RESSO file, and the stretch gizmo ("Accept Gizmo").
- **Display** draws the initial points (with `m_g`), the reference points and buckets, the trajectory (points or axes) and the from/to markers. The gizmo code in `display()` writes either `pipeline.state.m_g` (initial transformation) or the stretch transform.
- **Globals only the removed parts use:** `simple_gui`, `lastPar`, `step_1_done`, `step_2_done`, `calculations_failed`, `compute_all_seconds`, `show_initial_points`, `show_trajectory_as_axes`, `show_reference_points`, `show_reference_buckets`, `dec_reference_points`, `initial_transformation_gizmo`, the tooltip texts `omText`/`fiText`/`kaText`, and `full_debug_messages` (also passed by the CLI branch).
- **Pipeline functions only the GUI calls:** `load_reference_point_clouds`, `filter_reference_buckets` (`app/lidar_odometry.cpp`) and `align_to_reference` (`app/lidar_odometry_utils_optimizers.cpp`).
- **Settings at start-up:** `set_lidar_odometry_default_params` loads `<exe>/default_settings.toml` through `TomlConfigRepository`. Load parameters loads the chosen file onto the current `params`.

## Goals / Non-Goals

**Goals:**
- The GUI matches `specs/gui-workflow`: a Start menu, settings only from TOML, the active file shown, no georeferencing, and the kept tools and view options.
- After this change, the GUI changes `OdometryConfig` only through `ConfigRepository::load`.
- CLI output and GUI results unchanged (`44_short` identical through the CLI and through the GUI).

**Non-Goals:**
- A georeferencing module: a separate future change.
- Removing `OdometryState::reference_points`/`reference_buckets` and the `reference_points.size() == 0` branch in `compute_step_2`: step 5 (domain model). The branch always takes the empty path now.
- A GUI view-model or removing the remaining file-scope globals.
- A TOML key for debug messages, and remembering the last parameter file between sessions.
- Restyling the progress window beyond the active-file line.

## Decisions

1. **The Start menu replaces the button.** It is a `BeginMenu("Start")` placed first in the menu bar:
   ```
   Start
    ├─ 1. Load parameters…        → TomlConfigRepository::load onto params (+ paths, gui_prefs)
    ├─ 2. Load data and process…  → openData()                      Ctrl+O
    ├─────────
    └─ Parameters: <file name>     (disabled text line)
   ```
   - Load parameters keeps today's file dialog, try/catch and messages.
   - *Why a menu, not two buttons:* ImGui's menu bar already holds the other menus, a menu shows the order explicitly, and the disabled line has a natural place.

2. **The active parameter file** is a GUI global `std::string active_parameters`:
   - `"built-in defaults"` at start;
   - the full path after `set_lidar_odometry_default_params` loads `default_settings.toml`;
   - the chosen path after a successful Load parameters (unchanged if loading fails).

   The menu shows the file name. The progress window shows the file name plus the full path as a tooltip, captured when the run starts.
   - *Why the GUI and not `RunPaths`:* it describes where the settings came from, not the run. Putting it into the TOML `[paths]` would change the output format.

3. **The background colour** is applied from `gui_prefs.clear_color` by a small `apply_background()` that sets `bg_color`. It is called after the start-up defaults and after each Load parameters. The View → Colors item goes. `[misc] clear_color` stays in the TOML: saved with each result, loaded with each file.
   - *Rejected:* dropping `GuiPreferences`. That would change the TOML format, the golden test files and both baseline manifests, for no user benefit.

4. **The Tools window** is today's Settings window cut down to the scan range tools:
   - it has its own top-level **Tools** menu in the menu bar, after Start and before View (user request), with one item, "Scan range…". The item is disabled until a run has finished (`pipeline.worker_data` non-empty) and while a run is going. It toggles `is_tools_window`;
   - the disabled item's tooltip says "available after a run".

   The stretch gizmo code in `display()` stays. The initial-transformation branch of the gizmo (writing `m_g`) goes.

5. **Removal order** keeps every commit buildable and checkable:
   1. Start menu, active file, background colour;
   2. Presets and Parameters menus;
   3. Settings window → Tools window (parameter fields, the advanced flow, duplicate exports and georeferencing removed), plus the display parts (initial points, reference points/buckets, trajectory as axes) and the initial-transformation gizmo;
   4. the unused globals and the three reference functions.

   The CLI branch passes `false` for debug instead of `full_debug_messages`.

6. **Verification without a display in CI:** `44_short` through the CLI after each commit (the pipeline is untouched, so it must stay identical), plus a build of both presets and `ctest`. The GUI behaviour is checked by the user at the end. A grep check confirms no remaining writes to `params.` in the GUI file except through the repository.

## Risks / Trade-offs

- **[Removing too much from the 1,960-line file]:** for example a display path or the stretch gizmo that the Tools window still needs.
  → Remove section by section in the order above, building after each section. The user checks the Tools window after a real run.
- **[Hidden coupling through globals]:** `index_begin`/`index_end`, `gizmo_stretch_interval` and `stretch_gizmo_m` are shared by the Tools window and `display()`. They stay. Only globals whose last reader is removed are deleted (checked by the compiler as each one goes).
- **[Users relied on in-GUI tweaking]:** the decision is deliberate (settings only via TOML). → The Info text and `CLAUDE.md` say where the settings come from.
- **[Losing debug output in the GUI]:** accepted; can return as a TOML key.
- **[Removing `align_to_reference` changes code generation of `lidar_odometry_utils_optimizers.cpp`]** under `/fp:fast`.
  → The full 44 is run once at the end, judged with `--tolerance 1e-6`. `44_short` must stay identical.

## Migration Plan

Commits as in decision 5, each built in both presets, `ctest`, and checked on `44_short` via the CLI. Then the full 44 run, the user's GUI check, and the docs. Rollback: revert. The TOML and result formats are unchanged, so no data migration is needed.
