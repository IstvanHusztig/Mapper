# Proposal

## Why

Roadmap step 10. The GUI's state is a dozen file-scope globals in `lidar_odometry_gui.cpp` (`params`, `gui_prefs`, `console`, `gui_progress`, the current `pipeline`, `active_parameters` / `run_parameters`, `working_directory`, `loRunning`, the run's start time, the preview flags), read and written by the GLUT callbacks, the menus and the worker thread alike. Nothing of that logic can be tested, and which thread touches what is only visible by reading the whole file. Steps 2–9 gave the CLI a clean shape; the GUI front end is the last part that isn't.

## What Changes

- **`GuiViewModel`** (`src/presentation/gui/gui_view_model.{h,cpp}`) owns the GUI's state and its actions:
  - **state:** the settings (`OdometryConfig`, `GuiPreferences`), the parameter sources (active, and the one the current run started with), the current `OdometryPipeline`, the recording folder, the run flag and start time, the `GuiProgress`, the console reporter;
  - **actions,** with paths instead of dialogs: load the default settings, load a parameter file, start a run on a folder (load → odometry and post-processing → save to the next `lio_result_<n>`, on a worker thread), pause and resume, clear the point store at exit;
  - **queries** for the view: running, stage and fraction, elapsed time, the labels, the run's windows for drawing and the preview.

  It includes no ImGui, GLUT, OpenGL or file-dialog header.
- **`GuiProgress`** moves out of the GUI file into `src/presentation/gui/gui_progress.h`, unchanged.
- **`lidar_odometry_gui.cpp` keeps the view:** menus, progress window, dialogs and message boxes, drawing, the preview cloud's display list and the redraw timer. It reaches the view-model through one file-scope pointer, set in `run_gui`, because GLUT callbacks take no user data. What the run reports back to the user (the "saved" message, a failed load) comes from the view-model as a result the view shows.
- **A library `lo_gui_view_model`** (presentation layer) holds the view-model, with the include directories of `lo_infrastructure` only, so an ImGui or GL include in it fails to compile. The exe and the tests link it.
- **Tests** of the view-model without a display: default and loaded parameters and their labels, a failed start on a folder without a recording, pause and resume, and that a new run starts from fresh progress.
- **No behaviour change.** The menus, the progress window, the preview, the redraw rate and the results stay the same; `src/domain`, `src/application`, `src/infrastructure` and `core` don't change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None: a refactor of the GUI front end (`skip_specs`). The `gui-workflow` requirements (Start menu, settings only from TOML, the parameter file shown, view options, title, redraw rate, live preview) describe behaviour that stays as it is.

## Impact

- **Code:** `src/presentation/gui/lidar_odometry_gui.cpp`; new `gui_view_model.{h,cpp}` and `gui_progress.h`; `CMakeLists.txt` (the new library), `tests/CMakeLists.txt` and a new test file.
- **Docs:** `CLAUDE.md` (the GUI globals paragraph and the GUI description), `docs/ARCHITECTURE.md` (step 10 done).
- **Not in this change:**
  - the camera and view globals of `core`'s `utils.hpp` (`bg_color`, the rotation and translation, `lock_z`, `compass_ruler` …): they belong to `core` and are step 11;
  - the worker thread's lifetime: it stays detached, as today (closing the window during a run ends the process);
  - the existing data race of drawing the trajectory while the odometry writes its poses (reported in design, not fixed here).
