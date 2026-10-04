# Tasks

**Checks on every code commit** (design decision 7), run by the scratchpad's `step_check.sh`:
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- precise and fast `44_short` IDENTICAL (precise manifest from master; `tools/baseline/44_short.manifest.json`);
- `git diff --stat <change start> -- src/domain src/application src/infrastructure core` is empty.

Close the GUI before building.

## 1. Preparation

- [x] 1.1 Move `GuiProgress` verbatim from `lidar_odometry_gui.cpp` into `src/presentation/gui/gui_progress.h`, and include it. Verify: the step checks; the class text is unchanged (`git diff -M` / a diff of the moved block). Commit. Result: the 75-line block is identical to the one removed (diffed); the header adds only its includes. Both presets build, `ctest` passes, precise and fast `44_short` are IDENTICAL (11 files), and the layer folders and `core` are unchanged.

## 2. View-model

- [x] 2.1 Find out what today's code does when "Load data and process" picks an empty folder (read `make_folder_inputs`, `RecordingFolderSource::open` and `OdometryPipeline::load`, or run the CLI on an empty folder): load failure, or a run that saves an empty result. Record it here; the test of task 3.1 follows it (design risk 4). Result (from the code): with no files, `pair_recording_files` logs "Not enough files to continue: 0" and fails, so `make_folder_inputs` returns empty inputs, `OdometryPipeline::load()` returns false (`!inputs_.complete()`), and the GUI shows the load-failure message and exits. So an empty folder is a load failure; `working_directory` stays as it was (no first file to take it from), and no cache folder is created.
- [x] 2.2 Add `GuiViewModel` and `RunNotifier` (decisions 1 and 2) in `src/presentation/gui/gui_view_model.{h,cpp}`, and the library `lo_gui_view_model` (decision 4). Move the state and actions out of `lidar_odometry_gui.cpp`: the settings and sources, `select_input` without its dialog, `load_input`, `step2`, `save_results`, `openData`'s thread body, `on_exit`'s point-store clear. The view keeps the dialogs, message boxes (in its `RunNotifier`), menus, progress window, drawing, preview and redraw timer, and reaches the view-model through one file-scope pointer set in `run_gui` (decision 3). Then the check: temporarily include `<imgui.h>` and `<GL/freeglut.h>` in `gui_view_model.cpp`; both must fail to compile (record the error text); revert. Verify: the step checks; `grep -nE "^[A-Za-z].*[;=]" lidar_odometry_gui.cpp` lists only view state at file scope (list it here). Commit. Result: `GuiViewModel` owns the settings, their sources, the console, the progress, the pipeline, the recording folder, the running flag, the start time and the worker thread; `MessageBoxNotifier` in the GUI file shows the two message boxes as before (and exits after a failed load). The enforcement check: `<imgui.h>` → `fatal error C1083 … "imgui.h": No such file or directory`, `<GL/freeglut.h>` → the same for `GL/freeglut.h`; reverted. File scope of `lidar_odometry_gui.cpp` now: `winTitle`, `show_trajectory`, `view_model` (the pointer), `preview_cloud`, `show_point_cloud`, `preview_was_running`, `preview_final_pending`, `redraw_scheduled` (plus `GuiProgress`'s and `PreviewCloud`'s and the notifier's class definitions). Both presets build, `ctest` passes, precise and fast `44_short` are IDENTICAL (11 files), and the layer folders and `core` are unchanged.
- [ ] 2.3 GUI run by the user (decision 7): start-up with the default settings (title "dMapper", background), Start → Load parameters with `settings/lidar-vertical.toml` (its name in the Start menu), Start → Load data and process on `44_short` (progress window with Pause/Resume, the preview, the "saved" message), then close the program (cache removed). Verify: the result folder is IDENTICAL with `tools/baseline/44_short.manifest.json`; record what the user saw.

## 3. Tests

- [x] 3.1 `tests/gui_view_model_test.cpp` with a recording `RunNotifier` (decision 5); link `lo_gui_view_model` into the tests. Verify: both presets build, `ctest` passes (report the new cases' Debug time). Commit. Result: 5 cases, all passing (34 assertions): defaults from a missing and a real file with their source; Load parameters, and an unreadable file that changes nothing; a folder that doesn't exist starts nothing; an empty folder starts, then reports `load_failed` with the folder, stops running, keeps the source the run started with, leaves `working_directory` empty and creates no cache; `GuiProgress` pause holds `checkpoint` until resume, and `reset` starts afresh. **Finding:** the first link failed with `LNK1104 freeglut.lib`, a pragma from `GL/freeglut.h` that `core`'s `session.h` brings into the result and preview sinks; `lo_infrastructure` now builds with `FREEGLUT_LIB_PRAGMAS=0`, and the link has no unresolved symbol (no adapter calls GLUT); design decision 4 records it. Both presets build, `ctest` passes, precise and fast `44_short` are IDENTICAL (11 files) with the new flag, and the layer folders and `core` are unchanged. The new cases take 0.18 s in Debug.

## 4. Docs

- [ ] 4.1 Update:
  - `CLAUDE.md`: the GUI globals paragraph (Architecture) becomes the view-model and the one pointer; the GUI section names `GuiViewModel` and `gui_progress.h`; Testing (the new test file); the layer list (`lo_gui_view_model`);
  - `docs/ARCHITECTURE.md`: step 10 done; the trajectory drawing race as an open finding.

  Verify: `git diff --stat <change start> -- src/domain src/application src/infrastructure core` is empty, and `openspec validate gui-view-model --strict` passes. Commit.
