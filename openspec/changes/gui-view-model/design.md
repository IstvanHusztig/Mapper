# Design

## Context

`src/presentation/gui/lidar_odometry_gui.cpp` (827 lines, 2026-10-04) holds, at file scope:
- **settings and sources:** `params` (`OdometryConfig`), `gui_prefs`, `active_parameters` ("built-in defaults" or a TOML path), `run_parameters` (the source a run started with);
- **the run:** `console` (`ConsoleReporter`), `gui_progress` (`GuiProgress`, a `ProgressReporter` with Pause/Resume, defined in the file), `pipeline` (`OdometryPipeline`, replaced for each run in `select_input`), `working_directory` (the recording folder), `loRunning` (atomic), `loStartTime`;
- **view state:** `show_trajectory`, `show_point_cloud`, `preview_cloud` (`PreviewCloud`, points plus an OpenGL display list), `preview_was_running`, `preview_final_pending`, `redraw_scheduled`, `winTitle`.

The flows:
- **Start-up** (`run_gui`): `set_lidar_odometry_default_params` loads `default_settings.toml` next to the exe through `TomlConfigRepository`, then `apply_background` copies `gui_prefs.clear_color` into `core`'s `bg_color`.
- **Load parameters:** a file dialog, then the same repository load; `active_parameters` and the background follow.
- **Load data and process** (`openData`, menu or Ctrl+O, only while no run): a folder dialog lists the folder's files (`select_input`), resets the progress and the preview, and builds a new pipeline with `make_folder_inputs` and `make_run_services`. A detached `std::thread` then loads (`load_input`: on failure a message box, then `exit(1)`), runs `run_all`, saves to `get_next_result_path(working_directory)`, clears `loRunning`, and shows a message box with the folder, the trajectory length and the time.
- **Each frame** (`display`): the preview reads windows the odometry reported done; the trajectory and the last-pose cross are drawn from `pipeline.windows`; the menus are hidden while `loRunning`; the progress window shows the stage, the odometry percentage, the elapsed and estimated times, the parameter source and Pause/Resume.
- **Exit** (`on_exit`): clears the point store.

The camera, view and ImGui-docking globals (`bg_color`, `rotate_x`, `translate_*`, `lock_z`, `compass_ruler`, `initGL` …) live in `core`'s `utils.hpp` / `utils.cpp`.

Found while reading: `draw_run_data` draws every window's poses while the worker thread writes them (odometry and post-processing). The vectors aren't resized then, so it shows half-updated poses at worst; it is a data race nonetheless.

## Goals / Non-Goals

**Goals:**
- The GUI's state and actions in one class with no ImGui, GL, GLUT or dialog dependency, testable without a display.
- `lidar_odometry_gui.cpp` keeps only the view: layout, dialogs, drawing, timers.
- Behaviour and results unchanged.

**Non-Goals:**
- `core`'s camera and view globals (step 11).
- Joining the worker thread at exit; it stays detached (closing during a run ends the process, as today).
- Fixing the trajectory drawing race (above). It is recorded in `docs/ARCHITECTURE.md`; a fix (drawing only finished windows, as the preview does) is a visible behaviour change for its own proposal.

## Decisions

1. **`GuiViewModel`**, `src/presentation/gui/gui_view_model.{h,cpp}`:
   ```cpp
   class GuiViewModel {
   public:
       explicit GuiViewModel(RunNotifier &notifier);
       // settings
       void load_default_parameters(const std::filesystem::path &file);   // missing file: built-in defaults stay
       bool load_parameters(const std::string &file);
       const OdometryConfig &config() const;  const GuiPreferences &preferences() const;
       const std::string &active_parameters() const;  const std::string &run_parameters() const;
       // a run
       bool start_run(const std::string &folder);  // false: no such folder; else the worker starts
       void wait_for_run();                         // tests: join the worker
       bool running() const;  double elapsed_seconds() const;
       GuiProgress &progress();  const std::string &working_directory() const;
       const OdometryPipeline &run() const;         // windows and stored points for drawing
       void clear_point_store();                    // at exit
   };
   ```
   - It owns, as members, everything of the first two Context bullets. The parameter source labels and the TOML loads are the same code moved; `start_run` is `select_input` without the dialog plus `openData`'s thread body.
   - **The run keeps its order and its calls:** reset the progress, build the pipeline with `make_folder_inputs` and `make_run_services`, load, `run_all`, save through `FolderResultSink` on `get_next_result_path`, clear the running flag, report.
   - **The worker:** a `std::thread` member. `wait_for_run` joins it (tests); the destructor detaches it if it still runs, which is what today's detached thread amounts to. `running()` reads the same atomic flag.
   - `GuiProgress` moves unchanged into `gui_progress.h` (header only, standard library and `run_reporting.h`).

2. **`RunNotifier`, how the worker talks back to the view** (same header):
   ```cpp
   struct RunNotifier {
       virtual void load_failed(const std::string &folder) = 0;
       virtual void run_saved(const std::string &folder, double trajectory_length_m, double elapsed_s) = 0;
   };
   ```
   - The view implements it with today's message boxes, word for word, and `exit(1)` after a failed load, as today. Both are called on the worker thread, as the message boxes are today.
   - Tests implement it by recording the calls.

   Alternative: the view polls a "last result" each frame. Rejected: it would move the message box from the worker to the main thread, a visible change (the main loop would block while it is open).

3. **The view keeps a pointer.** `lidar_odometry_gui.cpp` has one file-scope `GuiViewModel *view_model`, set in `run_gui` to a local that lives until `glutMainLoop` returns, and its own view state (`show_trajectory`, `show_point_cloud`, the preview, the redraw flag, the title). GLUT callbacks (`display`, `mouse`, `on_exit`) take no user data, so one pointer is the minimum; everything else goes through it.
   - `PreviewCloud` stays in the view: it holds an OpenGL display list. It reads the run through `view_model->run()` as it reads `pipeline` today.
   - `apply_background` stays in the view (it writes `core`'s `bg_color`), reading `view_model->preferences()`.

4. **A library, `lo_gui_view_model`.** `gui_view_model.cpp` builds into a static library that links `lo_infrastructure` and gets only its include directories (no ImGui, GLUT, GL or dialog paths), so a view include there fails to compile. The exe and the tests link it. The check: temporarily include `<imgui.h>` and `<GL/freeglut.h>` in `gui_view_model.cpp`; both must fail; revert. **Added in task 3.1:** linking the view-model into the tests failed with `LNK1104 freeglut.lib`. `core`'s `export_laz.h` → `session.h` → `point_cloud.h` → `GL/freeglut.h` is compiled into the result and preview sinks (`WITH_GUI=1`), and freeglut's header auto-links `freeglut.lib` by pragma; the tests had never pulled those objects in before. `lo_infrastructure` is now compiled with `FREEGLUT_LIB_PRAGMAS=0`; the tests then link with no unresolved symbol, so no adapter calls GLUT.

5. **Tests** (`tests/gui_view_model_test.cpp`), with a recording notifier:
   - `load_default_parameters` on a missing file keeps the defaults and "built-in defaults"; on `tests/data/lidar_vertical.golden.toml` it loads (one setting compared) and the source becomes that path;
   - `load_parameters` on an unreadable file returns false and leaves the source;
   - `start_run` on a folder that doesn't exist returns false and starts nothing;
   - `start_run` on an empty temporary folder starts, and after `wait_for_run` the notifier got `load_failed` with that folder, `running()` is false, and `run_parameters()` is the source at the start. (Behaviour of today's code for an empty folder; checked first in the task, and the test follows what it does.)
   - `GuiProgress`: pause blocks `checkpoint` until resume (with a short-lived thread); `reset` clears pause, windows done and the stage.

6. **Commit order**, each one green:
   1. `GuiProgress` into its header (pure move);
   2. `GuiViewModel`, `RunNotifier` and the library, with the GUI using it; the enforcement check;
   3. the tests;
   4. the docs.

7. **Checks:**
   - every code commit: the step checks (both presets build `dMapper` and `lidar_odometry_tests`, `ctest`; precise and fast `44_short` IDENTICAL, which shows the CLI and the libraries are untouched);
   - `git diff --stat <change start> -- src/domain src/application src/infrastructure core` stays empty, so the full 44 isn't run: no processing code changes;
   - **a GUI run by the user** after task 2: start-up with the default settings (title, background), Load parameters (`lidar-vertical.toml`, its name in the Start menu), Load data and process on `44_short`: the progress window with Pause/Resume, the preview, the "saved" message, and `lio_result_<n>` IDENTICAL with the `44_short` manifest; closing the program clears the cache.

## Risks / Trade-offs

- **[Thread ownership changes timing]** Today the thread object is detached at once; now the view-model holds it. → Detaching in the destructor keeps the exit behaviour; nothing waits on it in the GUI.
- **[Static initialisation order]** Today `pipeline` and `gui_progress` are globals constructed before `main`, and `console` is referenced by both. As members of one object they are constructed in member order. → `console` is declared first, then `progress`, then `pipeline`.
- **[A GUI regression the command line can't see]** → The user's GUI run (decision 7) covers each menu entry and the progress window once.
- **[The empty-folder behaviour is not what the test assumes]** e.g. `open` succeeds with 0 chunks and the run saves an empty result. → Task 2 checks it before the test is written, and the test records what the code does; a surprising behaviour is reported, not changed.
