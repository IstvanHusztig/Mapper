# Tasks

Every code commit: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`.

## 1. Ports and the console adapter

- [x] 1.1 Add `app/run_reporting.h`:
  - `LogLevel`, `Logger`, `RunStage` and `ProgressReporter` (design decision 1);
  - the `LogMessage` builder with `log_debug/info/warning/error(logger)` (design decision 2).

  Verify it compiles in the tests target without Eigen or any app header.
- [x] 1.2 Add `app/console_reporting.{h,cpp}` with `ConsoleReporter` (design decision 4):
  - one mutex;
  - stdout for Debug/Info, stderr for Warning/Error;
  - `[stage] <name>` lines;
  - the in-place `running iterations: i/n` counter during Loading, closed before any log line and at the next stage.

  Add it to `SOURCES` in `CMakeLists.txt`; verify both presets build.
- [x] 1.3 Add `tests/run_reporting_test.cpp` and compile `app/console_reporting.cpp` into `lidar_odometry_tests`. Cover:
  - `LogMessage` makes one write per message, with no trailing newline;
  - level → stream routing;
  - 8 threads × 1000 lines all come out whole;
  - the counter is closed before a log line.

  Verify `ctest` passes; commit.

## 2. Ports into the pipeline API (no message converted yet)

- [x] 2.1 Change the signatures (design decision 3):
  - `compute_step_1` and `compute_step_2` take `Logger&, ProgressReporter&` instead of the atomics. Replace their pause loops with `progress.checkpoint()`, and the `loProgress.store` with `progress.progress(i + 1, n)`. The `running iterations` counter in `compute_step_1` becomes `progress.progress(...)`.
  - `load_data`, `calculate_trajectory`, `run_loop_closure`, `run_consistency`, `Consistency`, `Consistency2`, `save_result` and the helpers in design Context take `Logger&`.

  Verify both presets build.
- [x] 2.2 `OdometryPipeline`:
  - It takes `Logger&, ProgressReporter&` in its constructor (stored as pointers).
  - Stage reporting: `stage_started` with a `checkpoint()` in `load` (Loading), `run_odometry` (Odometry), `post_process` (LoopClosure / Consistency, only when each runs) and `save` (Saving).
  - `load`, `run_odometry` and `run_all` lose their atomic parameters.

  The CLI branch of `main()` builds a local `ConsoleReporter` and passes it as both ports. Verify the CLI console shows the five `[stage]` lines on `44_short` (Consistency only if enabled in `lidar-vertical.toml`). Run the step checks; commit.

## 3. Convert the messages (one commit per file)

- [x] 3.1 `app/lidar_odometry_utils.cpp` (`load_imu`, `load_point_cloud`, `save_poses`):
  - the `fprintf(stderr, "DLL ERROR …")` and `std::cerr` lines become `log_error`;
  - the per-file `total - filter = kept` line becomes one `LogMessage`.

  Verify that a grep for `std::cout|std::cerr|printf` in the file finds nothing, and that the `44_short` console shows whole per-file lines. Run the step checks; commit.
- [x] 3.2 `app/lidar_odometry.cpp`:
  - convert `load_data`, `calculate_trajectory`, `compute_step_1`, `run_consistency`, `save_result`, `save_parameters_toml`, `save_processing_results_json`, `load_reference_point_clouds`;
  - leave the trajectory `std::ofstream` writes (about lines 793/806) untouched.

  Verify the grep finds no console output calls. Run the step checks; commit.
- [ ] 3.3 `app/lidar_odometry_utils_optimizers.cpp`:
  - convert `compute_step_2`, `Consistency`, `Consistency2`, `optimize_lidar_odometry` (pass the `Logger&` through), `align_to_reference`;
  - the multi-statement `lm_factor`/`delta` output becomes one `LogMessage` per line;
  - drop the sticky `std::setprecision(10)` dependence: give any later message that printed doubles with it an explicit precision.

  Verify the grep finds no console output calls. Run the step checks; commit.
- [ ] 3.4 `app/loop_closure.cpp`. Verify the grep finds no console output calls, and that a grep for `sleep_for|std::atomic` across `app/lidar_odometry*.cpp`, `app/loop_closure.cpp` and `app/odometry_pipeline.*` finds nothing. Run the step checks; commit.

## 4. GUI

- [ ] 4.1 Add `GuiProgress` in `lidar_odometry_gui.cpp` (design decision 5):
  - atomics for stage, fraction and stage start;
  - `paused` with a `condition_variable`;
  - forwards to the `ConsoleReporter`.

  Wire the global `pipeline` to the global `ConsoleReporter` and `GuiProgress`. Remove `loProgress`, `loPause` and `loStep2StartTime`. Verify both presets build.
- [ ] 4.2 Progress window:
  - the stage name;
  - the percentage, elapsed time and estimated finish for Odometry only (estimate measured from the odometry stage start);
  - the stage name and elapsed time for other stages;
  - Pause and Resume through `GuiProgress`.

  Verify both presets build.
- [ ] 4.3 One-click flow (design decision 6):
  - split `step1()` into picking the folder (main thread) and loading;
  - `openData` runs `load` → `run_all` → `save` on the worker thread with `loRunning` set before it starts;
  - a load failure keeps the existing message box and exit.

  The advanced "Load data" stays on the main thread. Verify both presets build. Run the step checks; commit.

## 5. Integration

- [ ] 5.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json`.
- [ ] 5.2 GUI check, **done by the user**:
  - start the Release GUI and load `settings/lidar-vertical.toml`;
  - open `44_short` with the one-click flow and check that the progress window shows Loading → Odometry (%) → Loop closure (if enabled) → Saving;
  - press Pause during Odometry, wait, then Resume.

  Then I verify that the new `lio_result_*` is IDENTICAL with `44_short_baseline` on all 11 files.
- [ ] 5.3 Docs:
  - `CLAUDE.md` Architecture: the ports, `ConsoleReporter`, `GuiProgress`, and "no console output in pipeline code";
  - `docs/ARCHITECTURE.md`: mark step 3 done and fix the stale "Nothing here is implemented yet" status line.

  Commit.
- [ ] 5.4 Run `openspec validate logger-progress-ports --strict` and confirm it passes.
