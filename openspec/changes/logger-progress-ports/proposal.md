# Proposal

## Why

The pipeline code talks straight to the console and to the GUI.
- **Console:** about 125 `std::cout`/`std::cerr`/`printf` calls across `lidar_odometry.cpp`, `lidar_odometry_utils.cpp`, `lidar_odometry_utils_optimizers.cpp` and `loop_closure.cpp`.
- **GUI:** a `std::atomic<float>` progress value and a `std::atomic<bool>` pause flag, polled with `sleep_for` busy-waits inside `compute_step_1` and `compute_step_2`.

This blocks roadmap step 3 in `docs/ARCHITECTURE.md`. The application layer can't stay free of I/O and threads while this is in place, and nothing in it can run quietly in a unit test. It causes two user-visible problems:
- **Mixed-up console lines.** The parallel LAZ loader writes each line in several pieces from TBB threads, so lines from different files get mixed together.
- **A misleading progress bar.** The GUI only shows the odometry percentage, so it sits at 0 % while loading and at 100 % during loop closure, consistency and saving.

## What Changes

- **Two new ports in the application layer:**
  - `Logger` takes whole messages with a level (debug, info, warning, error).
  - `ProgressReporter` takes:
    - the current stage (Loading, Odometry, Loop closure, Consistency, Saving);
    - the fraction done within that stage;
    - a pause checkpoint.
- **Console adapters** (`ConsoleLogger`, `ConsoleProgress`):
  - `ConsoleLogger` is thread-safe and writes every message as one whole line. Info and debug go to stdout; warning and error go to stderr, as today.
  - `ConsoleProgress` prints the stage changes and the `running iterations: i/n` counter that `compute_step_1` prints today.
- **The pipeline functions use only the ports** (`load_data`, `compute_step_1`, `compute_step_2`, `run_loop_closure`, `run_consistency`, `save_result` and their helpers in `lidar_odometry_utils.cpp`):
  - no `std::cout`, `std::cerr` or `printf`;
  - no `std::atomic` parameters;
  - no `sleep_for`.

  `OdometryPipeline` gets the two ports in its constructor and reports each stage.
- **GUI progress window:**
  - It shows the current stage. A percentage appears only for stages that have one, and the time estimate covers only the odometry stage.
  - Pause and Resume keep working, through a GUI `ProgressReporter` adapter that blocks at the checkpoint while paused.
  - In the simple one-click flow, loading moves onto the worker thread, so the window is visible (showing "Loading") while the data loads. The folder dialog stays on the main thread.
- **Logging stays on the console only.** No log file is written to the output folder.
- **The output files don't change:** the CLI output stays bit-identical to the `44_short` and `44` baselines. Console messages keep their wording. The console differs in two ways: there is one new `[stage] <name>` line per stage, and a precision set by one message no longer carries over to later ones.

## Capabilities

### New Capabilities
- `run-progress`: how a run reports what it is doing.
  - The GUI shows the current stage and lets the user pause and resume.
  - The console shows whole, non-interleaved messages.
  - Progress and log output never change the result files.

### Modified Capabilities
- None. The `processing-pipeline` requirements (one sequence, GUI = CLI, settings sources) are unchanged.

## Impact

- **New files:**
  - `app/run_reporting.h` (the ports and a small message builder);
  - `app/console_reporting.{h,cpp}` (the console adapters);
  - unit tests in `tests/`.
- **Changed files:**
  - `app/lidar_odometry.{h,cpp}`, `app/lidar_odometry_utils.{h,cpp}`, `app/lidar_odometry_utils_optimizers.cpp`, `app/loop_closure.{h,cpp}`: the signatures gain `Logger&` and, where needed, `ProgressReporter&`, and lose the atomics;
  - `app/odometry_pipeline.{h,cpp}`;
  - `app/lidar_odometry_gui.cpp`: the CLI wiring, plus the GUI adapter and progress window;
  - `CMakeLists.txt` and `tests/CMakeLists.txt`.
- **Not touched:**
  - `core/`, which is wrapped and moved only when touched;
  - `app/toml_io.cpp`, which is infrastructure and becomes the `ConfigRepository` adapter in step 4;
  - the GUI's own console messages, which belong to the presentation layer.
- **Validation:**
  - `44_short` must be identical on every intermediate commit, and the full `44` once at the end.
  - The user checks the GUI by hand: the stages are shown, pause and resume work, and the result is identical to the CLI.
