# Design

## Context

See proposal.md for the motivation. Current state:

- **Console output in the pipeline** (`std::cout`/`std::cerr`/`printf`/`fprintf(stderr)`, counted per function):

  | Function | Prints |
  |---|---|
  | `compute_step_2` | 26 |
  | `save_result` | 20 |
  | `load_data` | 14 |
  | `compute_step_1` | 10 |
  | `Consistency2` | 10 |
  | `calculate_trajectory` | 9 |
  | `load_point_cloud` | 8 |
  | `Consistency` | 5 |
  | `save_poses` | 5 |
  | `run_loop_closure` | 4 |
  | `run_consistency` | 4 |
  | `save_processing_results_json` | 4 |
  | `save_parameters_toml` | 3 |
  | `load_imu` | 3 |
  | `load_reference_point_clouds` | 2 |
  | `optimize_lidar_odometry` | 1 |
  | `align_to_reference` | 1 |

- **Ordinary messages:** most messages are whole lines. Some are built from several `<<` pieces across statements: the per-file `total - filter = kept` line in `load_point_cloud`, and the lines in `compute_step_2` around `lm_factor`/`delta`.
- **Interleaving:** `load_point_cloud` and `load_imu` run inside `tbb::parallel_for`, so their pieces interleave across threads today.
- **In-place counter:** `compute_step_1` prints `\rrunning iterations: i/n` with `std::flush`, and ends with `(done)` and a newline.
- **Sticky precision:** `compute_step_2` (about line 2029) sets `std::setprecision(10)` on `std::cout` and never resets it, so later console numbers inherit it.
- **Progress:** only `compute_step_2` reports it, via `loProgress.store((i + 1) / n)`.
- **Pause:** `compute_step_1` and `compute_step_2` busy-wait once per window with `while (pause) sleep_for(1 s)`. Loop closure, consistency and saving have neither progress nor pause.
- **The GUI's simple flow** (`openData`) runs `step1()` (folder dialog, then `pipeline.load`) on the main thread. Then a detached thread runs `step2()` → `save_results`. The progress window is drawn only while `loRunning` is set, and the main loop is blocked during loading, so loading is never visible.
- **The advanced flow** ("Load data", "Compute all") runs on the main thread and never shows the progress window.
- **Untouched by this change:**
  - `core/` prints too; it is wrapped and moved only when touched.
  - `app/toml_io.cpp` is the future `ConfigRepository` adapter (step 4).
  - The GUI's own messages belong to the presentation layer.

## Goals / Non-Goals

**Goals:**
- The pipeline functions listed above reach the console only through a `Logger`, and report stage, progress and pause only through a `ProgressReporter`.
- Output files bit-identical, console wording kept.
- A visible "Loading" stage in the one-click GUI flow.

**Non-Goals:**
- Log levels configurable from settings, timestamps, or a log file (the user decided: console only).
- Replacing the `debugMsg` flag. Messages printed only under `if (debugMsg)` keep that guard and use the `Debug` level.
- Cancelling a run, which would be a new feature.
- Progress for loop closure, consistency or saving inside the stage. Only the stage name is shown there.
- Moving the advanced flow's "Load data"/"Compute all" onto a thread.
- Removing the GUI's file-scope globals.

## Decisions

1. **The ports are header-only interfaces in `app/run_reporting.h`:**
   ```cpp
   enum class LogLevel { Debug, Info, Warning, Error };
   class Logger {
   public:
       virtual ~Logger() = default;
       virtual void write(LogLevel level, std::string_view message) = 0; // one whole message, no trailing newline
   };

   enum class RunStage { Loading, Odometry, LoopClosure, Consistency, Saving };
   class ProgressReporter {
   public:
       virtual ~ProgressReporter() = default;
       virtual void stage_started(RunStage stage) = 0;
       virtual void progress(std::size_t done, std::size_t total) = 0; // within the current stage
       virtual void checkpoint() = 0;   // a safe point to pause; returns when the run may go on
   };
   ```
   They depend only on the standard library, so they can later move into `lo_application` (step 7) unchanged.

   *Alternatives:*
   - A global logger (`spdlog`-style singleton) breaks the rule "no globals in application code".
   - Passing a `std::ostream&` keeps the interleaving problem and the sticky stream state.

2. **A message builder keeps the call sites readable:**
   ```cpp
   log_info(logger) << "Loading point clouds (" << n << " files)..";
   ```
   - `LogMessage` collects into its own `std::ostringstream` and calls `Logger::write` once in its destructor.
   - Messages built from pieces over several statements become one `LogMessage` variable, so the line is written whole.
   - A trailing `\n` or `std::endl` in the old text is dropped; the adapter ends the line.
   - Each message has a fresh stream, so the sticky `setprecision(10)` disappears. A message that relied on it gets an explicit `std::setprecision`.

   *Alternative:* `std::format` strings would mean rewriting about 125 messages and their number formatting by hand, so they would risk changing wording.

3. **Passing the ports:** explicit parameters, like the config.
   - `load_data`, `calculate_trajectory`, `compute_step_1`, `compute_step_2`, `run_loop_closure`, `run_consistency`, `Consistency`, `Consistency2`, `save_result` and the helpers listed in Context get a `Logger&`.
   - `compute_step_1` and `compute_step_2` also get a `ProgressReporter&`, which replaces `std::atomic<float>&` and `const std::atomic<bool>&`.
   - `optimize_lidar_odometry` gains one more parameter; step 5 folds its arguments into option structs.
   - `OdometryPipeline` takes `Logger&` and `ProgressReporter&` in its constructor and stores pointers, so the GUI can still reassign its global `pipeline`. It calls `stage_started` at each stage. `Saving` is reported from `save()`, so the GUI's advanced "Save result" also shows it.
   - `load`, `run_odometry` and `run_all` lose their atomic parameters.

4. **`ConsoleReporter`** (`app/console_reporting.{h,cpp}`) implements both ports with one mutex, so log lines and the in-place counter never cut into each other:
   - **Messages:** `write` prints the message plus `\n` to stdout (Debug, Info) or stderr (Warning, Error) under the lock.
   - **Stage start:** prints one line such as `[stage] Odometry`.
   - **The in-place counter:** `progress` prints `\r<label> i/n` with a flush, only during `Loading`. The label is `running iterations:` there, matching today's text. `compute_step_2` already prints one line per window, so no counter is printed for the odometry stage.
   - **Interrupted counter:** if a log line arrives while the counter is open, the reporter ends the counter line first. The next stage start writes `(done)` and a newline, as today.
   - **Pause:** `checkpoint()` returns at once.
   - **Streams:** the reporter writes to `std::ostream&` targets given in the constructor (default `std::cout`/`std::cerr`), so tests can capture them.

5. **`GuiProgress`** (in `lidar_odometry_gui.cpp`, the presentation layer) implements `ProgressReporter`:
   - **State:** it stores the stage, the fraction and the stage start time in atomics that the progress window reads, and forwards `stage_started` and `progress` to the `ConsoleReporter`, so the GUI console looks like the CLI's.
   - **Pause:** `checkpoint()` waits on a `std::condition_variable` while `paused` is set. Pause and Resume set the flag and notify. The worker blocks without polling. The only `sleep_for` left is gone from the pipeline.
   - **Elapsed time and estimate:** both are computed from the stage start time. `loStep2StartTime` and the `loProgress` atomic are removed.

6. **One-click flow:**
   - `openData` keeps the folder dialog on the main thread, sets `loRunning`, and starts the worker thread, which runs `load` → `run_all` → `save`.
   - A load failure shows the existing message box from the worker thread and exits, as today. `pfd::message` is already used from that thread.
   - `step1()` is split into "pick the folder" and "load it", and the advanced "Load data" button calls both on the main thread as before.

7. **Pause checkpoints stay where the pause checks are today:** once per window in `compute_step_1` and `compute_step_2`, plus one at each stage start in `OdometryPipeline`. A checkpoint never touches data, so pausing can't change results.

8. **Tests** (`tests/`, doctest, no display):
   - `LogMessage` composes one `write` per message, with the right level and without a trailing newline.
   - `ConsoleReporter` sends levels to the right stream. Lines written from 8 threads at once all come out whole.
   - The in-place counter is closed before a log line.

   The tests compile `app/console_reporting.cpp` directly, as they do `toml_io.cpp`.

## Risks / Trade-offs

- **[A message changes the result]:** a print could have a side effect (an argument that calls a function, or stream state that a file writer shares).
  → Grep `std::cout` uses that build strings later written to files. The `save_result` file writers use their own `std::ofstream`, so they keep their manipulators (lines 793/806 write to the trajectory file, not to the console). Run `44_short` after each converted file, and `44` at the end.
- **[Missed prints]:** prints in `if (debugMsg)` branches don't run on the baseline, so a mistake there would go unnoticed.
  → A grep check in the tasks: zero `std::cout|std::cerr|printf` in the converted files. One `--debug`-free compile covers syntax; debug branches compile in the same build.
- **[Signature churn]:** `optimize_lidar_odometry` grows to about 41 parameters.
  → Accepted until step 5 (option structs).
- **[Thread and GUI lifetime]:** the worker thread uses the global `GuiProgress` and `ConsoleReporter`.
  → Both are file-scope globals that outlive the detached thread, like `pipeline` does today.
- **[Console diffs]:**
  - the extra `[stage]` lines;
  - default precision where the sticky `setprecision(10)` used to apply.

  → Console only, not part of the manifests; mentioned in the proposal.

## Migration Plan

Convert in small commits, each checked on `44_short`:
1. ports and adapters, with tests;
2. pipeline signatures and `OdometryPipeline` (the CLI uses `ConsoleReporter`);
3. one commit per converted file;
4. the GUI adapter and the threaded one-click flow.

Then run the full `44` once. The user checks the GUI by hand (stages, pause/resume, result identical). Rollback is a plain revert: no file formats or settings change.
