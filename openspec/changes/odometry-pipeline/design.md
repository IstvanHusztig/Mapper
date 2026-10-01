# Design

## Context

See proposal.md for the motivation. Current state:

- **CLI branch of `main()`** (`app/lidar_odometry_gui.cpp`):
  - `set_lidar_odometry_default_params` loads `default_settings.toml` from the current directory,
    then the given TOML is loaded (a parse failure is only printed).
  - Then `step1(folder, params, state, …)` (`load_data` → `calculate_trajectory` →
    `compute_step_1`) and `step2(…)` (`compute_step_2`).
  - Then `run_loop_closure` if `use_loop_closure`, and `run_consistency` if `use_consistency &&
    num_constistency_iter > 0`.
  - Finally `save_result(…, argv[3], elapsed)`.
- **GUI, simple flow** (`openData`): `step1()` on the main thread, then on a detached thread
  `step2()` → `save_results(false, elapsed)`. **No loop closure, no consistency.**
- **GUI, advanced flow:** "Load data" → `step1()`; "Compute all" → `step2()`; an optional
  "Point cloud consistency…" button → `run_consistency`; "Save result" → `save_results(true,
  0.0)`. **No loop closure.**
- **The two GUI wrappers** `step1()`/`step2()` duplicate the CLI wrappers but work on the globals
  `worker_data`, `trajectory`, `imu_data`, `pointsPerFile` and `state` (about 120 references in
  display, gizmo editing and export code).
- `compute_step_2` reports progress through `std::atomic<float>&` and pauses on `const
  std::atomic<bool>&`. Loop closure and consistency take no progress or pause.
- **Settings coverage:**
  - `settings/lidar-vertical.toml` contains every key of `app/default_settings.toml`, so a GUI
    that loads defaults and then that file has exactly the CLI's settings.
  - Six keys are in neither file and fall back to built-in defaults. One of them,
    `convergence_delta_threshold` in `[lidar_odometry_control]`, changes results when set.
- **What the baselines prove today:** the 44 baselines are CLI runs from the repo root, which has
  no `default_settings.toml`, so they were made with built-in defaults plus `lidar-vertical.toml`.
  That is exactly the new CLI rule.

## Goals / Non-Goals

**Goals:**
- One implementation of the stage sequence, used by the CLI and both GUI flows.
- GUI output identical to CLI output (all files), and CLI output unchanged.
- CLI results independent of the current directory.

**Non-Goals:**
- Progress or cancellation for loop closure and consistency (step 3, ProgressReporter).
- Replacing `std::cout` (step 3) and I/O ports (step 4).
- Removing UI globals (view flags, gizmo, progress timing, window). GLUT callbacks need
  file-scope state. A GUI view-model is out of scope.
- Changing what `compute_step_*`, loop closure or consistency compute.

## Decisions

1. **`OdometryPipeline` is a class with public data members** for the run's data (`points_per_file`,
   `imu`, `trajectory`, `worker_data`, `state`) and methods for the stages:
   - `bool load(const std::vector<std::string>& files, const std::atomic<bool>& pause)`
   - `bool run_odometry(std::atomic<float>& progress, const std::atomic<bool>& pause)`
   - `void post_process()`
   - `void save(const fs::path& out, const GuiPreferences& gui, double elapsed_s) const`
   - `bool run_all(...)`: `run_odometry` plus `post_process`.

   It holds `const OdometryConfig&` (the caller's config outlives it) and the `debug` flag.

   *Why public members:* the GUI's display and gizmo code reads and edits `worker_data` and
   `trajectory` in about 120 places. Accessors would add churn without adding safety, and
   encapsulating the run data is step 5's domain model. *Alternative:* free functions over a
   `RunData` struct. Rejected: the class gives `post_process` a single, obvious home.
2. **`post_process()` is the only place that decides loop closure and consistency.** It moves the
   CLI's two `if`s verbatim (same order, same conditions), so the CLI output can't change. The GUI
   calls it right after `run_odometry`, on the same worker thread.
3. **GUI wiring:**
   - The global data becomes `OdometryPipeline pipeline{params};`. A new run in the GUI (opening
     another folder) replaces it with `pipeline = OdometryPipeline{params};`, so nothing from the
     previous run leaks into the next, as is implicitly the case today with a fresh process.
   - Display, gizmo and export code switch to `pipeline.worker_data` and so on, by a mechanical
     rename. `mouse(…, int state, …)` shadows the old global name, so the rename must target
     `state.` and not that parameter; the compiler catches mistakes.
   - Simple flow: `load` on the main thread as today, then on the thread `run_all` → `save`.
   - Advanced flow: "Compute all" runs `run_all`. The consistency button and its tooltip are
     removed. "Save result" passes the time measured since "Compute all" instead of `0.0`.
4. **Settings loading:**
   - `set_lidar_odometry_default_params(path, …)` gets an explicit file path.
   - The CLI does **not** call it. It loads only `argv[2]` onto a value-initialized
     `OdometryConfig`, and returns 1 with an error message if loading fails (before any
     processing or output folder).
   - The GUI calls it with `<exe folder>/default_settings.toml`. The exe folder comes from
     `GetModuleFileNameW` on Windows, with `std::filesystem::canonical(argv[0])` elsewhere.
   - The "1 Velocity < 8km/h" preset menu item uses the same exe-folder path.
5. **The CLI keeps its own small driver** (folder listing as today with `!is_directory`, timing,
   and the "calculations finished" log) in `main()`, now around a local `OdometryPipeline`. The
   CLI `step1`/`step2` wrappers are deleted.
6. **Order of work, each commit checked on `44_short`:**
   - (a) a sensitivity check of the cwd test on today's code;
   - (b) `OdometryPipeline` plus the CLI on it, with the defaults rule;
   - (c) the GUI on it (both flows, button removed, exe-folder defaults);
   - (d) the full 44, the user's GUI check, and docs.

## Risks / Trade-offs

- [The CLI output changes because `post_process` reorders or skips something] → It is moved
  verbatim and checked on `44_short` and then the full 44.
- [The cwd test passes for the wrong reason (the planted setting has no effect)] → Step (a) runs
  today's CLI with a planted `default_settings.toml` (`convergence_delta_threshold = 0.001`) in
  the working directory and must show a **difference**. Only then is "identical" after the change
  meaningful.
- [A GUI rename slip, such as the `mouse(…, int state)` shadowing, or a missed reference] → The
  old globals are deleted, so every stale reference is a compile error. The user's GUI check
  compares all 11 files with the baseline.
- [The GUI run is now longer (loop closure and consistency) and the progress bar sits at 100 %
  while they run] → Accepted until step 3. A console line ("loop closure START", which exists
  today) shows activity.
- [Starting a second run in the same GUI session without restarting] → It gets a fresh
  `OdometryPipeline`. That is a behaviour improvement, not covered by the baselines; it is noted
  in the user check.

## Findings during implementation

- **The working-directory test is sensitive (task 1.1).** Today's CLI, run with a working
  directory containing a `default_settings.toml` that sets only `convergence_delta_threshold =
  0.001`, loaded that file ("No version information found in config file:
  default_settings.toml").
  - Its `44_short` output differed from the baseline in **10 of 11 files**: the params TOML,
    results JSON, `session.mjs`, both pose files, all LAZ and trajectories. Only
    `point_sizes_per_chunk.json` matched.
  - The run took 109 s instead of about 200 s, because the optimizer stopped earlier.
