# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

dMapper started as a standalone extraction of the LiDAR odometry (step 1) application of the HDMapping (hd-mapping) repository. It is a C++20 CMake project that builds a single executable, `dMapper`, which computes LiDAR+IMU odometry from a recording folder (lidar*.laz + imu*.csv). All dependencies are vendored under `3rdparty/` (Eigen, freeglut, glew-cmake, LASzip, oneTBB, Fusion AHRS, imgui/ImGuizmo/implot, manif, nlohmann json, observation_equations, portable-file-dialogs) — no package manager, no network needed. Only OpenGL/GLU must come from the system.

There are no linters or CI. Unit tests use doctest (`3rdparty/doctest`), and refactors are checked against a golden baseline (see Testing). Mechanical renames in refactors are done with `tools/rename_identifiers.py` (code only, never comments or strings; `-m` for member accesses only; `--dry-run` first).

## Build

Requires CMake >= 4.0. `CMakePresets.json` defines exactly two configurations, `x64-Release` and `x64-Debug` (Ninja + cl, build trees `out/build/<preset>`); Visual Studio uses them via Open Folder. Do not add other build trees. From a shell, `vcvars64.bat` must be loaded first (cl/ninja on PATH):

```
cmake --preset x64-Release
cmake --build --preset x64-Release --target dMapper
```

- Output: `out/build/<preset>/bin/dMapper.exe` (the name is `APP_NAME` in `CMakeLists.txt`). On Windows, runtime DLLs (TBB etc.) and `settings/default_settings.toml` are copied next to the exe post-build. Debug is unoptimized and many times slower; always process data with Release.
- `-DHD_CPU_OPTIMIZATION=AUTO|INTEL|AMD|ARM|GENERIC` selects release optimization flags (AMD on MSVC adds `/fp:fast`).
- The version (`HDMAPPING_VERSION_MAJOR/MINOR/PATCH`) is defined in the root `CMakeLists.txt`; `shared/include/HDMapping/Version.hpp` errors if it's missing.

## Testing

- **Unit tests:** `cmake --build --preset x64-Release --target lidar_odometry_tests`, then `ctest --test-dir out/build/x64-Release` (vcvars loaded). Tests live in `tests/` and must run without recorded data or a display. They link the layer libraries (`lo_infrastructure`, which brings `lo_application` and `lo_domain`) and don't link `core`. `tests/scan_to_map_test.cpp` runs `optimize_lidar_odometry` on a synthetic room and corridor against the true trajectory, with maps built with the exact NDT cell statistics. `tests/ndt_map_test.cpp` tests `update_rgd` itself in both modes.
- **Golden baseline:** a pure refactor is compared with `tools/compare_baseline.py`:
  - `44_short` must be bit-identical with `tools/baseline/44_short.manifest.json`; check it on every intermediate commit (about 3.5 min).
  - A refactor that reshapes numeric code (locals into structs, blocks into functions) can change the `/fp:fast` build's last bits even when it is exact. Then check it with a **precise build** instead: a Release build without `/fp:fast` (`-DHD_CPU_OPTIMIZATION=GENERIC`, built outside the repo's `out/` tree), whose `44_short` must stay bit-identical with a precise reference made from master, while the normal build only has to be within `--tolerance 1e-6` of `raw data/44_short_baseline`. Precise and fast results differ by millimetres over a run, so never compare one with the other. See `tools/README.md`.
  - The full 44 runs once per change (about 30 min). It must match `tools/baseline/44.manifest.json`, or else be within `--tolerance 1e-6` of the baseline folder `raw data/44_baseline`. The Release `/fp:fast` build can change the last bits of the long run when unrelated code changes. The datasets are in `C:\Users\User\Downloads\dLidar\raw data\` (`44_short` was made with the `cut_recording` target). Run Release with `settings/lidar-vertical.toml`, and never run two jobs on the same input folder at once (they share its `cache/`). See `tools/README.md`.

- **Subagents:** Claude may start subagents on its own when it sees a need, without asking first. Good candidates are long, self-contained jobs with a small result, such as a full 44 evaluation (runs, CloudCompare export, `eval_odometry.py`, counting log lines) or a broad code search. Give the subagent the exact scripts, paths and pass criteria, and check what it reports before acting on it.

## Run

- **Headless / CLI:** `dMapper <input_folder> <params.toml> <output_folder>`. Exactly 3 args triggers batch mode. The settings are the built-in defaults plus `<params.toml>` only, never a `default_settings.toml` from the working directory; an unreadable TOML exits 1.
- **GUI:** any other argument count opens the freeglut + ImGui window, titled "dMapper". The menu bar has Start, View and Camera:
  - **Start → 1. Load parameters…** loads a TOML. This is the only way to change settings in the GUI; at start-up it loads `default_settings.toml` from the exe folder. The Start menu and the progress window show the active parameter file.
  - **Start → 2. Load data and process…** (Ctrl+O) picks the recording folder, then loads it on a worker thread, processes it with the same stages as the CLI, and saves `lio_result_<n>`. For the same input and settings the result is identical to the CLI's.
  - **View** has Show point cloud, Show trajectory, compass/ruler and Lock Z; **Camera** has the fixed views and orthographic projection.
  - **Live point cloud preview** (`PreviewCloud` in `lidar_odometry_gui.cpp`):
    - built on the main thread from data the run already keeps: each window the odometry has reported done (`GuiProgress::windows_done()`) is read through `OdometryPipeline::load_window_points` and placed with its optimized trajectory;
    - thinned to 0.10 m voxels across all windows and coloured by height, at most 4 windows per frame;
    - rebuilt once from the final trajectory after the run (after loop closure and consistency);
    - it never changes processing or results.
  - The progress window shows the current stage (Loading, Odometry with percentage and estimate, Loop closure, Consistency, Saving) and can pause the run.
  - The window redraws on a timer (`schedule_redraw`): about 10 frames per second during a run and 60 otherwise, so the redraw doesn't slow the processing. A slower run would change results through `real_time_threshold_seconds`.
  - The background colour comes from `[misc] clear_color` in the loaded TOML.
  - There are no settings widgets, presets, georeferencing, or export/editing tools. Georeferencing is meant to be a separate post-processing module.

The input folder is scanned non-recursively. `pair_recording_files` (`src/infrastructure/recording_files.cpp`) sorts the files and splits them by extension (`.laz`/`.las` point clouds, `.csv` IMU data); everything else is ignored. LAZ and CSV files are paired by sorted order, and each must have a partner with the same last four characters of the name. There is no multi-LiDAR calibration: points are only shifted by the lever arm (`lidar_imu_lever_arm_*`).

## Architecture

Target architecture (Clean Architecture, layers, ports, migration roadmap) is in `docs/ARCHITECTURE.md`; refactors should move the code toward it. What follows describes the current code.

**Layers:** the code is in `src/`, one folder per layer, each built as a static library whose include directories admit only what the layer may use. A file that includes something its layer must not see doesn't compile:
- `src/domain` → `lo_domain`: entities and optimizers (`window.h`, `ndt_map`, `scan_to_map`, `robust_lidar_odometry`, `normal_equations`, `imu_kinematics`, `window_steps`, `optimizer_options`), plus `odometry_config.h`, `odometry_state.h`, `angle_units.h` and the `Logger`/`ProgressReporter` interfaces (`run_reporting.h`). Only Eigen, manif, observation_equations and `core/include`: `export_laz.h` fails here (no LASzip), but `core`'s session and point-cloud types still compile (their GL includes need `WITH_GUI`), so not using them in the domain is a review rule until roadmap step 11.
- `src/application` → `lo_application`: the ports (`input_ports.h`, `output_ports.h`) and the use cases (`odometry_pipeline`, `lidar_odometry`, `run_odometry`, `loop_closure`, `consistency`). Fusion and TBB are allowed here until roadmap step 9.
- `src/infrastructure` → `lo_infrastructure`: the adapters (folders, LAZ, CSV, TOML, JSON, console), built with `WITH_GUI=1` because they include core's I/O headers.
- `src/presentation/cli` (`run_cli`), `src/presentation/gui` (`run_gui`, `lidar_odometry_gui.cpp`) and `src/main.cpp`, the composition root that picks one by the argument count.

**Pipeline:** `OdometryPipeline` (`src/application/odometry_pipeline.{h,cpp}`) owns one run's data (`points_per_file`, `imu`, `imu_trajectory`, `windows`, `state`) and runs the stages `load` (1–3) → `run_odometry` (4) → `post_process` (loop closure if `use_loop_closure`, then consistency if `use_consistency`) → `save` (5). The CLI branch of `main()` and the GUI (global `pipeline`) both use it:
1. `load_data` — read the LAZ chunks and IMU CSV through the `RecordingSource`, shift the points by the lever arm, and sort them.
2. `calculate_trajectory` — IMU-only orientation via Fusion AHRS (NWU/ENU/NED convention flags in params).
3. `compute_step_1` — cut the data into `Window`s (explained in `docs/PIPELINE.md`; `src/domain/window.h`: `poses`, `motion_model_poses`, `timestamps`, `imu_angles`, `imu_acc`, `gyro_bias`); each window's points go to the `PointStore`.
4. `optimize_windows` (`src/application/run_odometry.cpp`, the window loop) — per-window scan-to-map optimization against an NDT-bucket reference map (built by `update_rgd` in `src/domain/ndt_map.{h,cpp}`, with the cell types in `core/include/ndt_types.h`; `use_exact_ndt_bucket_statistics` picks exact cell statistics over the previous ones, off by default, see `docs/degeneracy-information.html`): `optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`), or `optimize_sf2`/`optimize_rigid_sf` (`src/domain/robust_lidar_odometry.cpp`) when `use_robust_and_accurate_lidar_odometry` is set. The loop uses the IMU helpers in `src/domain/imu_kinematics.cpp` (world-frame acceleration, rest detection, gyro-bias removal, acceleration offset); the optimizers share the normal-equation helpers in `src/domain/normal_equations.cpp`. The loop body is a list of named steps: the per-run settings are in `OdometryRunSetup` and what passes from window to window in `WindowLoopCarry`; the steps that only need window data (`place_window_after`, `place_windows_after`, `estimate_previous_velocity`, `predict_window_poses`, `apply_gyro_bias_prior`, `is_turn_window`, `repropagate_window_tail`) are in `src/domain/window_steps.cpp` and tested on synthetic windows, the others (accelerometer term, robust mode, LM iterations, map update) are file-local in `run_odometry.cpp`. `optimize_lidar_odometry` takes the `Window` and named options from `src/domain/optimizer_options.h`: `ScanToMapOptions` and `MotionModelOptions` (built once per run by `get_scan_to_map_options` / `get_motion_model_options`; `optimize_windows` may raise a window's position sigmas when it is degenerate) and `VelocityPrior` (the previous window's final velocity), plus the rotation and translation IMU options (`MotionModelRotationOptions`, `MotionModelTranslationImuOptions`, same header). Also optional `Consistency`/`Consistency2` passes (`src/application/consistency.cpp`; `run_consistency`, run in `post_process` when `use_consistency = true`). Loop closure: `run_loop_closure` (`src/application/loop_closure.cpp`).
5. `save_result` (`src/application/lidar_odometry.cpp`) — concatenate the windows into output chunks, place the points and compute the relative trajectories, and hand everything to the `ResultSink`. It does no file I/O.

**Input I/O:** pipeline code has no file names, folders, LASzip calls or CSV parsing on the input side. It uses three ports from `src/application/input_ports.h`:
- `RecordingSource` reads the LiDAR + IMU chunk pairs.
- `PointStore` holds point sets between stages: each window's original and intermediate points, and the session chunks of `save_result`.
- `PreviewSink` writes `calibrationValidation.asc` and `temp_point_cloud_<i>.laz`.

`OdometryPipeline` owns them as `RunInputs`, built by `make_folder_inputs(files, logger)` (`src/infrastructure/folder_inputs.cpp`) for a recording folder. There is one adapter for each port:
- `RecordingFolderSource` (`src/infrastructure/recording_folder_source.{h,cpp}`, which also holds `load_imu` and `load_point_cloud`);
- `DiskPointStore` (`<folder>/cache/*.cache`, window files numbered from 0);
- `FolderPreviewSink` (`<folder>/preview`).

The pipeline records the cache and preview folders in `state.paths` from the ports' `location()`. The app doesn't use `core`'s `WorkerData` any more; it only remains because `core/src/session.cpp` uses it. **Output I/O and settings:** two ports from `src/application/output_ports.h`:
- `ResultSink` receives a finished run in a fixed order: point sizes, each chunk's cloud and `TrajectoryRow`s, the merged cloud, chunk poses, index poses, the `SessionSummary`, the parameters and the processing results.
- `ConfigRepository` loads and saves `OdometryConfig` + `RunPaths` + `GuiPreferences` by location.

Their adapters:
- `FolderResultSink` (`src/infrastructure/folder_result_sink.{h,cpp}`) holds every output file name and format: the LAZ files and the merged LAZ writer, the trajectory CSV, `.mri`/`.mrp`, `index_poses.json`, `session.mjs`, and the parameters TOML (written through its `ConfigRepository`), plus the results JSON. It also has `get_next_result_path` (the GUI's `lio_result_<n>`).
- `TomlConfigRepository` (`src/infrastructure/toml_config_repository.{h,cpp}`) wraps `TomlIO`.

`OdometryPipeline::save(ResultSink&, gui, elapsed)` writes a run. The CLI builds a `FolderResultSink` on `argv[3]`, and the GUI builds one on the next `lio_result_<n>`. The CLI, the GUI defaults and Start → Load parameters load settings, and `FolderResultSink` saves them, only through `TomlConfigRepository`; `TomlIO` is used only inside it.

**Reporting:** pipeline code never writes to the console and has no progress atomics or sleeps. It uses two ports from `src/domain/run_reporting.h`, passed in explicitly:
- `Logger` for messages: `log_info(logger) << ...`, also `log_debug`, `log_warning`, `log_error`. Each message has its own stream, so it sets its own precision.
- `ProgressReporter` for the stage (`RunStage`), progress within the stage, and `checkpoint()`, where a paused run waits.

`OdometryPipeline` takes both ports in its constructor and reports each stage. There are two adapters:
- `ConsoleReporter` (`src/infrastructure/console_reporting.{h,cpp}`) writes whole, thread-safe lines: info/debug to stdout, warning/error to stderr, a `[stage]` line per stage, and the loading counter. The CLI and the GUI both use it for messages.
- `GuiProgress` (in `lidar_odometry_gui.cpp`) feeds the progress window and implements Pause/Resume.

Logging is console only; nothing is written to the output folder.

**Settings and state** (`src/domain/odometry_config.h`: `OdometryConfig`, `RunPaths`, `GuiPreferences`; `src/domain/odometry_state.h`: `OdometryState`). Every file includes what it uses; there is no catch-all header:
- `OdometryConfig` holds every setting and is passed `const&` to the pipeline. Only TOML loading (`ConfigRepository`) writes to it; the GUI has no settings widgets.
- `OdometryState` holds what a run produces: the NDT maps, `m_g`, the initial and reference points, the trajectory length, and `RunPaths` (the `[paths]` keys plus the cache folder). Functions that change it take it explicitly.
- `GuiPreferences` holds GUI-only values (`clear_color`, saved in `[misc]`).

GUI defaults are loaded by `set_lidar_odometry_default_params` (in `lidar_odometry_gui.cpp`) from `default_settings.toml` next to the exe; the CLI doesn't load it. TOML I/O (`src/infrastructure/toml_io.h`, used only by `TomlConfigRepository`) is table-driven and takes `(OdometryConfig, RunPaths, GuiPreferences)`. To make a new setting loadable and savable, add it to both `TomlIO::POINTERS` (name → `OdometryConfig` member pointer; only bool/double/int/string supported) and the right `CATEGORIES` section, then regenerate the golden TOML test files (`LO_UPDATE_GOLDEN=1`, in their own commit). `settings/default_settings.toml` is the shipped config.

**IMU rotation prior and gyro bias** (in `optimize_lidar_odometry` / `optimize_windows`, all on by default):
- `[motion_model_uncertainty]`: `use_time_scaled_rotation_sigma` replaces the constant `*_om/fi/ka_1_sigma_deg` with `max(rotation_sigma_inflation_k * gyro_arw_{x,y,z}_deg_sqrt_s * sqrt(dt), rotation_sigma_floor_deg)` per consecutive-node edge.
- `[gyro_bias_estimation]`: `use_gyro_bias_estimation` adds a 3-axis gyro bias to each window's unknowns (`Window::gyro_bias`, a `GyroBiasState` with `mean` and `covariance`), carried to the next window as a prior. The prior sigma starts at `gyro_bias_initial_sigma_deg_s`, grows by `gyro_bias_rw_deg_s_sqrt_s`, and the bias is clamped at `gyro_bias_max_deg_s`. `handoff_tail_nodes` re-propagates each window's last nodes from the IMU before they seed the next window (0 = off). The per-window bias is written to `gyro_bias_per_window` in the results JSON.
- Switching all three off (`false`, `false`, `0`) reproduces the previous behaviour. The IMU CSV timestamps must be strictly increasing.
- `tools/eval_odometry.py` (see `tools/README.md`) computes the trajectory and map-sharpness metrics used to validate changes here.

**`core/` library:** shared HDMapping code (point cloud/session types, NDT, ICP, pose-graph SLAM, GNSS, LAZ export, observation-equation-based optimizers). Built as a static library `core` with `WITH_GUI=1`. `core/src/utils.cpp` is compiled directly into the exe, not into `core`.

**Critical link-order constraint:** `CORE_SOURCE_FILES` in `CMakeLists.txt` is an explicit list that must mirror the original hd-mapping `core/CMakeLists.txt` — do **not** replace it with a glob or compile core sources directly into the exe. `hash_utils.cpp` defines `get_index()`/`get_rgd_index()`; the app's own cell-index functions are named `ndt_cell_key()`/`ndt_cell_index()` so the two never collide. `surface.cpp` duplicates globals from `pose_graph_slam.cpp` (so it is intentionally excluded). Adding a new `core/src/*.cpp` that the app needs means adding it to that list and checking for symbol collisions.

GLEW is only used for headers (never initialized or linked). The GUI keeps file-scope globals in `lidar_odometry_gui.cpp`, because GLUT callbacks need them: `params`, `gui_prefs`, `console`, `gui_progress`, the current `pipeline`, `active_parameters`/`run_parameters`, and UI state (`show_trajectory`, `working_directory`, `loRunning`). The CLI (`run_cli`) uses only locals.
