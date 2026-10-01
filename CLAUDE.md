# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A standalone extraction of the `lidar_odometry_step_1` application from the HDMapping (hd-mapping) repository. It is a C++20 CMake project that builds a single executable, `lidar_odometry_step_1`, which computes LiDAR+IMU odometry from a folder of Mandeye-style recordings. All dependencies are vendored under `3rdparty/` (Eigen, freeglut, glew-cmake, LASzip, oneTBB, Fusion AHRS, imgui/ImGuizmo/implot, manif, nlohmann json, observation_equations, portable-file-dialogs) — no package manager, no network needed. Only OpenGL/GLU must come from the system.

There are no linters or CI. Unit tests use doctest (`3rdparty/doctest`), and refactors are checked against a golden baseline (see Testing).

## Build

Requires CMake >= 4.0. `CMakePresets.json` defines exactly two configurations, `x64-Release` and `x64-Debug` (Ninja + cl, build trees `out/build/<preset>`); Visual Studio uses them via Open Folder. Do not add other build trees. From a shell, `vcvars64.bat` must be loaded first (cl/ninja on PATH):

```
cmake --preset x64-Release
cmake --build --preset x64-Release --target lidar_odometry_step_1
```

- Output: `out/build/<preset>/bin/lidar_odometry_step_1.exe`. On Windows, runtime DLLs (TBB etc.) and `app/default_settings.toml` are copied next to the exe post-build. Debug is unoptimized and many times slower; always process data with Release.
- `-DHD_CPU_OPTIMIZATION=AUTO|INTEL|AMD|ARM|GENERIC` selects release optimization flags (AMD on MSVC adds `/fp:fast`).
- The version (`HDMAPPING_VERSION_MAJOR/MINOR/PATCH`) is defined in the root `CMakeLists.txt`; `shared/include/HDMapping/Version.hpp` errors if it's missing.

## Testing

- **Unit tests:** `cmake --build --preset x64-Release --target lidar_odometry_tests`, then `ctest --test-dir out/build/x64-Release` (vcvars loaded). Tests live in `tests/` and must run without recorded data or a display. They compile the app sources they need directly (today only `app/toml_io.cpp`) and don't link `core`.
- **Golden baseline:** a pure refactor must reproduce the reference output bit for bit, compared with `tools/compare_baseline.py` against `tools/baseline/44_short.manifest.json` (every intermediate commit, about 3.5 min) and `tools/baseline/44.manifest.json` (once per change, about 30 min). The datasets are in `C:\Users\User\Downloads\dLidar\raw data\` (`44_short` was made with the `cut_recording` target). Run Release with `settings/lidar-vertical.toml`, and never run two jobs on the same input folder at once (they share its `cache/`). See `tools/README.md`.

## Run

- **Headless / CLI:** `lidar_odometry_step_1 <input_folder> <params.toml> <output_folder>`. Exactly 3 args triggers batch mode. The settings are the built-in defaults plus `<params.toml>` only, never a `default_settings.toml` from the working directory; an unreadable TOML exits 1.
- **GUI:** any other argument count opens the freeglut + ImGui window. It loads `default_settings.toml` from the exe folder at start-up. "Compute all" and the simple one-click flow run the same stages as the CLI, so for the same input and settings the GUI result is identical to the CLI's.

The input folder is scanned non-recursively; `load_data` (`app/lidar_odometry.cpp`) sorts files by extension: `.laz`/`.las` point clouds, `.csv` IMU data; everything else is ignored. There is no multi-LiDAR calibration: points are only shifted by the lever arm (`lidar_imu_lever_arm_*`).

## Architecture

Target architecture (Clean Architecture, layers, ports, migration roadmap) is in `docs/ARCHITECTURE.md`; refactors should move the code toward it. What follows describes the current code.

**Pipeline:** `OdometryPipeline` (`app/odometry_pipeline.{h,cpp}`) owns one run's data (`points_per_file`, `imu`, `trajectory`, `worker_data`, `state`) and runs the stages `load` (1–3) → `run_odometry` (4) → `post_process` (loop closure if `use_loop_closure`, then consistency if `use_consistency`) → `save` (5). The CLI branch of `main()` and the GUI (global `pipeline`) both use it:
1. `load_data` — read LAZ chunks + IMU CSV, apply calibration, decimate.
2. `calculate_trajectory` — IMU-only orientation via Fusion AHRS (NWU/ENU/NED convention flags in params).
3. `compute_step_1` — cut the data into `WorkerData` windows (points + intermediate trajectory per chunk).
4. `compute_step_2` (`app/lidar_odometry_utils_optimizers.cpp`) — per-window scan-to-map optimization against an NDT-bucket reference map (`optimize_lidar_odometry`, `optimize_sf*`, `optimize_icp`), optional `Consistency`/`Consistency2` passes (`run_consistency`, run in `post_process` when `use_consistency = true`). Loop closure: `run_loop_closure` (`app/loop_closure.cpp`).
5. `save_result` (`app/lidar_odometry.cpp`) — write session, trajectories, LAZ, `parameters.toml` and processing-results JSON to the output folder.

**Settings and state** (all in `app/lidar_odometry_utils.h`):
- `OdometryConfig` holds every setting and is passed `const&` to the pipeline. Only TOML loading and the GUI's settings widgets write to it.
- `OdometryState` holds what a run produces: the NDT maps, `m_g`, the initial and reference points, the trajectory length, and `RunPaths` (the `[paths]` keys plus the cache folder). Functions that change it take it explicitly.
- `GuiPreferences` holds GUI-only values (`clear_color`, saved in `[misc]`).

GUI defaults are loaded by `set_lidar_odometry_default_params` (in `lidar_odometry_gui.cpp`) from `default_settings.toml` next to the exe; the CLI doesn't load it. TOML I/O (`app/toml_io.h`) is table-driven and takes `(OdometryConfig, RunPaths, GuiPreferences)`. To make a new setting loadable and savable, add it to both `TomlIO::POINTERS` (name → `OdometryConfig` member pointer; only bool/double/int/string supported) and the right `CATEGORIES` section, then regenerate the golden TOML test files (`LO_UPDATE_GOLDEN=1`, in their own commit). `app/default_settings.toml` is the shipped config.

**IMU rotation prior and gyro bias** (in `optimize_lidar_odometry` / `compute_step_2`, all on by default):
- `[motion_model_uncertainty]`: `use_time_scaled_rotation_sigma` replaces the constant `*_om/fi/ka_1_sigma_deg` with `max(rotation_sigma_inflation_k * gyro_arw_{x,y,z}_deg_sqrt_s * sqrt(dt), rotation_sigma_floor_deg)` per consecutive-node edge.
- `[gyro_bias_estimation]`: `use_gyro_bias_estimation` adds a 3-axis gyro bias to each window's unknowns, carried to the next window as a prior. The prior sigma starts at `gyro_bias_initial_sigma_deg_s`, grows by `gyro_bias_rw_deg_s_sqrt_s`, and the bias is clamped at `gyro_bias_max_deg_s`. `handoff_tail_nodes` re-propagates each window's last nodes from the IMU before they seed the next window (0 = off). The per-window bias is written to `gyro_bias_per_window` in the results JSON.
- Switching all three off (`false`, `false`, `0`) reproduces the previous behaviour. The IMU CSV timestamps must be strictly increasing.
- `tools/eval_odometry.py` (see `tools/README.md`) computes the trajectory and map-sharpness metrics used to validate changes here.

**`core/` library:** shared HDMapping code (point cloud/session types, NDT, ICP, pose-graph SLAM, GNSS, LAZ export, observation-equation-based optimizers). Built as a static library `core` with `WITH_GUI=1`. `core/src/utils.cpp` is compiled directly into the exe, not into `core`.

**Critical link-order constraint:** `CORE_SOURCE_FILES` in `CMakeLists.txt` is an explicit list that must mirror the original hd-mapping `core/CMakeLists.txt` — do **not** replace it with a glob or compile core sources directly into the exe. The app relies on static-archive semantics: `hash_utils.cpp` defines `get_index()`/`get_rgd_index()` that would collide with the app's own definitions, and `surface.cpp` duplicates globals from `pose_graph_slam.cpp` (so it is intentionally excluded). Adding a new `core/src/*.cpp` that the app needs means adding it to that list and checking for symbol collisions.

GLEW is only used for headers (never initialized or linked). The GUI keeps file-scope globals in `lidar_odometry_gui.cpp`, because GLUT callbacks need them: `params`, `gui_prefs`, the current `pipeline`, and UI state (view flags, gizmo, progress). The CLI branch of `main()` uses only locals.
