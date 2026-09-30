# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A standalone extraction of the `lidar_odometry_step_1` application from the HDMapping (hd-mapping) repository. It is a C++20 CMake project that builds a single executable, `lidar_odometry_step_1`, which computes LiDAR+IMU odometry from a folder of Mandeye-style recordings. All dependencies are vendored under `3rdparty/` (Eigen, freeglut, glew-cmake, LASzip, oneTBB, Fusion AHRS, imgui/ImGuizmo/implot, manif, nlohmann json, observation_equations, portable-file-dialogs) — no package manager, no network needed. Only OpenGL/GLU must come from the system.

There are no tests, linters, or CI in this repo.

## Build

Requires CMake >= 4.0. On Windows the existing `build/` tree uses the "Visual Studio 18 2026" generator; `out/build/x64-Debug` is from Visual Studio's CMake integration.

```
cmake -S . -B build
cmake --build build --config Release --target lidar_odometry_step_1
```

- Output: `build/bin/<Config>/lidar_odometry_step_1.exe` (multi-config) or `build/bin/lidar_odometry_step_1`. On Windows, runtime DLLs (TBB etc.) and `app/default_settings.toml` are copied next to the exe post-build.
- `-DHD_CPU_OPTIMIZATION=AUTO|INTEL|AMD|ARM|GENERIC` selects release optimization flags (AMD on MSVC adds `/fp:fast`).
- The version (`HDMAPPING_VERSION_MAJOR/MINOR/PATCH`) is defined in the root `CMakeLists.txt`; `shared/include/HDMapping/Version.hpp` errors if it's missing.

## Run

- **Headless / CLI:** `lidar_odometry_step_1 <input_folder> <params.toml> <output_folder>` — exactly 3 args triggers batch mode (step1 → step2 → save_result).
- **GUI:** any other argument count opens the freeglut + ImGui window.

The input folder is scanned non-recursively; `load_data` (`app/lidar_odometry.cpp`) sorts files by extension: `.laz`/`.las` point clouds, `.csv` IMU data, `.json` LiDAR calibration, `.sn` serial-number mapping.

## Architecture

**Pipeline** (driven from `main()` / `step1()` / `step2()` in `app/lidar_odometry_gui.cpp`):
1. `load_data` — read LAZ chunks + IMU CSV, apply calibration, decimate.
2. `calculate_trajectory` — IMU-only orientation via Fusion AHRS (NWU/ENU/NED convention flags in params).
3. `compute_step_1` — cut the data into `WorkerData` windows (points + intermediate trajectory per chunk).
4. `compute_step_2` (`app/lidar_odometry_utils_optimizers.cpp`) — per-window scan-to-map optimization against an NDT-bucket reference map (`optimize_lidar_odometry`, `optimize_sf*`, `optimize_icp`), optional `Consistency`/`Consistency2` passes (`run_consistency`, run by the CLI when `use_consistency = true`).
5. `save_result` (`app/lidar_odometry.cpp`) — write session, trajectories, LAZ, `parameters.toml` and processing-results JSON to the output folder.

**Parameters:** everything tunable lives in `LidarOdometryParams` (`app/lidar_odometry_utils.h`), defaults set by `set_lidar_odometry_default_params` (in `lidar_odometry_gui.cpp`). TOML I/O (`app/toml_io.h`) is table-driven: to make a new field loadable/savable, add it to both `TomlIO::POINTERS` (name → member pointer; only bool/double/int/string supported) and the appropriate `CATEGORIES` section. `app/default_settings.toml` is the shipped config.

**IMU rotation prior and gyro bias** (in `optimize_lidar_odometry` / `compute_step_2`, all on by default):
- `[motion_model_uncertainty]`: `use_time_scaled_rotation_sigma` replaces the constant `*_om/fi/ka_1_sigma_deg` with `max(rotation_sigma_inflation_k * gyro_arw_{x,y,z}_deg_sqrt_s * sqrt(dt), rotation_sigma_floor_deg)` per consecutive-node edge.
- `[gyro_bias_estimation]`: `use_gyro_bias_estimation` adds a 3-axis gyro bias to each window's unknowns, carried to the next window as a prior. The prior sigma starts at `gyro_bias_initial_sigma_deg_s`, grows by `gyro_bias_rw_deg_s_sqrt_s`, and the bias is clamped at `gyro_bias_max_deg_s`. `handoff_tail_nodes` re-propagates each window's last nodes from the IMU before they seed the next window (0 = off). The per-window bias is written to `gyro_bias_per_window` in the results JSON.
- Switching all three off (`false`, `false`, `0`) reproduces the previous behaviour. The IMU CSV timestamps must be strictly increasing.
- `tools/eval_odometry.py` (see `tools/README.md`) computes the trajectory and map-sharpness metrics used to validate changes here.

**`core/` library:** shared HDMapping code (point cloud/session types, NDT, ICP, pose-graph SLAM, GNSS, LAZ export, observation-equation-based optimizers). Built as a static library `core` with `WITH_GUI=1`. `core/src/utils.cpp` is compiled directly into the exe, not into `core`.

**Critical link-order constraint:** `CORE_SOURCE_FILES` in `CMakeLists.txt` is an explicit list that must mirror the original hd-mapping `core/CMakeLists.txt` — do **not** replace it with a glob or compile core sources directly into the exe. The app relies on static-archive semantics: `hash_utils.cpp` defines `get_index()`/`get_rgd_index()` that would collide with the app's own definitions, and `surface.cpp` duplicates globals from `pose_graph_slam.cpp` (so it is intentionally excluded). Adding a new `core/src/*.cpp` that the app needs means adding it to that list and checking for symbol collisions.

GLEW is only used for headers (never initialized or linked). Much of the app code uses file-scope globals in `lidar_odometry_gui.cpp` (params, point data, trajectory) shared between the GUI and CLI paths.
