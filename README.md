# dMapper — LiDAR + IMU mapping

dMapper started as a standalone build of the LiDAR odometry (step 1) application from [HDMapping](https://github.com/MapsHD/HDMapping), for a handheld LiDAR with a built-in IMU (caves and outdoors). It turns a folder of LiDAR point clouds (`.laz`) and IMU samples (`.csv`) into a motion-compensated point cloud and a trajectory.

**Full documentation:** open [`docs/index.html`](docs/index.html) in a browser. It covers the processing pipeline step by step and every parameter of the sensor settings files in `settings/`.

**How a run works:** [`docs/PIPELINE.md`](docs/PIPELINE.md) explains the windows a run is cut into and how each stage uses them.

**Performance:** [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md) lists the performance fixes, the settings that trade speed, and the open findings.

**Architecture goals:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) describes the target Clean Architecture and the migration roadmap (draft).

## Supported sensors

dMapper works with any LiDAR with an IMU whose recording is a folder of `lidar*.laz` point clouds (GPS time in s) and `imu*.csv` samples (`timestamp` in ns, gyro in rad/s, acceleration in g), in a Z-up frame. Everything that differs between sensors (IMU rate and noise, lever arm, range, window length) is set in the settings file; there is one per supported sensor:

| settings file | sensor | mount | state |
|---|---|---|---|
| `settings/l2-vertical.toml` | Unitree L2 | vertical, handheld | tuned |
| `settings/livox-vertical.toml` | Livox Mid-360S | vertical, handheld | provisional (noise values and lever arm still to be measured) |

Each has a hardware sheet next to it (`l2-hardware-specs.md`, `livox-mid360s-hardware-specs.md`). `settings/lidar-vertical.toml` was renamed to `settings/l2-vertical.toml`, with the same values.

## Build

Requires CMake ≥ 4.0 and a C++20 compiler. All dependencies are vendored in `3rdparty/`; only OpenGL/GLU come from the system.

`CMakePresets.json` defines two configurations, `x64-Release` (optimized, use it for processing) and `x64-Debug` (unoptimized, many times slower, for debugging only). In Visual Studio (Open Folder), pick one in the configuration dropdown; Run / Ctrl+F5 rebuilds changed sources first. From a Developer Command Prompt:

```
cmake --preset x64-Release
cmake --build --preset x64-Release --target dMapper
```

The executable is written to `out/build/x64-Release/bin/dMapper.exe` (or `out/build/x64-Debug/bin/...`).

## Run

```
dMapper <input_folder> <sensor_settings.toml> <output_folder>
```

- `<input_folder>`: one recording. The files `lidar*.laz` and `imu*.csv` are required; other files are ignored. The folder is scanned non-recursively, and **every** `.csv` in it is read as IMU data, so keep other CSV files out of it.
- Any other number of arguments opens the GUI (freeglut + ImGui).
- Output: `scan_lio_*.laz` (the point cloud in chunks), `scan_lio_merged.laz` (all chunks in one file, world coordinates, with `export_merged_laz = true`), `trajectory_lio_*.csv` (one pose per IMU sample: the optimized pose as a 4×3 matrix relative to the chunk; the `om_rad fi_rad ka_rad` columns are the raw AHRS angles of the IMU, not the optimized orientation), `point_sizes_per_chunk.json` (despite its name, the number of points of each window), `session.mjs` with pose files, the parameters used (`dMapper_params_*.toml`) and processing results (`dMapper_results_*.json`, including the estimated gyro bias per window).

The IMU timestamps must be strictly increasing, and LiDAR and IMU timestamps must come from the same clock (they must agree within a few ms).

## What happens to the points (short version)

```
LAZ points ──> load & filter ──> assign to IMU pose ──> window (threshold_nr_poses)
                  │                                          │
IMU samples ──> AHRS orientation ──> initial trajectory ─────┤
                                                             v
                                   per-window optimization against an NDT map
                                   (point-to-distribution + IMU motion prior
                                    + gyro bias + velocity prior)
                                                             │
                     map update <────────────────────────────┤
                                                             v
                                  export: points transformed by their own pose
```

1. **Load:** each point is read, shifted by the LiDAR→IMU lever arm, and dropped if it is too close to or too far from the sensor (horizontal radius `filter_threshold_xy_inner` … `_outer`) or has no timestamp.
2. **IMU trajectory:** a Madgwick-type filter (Fusion AHRS) integrates gyro and accelerometer into one orientation per IMU sample.
3. **Windows:** the IMU poses are cut into windows of `threshold_nr_poses` poses. Every point is attached to the pose just before its timestamp (at most 10 ms), so each point gets its own pose (motion compensation). A voxel-decimated copy (`decimation`) is used for optimization.
4. **Optimization:** window by window, all poses of the window (and the gyro bias) are adjusted so that the points fit the map, a hierarchical NDT grid of Gaussians built from all previous windows, while staying close to what the IMU measured.
5. **Map update and hand-off:** the optimized points are added to the map, the last poses of the window are re-derived from the IMU, and the next window starts where this one ended.
6. **Loop closure** (`use_loop_closure = true`, on in the shipped sensor settings): places visited twice, at least 30 s apart, are recognised, aligned, and the trajectory in between is bent so both visits coincide (doubled rooms after a featureless corridor).
7. **Optional consistency pass** (`use_consistency = true`, on in the shipped sensor settings): the whole trajectory is re-optimized against one map of the entire recording. It makes walls sharper for a little more runtime.
8. **Export:** every original (non-decimated) point is transformed with its own optimized pose and written to LAZ.

## Repository layout

| path | content |
|---|---|
| `app/` | the application: loading, IMU trajectory, windowing, optimizer, export, GUI, TOML I/O |
| `core/` | shared HDMapping library (point clouds, NDT, sessions, LAZ export) |
| `3rdparty/` | vendored dependencies |
| `settings/` | one settings file per supported sensor, with its hardware sheet |
| `tools/eval_odometry.py` | quality metrics (jitter, heading, map sharpness) for result folders |
| `tools/merge_chunks.sh` | merges the `scan_lio_N.laz` chunks of a result into one LAZ in world coordinates (CloudCompare) |
| `docs/` | HTML documentation |
| `openspec/` | specifications and archived changes (design decisions, measurements) |

## Acknowledgements

Janusz Będkowski, the author of [HDMapping](https://github.com/MapsHD/HDMapping), helped a lot with getting this project started. I am thankful for his help.
