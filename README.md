# Mapper — LiDAR + IMU odometry (lidar_odometry_step_1)

A standalone build of the `lidar_odometry_step_1` application from [HDMapping](https://github.com/MapsHD/HDMapping), tuned for a handheld **Unitree L2** recorded with **dLidar** (caves and outdoors). It turns a folder of LiDAR point clouds (`.laz`) and IMU samples (`.csv`) into a motion-compensated point cloud and a trajectory.

**Full documentation:** open [`docs/index.html`](docs/index.html) in a browser. It covers the processing pipeline step by step and every parameter of `settings/lidar-vertical.toml`.

**Architecture goals:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) describes the target Clean Architecture and the migration roadmap (draft).

## Build

Requires CMake ≥ 4.0 and a C++20 compiler. All dependencies are vendored in `3rdparty/`; only OpenGL/GLU come from the system.

`CMakePresets.json` defines two configurations, `x64-Release` (optimized, use it for processing) and `x64-Debug` (unoptimized, many times slower, for debugging only). In Visual Studio (Open Folder), pick one in the configuration dropdown; Run / Ctrl+F5 rebuilds changed sources first. From a Developer Command Prompt:

```
cmake --preset x64-Release
cmake --build --preset x64-Release --target lidar_odometry_step_1
```

The executable is written to `out/build/x64-Release/bin/lidar_odometry_step_1.exe` (or `out/build/x64-Debug/bin/...`).

## Run

```
lidar_odometry_step_1 <input_folder> settings/lidar-vertical.toml <output_folder>
```

- `<input_folder>`: one recording. The files `lidar*.laz` and `imu*.csv` are required; other files are ignored. The folder is scanned non-recursively, and **every** `.csv` in it is read as IMU data, so keep other CSV files out of it.
- Any other number of arguments opens the GUI (freeglut + ImGui).
- Output: `scan_lio_*.laz` (the point cloud in chunks), `scan_lio_merged.laz` (all chunks in one file, world coordinates, with `export_merged_laz = true`), `trajectory_lio_*.csv` (one pose per IMU sample), `session.mjs` with pose files, the parameters used (`HDMapping_params_*.toml`) and processing results (`HDMapping_results_*.json`, including the estimated gyro bias per window).

The IMU timestamps must be strictly increasing. Recordings from dLidar builds before the timestamp fix should be repaired first: `python tools/fix_imu_timestamps.py <input>/imu.csv -o <new_folder>/imu.csv` (copy the LAZ files next to it; never leave two IMU CSVs in one input folder).

## What happens to the points (short version)

```
LAZ points ──> load & filter ──> assign to IMU pose ──> window (360 poses, ~0.4 s)
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
6. **Loop closure** (`use_loop_closure = true`, on in `settings/lidar-vertical.toml`): places visited twice, at least 30 s apart, are recognised, aligned, and the trajectory in between is bent so both visits coincide (doubled rooms after a featureless corridor).
7. **Optional consistency pass** (`use_consistency = true`, on in `settings/lidar-vertical.toml`): the whole trajectory is re-optimized against one map of the entire recording. On the test recordings it gave about 6–7% sharper walls for about 20–30 s more runtime.
8. **Export:** every original (non-decimated) point is transformed with its own optimized pose and written to LAZ.

## Repository layout

| path | content |
|---|---|
| `app/` | the application: loading, IMU trajectory, windowing, optimizer, export, GUI, TOML I/O |
| `core/` | shared HDMapping library (point clouds, NDT, sessions, LAZ export) |
| `3rdparty/` | vendored dependencies |
| `settings/` | `lidar-vertical.toml` (tuned for the L2) and sensor notes |
| `tools/eval_odometry.py` | quality metrics (jitter, heading, map sharpness) for result folders |
| `tools/fix_imu_timestamps.py` | repairs bursty or stepped IMU timestamps of older dLidar recordings |
| `tools/merge_chunks.sh` | merges the `scan_lio_N.laz` chunks of a result into one LAZ in world coordinates (CloudCompare) |
| `docs/` | HTML documentation |
| `openspec/` | specifications and archived changes (design decisions, measurements) |
