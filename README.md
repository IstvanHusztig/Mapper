# dMapper — LiDAR + IMU mapping

dMapper turns a recording of a handheld LiDAR with a built-in IMU (a folder of `.laz` point clouds and `.csv` IMU
samples) into a motion-compensated point cloud and a trajectory. It started as a standalone build of the LiDAR odometry
(step 1) application of [HDMapping](https://github.com/MapsHD/HDMapping) and has since been rebuilt around its own
layers, tests and tools. The target is cave surveys hundreds of metres long, so precision and map quality come before
speed. Version 0.1.0.

**Full documentation:** open [`docs/index.html`](docs/index.html) in a browser: the user guide (the pipeline step by
step, every setting of the sensor files in `settings/`, the analysis tools) with a map of every other page.

Other pages to start from:
- [`docs/PIPELINE.md`](docs/PIPELINE.md): the windows a run is cut into and how each stage uses them.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): the layers (Clean Architecture) and the ports.
- [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md): performance fixes and the settings that trade speed.
- [`docs/regions/`](docs/regions/README.md): the code's eight regions, each with its contract, checks and known flaws.

## Supported sensors

dMapper works with a LiDAR with an IMU whose recording is a folder of `lidar*.laz` point clouds (GPS time in s) and
`imu*.csv` samples (`timestamp` in ns, gyro in rad/s, acceleration in g), in a Z-up frame. Everything that differs between
sensors (IMU rate and noise, lever arm, range, window length) is set in the settings file; there is one per sensor, one
sensor at a time:

| settings file | sensor | mount |
|---|---|---|
| `settings/l2-vertical.toml` | Unitree L2 | vertical, handheld |
| `settings/livox-vertical.toml` | Livox Mid-360S | vertical, handheld |

Each has a hardware sheet next to it (`l2-hardware-specs.md`, `livox-mid360s-hardware-specs.md`). The Livox file was
tuned on two indoor recordings; a few noise values are still marked TODO in it, and its gyro scale (`gyro_scale_z`) is
that of the unit the recordings were made with (1.0 for an unmeasured unit).

## Build

Requires CMake ≥ 4.0 and a C++20 compiler (MSVC with Ninja on Windows). All dependencies are vendored in `3rdparty/`;
only OpenGL/GLU come from the system.

`CMakePresets.json` defines two configurations, `x64-Release` (optimized, use it for processing) and `x64-Debug`
(unoptimized, many times slower, for debugging only). In Visual Studio (Open Folder), pick one in the configuration
dropdown. From a Developer Command Prompt (`vcvars64.bat` loaded):

```
cmake --preset x64-Release
cmake --build --preset x64-Release --target dMapper
```

The executable is written to `out/build/x64-Release/bin/dMapper.exe`.

## Run

```
dMapper <input_folder> <sensor_settings.toml> <output_folder>
```

- `<input_folder>`: one recording, scanned non-recursively. `.laz`/`.las` files are point clouds and `.csv` files are IMU
  data; everything else is ignored. Each point cloud is paired with the CSV whose name ends in the same four characters
  (`lidar0001.laz` with `imu0001.csv`), so keep other CSV files out of the folder.
- The settings are the built-in defaults plus the given file, nothing else. Use the file of your sensor.
- Any other number of arguments opens the GUI (freeglut + ImGui): Start → 1. Load parameters… picks the settings file
  (at start-up it loads `default_settings.toml` next to the exe, which is not a sensor file), Start → 2. Load data and
  process… the recording. The result is the same as the command line's.
- Output: `scan_lio_*.laz` (the point cloud in chunks), `scan_lio_merged.laz` (all chunks in one file, world
  coordinates, with `export_merged_laz = true`), `trajectory_lio_*.csv` (one pose per IMU sample: the optimized pose as a
  4×3 matrix relative to the chunk; the `om_rad fi_rad ka_rad` columns are the raw AHRS angles of the IMU, not the
  optimized orientation), `point_sizes_per_chunk.json` (despite its name, the number of points of each window),
  `session.mjs` with pose files, the parameters used (`dMapper_params_*.toml`) and processing results
  (`dMapper_results_*.json`, including the estimated gyro bias per window).
- For experiments: reprocessing from a saved odometry (`--from-snapshot`), replaying a stretch of windows (`--replay`) or
  only the loop closure (`--replay-loop-closure`); see the guide and `CLAUDE.md`.

The IMU timestamps must be strictly increasing, and LiDAR and IMU timestamps must come from the same clock (they must
agree within a few ms). Stand still for 1–2 s at the start of a recording.

## What happens to the points (short version)

```
LAZ points ──> load & filter ──> assign to IMU pose ──> windows
                  │                                        │
IMU samples ──> AHRS orientation ──> initial trajectory ───┤
                                                           v
                          per-window optimization against an NDT map
                          (point-to-distribution, IMU motion model, accelerometer
                           with velocity and offset, gyro bias)
                                                           │
                    map update <───────────────────────────┤
                                                           v
                    loop closure (pose graph over segments) ──> optional consistency pass
                                                           │
                                                           v
                           export: points transformed by their own pose
```

1. **Load:** each point is read, shifted by the LiDAR→IMU lever arm, and dropped if it is too close to or too far from
   the sensor (horizontal radius `filter_threshold_xy_inner` … `_outer`) or has no timestamp.
2. **IMU trajectory:** an AHRS filter (Fusion) integrates gyro and accelerometer into one orientation per IMU sample.
3. **Windows:** the IMU poses are cut into windows of `threshold_nr_poses` poses (about 0.36–0.4 s with the shipped files). Every point is attached
   to the pose just before its timestamp, so each point gets its own pose (motion compensation). A voxel-decimated copy
   (`decimation`) is used for the optimization.
4. **Optimization:** window by window, all poses of the window are adjusted so that the points fit the map (a
   hierarchical NDT grid built from the previous windows), while staying close to what the IMU measured: the motion
   model between nodes, the accelerometer with the window's start velocity and acceleration offset (`imu_always_on`), and
   the gyro bias. Along a direction the LiDAR cannot see (a featureless corridor) the IMU carries the motion.
5. **Map update and hand-off:** the optimized points are added to the map, and the next window starts where this one
   ended, with its velocity, offset and bias carried over.
6. **Loop closure** (`use_loop_closure`, on in both sensor files): places revisited after at least 8 m of walking are
   registered against each other, and a pose graph over segments of 6 windows bends the trajectory so both visits
   coincide; false matches are dropped. In the Livox file two more rules are on: the odometry steps of a stretch the
   LiDAR could not see are loosened along it (so the correction lands there, not all over the walk), and a match the rest
   of the graph contradicts is dropped (`loop_closure_odometry_from_readings`, `loop_closure_match_check`).
7. **Optional consistency pass** (`use_consistency`, on in the L2 file, off in the Livox file): the whole trajectory is
   re-optimized against one map of the entire recording.
8. **Export:** every original (non-decimated) point is transformed with its own optimized pose and written to LAZ.

## Tests and checks

- Unit tests (doctest, no recorded data needed): `cmake --build --preset x64-Release --target lidar_odometry_tests`, then
  `ctest --test-dir out/build/x64-Release`.
- `python tools/step_checks.py`: build, tests, guards, replays and a short recording against stored references (about 7
  min); `tools/end_runs.py`: the full recordings against their manifests. Both need the recordings on this machine
  (`tools/local_paths.toml`).
- `tools/` holds the analysis tools (map quality, wall pairs, trajectory comparison, the loop closure graph in Python and
  more); see [`tools/README.md`](tools/README.md).

## Repository layout

| path | content |
|---|---|
| `src/domain/` | entities and optimizers: windows, NDT map, scan-to-map terms, IMU kinematics, the window's noise model and solver |
| `src/application/` | the use cases and ports: pipeline, window loop, loop closure, consistency, replays |
| `src/infrastructure/` | adapters: recording folder, LAZ, CSV, TOML, result files, console, Fusion AHRS, TBB |
| `src/presentation/` | command line and GUI |
| `tests/` | unit tests and synthetic recordings |
| `tools/` | Python analysis and verification tools, baselines (`tools/baseline/`) |
| `settings/` | one settings file per supported sensor, with its hardware sheet |
| `3rdparty/` | vendored dependencies |
| `docs/` | the user guide (`docs/index.html`) and the other documentation |
| `openspec/` | specifications and changes, with their design decisions and measurements |

## Sources

Where the methods in dMapper came from. Each change, its origin and the evidence recorded for it are in
[docs/provenance.html](docs/provenance.html).

- [HDMapping](https://github.com/MapsHD/HDMapping) (Janusz Będkowski and co-authors): the base of the odometry (NDT
  scan-to-map, motion model, observation equations, the consistency pass). No HDMapping paper is cited in this repo.
- Zhang, Kaess, Singh, "On Degeneracy of Optimization-based State Estimation Problems", ICRA 2016.
- Hatleskog & Alexis, probabilistic degeneracy detection, RA-L 2025, arXiv:2410.10784.
- X-ICP, Tuna et al., T-RO 2024, arXiv:2211.16335; "Informed, Constrained, Aligned", Tuna et al., arXiv:2408.11809.
- Segal, Haehnel, Thrun, Generalized-ICP, RSS 2009.
- Wildcat, Ramezani et al., arXiv:2205.12595.
- Loop closure survey ([docs/loop-closure-research.html](docs/loop-closure-research.html)): KISS-SLAM arXiv:2503.12660,
  MULLS arXiv:2102.03771, HBA arXiv:2209.11939, LAMP 2.0 arXiv:2205.13135; also LIO-SAM, SC-PGO, hdl_graph_slam, GLIM.
- Odometry map model: Laconte et al., ICRA 2019; Gupta et al., ECMR 2023.
- Further reading from the Codex notes: FAST-LIO2 arXiv:2107.06829, KISS-ICP arXiv:2209.15397, ICP covariance
  arXiv:1410.7632 and arXiv:1909.05722, CT-ICP arXiv:2109.12979.
- The first literature survey in full:
  [openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md](openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md)
  (its citations were not checked one by one).
- Claude (Anthropic): Claude Opus 5.5, Claude Sonnet 5.5 (mostly reviews) and Claude Fable 5.2, as named in the commit
  trailers, and the region agents.
- Codex (OpenAI): design notes and experiments, e.g. the fine-time placement of a blind stretch.
- The author: the recordings and sensors, the static IMU measurement, the building plan and wall marks, every decision to
  ship or reject a change, and the look at the clouds.

## Acknowledgements

Janusz Będkowski, the author of [HDMapping](https://github.com/MapsHD/HDMapping), helped a lot with getting this project
started. I am thankful for his help.
