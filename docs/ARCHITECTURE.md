# Architecture goals

Status: **in progress.** This is the target architecture for dMapper (`dMapper.exe`) and the
plan for getting there. Each migration step is its own OpenSpec change; steps 0–5 and 6a are done (see
the roadmap).

## Why

The app works and is validated on real scans, but its structure is inherited from HDMapping:

- the pipeline, the GUI and the CLI shared file-scope globals in `app/lidar_odometry_gui.cpp`
  (fixed in step 2);
- `LidarOdometryParams` mixed settings with runtime state (fixed in step 1);
- the optimizers printed to the console (fixed in step 3) and read/write cache files;
- nothing can be tested in isolation.

We follow **Clean Architecture** (Robert C. Martin), close in spirit to Fowler's layering and
Cockburn's ports & adapters: a pure domain at the centre, application use cases around it,
infrastructure and UI at the edge, and all source dependencies pointing inward.

## 1. Goals

- **The dependency rule:** source dependencies point inward only. The domain knows nothing about
  files, LAZ, TOML, ImGui, GLUT, console output or threads.
- **Testable core:** every algorithm (motion model, scan-to-map, gyro bias, loop closure,
  consistency) can run in a unit test on synthetic data, without disk or GUI.
- **One pipeline, two front ends:** CLI and GUI call the same use case; no globals are shared
  between them.
- **Results don't change by accident:** each refactor step must reproduce the golden baseline:
  `44_short` bit for bit, the full `raw data/44` within 1e-6 (see Decisions).
- **Incremental, never a rewrite:** strangler-style; the app builds and runs after every step.

## 2. Layers (inside → outside)

| Layer | Contains | May depend on |
|---|---|---|
| **Domain** | Entities and value objects: `Pose`, `Trajectory`, `ImuSample`, `PointCloud`/`Point3Di`, `Window` (was `WorkerData`), `GyroBiasState`, `NdtMap`. Domain services: motion model, scan-to-map optimizer (`optimize_lidar_odometry`), gyro-bias propagation, loop-closure detection and pose graph, consistency. | Eigen, manif, observation_equations (pure math), nothing else |
| **Application** | Use cases: `LoadRecording`, `EstimateOrientation` (AHRS), `SplitIntoWindows` (step 1), `RunOdometry` (step 2), `CloseLoops`, `RefineConsistency`, `ExportResults`; the `OdometryPipeline` that orchestrates them; the **ports** (interfaces) the use cases need. | Domain |
| **Infrastructure (adapters)** | LAZ reader/writer (LASzip), IMU CSV reader, TOML config repository, on-disk point cache, session/JSON/trajectory writers, Fusion AHRS adapter, TBB executor, console logger. | Application ports, Domain, 3rd-party libraries |
| **Presentation** | CLI (3-argument batch mode), GUI (freeglut + ImGui + ImGuizmo viewer). | Application |
| **Composition root** | `main()`: wires the adapters into the pipeline and picks CLI or GUI. | Everything |

## 3. Ports (first cut)

- `RecordingSource`: lists and reads LiDAR chunks and IMU samples from an input folder. ✅ step 4a
- `PointStore`: keeps and fetches a window's points (in memory or in a disk cache); replaces the
  `*_cache_file_name` fields in `WorkerData`. ✅ step 4a
- `PreviewSink`: preview output during a run (`calibrationValidation.asc`,
  `temp_point_cloud_<i>.laz`). ✅ step 4a
- `ConfigRepository`: loads and saves `OdometryConfig` (TOML today). ✅ step 4b
- `ResultSink`: session, trajectories, LAZ, `parameters.toml`, processing-results JSON. ✅ step 4b
- `ProgressReporter` / `Logger`: replace `std::cout` (about 100 calls in the pipeline) and the
  `std::atomic<float>` progress and pause flags.
- `OrientationEstimator`: AHRS (Fusion) behind an interface.
- `ParallelExecutor`: hides TBB (see Decisions).

## 4. Problems in today's code this addresses

- ✅ `LidarOdometryParams` (`app/lidar_odometry_utils.h`) held settings **and** runtime state
  (`reference_buckets`, `buckets_indoor`/`buckets_outdoor`, `m_g`, `initial_points`, paths,
  `ImVec4 clear_color`). Step 1 split it into `OdometryConfig` (passed `const&`),
  `OdometryState` and `GuiPreferences`. Grouping `OdometryConfig` by section is left to step 5.
- ✅ `WorkerData` (`core/include/structures.h`) carried cache file names and a GUI `show` flag.
  Step 4a moved the points behind the `PointStore` port, and step 5a replaced it in the app with
  the domain `Window` (`app/window.h`). `WorkerData` stays in `core` only for `session.cpp`.
- ✅ Globals in `lidar_odometry_gui.cpp` (`params`, `worker_data`, `imu_data`, `trajectory`,
  `pointsPerFile`, step flags). Step 2 moved the run data into `OdometryPipeline`. The GUI still
  keeps `params`, `gui_prefs`, its current `pipeline` and UI state as file-scope globals for the
  GLUT callbacks; a GUI view-model is left for later. Since the GUI cleanup (2026-10-02) the GUI
  changes `OdometryConfig` only through `ConfigRepository` (Start → Load parameters). It has no
  settings widgets and no georeferencing or editing tools. Georeferencing is planned as a
  separate post-processing module on result folders.
- ✅ `optimize_lidar_odometry` took about 35 positional arguments → since step 5b it takes a
  `Window` and option structs built from the config (15 parameters).
- ✅ `lidar_odometry_utils_optimizers.cpp` (about 3,000 lines) mixed optimizer math, the window loop,
  state hand-off and I/O → since step 6a one file per service: `scan_to_map`, `robust_lidar_odometry`,
  `consistency`, `imu_kinematics`, `normal_equations` and the window loop in `run_odometry`.
- `core/` mixes pure math (NDT, ICP, pose graph) with I/O (LAZ export, session, file dialogs)
  → the pure parts feed the domain, the rest becomes infrastructure.

## 5. Target folder and CMake layout

```
src/domain/          -> lib  lo_domain          (Eigen, manif, observation_equations)
src/application/     -> lib  lo_application     (lo_domain)
src/infrastructure/  -> lib  lo_infrastructure  (lo_application, LASzip, TBB, Fusion, toml, json)
src/presentation/cli, src/presentation/gui
src/main.cpp         -> exe  dMapper
tests/               -> unit tests on lo_domain / lo_application
```

CMake target links enforce the dependency rule: a domain file that includes LASzip won't link.
The `core` static library and its `CORE_SOURCE_FILES` link-order constraint stay; pieces move out
of it only when a roadmap step touches them. The vendored `3rdparty/` stays as it is (plus
doctest).

## 6. Migration roadmap

Each step is one OpenSpec change, validated against the baseline.

0. ✅ **Golden baseline and test harness** (done 2026-10-01): `tools/compare_baseline.py`,
   manifests for `44` and `44_short` in `tools/baseline/`, the `cut_recording` tool, and doctest
   with the `lidar_odometry_tests` target. Two identical runs were confirmed bit-identical on both
   datasets. See `tools/README.md`.
1. ✅ **Split config from state** (done 2026-10-01): `OdometryConfig` (const in the pipeline),
   `OdometryState` (incl. `RunPaths`) and `GuiPreferences`; the TOML file is byte-identical
   (golden test in `tests/`).
2. ✅ **Extract `OdometryPipeline`** (done 2026-10-01): `app/odometry_pipeline.{h,cpp}`. CLI and
   GUI both run load → odometry → loop closure → consistency → save. The CLI ignores the working
   directory's `default_settings.toml`, and the GUI loads it from the exe folder.
   - This also fixes a known GUI/CLI gap: today the GUI never runs loop closure, which the CLI
     does. On 44 the CLI result with loop closure is the better one.
   - After this step, the GUI's "Compute all" must run exactly the CLI sequence (step 2 → loop
     closure → consistency) and produce output bit-identical to the baseline.
   - The CLI output must not change.
3. ✅ **Logger and ProgressReporter ports** (done 2026-10-02): `app/run_reporting.h` (ports),
   `ConsoleReporter` (console adapter, whole thread-safe lines) and the GUI's `GuiProgress`. The
   pipeline code has no `std::cout`, progress atomics or sleeps. The GUI progress window shows
   the current stage, and Pause blocks the run at its next checkpoint. Logging stays on the
   console.
4. **I/O ports**, split into two changes:
   - 4a. ✅ **Input side** (done 2026-10-02): `RecordingSource`, `PointStore` and `PreviewSink`
     (`app/input_ports.h`) with the adapters `RecordingFolderSource`, `DiskPointStore` and
     `FolderPreviewSink`. The pipeline owns them (`RunInputs`, `make_folder_inputs`). The pairing
     rules and the point store have unit tests.
   - 4b. ✅ **Output side** (done 2026-10-02): `ResultSink` and `ConfigRepository`
     (`app/output_ports.h`) with the adapters `FolderResultSink` (every output file name and
     format) and `TomlConfigRepository`. `save_result` keeps only the computation, and the CLI and
     GUI change settings only through the repository.
5. **Domain model**, split into two changes:
   - 5a. ✅ **Domain types** (done 2026-10-03): `Window` (`poses`, `motion_model_poses`,
     `timestamps`, `imu_angles`, `imu_acc`, `gyro_bias`), `GyroBiasState` and `ImuTrajectory` in
     `app/window.h`; the app uses `windows` instead of `worker_data`.
   - 5b. ✅ **Option structs** (done 2026-10-03): `optimize_lidar_odometry` takes the `Window` and
     `ScanToMapOptions`, `MotionModelOptions` and `VelocityPrior` (`app/optimizer_options.h`, built
     from `OdometryConfig`) instead of about 35 positional parameters.
6. **Split the optimizer file** into domain services, split into three changes:
   - 6a. ✅ **One file per service** (done 2026-10-03): pure moves out of the 2,922-line file;
     the first tests on synthetic data cover the IMU kinematics and the normal-equation helpers.
   - 6b. **Window loop into named steps:** `compute_step_2` broken into functions with a
     carried-state struct (velocity prior, gyro bias, acceleration offset, degeneracy gating, hand-off).
   - 6c. **Synthetic scan-to-map test** of `optimize_lidar_odometry` (the tests then link `core`).
7. **Layered CMake targets and folders;** move the `core/` pieces touched so far.

## 7. Rules and conventions

- Domain and application code: no `std::cout`, no file paths, no globals, no `#if WITH_GUI`.
- Config is passed in, never read from a global; state is returned, not mutated through params.
- Numerical behaviour changes and refactors never share a commit.

## 8. Decisions

- **Parallelism is behind a port.** Domain and application code never include TBB; they use the
  `ParallelExecutor` port. The TBB adapter lives in infrastructure, and tests can use a serial,
  deterministic executor. Eigen, manif and observation_equations are allowed in the domain.
- **`core/` is wrapped and moved on touch.** For now `core/` is treated as an external "math
  kernel" dependency. A piece of it moves into the layers only when a roadmap step touches it.
  There is no separate step for sorting all of `core/`.
- **Test framework: doctest**, vendored under `3rdparty/` (single header, fast to compile).
- **Baseline: `raw data/44`.** The validated scan (loops on every floor) with
  `settings/lidar-vertical.toml` is the golden reference.
- **Short cut for intermediate checks.** `44_short` (the first 30 s of 44, about 3–4 min to
  process) has its own manifest and checks every intermediate commit. The full 44 (about 30 min)
  runs once at the end of each change, because only it exercises loop closure.
- **Refactors must be bit-identical on `44_short`.** A pure refactor step must reproduce the
  `44_short` baseline exactly (trajectories, LAZ, results JSON apart from timing fields). This
  relies on `useMultithread = false` keeping the run deterministic.
- **The full 44 may differ within 1e-6.** The Release build on this machine uses `/fp:fast` with
  whole-program optimization, so a change to unrelated code can change floating-point evaluation
  order and the last bits of the 30-minute run (seen in step 3: at most 1e-9 m). A refactor
  passes on 44 if every text output is within `--tolerance 1e-6` of the baseline folder (decided
  2026-10-02). A change that deliberately alters numerical behaviour sets a new baseline in its
  own commit.
- **Domain names in code.** Use cases are `SplitIntoWindows`, `RunOdometry`, `CloseLoops` and so
  on. "Step 1" and "Step 2" remain only as GUI labels and in the user docs.

## 9. Open questions

- None right now.
