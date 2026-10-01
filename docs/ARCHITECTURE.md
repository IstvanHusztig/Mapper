# Architecture goals

Status: **draft, under refinement.** This is the target architecture for `lidar_odometry_step_1`
and the plan for getting there. Nothing here is implemented yet; each migration step becomes its
own OpenSpec change.

## Why

The app works and is validated on real scans, but its structure is inherited from HDMapping:

- the pipeline, the GUI and the CLI share file-scope globals in `app/lidar_odometry_gui.cpp`;
- `LidarOdometryParams` mixes settings with runtime state;
- the optimizers print to the console and read/write cache files;
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
- **Results don't change by accident:** each refactor step must reproduce the golden baseline on
  `raw data/44` bit for bit (see Decisions).
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

- `RecordingSource`: lists and reads LiDAR chunks and IMU samples from an input folder.
- `PointStore`: keeps and fetches a window's points (in memory or in a disk cache); replaces the
  `*_cache_file_name` fields in `WorkerData`.
- `ConfigRepository`: loads and saves `OdometryConfig` (TOML today).
- `ResultSink`: session, trajectories, LAZ, `parameters.toml`, processing-results JSON.
- `ProgressReporter` / `Logger`: replace `std::cout` (about 100 calls in the pipeline) and the
  `std::atomic<float>` progress and pause flags.
- `OrientationEstimator`: AHRS (Fusion) behind an interface.
- `ParallelExecutor`: hides TBB (see Decisions).

## 4. Problems in today's code this addresses

- `LidarOdometryParams` (`app/lidar_odometry_utils.h`) holds settings **and** runtime state
  (`reference_buckets`, `buckets_indoor`/`buckets_outdoor`, `m_g`, `initial_points`, paths,
  `ImVec4 clear_color`) → split into an immutable `OdometryConfig` (grouped by section) plus
  pipeline state.
- `WorkerData` (`core/include/structures.h`) carries cache file names and a GUI `show` flag
  → a domain `Window` plus the `PointStore` port; view flags move to the GUI.
- Globals in `lidar_odometry_gui.cpp` (`params`, `worker_data`, `imu_data`, `trajectory`,
  `pointsPerFile`, step flags) → owned by `OdometryPipeline` or a GUI view-model.
- `optimize_lidar_odometry` takes about 40 positional arguments → option structs built from the
  config.
- `lidar_odometry_utils_optimizers.cpp` (about 3,000 lines) mixes optimizer math, the window loop,
  state hand-off and I/O → split into one domain service each.
- `core/` mixes pure math (NDT, ICP, pose graph) with I/O (LAZ export, session, file dialogs)
  → the pure parts feed the domain, the rest becomes infrastructure.

## 5. Target folder and CMake layout

```
src/domain/          -> lib  lo_domain          (Eigen, manif, observation_equations)
src/application/     -> lib  lo_application     (lo_domain)
src/infrastructure/  -> lib  lo_infrastructure  (lo_application, LASzip, TBB, Fusion, toml, json)
src/presentation/cli, src/presentation/gui
src/main.cpp         -> exe  lidar_odometry_step_1
tests/               -> unit tests on lo_domain / lo_application
```

CMake target links enforce the dependency rule: a domain file that includes LASzip won't link.
The `core` static library and its `CORE_SOURCE_FILES` link-order constraint stay; pieces move out
of it only when a roadmap step touches them. The vendored `3rdparty/` stays as it is (plus
doctest).

## 6. Migration roadmap

Each step is one OpenSpec change, validated against the baseline.

0. **Golden baseline and test harness:** a stored baseline run of `raw data/44`, a compare script
   that checks it bit for bit (ignoring timing fields; first confirm that two identical runs
   really match), and doctest vendored with a `tests` target.
1. **Split config from state:** `OdometryConfig` vs runtime state; the TOML file format is kept.
2. **Extract `OdometryPipeline`** out of `lidar_odometry_gui.cpp`; CLI and GUI both call it; the
   globals go away.
3. **Logger and ProgressReporter ports:** remove `std::cout` and the atomics from the pipeline.
4. **I/O ports:** `RecordingSource`, `PointStore`, `ResultSink`, `ConfigRepository`, with adapters.
5. **Domain model:** `Window`, `Trajectory`, `GyroBiasState`; option structs for the optimizer.
6. **Split the optimizer file** into domain services; first unit tests on synthetic data.
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
- **Refactors must be bit-identical.** A pure refactor step must reproduce the baseline output
  exactly (trajectories, LAZ, results JSON apart from timing fields). This relies on
  `useMultithread = false` keeping the run deterministic. Only a change that deliberately alters
  numerical behaviour may use a tolerance, and it then sets a new baseline in its own commit.
- **Domain names in code.** Use cases are `SplitIntoWindows`, `RunOdometry`, `CloseLoops` and so
  on. "Step 1" and "Step 2" remain only as GUI labels and in the user docs.

## 9. Open questions

- None right now.
