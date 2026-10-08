# Architecture goals

Status: **in progress.** This is the target architecture for dMapper (`dMapper.exe`) and the
plan for getting there. Each migration step is its own OpenSpec change; steps 0–6 are done (see
the roadmap).

## Why

The app works and is validated on real scans, but its structure is inherited from HDMapping:

- the pipeline, the GUI and the CLI shared file-scope globals in `src/presentation/gui/lidar_odometry_gui.cpp`
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

- ✅ `LidarOdometryParams` (then in `app/lidar_odometry_utils.h`, split up in step 7a) held settings **and** runtime state
  (`reference_buckets`, `buckets_indoor`/`buckets_outdoor`, `m_g`, `initial_points`, paths,
  `ImVec4 clear_color`). Step 1 split it into `OdometryConfig` (passed `const&`),
  `OdometryState` and `GuiPreferences`. Grouping `OdometryConfig` by section is left to step 5.
- ✅ `WorkerData` (`core/include/structures.h`) carried cache file names and a GUI `show` flag.
  Step 4a moved the points behind the `PointStore` port, and step 5a replaced it in the app with
  the domain `Window` (`src/domain/window.h`). `WorkerData` stays in `core` only for `session.cpp` (since step 11 it is an unused struct in `src/domain/structures.h`).
- ✅ Globals in `lidar_odometry_gui.cpp` (`params`, `worker_data`, `imu_data`, `trajectory`,
  `pointsPerFile`, step flags). Step 2 moved the run data into `OdometryPipeline`. The GUI still
  keeps `params`, `gui_prefs`, its current `pipeline` and UI state as file-scope globals for the
  GLUT callbacks; a GUI view-model is step 10. Since the GUI cleanup (2026-10-02) the GUI
  changes `OdometryConfig` only through `ConfigRepository` (Start → Load parameters). It has no
  settings widgets and no georeferencing or editing tools. Georeferencing is planned as a
  separate post-processing module on result folders.
- ✅ `optimize_lidar_odometry` took about 35 positional arguments → since step 5b it takes a
  `Window` and option structs built from the config (15 parameters).
- ✅ `lidar_odometry_utils_optimizers.cpp` (about 3,000 lines) mixed optimizer math, the window loop,
  state hand-off and I/O → since step 6a one file per service: `scan_to_map`, `robust_lidar_odometry`,
  `consistency`, `imu_kinematics`, `normal_equations` and the window loop in `run_odometry`.
- `core/` mixes pure math (NDT, ICP, pose graph) with I/O (LAZ export, session, file dialogs)
  → ✅ step 11: dMapper took over the few parts it used, and `core/` is gone.

## 5. Folder and CMake layout (built in step 7b)

```
src/domain/          -> lib  lo_domain          (Eigen, manif, observation_equations)
src/application/     -> lib  lo_application     (lo_domain only)
src/infrastructure/  -> lib  lo_infrastructure  (lo_application, LASzip, toml, json, csv-parser, Fusion, TBB)
src/presentation/gui -> lib  lo_gui_view_model  (lo_infrastructure; the view-model only)
src/presentation/cli, src/presentation/gui (in the dMapper exe: ImGui, freeglut, GL, file dialogs)
src/main.cpp         -> exe  dMapper
tests/               -> unit tests, linking lo_infrastructure (and with it the inner layers)
```

Each library's include directories enforce the dependency rule: a domain file that includes LASzip doesn't
compile.
There is no `core` library any more (step 11). The vendored `3rdparty/` holds only what dMapper
builds against.

## 6. Migration roadmap

Each step is one OpenSpec change, validated against the baseline.

0. ✅ **Golden baseline and test harness** (done 2026-10-01): `tools/compare_baseline.py`,
   manifests for `44` and `44_short` in `tools/baseline/`, the `cut_recording` tool, and doctest
   with the `lidar_odometry_tests` target. Two identical runs were confirmed bit-identical on both
   datasets. See `tools/README.md`.
1. ✅ **Split config from state** (done 2026-10-01): `OdometryConfig` (const in the pipeline),
   `OdometryState` (incl. `RunPaths`) and `GuiPreferences`; the TOML file is byte-identical
   (golden test in `tests/`).
2. ✅ **Extract `OdometryPipeline`** (done 2026-10-01): `src/application/odometry_pipeline.{h,cpp}`. CLI and
   GUI both run load → odometry → loop closure → consistency → save. The CLI ignores the working
   directory's `default_settings.toml`, and the GUI loads it from the exe folder.
   - This also fixes a known GUI/CLI gap: today the GUI never runs loop closure, which the CLI
     does. On 44 the CLI result with loop closure is the better one.
   - After this step, the GUI's "Compute all" must run exactly the CLI sequence (step 2 → loop
     closure → consistency) and produce output bit-identical to the baseline.
   - The CLI output must not change.
3. ✅ **Logger and ProgressReporter ports** (done 2026-10-02): `src/domain/run_reporting.h` (ports),
   `ConsoleReporter` (console adapter, whole thread-safe lines) and the GUI's `GuiProgress`. The
   pipeline code has no `std::cout`, progress atomics or sleeps. The GUI progress window shows
   the current stage, and Pause blocks the run at its next checkpoint. Logging stays on the
   console.
4. **I/O ports**, split into two changes:
   - 4a. ✅ **Input side** (done 2026-10-02): `RecordingSource`, `PointStore` and `PreviewSink`
     (`src/application/input_ports.h`) with the adapters `RecordingFolderSource`, `DiskPointStore` and
     `FolderPreviewSink`. The pipeline owns them (`RunInputs`, `make_folder_inputs`). The pairing
     rules and the point store have unit tests.
   - 4b. ✅ **Output side** (done 2026-10-02): `ResultSink` and `ConfigRepository`
     (`src/application/output_ports.h`) with the adapters `FolderResultSink` (every output file name and
     format) and `TomlConfigRepository`. `save_result` keeps only the computation, and the CLI and
     GUI change settings only through the repository.
5. **Domain model**, split into two changes:
   - 5a. ✅ **Domain types** (done 2026-10-03): `Window` (`poses`, `motion_model_poses`,
     `timestamps`, `imu_angles`, `imu_acc`, `gyro_bias`), `GyroBiasState` and `ImuTrajectory` in
     `src/domain/window.h`; the app uses `windows` instead of `worker_data`.
   - 5b. ✅ **Option structs** (done 2026-10-03): `optimize_lidar_odometry` takes the `Window` and
     `ScanToMapOptions`, `MotionModelOptions` and `VelocityPrior` (`src/domain/optimizer_options.h`, built
     from `OdometryConfig`) instead of about 35 positional parameters.
6. **Split the optimizer file** into domain services, split into three changes:
   - 6a. ✅ **One file per service** (done 2026-10-03): pure moves out of the 2,922-line file;
     the first tests on synthetic data cover the IMU kinematics and the normal-equation helpers.
   - 6b. ✅ **Window loop into named steps** (done 2026-10-03): `compute_step_2` became
     `optimize_windows`, a 126-line loop of named steps with `OdometryRunSetup` and `WindowLoopCarry`;
     the steps that only need window data are in `src/domain/window_steps.cpp`, with tests. Found on the way:
     after every window the loop placed all later windows again (about 238,000 placements on 44). Fixed in
     `chain-next-window-only` (2026-10-03): only the next window is placed; new baselines, see
     `docs/window-chaining.html`.
   - 6c. ✅ **Synthetic scan-to-map test** (done 2026-10-04): `tests/scan_to_map_test.cpp` runs
     `optimize_lidar_odometry` on a synthetic room and corridor against the truth. No `core` link was needed: the
     tests compile `scan_to_map.cpp` and `lidar_odometry_utils.cpp` (now `ndt_map.cpp`) directly. Found on the way: `update_rgd` keeps a
     cell's first point as its mean and fixes its normal from the first 3 points, which costs about 1 mm and makes the
     degeneracy information unreliable (`docs/degeneracy-information.html`). Change `ndt-bucket-statistics` (2026-10-04)
     added exact cell statistics behind `use_exact_ndt_bucket_statistics`; it was rejected for now (off), because on 44 the
     degeneracy threshold then marked 439 instead of 95 windows and the map was 5 % less sharp.
7. **Layered CMake targets and folders,** split into two changes:
   - 7a. ✅ **Includes first** (done 2026-10-04, `split-utils-header`): the catch-all `app/lidar_odometry_utils.h`
     (LASzip, Fusion, json, `core`'s NDT and transformations, the jacobians) became `ndt_map.{h,cpp}`,
     `odometry_config.h` and `odometry_state.h`, and every file includes what it uses. Found on the way: `core`'s
     `ndt.h` includes `point_cloud.h` (and with it `GL/freeglut.h`) and `session.h`; the four NDT value structs moved to
     `core/include/ndt_types.h`, with aliases left in class `NDT`. A `/showIncludes` check confirmed that `scan_to_map`,
     `normal_equations`, `imu_kinematics`, `window_steps`, `ndt_map`, `optimizer_options` and `robust_lidar_odometry`
     reach no LASzip, json, GL, `session.h` or Fusion. The window loop still does: Fusion through `input_ports.h` (step 9)
     and `session.h` through `export_laz.h` in `run_odometry.cpp`.
   - 7b. ✅ **Folders and libraries** (done 2026-10-04, `layer-folders`): `app/` became `src/domain`,
     `src/application`, `src/infrastructure`, `src/presentation/{cli,gui}` and `src/main.cpp` (the composition root,
     with `run_cli` / `run_gui`), built as `lo_domain`, `lo_application` and `lo_infrastructure`. Each library gets
     only the include directories of its layer, so a LASzip, json, Fusion or TBB include in the domain doesn't compile
     (checked). The tests link the libraries. Prepared by renaming the app's cell-index functions
     (`ndt_cell_key`/`ndt_cell_index`, no clash with `core`'s `hash_utils`) and moving `points_to_vector` to
     `core/include/point_placement.h`; `default_settings.toml` moved to `settings/`. Remaining exceptions:
     `lo_application` uses Fusion and TBB (step 9), and `core`'s session and point-cloud types still compile in the
     domain, because their GL includes need `WITH_GUI` (a review rule until step 11).

Steps 8–11 were added on 2026-10-03. They cover the goals that steps 0–7 still leave open.

8. ✅ **Tests for loop closure and consistency** (done 2026-10-04, `loop-closure-consistency-tests`):
   `tests/synthetic_recording.{h,cpp}` drives two laps around a block (44 windows, 132,000 points, corners with the
   turn rate ramped over 0.5 s), shared with 6c's scene helpers, and `tests/memory_point_store.h` stands in for the
   `PointStore`. Findings, kept as expected failures (`doctest::should_fail`) with their cause:
   - **Loop closure** halves second-lap drift (575 → 112 mm, 5 loops), but moves a trajectory that is already right
     by 32 mm / 0.09°: each aligned revisit still registers a 1–3 cm "correction", and the pose graph spreads it.
     With points only within 6 m it accepted a false loop (1.5 m). Its scenarios take about 20 s each in Release and
     more than 15 minutes in Debug, so they are skipped in the regular run and run with `--no-skip` (user decision).
   - **Consistency** reduces window seams in both variants, but moves the truth by 6.4 mm (`Consistency`, even
     NDT-fit noise) and 16 mm / 0.46° (`Consistency2`, whose 0.01° rotation smoothness pulls against every change of
     turn rate, so the error sits at the corners).
9. ✅ **Remaining ports** (done 2026-10-04, `remaining-ports`): `OrientationEstimator` (Fusion AHRS)
   and `ParallelExecutor` (TBB) in `src/application/service_ports.h`, with their adapters
   (`FusionOrientationEstimator`, `TbbExecutor`) in infrastructure, passed to the pipeline as
   `RunServices`. IMU samples became the domain type `ImuSample` (float vectors, so the values stay
   bit-identical). `lo_application` now links only `lo_domain`; a Fusion or TBB include there doesn't
   compile. `calculate_trajectory`'s bias, gain and time-step rules are tested with a fake estimator.
   Found on the way: the Windows .NET SDK has its own `Fusion.h` on the system include path, so a
   bare `#include <Fusion.h>` compiles anywhere; `<FusionAhrs.h>` is the reliable check.
10. ✅ **GUI view-model** (done 2026-10-04, `gui-view-model`): `GuiViewModel` holds the GUI's state
    (settings and their source, the console, the progress, the current pipeline, the recording folder,
    the run flag and the worker thread) and its actions with paths instead of dialogs; the worker reports
    through `RunNotifier`, which the view implements with its message boxes. `lidar_odometry_gui.cpp`
    keeps the view and one `view_model` pointer for the GLUT callbacks. `lo_gui_view_model` gets no
    ImGui/GLUT/GL include paths and is unit-tested. Found on the way: `core`'s `session.h` reaches
    `GL/freeglut.h` in the infrastructure, whose auto-link pragma made the tests need `freeglut.lib` once
    they linked the result sink; `lo_infrastructure` now builds with `FREEGLUT_LIB_PRAGMAS=0` (no adapter
    calls GLUT). `core`'s camera globals stay for step 11.
    - 10b. ✅ **Trajectory drawing race** (done 2026-10-04, `trajectory-view-race`; found while proposing step 10). The GUI
      draws every window's poses while the worker thread writes them (odometry and post-processing). The vectors
      aren't resized then, so the worst it shows is half-updated poses, but it is a data race. Fix: draw only the
      windows the odometry reported done, as the point cloud preview does, and the whole trajectory after the run.
      A visible behaviour change, so its own proposal. Done: the view draws a main-thread copy (`TrajectoryCopy`)
      filled inside `GuiProgress::read_finished_windows`, whose lock a stage change also takes, and
      `optimize_windows` reports a window done only after the failure check that could still restore it.
11. ✅ **Retire `core/`** (done 2026-10-05, `retire-core`; planned as "split `core/`"). Measured first:
    the exe linked `core.lib` for three file-dialog functions only, infrastructure used only `exportLaz`
    (LASzip + Eigen, but its header pulled `session.h` and GL in), the GUI used the viewer part of
    `utils.cpp`, and the domain four header-only files. The user chose to retire `core/` rather than
    keep it as a math kernel: `structures.h`, `transformations.h`, `ndt_types.h` and `point_placement.h`
    moved into `src/domain` unchanged, `exportLaz` into `src/infrastructure/laz_export`, the viewer and
    the dialogs into `src/presentation/gui`, `csv.hpp` into `3rdparty/csv-parser`; the rest of `core/`
    (about 38,000 lines) was deleted, with `CORE_SOURCE_FILES` and its link-order rules, `WITH_GUI` and
    the `FREEGLUT_LIB_PRAGMAS` workaround. GLEW, ImGuizmo, implot and glm went too; GLEW was only
    needed for one OpenGL 2.0 constant in the unused Info window, which went with it.

## 7. Rules and conventions

- Domain and application code: no `std::cout`, no file paths, no globals, no `#if WITH_GUI`.
- Config is passed in, never read from a global; state is returned, not mutated through params.
- Numerical behaviour changes and refactors never share a commit.

## 8. Decisions

- **Parallelism is behind a port.** Domain and application code never include TBB; they use the
  `ParallelExecutor` port. The TBB adapter lives in infrastructure, and tests can use a serial,
  deterministic executor. What remains: the domain's parallel loops use `<execution>` from the standard
  library directly (`scan_to_map` and `robust_lidar_odometry`, always parallel since `deterministic-multithread` and
  `builtin-multithread`; there is no setting). They are deterministic: `scan_to_map` sums each trajectory node's point terms
  in point order, with the nodes in parallel, and `robust_lidar_odometry` sums per-point blocks in index
  order, so the results are the same bits with or without threads. `consistency` has its own flags and runs
  serially. Eigen, manif and observation_equations are allowed in the domain.
- **`core/` was wrapped and moved on touch** until step 11, which retired it (2026-10-05): what
  dMapper used lives in the layers, and new code doesn't take anything from HDMapping's `core` again.
- **Test framework: doctest**, vendored under `3rdparty/` (single header, fast to compile).
- **Baseline: `raw data/44`.** The validated scan (loops on every floor) with
  `settings/lidar-vertical.toml` is the golden reference.
- **Short cut for intermediate checks.** `44_short` (the first 30 s of 44, about 2.5 min to
  process) has its own manifest and checks every intermediate commit. The full 44 (about 25 min)
  runs once at the end of each change, because only it exercises loop closure.
- **Refactors must be bit-identical on `44_short`.** A pure refactor step must reproduce the
  `44_short` baseline exactly (trajectories, LAZ, results JSON apart from timing fields). This
  relies on the run being deterministic, which it is, threads included.
- **One floating-point mode, exact refactors are bit-identical** (decided 2026-10-08,
  `fixed-pattern-precise-fp`). MSVC Release builds use `/fp:precise`; `/fp:fast` gained 0-2 % and made
  the long run depend on how the compiler ordered sums, which needed a second "precise" build, a second
  reference and a tolerance rule (decided 2026-10-02 and 2026-10-03, now gone). An exact refactor
  reproduces `44_short` and the full 44 bit for bit against the two manifests. A change that
  deliberately alters numerical behaviour sets new baselines in its own commit, after its effect is
  measured (`tools/compare_baseline.py --tolerance`, `tools/eval_odometry.py`).
- **Domain names in code.** Use cases are `SplitIntoWindows`, `RunOdometry`, `CloseLoops` and so
  on. "Step 1" and "Step 2" remain only as GUI labels and in the user docs.

## 9. Open questions

- None right now.
