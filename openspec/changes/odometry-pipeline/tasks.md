# Tasks

Every code commit: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`.

## 1. Sensitivity of the working-directory test (on today's code)

- [x] 1.1 Create a scratch folder holding a `default_settings.toml` that sets only `[lidar_odometry_control] convergence_delta_threshold = 0.001`; run today's Release CLI on `44_short` with that folder as the working directory; verify that `compare_baseline.py` reports **differences** against the `44_short` manifest (record which files in design.md). If it is identical, choose another key from the six that `lidar-vertical.toml` lacks before continuing

## 2. OdometryPipeline and the CLI

- [x] 2.1 Add `app/odometry_pipeline.{h,cpp}` (design decision 1) with `load`, `run_odometry`, `post_process` (the CLI's loop-closure and consistency `if`s, verbatim), `save` and `run_all`; add it to `SOURCES` in `CMakeLists.txt`; verify both presets build
- [x] 2.2 CLI branch of `main()`: load only `argv[2]` onto a value-initialized config (no `default_settings.toml`), exit 1 with a message when it fails; run a local `OdometryPipeline` (`load` → `run_all` → `save`); delete the CLI `step1`/`step2` wrappers; verify a missing or invalid TOML path exits non-zero with no output folder
- [x] 2.3 Run the step checks; also repeat the 1.1 run (planted `default_settings.toml` as the working directory) and verify it is now IDENTICAL to the manifest; commit

## 3. GUI on the pipeline

- [x] 3.1 Replace the globals `worker_data`, `trajectory`, `imu_data`, `pointsPerFile`, `state` with `OdometryPipeline pipeline{params}` and rename the references (watch `mouse(…, int state, …)`); a new folder gets a fresh pipeline; verify it compiles with the old globals deleted
- [x] 3.2 Simple flow (`openData`): `load`, then on the thread `run_all` → `save`; advanced flow: "Compute all" → `run_all`, remove the consistency button and its tooltip, and have "Save result" pass the elapsed time since "Compute all"
- [x] 3.3 GUI defaults: `set_lidar_odometry_default_params` resolves `<exe folder>/default_settings.toml` itself (`exe_folder()`); the GUI start-up and the "1 Velocity < 8km/h" preset use `<exe folder>/default_settings.toml`; verify by starting the GUI with another working directory and checking the console line that the file next to the exe was loaded
- [x] 3.4 Run the step checks and commit

## 4. Integration

- [x] 4.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json`
- [x] 4.2 GUI check, **done by the user**: start the Release GUI, load `settings/lidar-vertical.toml` (Load parameters), open `44_short`, let the simple flow finish and save; then I verify that the new `lio_result_*` is IDENTICAL with `44_short_baseline` on all 11 files
- [x] 4.3 Update `CLAUDE.md` (Run section: same stages in CLI and GUI, where the settings come from; Architecture: `OdometryPipeline`, remaining UI globals) and mark step 2 done in `docs/ARCHITECTURE.md`; commit
- [x] 4.4 Run `openspec validate odometry-pipeline --strict` and confirm it passes
