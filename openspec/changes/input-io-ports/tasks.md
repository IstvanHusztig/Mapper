# Tasks

Every code commit: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`.

## 1. Ports and file pairing

- [x] 1.1 Add `app/input_ports.h` with `PointFilter`, `RecordingSource`, `PointSet`, `PointStore`, `PreviewSink` (design decision 1), and move the `Imu` alias there (`lidar_odometry.h` includes it). Verify both presets build.
- [x] 1.2 Add `app/recording_files.{h,cpp}` with `pair_recording_files`: steps 1–4 of design Context, copied verbatim from `load_data`, with the same messages. Add it to `SOURCES`. `load_data` doesn't use it yet. Verify both presets build.
- [x] 1.3 Add `tests/recording_files_test.cpp` and compile `recording_files.cpp` into the tests. Cover:
  - matching pairs;
  - upper-case `.LAZ`/`.CSV`;
  - other files ignored;
  - a middle mismatch returns false;
  - a last-file mismatch returns true with `min` pairs;
  - fewer than 2 files returns false;
  - the folder is the parent of the first sorted name.

  Verify `ctest` passes. Commit.

## 2. Point store

- [ ] 2.1 Add `app/disk_point_store.{h,cpp}` (`DiskPointStore`, design decision 3): today's file names, folder created on the first save, `save_vector_data`/`load_vector_data`, `clear()` = `remove_all`. Add `tests/disk_point_store_test.cpp`:
  - round trip for each `PointSet`;
  - exact file names;
  - a missing index returns false;
  - `clear()` removes the folder.

  Verify `ctest` passes.
- [ ] 2.2 Switch the cache users to a `PointStore&`: `compute_step_1`, `compute_step_2`, `Consistency`, `Consistency2`, `run_consistency`, `run_loop_closure` and `save_result` (session chunks). They no longer set or read `WorkerData::*_cache_file_name`. Until group 4, `OdometryPipeline` creates a `DiskPointStore` on `state.paths.working_directory_cache` after `load_data`.

  Verify that a grep for `_cache_file_name|save_vector_data|load_vector_data` in `app/lidar_odometry.cpp`, `app/lidar_odometry_utils_optimizers.cpp` and `app/loop_closure.cpp` finds nothing. Run the step checks. Commit.

## 3. Preview sink

- [ ] 3.1 Add `app/folder_preview_sink.{h,cpp}` (`FolderPreviewSink`): `calibration_validation` writes `calibrationValidation.asc` with today's formatting; `window_cloud` writes `temp_point_cloud_<i>.laz` through `exportLaz`; the folder is created on the first write.

  Switch `load_data` (calibration validation) and `compute_step_2` (every 100th window) to a `PreviewSink&`. Until group 4, `OdometryPipeline` creates it on the preview folder.

  Verify that a grep for `working_directory_preview|exportLaz|ofstream` in `app/lidar_odometry_utils_optimizers.cpp` and in `load_data` finds nothing. Run the step checks, then check that `44_short/preview/temp_point_cloud_0.laz` was written. Commit.

## 4. Recording source and pipeline-owned inputs

- [ ] 4.1 Add `app/mandeye_recording_source.{h,cpp}` (`MandeyeFolderSource`): move `load_imu` and `load_point_cloud` there from `lidar_odometry_utils.{h,cpp}` unchanged. `read_imu` and `read_points` call them, and `open` re-checks that the paired files exist. Verify both presets build.
- [ ] 4.2 Add `app/folder_inputs.{h,cpp}` with `RunInputs` and `make_folder_inputs(file names, logger)` (design decision 4): pairing, then the three adapters on `<folder>/cache` and `<folder>/preview`, or empty inputs if pairing fails. Verify both presets build.
- [ ] 4.3 Pipeline owns its inputs:
  - **Pipeline:** `OdometryPipeline(config, logger, progress, RunInputs, RunPaths, debug)`; `load()` takes no arguments and returns false for empty inputs or a failed `open`; it sets `state.paths` from the store's and sink's `location()`.
  - **`load_data`:** reads through `RecordingSource`; no pairing, paths or LASzip.
  - **CLI:** builds the inputs from the `argv[1]` listing.
  - **GUI:** `select_input` builds them for the picked folder; `on_exit` calls `pipeline.clear_point_store()`; the reference-cloud loader includes `mandeye_recording_source.h`.

  Verify:
  - a CLI run on a folder with one file exits without an output folder, as before;
  - a grep for `laszip|ifstream|parent_path|fs::exists` in `load_data` finds nothing.

  Run the step checks. Commit.

## 5. Integration

- [ ] 5.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json`, or `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes.
- [ ] 5.2 GUI check, **done by the user**:
  1. start the Release GUI and load `settings/lidar-vertical.toml`;
  2. run `44_short` with the one-click flow, and say which `lio_result_*` it saved to;
  3. in the advanced flow, use "Load data" and "Compute all", then save;
  4. close the GUI and check that `44_short/cache` is gone.

  Then I verify that both results are IDENTICAL with the `44_short` manifest.
- [ ] 5.3 Docs:
  - `CLAUDE.md` Architecture: the input ports and adapters, `OdometryPipeline` owning `RunInputs`, and where `load_imu`/`load_point_cloud` live now. Testing: the new test sources.
  - `docs/ARCHITECTURE.md`: split roadmap step 4 into 4a (done) and 4b (`ResultSink`, `ConfigRepository`); add `PreviewSink` to §3 Ports; in §4, the `WorkerData` cache fields are unused (removed in step 5).

  Commit.
- [ ] 5.4 Run `openspec validate input-io-ports --strict` and confirm it passes.
