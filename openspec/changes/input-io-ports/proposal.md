# Proposal

## Why

Roadmap step 4 in `docs/ARCHITECTURE.md` puts the pipeline's file I/O behind ports. The user split it into two changes; this one, **4a**, covers the input side. Today the pipeline functions handle files directly:
- **Recording:** `load_data` sorts and pairs `lidar*.laz`/`imu*.csv` file names, checks the folder, and reads both formats (`load_point_cloud` with LASzip, `load_imu` with CSV parsing).
- **Point cache:** `compute_step_1`, `compute_step_2`, `Consistency`/`Consistency2`, `run_loop_closure` and `save_result` read and write `cache/*.cache` files through paths stored in `WorkerData` (`original_points_cache_file_name`, `intermediate_points_cache_file_name`, `original_points_to_save_cache_file_name`) and `OdometryState::paths`.
- **Preview files:** `load_data` writes `preview/calibrationValidation.asc`, and `compute_step_2` writes `preview/temp_point_cloud_N.laz` every 100 windows.

So none of these stages can run without a disk, a real recording and the folder layout. That blocks the testable core goal, and steps 5 and 6 (domain model, splitting the optimizer).

## What Changes

- **Application-layer ports** (`app/input_ports.h`):
  - `RecordingSource`: the number of LiDAR/IMU chunk pairs, and reading one chunk's IMU samples or filtered points.
  - `PointStore`: save and load a point set by kind (window's original points, window's intermediate points, session chunk) and index, plus clear.
  - `PreviewSink`: the calibration-validation points and the periodic window preview cloud.
- **Infrastructure adapters:**
  - `MandeyeFolderSource`: today's file pairing, LAZ and IMU CSV reading, moved out of `load_data` and `lidar_odometry_utils.cpp`.
  - `DiskPointStore`: today's `cache/` folder and file names.
  - `FolderPreviewSink`: today's `preview/` folder and file formats.
  - A factory that builds all three from the input file list, the way `load_data` derives the folders today.
- **Pipeline:** the functions take the ports instead of building paths. `OdometryPipeline` owns the run's input adapters, and `load()` no longer takes a file list. `WorkerData`'s cache-path fields stay in `core` but the app stops using them (they go away with the domain `Window` in step 5).
- **CLI and GUI:** both build the adapters for the chosen folder. The GUI's cache removal at exit goes through `PointStore::clear()`.
- **Unchanged:**
  - output files: `44_short` bit-identical, `44` identical or within `--tolerance 1e-6`;
  - cache and preview file names and formats;
  - console wording, except the cache-path messages.
- **Docs:** `docs/ARCHITECTURE.md` splits roadmap step 4 into 4a (this change) and 4b (`ResultSink` and `ConfigRepository`), and adds `PreviewSink` to the ports.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- None. This is a pure refactor with no behaviour change, so the change sets `skip_specs: true`.

## Impact

- **New files:** `app/input_ports.h`, `app/recording_files.{h,cpp}`, `app/mandeye_recording_source.{h,cpp}`, `app/disk_point_store.{h,cpp}`, `app/folder_preview_sink.{h,cpp}`, `app/folder_inputs.{h,cpp}`, and tests in `tests/`.
- **Changed files:** `app/lidar_odometry.{h,cpp}`, `app/lidar_odometry_utils.{h,cpp}` (`load_imu` and `load_point_cloud` move out), `app/lidar_odometry_utils_optimizers.cpp`, `app/loop_closure.{h,cpp}`, `app/odometry_pipeline.{h,cpp}`, `app/lidar_odometry_gui.cpp`, `CMakeLists.txt`, `tests/CMakeLists.txt`, `CLAUDE.md`, `docs/ARCHITECTURE.md`.
- **Not touched:**
  - `core/`: `WorkerData` keeps its path fields, which `core/src/session.cpp` still reads;
  - the output folder (`save_result`'s writing, step 4b);
  - `toml_io` (step 4b);
  - the TBB loops in `load_data`, which stay for now: TBB goes behind its own port later.
- **Validation:**
  - `44_short` on every commit, the full `44` at the end;
  - the user checks the GUI: the one-click flow, the advanced flow, and the cache removed at exit.
