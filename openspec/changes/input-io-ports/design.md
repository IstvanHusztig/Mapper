# Design

## Context

See proposal.md for the motivation. Current state of the input-side I/O:

- **`load_data`** (`app/lidar_odometry.cpp`):
  1. Sorts the input file names and splits them by extension (`.csv`, and `.laz`/`.las`, case-insensitive).
  2. Checks that every LAZ has a CSV with the same last four characters of the stem, and the other way round. A mismatch in the middle stops the run; a mismatch on the last file only prints a warning.
  3. Requires at least 2 input files.
  4. Takes the working folder as the parent of the first sorted file, and checks that it exists.
  5. Reads the IMU of each pair (`load_imu`, TBB `blocked_range`), merges the samples and sorts them by timestamp.
  6. Sets `state.paths.working_directory_preview`/`_cache` to `<folder>/preview` and `<folder>/cache`.
  7. Reads each pair's LAZ (`load_point_cloud`, TBB `parallel_for`) and sorts its points by timestamp.
  8. With `save_calibration_validation`, writes the first `calibration_validation_points` points of chunk 0 to `preview/calibrationValidation.asc` (tab-separated x y z intensity lidarid).

  Pairing uses only the sorted order: pair `i` is `csv_files[i]` + `laz_files[i]`, and `min(#laz, #csv)` pairs are read.
- **The point cache:** `save_vector_data`/`load_vector_data` (`core/include/structures.h`, binary `Point3DiDisk`). Readers and writers:
  - `compute_step_1` creates `cache/` and writes `original_points_<n>.cache` and `intermediate_points_<n>.cache`, where `n` is the window index. The intermediate set is the decimated points, or the original points when decimation gives none.
  - `compute_step_2`, `Consistency`, `Consistency2` and `run_loop_closure` read `intermediate_points_<i>`.
  - `save_result` reads `original_points_<i>`, writes `session_chunk_<k>.cache` per 51-window chunk (plus the remainder), and reads them back to write the LAZ files.
  - All of them reach the files through `WorkerData::*_cache_file_name` (`std::filesystem::path`, set by `compute_step_1` and `save_result`).
  - Every access is sequential (no parallel loop around a load or save).
- **Preview:** `compute_step_2` creates `preview/` and writes `temp_point_cloud_<i>.laz` (`exportLaz`) of the window's intermediate points in world coordinates when `i % 100 == 0`.
- **Composition:** the CLI lists `argv[1]` (every non-directory entry), the GUI lists the picked folder (regular files), and both pass the list to `OdometryPipeline::load`. The GUI's `on_exit` removes `pipeline.state.paths.working_directory_cache`.
- **Other users of the readers:** the GUI loads reference clouds with `load_point_cloud` (`load_reference_point_clouds`).
- **`core/src/session.cpp`** still reads `WorkerData::intermediate_points_cache_file_name`. The app doesn't call that function, but `core` must keep compiling.

## Goals / Non-Goals

**Goals:**
- `load_data`, `compute_step_1`, `compute_step_2`, `Consistency`, `Consistency2`, `run_loop_closure` and the cache part of `save_result` contain no file names, folder logic, LASzip calls or CSV parsing.
- The same file names, formats and folders as today; result bit-identical on `44_short`, and within tolerance on `44`.
- Unit tests for the pairing rules and the point store, without recorded data.

**Non-Goals:**
- The output folder (`save_result`'s files, the parameters TOML): step 4b.
- Removing `WorkerData`'s path fields (step 5, `Window`), and moving `Imu`/`Point3Di` into a domain model.
- TBB behind a `ParallelExecutor` port: the two loops in `load_data` stay as they are.
- New recording formats or pairing rules.

## Decisions

1. **Ports** in `app/input_ports.h` (application layer; depends on `Point3Di`, Eigen and the `Imu` sample type only):
   ```cpp
   struct PointFilter { bool omit_zero_timestamp; double xy_inner; double xy_outer; Eigen::Vector3d lever_arm; };

   class RecordingSource {
   public:
       virtual ~RecordingSource() = default;
       // checks the recording and prints why it can't be processed; must be called before the rest
       virtual bool open(Logger &logger) = 0;
       virtual std::size_t chunk_count() const = 0;                       // complete LiDAR + IMU pairs
       virtual Imu read_imu(std::size_t chunk, Logger &logger) = 0;       // may be called from several threads
       virtual std::vector<Point3Di> read_points(std::size_t chunk, const PointFilter &filter, Logger &logger) = 0; // idem
   };

   enum class PointSet { WindowOriginal, WindowIntermediate, SessionChunk };
   class PointStore {
   public:
       virtual ~PointStore() = default;
       virtual bool save(PointSet set, std::size_t index, const std::vector<Point3Di> &points) = 0;
       virtual bool load(PointSet set, std::size_t index, std::vector<Point3Di> &points) = 0;
       virtual void clear() = 0;                    // removes everything stored (GUI at exit)
       virtual std::string location() const = 0;    // recorded in the outputs as working_directory_cache
   };

   class PreviewSink {
   public:
       virtual ~PreviewSink() = default;
       virtual void calibration_validation(const std::vector<Point3Di> &points, std::size_t max_points, Logger &logger) = 0;
       virtual void window_cloud(std::size_t window, const std::vector<Eigen::Vector3d> &points,
                                 const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
       virtual std::string location() const = 0;    // recorded as working_directory_preview
   };
   ```
   The `Imu` alias moves from `lidar_odometry.h` to `input_ports.h` (`lidar_odometry.h` includes it).
   - *`location()`:* the two folders are still written to the results JSON and the `[paths]` TOML section, so the pipeline sets `state.paths` from the ports. Hiding them would change the outputs.
   - *Store by kind and index instead of by path:* this lets an in-memory store replace the disk in tests later, and removes path building from four stages.

2. **Pairing as a pure function:** `app/recording_files.{h,cpp}`:
   ```cpp
   struct RecordingFiles { std::vector<std::string> laz, csv; std::string folder; };
   bool pair_recording_files(std::vector<std::string> names, RecordingFiles &out, Logger &logger);
   ```
   - It contains steps 1–4 of today's `load_data` verbatim (sorting, extension split, 4-character pairing check, the warnings, the minimum of 2 files, folder = parent of the first sorted name, existence check).
   - It has no LASzip or Fusion dependency, so the tests can compile it.
   - `MandeyeFolderSource::open` calls it.

3. **Adapters** (infrastructure, in `app/` until step 7 creates the folders):
   - `MandeyeFolderSource(file names)`: `read_imu`/`read_points` are today's `load_imu` and `load_point_cloud`, moved from `lidar_odometry_utils.cpp`. The free function `load_point_cloud` stays available to the GUI's reference-cloud loader through `mandeye_recording_source.h`.
   - `DiskPointStore(cache folder)`:
     - creates the folder on the first `save`;
     - maps `(set, index)` to `original_points_<i>.cache`, `intermediate_points_<i>.cache` or `session_chunk_<i>.cache`. (Found during apply: `compute_step_1` used to number window files from 1, `worker_data.size()` after adding the window. The store uses the window index, so they now start at 0. Cache files are internal, and every run rewrites what it reads.)
     - uses `save_vector_data`/`load_vector_data` unchanged;
     - `clear()` is today's `remove_all`.
   - `FolderPreviewSink(preview folder)`:
     - creates the folder on the first write;
     - writes the `.asc` and `temp_point_cloud_<i>.laz` exactly as today (the same stream formatting and `exportLaz`).
   - `make_folder_inputs(file names)` (`app/folder_inputs.{h,cpp}`) returns `RunInputs { unique_ptr<RecordingSource>, unique_ptr<PointStore>, unique_ptr<PreviewSink> }`. The cache and preview folders sit under the recording's folder. That folder is known only once the files are paired, so the store and sink get it when the pipeline opens the source (see 4).

4. **The pipeline owns its inputs:**
   - Constructor: `OdometryPipeline(config, logger, progress, RunInputs inputs, RunPaths paths, debug)`, and `load()` takes no arguments.
   - `make_folder_inputs` pairs the files first (`pair_recording_files`). It returns the three adapters already pointed at `<folder>/cache` and `<folder>/preview`, or empty inputs (after the pairing messages) if pairing fails. `RecordingSource::open` then only re-checks that the paired files exist.
   - `OdometryPipeline::load` returns false for empty inputs or a failed `open`, which covers the cases where `load_data` returned false. On success it sets `state.paths.working_directory_cache`/`_preview` from `PointStore::location()`/`PreviewSink::location()` before reading the data. These are the same strings `load_data` set, and no step between the old and new place reads them.
   - The GUI's initial global pipeline has empty inputs and is never run.
   - *Alternative:* the caller owns the adapters and passes references. Rejected: the GUI would need three more globals whose lifetime must match the global `pipeline`.
   - *Message order:* the pairing messages were printed after "Loading data..". They are now printed when the inputs are built, just before the pipeline's Loading stage. Console only.

5. **Function signatures:**
   - `load_data(RecordingSource&, PreviewSink&, params, points_per_file, imu, debug, logger)`: steps 5, 7 and 8 of Context (reading, merging, sorting, calibration validation). It no longer pairs files or sets `state.paths`.
   - `compute_step_1(..., PointStore&, ...)`
   - `compute_step_2(..., PointStore&, PreviewSink&, ...)`
   - `Consistency`/`Consistency2`/`run_consistency(..., PointStore&)`
   - `run_loop_closure(..., PointStore&)`
   - `save_result(..., PointStore&)`

   `WorkerData`'s path fields are no longer set or read by the app.

6. **Messages:** the cache-path lines (`original_points_to_save_cache_file_name <path>`) and the `problem with save/load_vector_data '<path>'` lines become `problem with point store: <set> <index>` without a path. Folder-creation messages move into the adapters as plain console lines through their logger. Console only.

7. **Tests** (doctest, no recorded data):
   - `pair_recording_files`:
     - matching pairs;
     - upper-case extensions;
     - a mismatch in the middle (false);
     - a mismatch on the last file (true, `min` pairs);
     - fewer than 2 files;
     - other files ignored.
   - `DiskPointStore`:
     - round trip for each `PointSet`, checked on a few `Point3Di` fields;
     - the exact file names;
     - `load` of a missing index returns false;
     - `clear()` removes the folder.

   Both use a temp folder. The tests compile `recording_files.cpp` and `disk_point_store.cpp` directly.

## Risks / Trade-offs

- **[Folder derivation changes]:** the cache/preview folder and `state.paths` must stay byte-identical, because they are written to the outputs (normalized by `<IN>` only when they match the input prefix).
  → `pair_recording_files` keeps `fs::path(sorted.front()).parent_path().string()` verbatim. The `44_short` check covers it.
- **[Pairing behaviour drifts while moving]:** → Move the code verbatim and cover the branches with the unit tests before switching `load_data` to it.
- **[`/fp:fast` noise on 44]:** expected. The full run is judged with `--tolerance 1e-6` against `raw data/44_baseline` (decided in step 3). `44_short` must stay identical.
- **[GUI lifetime]:** `pipeline = OdometryPipeline(...)` now moves owned adapters. `on_exit` calls `pipeline.clear_point_store()`, which does nothing with empty inputs.
- **[Stale cache from older runs]:** every run rewrites what it reads. No change.
- **[GUI exit removed a folder from the settings]** (found during apply): `on_exit` used to `remove_all(state.paths.working_directory_cache)` even when no data was loaded. That path can come from the `[paths]` section of a loaded TOML.
  → Now `clear_point_store()` removes only the current run's cache, and does nothing before a folder is loaded.
- **[Existing crash on a failed load]** (found during apply, not changed): the CLI calls `run_all` and `save` even when `load` fails. A folder with fewer than 2 files crashed (segmentation fault) before this change and still does, with the same messages in a different order and no output folder. This is out of scope for a pure refactor; reported to the user.

## Migration Plan

Small commits, each built in both presets with `ctest`, and each checked on `44_short`:
1. ports and pairing, with tests;
2. `DiskPointStore` and the switch of the cache users;
3. `FolderPreviewSink`;
4. `MandeyeFolderSource` and `make_folder_inputs`, and the pipeline owning its inputs (CLI and GUI);
5. the full 44 run, the user's GUI check, and the docs.

Rollback: revert. No format changes.
