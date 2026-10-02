# Design

## Context

See proposal.md for the motivation. Current state of the output side:

- **`save_result`** (`app/lidar_odometry.cpp`) does, in this order:
  1. `create_directory(outwd)`.
  2. Chunking: each window's original points come from `PointStore` (step 4a). They are concatenated into session chunks of 51 windows, plus the remainder if it has more than `min_counter_concatenated_trajectory_nodes` windows, and stored back as `SessionChunk`. Window point counts go into `point_sizes_per_chunk`.
  3. `point_sizes_per_chunk.json` (`dump(2)`).
  4. With `export_merged_laz`: open `scan_lio_merged.laz` (`MergedLazWriter`, streaming LASzip; an error message if it can't be opened).
  5. For each chunk `i`:
     - prints `------------------------` and `saving to: <laz>`;
     - loads the chunk; `points_to_vector` (threshold filter, `index_poses_i`, chunk-local points); `exportLaz` → `scan_lio_<i>.laz`;
     - for the merged file, transforms the points to world with the chunk's first pose;
     - collects that pose and the file name, and with `save_index_pose` the index poses;
     - prints `saving to: <csv>` and writes `trajectory_lio_<i>.csv`:
       - a header, then for each node: the timestamp ×1e9 at `setprecision(20)`, the 12 entries of `traj[0].inverse() * traj[j]` at `setprecision(10)`, and the unix timestamp ×1e9 and `om fi ka` at `setprecision(20)`;
       - **if the file can't be opened, `save_result` returns at once.**
  6. Closes the merged LAZ and prints `saved <n> points to ...`.
  7. `save_poses` → `session_ini_poses.mri` and `session_poses.mrp`, with the same content: the pose count, then per chunk the file name and the 4×4 matrix.
  8. With `save_index_pose`: `index_poses.json`.
  9. `session.mjs`, a JSON `dump(2)`:
     - `Session Settings`: the offsets, the folder names (all `outwd`), the pose file paths, the version, the trajectory length, the elapsed time, `index_poses_path` (only with index poses), `point_sizes_path`, `decimation`, `threshold_nr_poses`;
     - `laz_file_names`.

     It prints `saving file:` and one `adding file:` per chunk.
  10. `save_parameters_toml` → `HDMapping_params_<version>_<date-time>.toml` through `TomlIO::SaveParametersToTomlFile`.
  11. `save_processing_results_json` → `HDMapping_results_<version>_<date-time>.json` (`dump(4)`), from `params`, `state` and the per-window gyro bias.

  Steps 10 and 11 each read the clock for their file name.
- **TOML users:**
  - CLI `main()`: loads `argv[2]` onto built-in defaults.
  - GUI `set_lidar_odometry_default_params`: `<exe>/default_settings.toml`.
  - GUI Load parameters and Save parameters (Parameters menu).
  - `save_parameters_toml`.

  All of them create a `TomlIO` and call `LoadParametersFromTomlFile`/`SaveParametersToTomlFile`. The golden tests (`tests/toml_golden_test.cpp`, `tests/smoke_test.cpp`) use `TomlIO` directly.
- **`get_next_result_path`** (`lidar_odometry_utils.cpp`) picks `lio_result_<max+1>` for the GUI. `save_poses` lives there too; its only caller is `save_result`.

## Goals / Non-Goals

**Goals:**
- `save_result` contains no file names, file streams, LASzip, JSON or TOML. It decides what to write and computes the data; the sink formats and writes it.
- The CLI and GUI load and save settings only through `ConfigRepository`.
- Every output byte unchanged (`44_short` identical, `44` within `--tolerance 1e-6`); the same console messages in the same order.

**Non-Goals:**
- Changing the output formats, the file set, or the two separate clock reads for the TOML and JSON names.
- The GUI cleanup (menus, advanced flow, settings widgets): a later change.
- `save_trajectory_to_ascii` (a GUI export) and `load_reference_point_clouds`.
- Moving `TomlIO` itself or changing its table-driven mapping.

## Decisions

1. **Ports** in `app/output_ports.h`:
   ```cpp
   struct TrajectoryRow { std::pair<double, double> timestamps; Eigen::Affine3d pose; Eigen::Vector3d om_fi_ka; }; // pose relative to the chunk's first node
   struct SessionSummary { double trajectory_length_m; double elapsed_seconds; double decimation; int threshold_nr_poses;
                           bool index_poses; std::size_t chunk_count; };

   class ResultSink {
   public:
       virtual ~ResultSink() = default;
       virtual bool begin() = 0;                                              // e.g. create the folder
       virtual void point_sizes(const std::vector<int> &sizes) = 0;
       virtual bool open_merged_cloud() = 0;
       virtual void chunk_cloud(std::size_t chunk, const std::vector<Eigen::Vector3d> &points,
                                const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
       virtual void merged_cloud(const std::vector<Eigen::Vector3d> &world_points,
                                 const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
       virtual bool chunk_trajectory(std::size_t chunk, const std::vector<TrajectoryRow> &rows) = 0;  // false: stop saving
       virtual void close_merged_cloud() = 0;
       virtual void chunk_poses(const std::vector<Eigen::Affine3d> &first_poses) = 0;              // one per chunk
       virtual void index_poses(const std::vector<std::vector<int>> &index_poses) = 0;
       virtual void session(const SessionSummary &summary) = 0;
       virtual void parameters(const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) = 0;
       virtual void processing_results(const OdometryConfig &config, const OdometryState &state,
                                       const std::vector<WorkerData> &windows, double elapsed_seconds) = 0;
   };

   class ConfigRepository {
   public:
       virtual ~ConfigRepository() = default;
       virtual bool load(const std::string &location, OdometryConfig &config, RunPaths &paths, GuiPreferences &gui) = 0;
       virtual bool save(const std::string &location, const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) = 0;
   };
   ```
   - *Method per output, called in today's order:* `save_result` keeps its control flow (which files, early return when the trajectory file fails, merged only with `export_merged_laz`, index poses only with `save_index_pose`). The order of files and console messages stays the same, so the console log and any partial output after an error match today's.
   - *What stays in the application:* the relative pose `traj[0].inverse() * traj[j]` and the world transform for the merged cloud are computations. They stay in `save_result`, with exactly the same expressions, so the arithmetic is unchanged.
   - *What goes to the sink:* number formatting (`setprecision`, ×1e9), file names and the JSON layouts.
   - *Logger:* the adapter gets it in its constructor. A sink exists for one run, so the interface stays free of it.
   - *`processing_results` takes config, state and windows:* the results JSON is a report over these. Building an intermediate struct would duplicate about 40 fields for no gain now. Step 5's domain types can narrow it.

2. **Adapters** (infrastructure, in `app/` until step 7):
   - `FolderResultSink(fs::path folder, ConfigRepository &config_repository, Logger &logger)` (`app/folder_result_sink.{h,cpp}`):
     - steps 1 and 3–11 moved verbatim from `save_result`, `save_parameters_toml`, `save_processing_results_json` and `save_poses`;
     - `MergedLazWriter` moves into its `.cpp`;
     - `parameters()` writes through the injected `ConfigRepository` (today's `TomlIO` call);
     - `get_next_result_path` moves to `folder_result_sink.h`.
   - `TomlConfigRepository` (`app/toml_config_repository.{h,cpp}`): `load`/`save` call `TomlIO::LoadParametersFromTomlFile`/`SaveParametersToTomlFile` unchanged, so the format and messages stay the same.

3. **Pipeline:** `OdometryPipeline::save(ResultSink &sink, const GuiPreferences &gui, double elapsed_seconds)` reports the Saving stage and calls `save_result(..., sink, point_store, logger)`.
   - The CLI builds `TomlConfigRepository config_repository;` and `FolderResultSink sink(argv[3], config_repository, reporter);`.
   - The GUI's `save_results` builds the sink on `get_next_result_path(working_directory)`.
   - The output folder isn't part of `RunInputs`, because the GUI decides it only when saving.

4. **Settings in the CLI and GUI:** one `TomlConfigRepository` per use, replacing each `TomlIO toml_io;`.
   - The CLI's "unreadable TOML exits 1" check stays: `fs::is_regular_file || !repo.load(...)`.
   - The GUI's try/catch around Load parameters stays.

5. **Tests:** `tests/toml_config_repository_test.cpp`:
   - `save` then `load` round-trips the default config and a changed one (a few fields of each type, plus the `[paths]` and `[misc]` values);
   - `load` of a missing file returns false.

   It compiles `toml_config_repository.cpp` with `toml_io.cpp`. `FolderResultSink` is not unit-tested: it needs LASzip and `core`'s `exportLaz`, which the tests don't link. It is checked bit for bit by `44_short` (all 11 files) on every commit.

## Risks / Trade-offs

- **[A formatting detail changes while moving]:** for example stream precision carried from one output line to the next, `std::endl` versus `\n`, or `dump(2)` versus `dump(4)`.
  → Move the code verbatim, with each file's stream created and used exactly as today; `44_short` compares every byte.
- **[The arithmetic moves]:** the relative pose and the world transform stay in `save_result` with the same expressions. Under `/fp:fast` a moved expression could round differently; that would show only on the full 44, which is judged with the tolerance.
- **[Partial output after an error]:** the early return on a trajectory-file failure is kept through `chunk_trajectory` returning false. → It is reviewed in the diff; there is no data to trigger it.
- **[`GuiPreferences` in an application port]:** `ResultSink::parameters` and `ConfigRepository` carry the GUI's `clear_color`, because it is part of the saved TOML (`[misc]`). This is accepted until the GUI cleanup decides whether `[misc]` stays.

## Migration Plan

Small commits, each built in both presets with `ctest`, and each checked on `44_short`:
1. ports and `TomlConfigRepository`, with tests, and the CLI/GUI settings switched to it;
2. `FolderResultSink` with the JSON, TOML and pose-file outputs (steps 3 and 7–11), with `save_result` calling it;
3. the LAZ and trajectory outputs (steps 4–6) through the sink, and `save_result` stripped of file I/O;
4. `OdometryPipeline::save(ResultSink&)`, CLI/GUI wiring, and `get_next_result_path` moved;
5. the full 44 run, the user's GUI check, and the docs.

Rollback: revert. No format changes.
