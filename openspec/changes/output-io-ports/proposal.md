# Proposal

## Why

Roadmap step 4b in `docs/ARCHITECTURE.md` finishes the I/O ports that 4a started on the input side. Today the output side is written directly from pipeline code:
- **`save_result`** (`app/lidar_odometry.cpp`) both computes and writes. It chunks the windows, places points in world coordinates and computes relative poses. It also opens files and formats them: `point_sizes_per_chunk.json`, `scan_lio_<i>.laz` (`exportLaz`), `scan_lio_merged.laz` (LASzip), `trajectory_lio_<i>.csv`, `session_ini_poses.mri`/`session_poses.mrp` (`save_poses`), `index_poses.json`, `session.mjs`, `HDMapping_params_*.toml` and `HDMapping_results_*.json`.
- **TOML:** `TomlIO` is created ad hoc in five places: the CLI loading its settings, the GUI loading its defaults, Load parameters, Save parameters, and the parameters TOML written with each result.

The pipeline's `save` stage can't run without a disk, and the CLI and GUI depend on the TOML class directly. 4b closes the I/O part of the dependency rule before the domain model work (step 5). It also gives the planned GUI cleanup, where loading a TOML becomes the only way to change settings, one entry point for loading settings.

## What Changes

- **Application-layer ports** (`app/output_ports.h`):
  - `ResultSink`: the result of one run. It receives:
    - the point sizes;
    - each output chunk's cloud and trajectory rows;
    - the merged cloud;
    - the chunk poses;
    - the index poses;
    - the session summary;
    - the parameters;
    - the processing results.
  - `ConfigRepository`: loads and saves `OdometryConfig` + `RunPaths` + `GuiPreferences` by location.
- **Infrastructure adapters:**
  - `FolderResultSink`: one output folder. It holds today's file names, formats and number formatting: `exportLaz`, the merged LAZ writer, the CSV, `save_poses` and the JSON layouts, all moved out of `lidar_odometry.cpp`/`lidar_odometry_utils.cpp`. It writes the parameters TOML through a `ConfigRepository`.
  - `TomlConfigRepository`: wraps `TomlIO`.
  - `get_next_result_path` moves next to `FolderResultSink`.
- **`save_result`** keeps only the computation: chunking through `PointStore`, `points_to_vector`, world coordinates, relative poses, and which files exist (`export_merged_laz`, `save_index_pose`). It passes data to the sink and contains no file names, streams or JSON.
- **`OdometryPipeline::save`** takes a `ResultSink&` instead of an output folder. The CLI and GUI build a `FolderResultSink` for the output folder.
- **Settings:** the CLI and GUI load and save settings only through `ConfigRepository`; `TomlIO` is used only inside the adapter.
- **Unchanged:**
  - every output file: `44_short` bit-identical, `44` identical or within `--tolerance 1e-6` (1 µm);
  - the file set and file names;
  - console wording;
  - TOML format (golden tests).
- **Docs:** roadmap step 4b is marked done in `docs/ARCHITECTURE.md` and described in `CLAUDE.md`.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- None. This is a pure refactor with no behaviour change, so the change sets `skip_specs: true`.

## Impact

- **New files:** `app/output_ports.h`, `app/folder_result_sink.{h,cpp}`, `app/toml_config_repository.{h,cpp}`, and tests in `tests/`.
- **Changed files:**
  - `app/lidar_odometry.{h,cpp}`: `save_result` slimmed; `save_parameters_toml`, `save_processing_results_json` and `MergedLazWriter` move out;
  - `app/lidar_odometry_utils.{h,cpp}`: `save_poses` and `get_next_result_path` move out;
  - `app/odometry_pipeline.{h,cpp}`;
  - `app/lidar_odometry_gui.cpp`: CLI, defaults, Load/Save parameters, saving results;
  - `CMakeLists.txt`, `tests/CMakeLists.txt`, `CLAUDE.md`, `docs/ARCHITECTURE.md`.
- **Not touched:**
  - `core/`;
  - `app/toml_io.{h,cpp}` itself (it becomes an implementation detail of the adapter);
  - the GUI-only export `save_trajectory_to_ascii`, which is presentation and goes with the GUI cleanup if unused;
  - `load_reference_point_clouds`.
- **Validation:**
  - `44_short` on every commit, the full `44` at the end;
  - the user checks the GUI: a one-click run identical to the baseline, Load parameters, and Save parameters producing the same TOML as before.
