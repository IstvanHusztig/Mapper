# Tasks

Every code commit: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`.

## 1. Ports and the config repository

- [x] 1.1 Add `app/output_ports.h` with `TrajectoryRow`, `SessionSummary`, `ResultSink` and `ConfigRepository` (design decision 1). Verify both presets build.
- [x] 1.2 Add `app/toml_config_repository.{h,cpp}` (`TomlConfigRepository`, calling `TomlIO` unchanged), and `tests/toml_config_repository_test.cpp`:
  - a round trip of the defaults;
  - a round trip of a changed config (bool, int, double and string fields, `[paths]`, `[misc]` `clear_color`);
  - a missing file returns false.

  Verify `ctest` passes.
- [x] 1.3 Replace every `TomlIO toml_io;` outside `toml_io.cpp` and the adapter with `TomlConfigRepository`: CLI `main()`, `set_lidar_odometry_default_params`, Load parameters, Save parameters. Keep the CLI's exit-1 check and the GUI's try/catch.

  Verify that a grep for `TomlIO` in `app/lidar_odometry_gui.cpp` finds nothing, and that a CLI run with a missing TOML path still exits 1. Run the step checks. Commit.

## 2. Result sink: JSON, TOML and pose files

- [x] 2.1 Add `app/folder_result_sink.{h,cpp}` (`FolderResultSink(folder, ConfigRepository&, Logger&)`). Move these into it verbatim:
  - `begin` (create the folder);
  - `point_sizes`;
  - `chunk_poses` (`save_poses` → `.mri` and `.mrp`, moved from `lidar_odometry_utils.cpp`);
  - `index_poses`;
  - `session`;
  - `parameters` (from `save_parameters_toml`, through the repository);
  - `processing_results` (from `save_processing_results_json`).

  Until group 3, the LAZ and trajectory methods are not used. `save_result` takes a `ResultSink&` and calls these methods instead of its own code for steps 1, 3 and 7–11 of design Context. `OdometryPipeline::save` builds the sink on the given folder for now.

  Verify:
  - a grep for `nlohmann|save_poses|TomlIO|session.mjs` in `save_result` finds nothing;
  - the run prints the same console messages in the same order (diff the log of a `44_short` run with the previous one).

  Run the step checks. Commit.

## 3. Result sink: clouds and trajectories

- [x] 3.1 Move into `FolderResultSink`:
  - `MergedLazWriter` (the merged LAZ: `open_merged_cloud`, `merged_cloud`, `close_merged_cloud` with the `saved <n> points` message);
  - `chunk_cloud` (`exportLaz` → `scan_lio_<i>.laz`, with the `------------------------` and `saving to:` lines);
  - `chunk_trajectory`: the CSV header and the rows with today's precisions; it returns false when the file can't be opened.

  `save_result` builds the `TrajectoryRow`s with the same `traj[0].inverse() * traj[j]` expression, and keeps the world transform for the merged cloud.

  Verify:
  - a grep for `exportLaz|ofstream|laszip|fs::path|std::setprecision` in `save_result` finds nothing;
  - the console log diff against group 2's run shows no change.

  Run the step checks. Commit.

## 4. Pipeline and wiring

- [x] 4.1 Changes:
  - `OdometryPipeline::save(ResultSink &sink, const GuiPreferences &gui, double elapsed_seconds)`;
  - the CLI builds `TomlConfigRepository` + `FolderResultSink(argv[3], ...)`;
  - the GUI's `save_results` builds the sink on `get_next_result_path(working_directory)`;
  - `get_next_result_path` moves to `folder_result_sink.{h,cpp}`.

  Verify that `lidar_odometry.cpp` and `lidar_odometry_utils.cpp` no longer include `nlohmann`/`export_laz.h`/`toml_io.h` for outputs. Run the step checks. Commit.

  Result: `lidar_odometry.h` dropped `laszip_api.h`, `nlohmann/json.hpp` and `toml_io.h`. It keeps `export_laz.h`, because `points_to_vector` lives there. `lidar_odometry_utils.h` still includes `laszip_api.h` and `nlohmann/json.hpp`; it is the shared header of the old app code and is left for step 7.

## 5. Integration

- [ ] 5.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json`, or `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes.
- [ ] 5.2 GUI check, **done by the user**:
  1. start the Release GUI and use Load parameters with `settings/lidar-vertical.toml`;
  2. run `44_short` with "Load & process scanning", and say which `lio_result_*` it saved to;
  3. use Save parameters to save a TOML.

  Then I verify:
  - the result is IDENTICAL with the `44_short` manifest;
  - the saved TOML equals one saved by the previous build (apart from the build date line), using `settings/lidar-vertical.toml` loaded the same way.
- [ ] 5.3 Docs:
  - `CLAUDE.md` Architecture: the output ports and adapters, `save_result`'s role, settings through `ConfigRepository`. Testing: the new test source.
  - `docs/ARCHITECTURE.md`: mark roadmap step 4b done and update §3 Ports.

  Commit.
- [ ] 5.4 Run `openspec validate output-io-ports --strict` and confirm it passes.
