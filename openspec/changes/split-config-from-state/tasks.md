# Tasks

Every step ends with: both presets build `lidar_odometry_step_1` and `lidar_odometry_tests`; `ctest` passes in Release; a Release run on `44_short` compares IDENTICAL with `tools/baseline/44_short.manifest.json` (run from the repo root, as in `tools/README.md`); then one commit.

## 1. Golden TOML test (before any refactor)

- [x] 1.1 Add a compile definition with the repo path to `lidar_odometry_tests`, and a test that saves the default params and the loaded `settings/lidar-vertical.toml` (both with a fixed `build_date`) and compares them with `tests/data/default_params.golden.toml` / `tests/data/lidar_vertical.golden.toml`; generate the golden files from the current code, check they contain `[paths]` and `[misc] clear_color`, and verify the test passes, and fails after editing one golden byte
- [x] 1.2 Run the step checks and commit

## 2. OdometryState

- [x] 2.1 Add `OdometryState` (with `RunPaths paths`) and move `m_g`, `initial_points`, `reference_points`, `reference_buckets`, `buckets_indoor`, `buckets_outdoor`, `consecutive_distance`, `total_length_of_calculated_trajectory`, `working_directory_cache` out of `LidarOdometryParams`; keep the TOML-persisted `[paths]` fields (`working_directory_preview`, `current_output_dir`) in params until 3.1
- [x] 2.2 Add an `OdometryState&` (or `const&` for readers) parameter to `load_data`, `compute_step_1`, `compute_step_2`, `run_consistency`, `filter_reference_buckets`, `load_reference_point_clouds`, `save_result`, `save_parameters_toml`, `save_processing_results_json`, and to whatever else the compiler flags; keep every copy a copy
- [x] 2.3 CLI and GUI: add an `OdometryState` next to `params` (a global `state` in the GUI) and pass it; verify the build has no remaining references to the moved fields on params
- [x] 2.4 Run the step checks and commit

## 3. SettingsDocument and TOML I/O

- [x] 3.1 Add `GuiPreferences` (`float clear_color[4]`) (no `SettingsDocument`, see design.md findings); move `clear_color`, `current_output_dir` and `working_directory_preview` out of params (the latter two into `RunPaths`); the GUI converts `clear_color` to and from `ImVec4` where it uses it
- [x] 3.2 Change `TomlIO` load/save to take config, `RunPaths` and `GuiPreferences` (`POINTERS` into the config, a small `RunPaths` key table, `[misc] clear_color` from `gui`); `CATEGORIES` unchanged; `save_parameters_toml` builds the document from the config, `state.paths` and the GUI prefs
- [x] 3.3 CLI and GUI: load into config, `state.paths` and `gui_prefs`; update the round-trip smoke test; verify the golden TOML test still passes byte for byte
- [x] 3.4 Run the step checks and commit

## 4. Rename and const

- [ ] 4.1 Rename `LidarOdometryParams` to `OdometryConfig` everywhere (types only, no logic change); verify `grep LidarOdometryParams` finds nothing outside archived OpenSpec changes
- [ ] 4.2 Make pipeline functions take `const OdometryConfig&` (the functions listed in design.md Goals); make the grid-parameter arguments of `update_rgd`, `update_rgd_spherical_coordinates`, `align_to_reference`, `optimize_lidar_odometry` and `optimize_sf*` `const&`; record in design.md any that turn out to write
- [ ] 4.3 Run the step checks and commit

## 5. Integration

- [ ] 5.1 Full 44: run in the background and verify IDENTICAL with `tools/baseline/44.manifest.json`
- [ ] 5.2 GUI check on `44_short`, **done by the user** (the GUI can't be driven from the CLI): repeat exactly the clicks of the pre-refactor run `44_short/lio_result_0` (2026-10-01 21:56, manifest `44_short_gui_before.manifest.json` in this change) with the refactored Release build; verify with `compare_baseline.py` that the new GUI output is IDENTICAL on all 11 files
- [ ] 5.3 Update `CLAUDE.md` (type names and the Parameters paragraph: `OdometryConfig`, `OdometryState`, `SettingsDocument`; where TOML fields are registered) and mark step 1 done in `docs/ARCHITECTURE.md`; commit
- [ ] 5.4 Run `openspec validate split-config-from-state --strict` and confirm it passes
