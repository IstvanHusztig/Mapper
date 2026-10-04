# Proposal

## Why

Roadmap step 7b finishes step 7. The code sits in one flat `app/` folder of 55 files, and nothing stops a domain file from including LASzip, json, ImGui or Fusion; today only reviews catch that. Step 7a cleaned up the includes, so the files can now move into the layer folders of `docs/ARCHITECTURE.md` §5, with CMake libraries whose include paths and links enforce the dependency rule: a domain file that includes an infrastructure header no longer compiles.

## What Changes

- **Folders.** `app/` is replaced by:
  - `src/domain/`: `window.h`, `angle_units.h`, `run_reporting.h` (the `Logger` interface the optimizers report through), `odometry_config.h`, `odometry_state.h`, `ndt_map`, `normal_equations`, `imu_kinematics`, `window_steps`, `optimizer_options`, `scan_to_map`, `robust_lidar_odometry`;
  - `src/application/`: the ports (`input_ports.h`, `output_ports.h`) and the use cases (`odometry_pipeline`, `lidar_odometry`, `run_odometry`, `loop_closure`, `consistency`);
  - `src/infrastructure/`: the adapters (`recording_folder_source`, `recording_files`, `disk_point_store`, `folder_preview_sink`, `folder_inputs`, `folder_result_sink`, `toml_io`, `toml.hpp`, `toml_config_repository`, `console_reporting`);
  - `src/presentation/cli/` and `src/presentation/gui/`: the CLI branch of today's `main()`, and the GUI (`lidar_odometry_gui.cpp`, `resource.rc`, `resource.h`, `icon.ico`);
  - `src/main.cpp`: the composition root, which picks the CLI or the GUI by the argument count.
- **Libraries.** These static libraries replace the flat source list of `dMapper`:
  - `lo_domain`: Eigen, manif, observation_equations and `core/include` (the "math kernel"). No LASzip, json, GL, Fusion or TBB include paths.
  - `lo_application`: depends on `lo_domain`. Fusion and TBB stay as listed exceptions until roadmap step 9.
  - `lo_infrastructure`: depends on `lo_application`, plus LASzip, json and toml.

  `dMapper` links them, plus `core` and the GUI libraries. The tests link the libraries instead of compiling app sources.
- **Three small preparations, so that the moves stay pure:**
  - **(a)** The app's `get_index` / `get_rgd_index` are renamed to `ndt_cell_key` / `ndt_cell_index`. Today the app and `core/src/hash_utils.cpp` both define them, and only "the exe's objects win" decides which one links. Inside static libraries, the library search order would decide instead.
  - **(b)** `points_to_vector`, the only thing the use cases take from `core`'s `export_laz.h`, moves into a light `core/include/point_placement.h`, with `export_laz.h` including it. The same pattern as `ndt_types.h`.
  - **(c)** The unused `session.h`, `Version.hpp` and `export_laz.h` includes go out of the use cases.
- **`default_settings.toml`** moves from `app/` to `settings/`, next to `lidar-vertical.toml`. It is still copied next to the exe.
- **No behaviour change.** Results, outputs and settings stay the same. The precise build must stay bit-identical. The normal build must be IDENTICAL or within 1e-6.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: the scenario "Default is single-threaded" names the shipped settings file by its path, which changes from `app/default_settings.toml` to `settings/default_settings.toml`.

## Impact

- **Code:** every file in `app/` moves with `git mv` (history follows). `CMakeLists.txt` gets the libraries, and `tests/CMakeLists.txt` links them. `core` gets the new `point_placement.h`; `CORE_SOURCE_FILES` and its link order are untouched.
- **Docs:** every path under `app/` in `CLAUDE.md`, `docs/*.md`, `docs/*.html` and `tools/README.md`. `docs/ARCHITECTURE.md` marks step 7 done.
- **Not in this change:**
  - the Fusion / TBB ports (step 9) and the GUI globals (step 10);
  - sorting `core` (step 11);
  - `GuiPreferences`, which stays in `odometry_config.h` because the `ConfigRepository` port loads it with the settings.
