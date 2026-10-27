# Proposal

## Why

The Unitree L2's IMU produces false gyro offsets after turns, so the handheld moves to the Livox Mid-360S, mounted the same way (vertical, same Z-up output, same `lidar*.laz` + `imu*.csv`). dMapper must run with either sensor without code changes. Every sensor difference has to come from the settings file, and code and user docs must not be written for one model.

## What Changes

- **BREAKING (file name):** `settings/lidar-vertical.toml` is renamed to `settings/l2-vertical.toml`, and `settings/lidar-hardware-specs.md` to `settings/l2-hardware-specs.md`. The values don't change.
- New `settings/livox-vertical.toml` for the Mid-360S (vertical mount), with a sensor sheet `settings/livox-mid360s-hardware-specs.md`. Its IMU-rate and noise values are provisional until the first Mid-360S recordings are measured.
- The start-up gyro bias, hard-coded today to the first 1000 IMU samples with a variance gate of 5.0 deg²/s², becomes two TOML settings: `startup_gyro_bias_nr_samples` and `startup_gyro_bias_max_variance_deg2_s2`. Their defaults are the old values, so results don't change.
- Code comments in `odometry_config.h` and `recording_folder_source.cpp` describe the quantities, not the L2. Where the L2 values come from is recorded in `l2-vertical.toml`.
- User docs drop L2- and recording-44-specific content and use general terms: `README.md`, `docs/*.html` (en + hu), and code/test comments. README gains a "Supported sensors" note.
- Developer markdown stays as it is for now (`CLAUDE.md`, `tools/README.md`, `docs/*.md`), to be generalized after the Mid-360S has been tested. The same goes for tool docstrings and `openspec/changes/**`. Only their `lidar-vertical` paths are renamed.

## Capabilities

### New Capabilities
- `sensor-settings`: sensor-specific values come only from the settings file. The repository ships one TOML per supported sensor, and the start-up gyro bias window and gate are settings.

### Modified Capabilities
<!-- none: the existing specs name lidar-vertical.toml only as the settings file of their scenarios; the path is updated in place (see design) -->

## Impact

- **Code:**
  - `src/domain/odometry_config.h` (2 new settings, comments);
  - `src/application/lidar_odometry.{h,cpp}` (`calculate_trajectory` takes the settings);
  - its caller in `src/application/odometry_pipeline.cpp`;
  - `src/infrastructure/toml_io.h` (POINTERS, CATEGORIES);
  - the comment in `src/infrastructure/recording_folder_source.cpp`.
- **Settings:** `settings/l2-vertical.toml` (renamed, plus 2 keys), `settings/default_settings.toml` (plus 2 keys), and the new `settings/livox-vertical.toml`.
- **Tests:**
  - `tests/toml_golden_test.cpp` and `tests/gui_view_model_test.cpp` (renamed golden file, a new livox case);
  - the golden TOMLs, regenerated;
  - comment references in four other test files.
- **Tools:** `tools/compare_baseline.py` usage line, and the `"params"` field of the two baseline manifests (metadata).
- **Results:** none. `44_short` and `44` stay bit-identical with their manifests.
