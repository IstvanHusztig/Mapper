# Proposal

## Why

dMapper has 130 loadable settings (`TomlIO::POINTERS`), but the shipped `settings/default_settings.toml` lists only 77 of them; the other 53 exist only as code defaults (all of loop closure, the IMU translation terms, `ndt_map_horizon_m`). At the same time, some values that depend on the environment are fixed in the code, for example the loop-closure registration search grid, which limits how much drift a loop can catch on a long cave route. Some switches only reproduce behaviour that has since been replaced (the degenerate gate before `imu_always_on`). A cave survey needs the settings that depend on the passage to be visible and changeable, and the rest to be fixed in one place. Planned for after `odometry-map-horizon` has shipped or been rejected (user, 2026-10-12), so the audit sees the final set of settings.

## What Changes

- **An audit table** (`settings-audit.md` in this change): each of the 130 settings classified as *sensor* (depends on the LiDAR/IMU), *environment* (depends on the site or the walk), *algorithm* (a numerical detail nobody should tune per dataset), *debug* (diagnostics and outputs) or *dead* (only reproduces replaced behaviour, or not read at all), with where it is used and the values in `default_settings.toml` and `lidar-vertical.toml`.
- **Hard-coded values that depend on the environment become settings**, with today's values as defaults, so the results stay the same:
  - the loop-closure registration search grid (`loop_closure.cpp`: coarse radius 0.5 m and 2.5° yaw steps, fine radius 0.2 m, 0.75° yaw steps, 0.1 m / 0.2 m translation steps);
  - the weak-geometry eigenvalue ratio 0.2 in loop closure;
  - the minimum point range 0.5 m in `run_odometry.cpp`, next to `min_distance_lidar` (resolved: one setting, or a stated reason for two);
  - the gravity calibration time constant 2 s;
  - the fixed angle sigma 0.1°.
  The audit may add others it finds; each one is listed in the table with its reason.
- **Algorithm settings become named constants** in the code that uses them, when the audit finds nobody needs to change them (no difference between the two shipped files, no recording that needs another value). Each move is listed in the table.
- **Dead switches are removed with their code paths**, but only those the user approves item by item (they may change the built-in defaults, which `default_settings.toml` users rely on). **BREAKING** for a TOML that still sets a removed key: it is ignored, as unknown keys are today.
- **Unknown keys are reported:** today the loader silently ignores a key it doesn't know, so a removed or misspelled setting has no effect without notice. Loading now logs a warning naming each unknown key and its table; the run goes on.
- **One source for the defaults:** today the shipped `settings/default_settings.toml` differs from the built-in defaults in 13 values (`sliding_window_trajectory_length_threshold` 200 against 5 m, `max_distance_lidar` 70 against 30 m, `resolution_X/Y/Z` 0.3 against 0.0, `threshold_nr_poses`, `threshould_output_filter`, `use_mutliple_gaussian`, the motion-model sigmas), so the CLI with a parameter file that omits a key runs with other values than the GUI. The built-in defaults take the shipped values, and `settings/default_settings.toml` is written from them by the save path: complete (every setting) and equal by construction. A unit test fails when the file is missing a setting or differs from the built-in defaults.
- Not in this change: retiring `sliding_window_trajectory_length_threshold` or anything else that changes results. The audit table names such candidates; each becomes its own measured change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: adds the requirements that the shipped settings file lists every loadable setting, that unknown keys in a parameter file are reported, and that a value which depends on the sensor or the environment is a setting, not a constant in the code.

## Impact

- Code: `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h` (`POINTERS`, `CATEGORIES`), `src/application/loop_closure.cpp`, `src/application/run_odometry.cpp`, the files of any setting that becomes a constant or is removed.
- Settings: `settings/default_settings.toml` (regenerated, complete); the built-in defaults in `odometry_config.h` (13 values), `settings/lidar-vertical.toml` (removed keys only).
- Tests: the golden TOML files regenerated (`LO_UPDATE_GOLDEN=1`, own commit); a new test for the completeness of `default_settings.toml`.
- Results: bit-identical with the `lidar-vertical.toml` baselines (that file sets every key whose built-in default changes; checked in the tasks). A user TOML that omits one of the 13 keys now gets the GUI's value. The output's parameters TOML gains the new keys and loses removed ones, so the 44 and 44_short manifests are rewritten in their own commit, as after earlier setting changes.
- Docs: the parameter tables (EN/HU) and `CLAUDE.md` where it names settings.
