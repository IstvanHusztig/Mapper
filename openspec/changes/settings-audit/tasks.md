# Tasks

Start only after `odometry-map-horizon` has shipped or been archived as rejected (user, 2026-10-12).

## 1. The audit table

- [ ] 1.1 `settings-audit.md` in this change: one row per `TomlIO::POINTERS` entry with table, class (design decision 1), where it is read (`file:line`), built-in default, `default_settings.toml` and `l2-vertical.toml` values, the evidence, and the proposal (keep / constant / dead / rename later). Also a section for the hard-coded values that should become settings: the five of the proposal, plus any others found in `src/domain` and `src/application`. Verify: the row count equals the `POINTERS` count; every *constant* or *dead* row names its evidence.
- [ ] 1.2 Check that `l2-vertical.toml` sets every key whose built-in default changes in group 4 (today 13), and every key a *dead* proposal touches. Verify: a list in the table, with any missing key named. A missing key blocks group 4 until the user decides.
- [ ] 1.3 The user decides on every *dead* and *constant* proposal (design decision 6). Verify: each row carries the decision and the date.

## 2. Hard-coded values become settings

- [ ] 2.1 The loop-closure search grid, the weak-geometry ratio, the gravity calibration time constant and the angle sigma become `OdometryConfig` members with today's values (design decision 3), in `POINTERS` and `CATEGORIES`. They are read where the literals were. Verify: Release build; `44_short` point clouds and trajectories bit-identical with `tools/baseline/44_short.manifest.json` (only the parameters TOML differs).
- [ ] 2.2 The 0.5 m minimum range in `run_odometry.cpp`: merged with `min_distance_lidar` or made its own setting (design decision 3), as the table decided. Verify: as 2.1.
- [ ] 2.3 Docs: the new settings in the parameter tables (EN/HU). Verify: each new key appears in both.

## 3. Settings become constants

- [ ] 3.1 Each approved *constant* row: the member and its `POINTERS`/`CATEGORIES` entries go, and a named `constexpr` with the same literal goes where it is read (design decision 2). Settings files and docs lose the key. Verify: Release build and `ctest`; `44_short` as 2.1; `grep` finds the old key nowhere in `src/`, `settings/` or `docs/`.

## 4. One source for the defaults

- [ ] 4.1 The 13 differing in-class defaults in `odometry_config.h` take the shipped values (design decision 4). Unit tests that relied on the old defaults set them explicitly. Verify: Release `ctest`; `44_short` as 2.1.
- [ ] 4.2 The golden test writes `settings/default_settings.toml` from the built-in defaults with `LO_UPDATE_GOLDEN=1`. A new test loads the shipped file and checks every `POINTERS` key is present and equal to its built-in default. Golden files and the shipped file are regenerated in their own commit. Verify: `ctest` passes; the new test fails when one key is deleted from the shipped file (tried once, then restored); the GUI starts with the regenerated file and shows it as the active parameter file.
- [ ] 4.3 Docs: the parameter tables (EN/HU) note the 13 changed built-in defaults; `CLAUDE.md`'s settings paragraph says the shipped file is generated. Verify: both pages and `CLAUDE.md` say so.

## 5. Unknown keys are reported

- [ ] 5.1 Loading logs one warning per unknown key with its table (design decision 5). Unit test in `tests/toml_config_repository_test.cpp`: a file with `ndt_map_horizon = 15.0` in `[lidar_odometry_control]` loads, keeps the built-in `ndt_map_horizon_m`, and the warning names the key and the table; `l2-vertical.toml` and the shipped file load without warnings. Verify: Release `ctest`.

## 6. Dead switches removed

- [ ] 6.1 Each approved *dead* row: the setting and its code path removed, one commit each, the golden files regenerated in the same group (own commit). Verify: Release build and `ctest`; `44_short` as 2.1; no trace in `src/`, `tests/`, `docs/`, `settings/`.
- [ ] 6.2 A Debug build and `ctest` (code paths removed, structs changed). Verify: both pass.

## 7. Baselines and wrap-up

- [ ] 7.1 Full 44 with `settings/l2-vertical.toml`. Verify: `tools/compare_baseline.py` against `tools/baseline/44.manifest.json` reports point clouds and trajectories identical; only the parameters TOML differs.
- [ ] 7.2 New 44 and 44_short manifests in their own commit (parameters TOML only), old baseline folders renamed with a date suffix. Verify: `compare_baseline.py` IDENTICAL against the new manifests.
- [ ] 7.3 Memory updated; candidates for measured follow-up changes (renames with aliases, `sliding_window_trajectory_length_threshold`, changed defaults such as `imu_always_on`) listed in the table's last section. Verify: the entries exist.
