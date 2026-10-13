# Design

## Context

- Settings are `OdometryConfig` members with in-class defaults (`src/domain/odometry_config.h`). `TomlIO` loads and saves them through `POINTERS` (name → member pointer, 130 entries) and `CATEGORIES` (table → keys). Loading walks `CATEGORIES` and reads only the keys it knows; anything else in the file is skipped without a word (`toml_io.cpp`, the loop around `POINTERS.find`).
- The CLI uses the built-in defaults plus its TOML. The GUI loads `settings/default_settings.toml` (copied next to the exe). That file was written by HDMapping 0.95's save path in March, has 96 keys, and differs from the built-in defaults in 13 values.
- The golden tests (`tests/toml_golden_test.cpp`) save the built-in defaults and `lidar-vertical.toml` and compare them byte for byte with `tests/data/*.golden.toml`; `LO_UPDATE_GOLDEN=1` rewrites them.
- The baselines run with `settings/lidar-vertical.toml`. Its results must not move. The output's parameters TOML is part of the manifests, so any key added or removed rewrites them (done before, for example in 8597f5f).
- Planned for after `odometry-map-horizon`. The audit then sees the final set of settings, including `ndt_map_horizon_m` and whatever happened to `sliding_window_trajectory_length_threshold`.

## Goals / Non-Goals

**Goals:**
- One table that says, for every setting, what it is for, which class it belongs to, and why.
- One source of defaults, shown complete in the shipped file.
- No silent ignoring of keys.
- Every move bit-identical with the `lidar-vertical.toml` baselines.

**Non-Goals:**
- Changing any value in `lidar-vertical.toml`, or which algorithm runs with it.
- Renaming settings, including the misspelled ones (`use_mutliple_gaussian`, `threshould_output_filter`). A rename breaks every saved TOML. The table lists the misspellings, and a rename can come later, together with an alias.
- New TOML value types (arrays, enums). Only bool/double/int/string, as today.

## Decisions

1. **The classes, and how a setting is placed:**
   - *sensor*: changes with another LiDAR or IMU (ranges, noise densities, lever arm, frequencies).
   - *environment*: changes with the site or the walk (voxel and NDT sizes, map horizon, loop-closure search and gaps, rest detection thresholds that depend on how the operator walks).
   - *algorithm*: a numerical detail of a method (finite-difference steps, iteration limits, convergence thresholds, batch sizes).
   - *debug*: diagnostics and optional outputs.
   - *dead*: not read by the code, or only selecting a path a later change replaced.

   The deciding question is "would another recording or another sensor need another value?" The evidence for each setting goes in the table: the two shipped files differ, a change tuned it (`openspec/changes/archive`), or the docs explain it per site. An *algorithm* setting with none of that evidence becomes a constant. When in doubt, it stays a setting.
   *Alternative:* keep everything as settings and only document. Rejected: 130 settings hide the dozen a cave survey needs to touch.

2. **Former settings become constants next to their use.** Each becomes a `constexpr` in the function or file that reads it, named like the setting and commented with the table's reason. A shared header of constants was rejected, because it would bring back the catch-all header `CLAUDE.md` says the code doesn't have.

3. **New settings keep today's value as their default**, go into the table where the code that uses them lives (`[loop_closure]` for the search grid, `[lidar_odometry_control]` for the range and the angle sigma, `[imu_translation]` for the gravity time constant), and are named with their unit (`_m`, `_deg`, `_s`). The duplicate 0.5 m minimum range is read first. If it is the same filter as `min_distance_lidar`, the code uses the setting. That is bit-identical only if the values match in `lidar-vertical.toml`; if they don't, it becomes its own setting and the table says why there are two.

4. **Built-in defaults take the shipped values, and the shipped file is generated.** The 13 differing in-class defaults in `odometry_config.h` become the shipped file's values. `settings/default_settings.toml` is then written by the save path from a default-constructed config, with `LO_UPDATE_GOLDEN=1` in the golden test, which already does that for `default_params.golden.toml`. With that, `default_params.golden.toml` and the shipped file differ only in the build date. A test loads the shipped file and checks that every `POINTERS` key is present and equal to the built-in default.
   *Alternative:* edit the shipped file by hand and keep the built-in defaults. Rejected: two sources drift again, as they did since March.
   **Effect on results:** `lidar-vertical.toml` sets all 13 keys (checked in task 1.2), so the baselines don't move. The unit tests that build `OdometryConfig{}` see the new defaults, and any that depended on the old ones are adjusted to set the old values explicitly.

5. **Unknown keys:** after loading, each table's keys are compared with `CATEGORIES` (plus `[paths]`, `[misc]` `clear_color`, `[version_info]`, the in/out grid tables), and each unknown key is logged as a warning with its table. `TomlIO` writes to `std::cout` today, so the warning goes there too. Routing through the `Logger` port is out of scope.
   *Alternative:* refuse the file. Rejected: an old TOML with a removed key must still run.

6. **Dead switches need the user's yes, one by one.** The table proposes them with their evidence. Each removal is one commit with its code path, in line with the no-dead-code rule. A removal that changes what the built-in defaults compute is named as such in the table. An example is the degenerate gate, which the built-in `imu_always_on = false` still uses. It needs its own decision, because a user TOML without `imu_always_on` runs that path.

## Risks / Trade-offs

- **[A value that looks like an algorithm detail turns out to matter on a cave recording]** → Only settings with no evidence of tuning become constants, and turning a constant back into a setting is a small change. When in doubt, it stays a setting.
- **[A user TOML relies on an old built-in default]** → The 13 changes are listed in the docs' parameter table and in the commit message; `lidar-vertical.toml` and the GUI default are unaffected.
- **[Bit-identity breaks through a moved value's type or order of operations]** → Each constant keeps the exact literal and expression. `44_short` is checked after every commit, and the full 44 once at the end.
- **[Unknown-key warnings flood old TOMLs]** → One line per key, printed once per load.

## Migration Plan

- Order: table → user decisions on dead switches → new settings → constants → defaults and the generated file → unknown-key warning → removals. Each group in its own commit, `44_short` bit-identical after each.
- The golden files and the manifests are regenerated in their own commits.
- Rollback: revert the commits. No data formats change, apart from the keys in the parameters TOML.
