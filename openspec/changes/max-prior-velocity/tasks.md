# Tasks

Every code commit: both presets build `dMapper` and `lidar_odometry_tests`. The GUI must be closed while building.

## 1. Setting

- [x] 1.1 Add the setting (design decisions 1–4):
  - `OdometryConfig::max_prior_velocity_mps = 10.0` in `app/lidar_odometry_utils.h`, with a comment;
  - an entry in `TomlIO::POINTERS` and in the `motion_model_uncertainty` entry of `CATEGORIES` (`app/toml_io.h`);
  - in `compute_step_2`, replace `10.0` with `params.max_prior_velocity_mps` and use the new console message;
  - `settings/lidar-vertical.toml` 5.0, `app/default_settings.toml` 10.0, both in `[motion_model_uncertainty]`.

  Verify:
  - a grep for `> 10.0` near `prev_velocity` finds nothing;
  - `ctest` fails only in the two golden TOML tests, and each diff is just the new `max_prior_velocity_mps` line.

  Commit.
- [x] 1.2 Regenerate the golden TOML files with `LO_UPDATE_GOLDEN=1` (design decision 5). Verify:
  - `git diff` shows only one new line per file: 10.0 in `default_params.golden.toml`, 5.0 in `lidar_vertical.golden.toml`;
  - `ctest` passes.

  Commit on its own.

## 2. Results

- [ ] 2.1 Release CLI run on `44_short` with `settings/lidar-vertical.toml` and compare with `tools/baseline/44_short.manifest.json`. Count the "not used as prior" lines in the console log. Then:
  - **only the parameters TOML differs:** update that entry in `44_short.manifest.json`;
  - **other files differ too:** refresh `raw data/44_short_baseline` from the run and rewrite `44_short.manifest.json` with `--write-manifest`.

  In both cases, set the parameters TOML entry of `tools/baseline/44.manifest.json` to the new hash (design, Risks). No full 44 run (user decision, 2026-10-03).

  Verify that the run compares IDENTICAL with the updated `44_short.manifest.json`. Report the drop count and which case applied. Commit the manifests on their own.

## 3. Docs

- [ ] 3.1 `docs/index.html`:
  - a row for `max_prior_velocity_mps` (5 m/s) in the `[motion_model_uncertainty]` table, after `lidar_odometry_motion_model_acc_1_sigma_mps2`. It says the setting is measured over the last 20 nodes (about 22 ms) and that 0 or below disables the prior;
  - one sentence in the "Velocity continuity" bullet of the Window optimization step.

  Verify that the page renders, by opening it in a browser or checking that the tags are balanced. Commit.
- [ ] 3.2 Run `openspec validate max-prior-velocity --strict` and confirm it passes.
