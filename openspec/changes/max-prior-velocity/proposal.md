# Proposal

## Why

Each window after the first uses the previous window's final velocity as a prior:
- for the velocity-continuity term;
- for the IMU acceleration term;
- for the starting guess of the window's positions.

As a sanity check, that velocity is dropped when it is faster than 10 m/s. The limit is hard-coded (`prev_velocity.norm() > 10.0` in `compute_step_2`), and 10 m/s is far above what the handheld recordings can reach: in a cave, walking speed stays under 5 m/s. So a bad velocity from a failed window, for example 7 m/s, still seeds the next window. The limit should be a setting, and `settings/lidar-vertical.toml` should use 5 m/s.

## What Changes

- **New setting** `max_prior_velocity_mps` in `[motion_model_uncertainty]`. It is the speed above which the previous window's velocity is not used as a prior. Its built-in default is 10.0, so a TOML file without the key behaves as before.
- **Loading and saving:** the setting is read from and saved to the parameters TOML like every other setting. The saved `HDMapping_params_*.toml` gains the key.
- **`settings/lidar-vertical.toml`** sets it to 5.0. This can change results, but only when a window ends faster than 5 m/s.
- **Console message:** the message logged when the velocity is dropped shows the limit and the speed instead of the fixed text `!!!prev_velocity.norm() > 10.0!!!`.
- **Docs:** `docs/index.html` documents the setting.

Behaviour with the built-in defaults, and with any TOML file that doesn't set the key, stays the same.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: a new requirement, "Velocity prior limit": the previous window's velocity is used as a prior only up to a configurable speed.

## Impact

- **Code:**
  - `app/lidar_odometry_utils.h` (`OdometryConfig` member);
  - `app/toml_io.h` (`POINTERS` and the `motion_model_uncertainty` entry of `CATEGORIES`);
  - `app/lidar_odometry_utils_optimizers.cpp` (`compute_step_2`).
- **Settings:** `settings/lidar-vertical.toml` (5.0) and `app/default_settings.toml` (10.0, the GUI's start-up file, so its behaviour stays the same).
- **Tests:** the golden TOML files `tests/data/default_params.golden.toml` and `tests/data/lidar_vertical.golden.toml` gain the key. They are regenerated in their own commit.
- **Docs:** `docs/index.html`.
- **Results:**
  - `44_short` and the full 44 run with `settings/lidar-vertical.toml`, so they change only if a window there ends faster than 5 m/s.
  - The user decided (2026-10-03) not to run the full 44 for this change. `44_short` is the check.
  - The saved parameters TOML gains a line, so the `HDMapping_params_*.toml` entry of the manifests no longer matches. That file is expected to differ; the other files are compared as usual.
