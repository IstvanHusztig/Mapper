# Proposal

## Why

Handheld Unitree L2 recordings (caves, outdoors) produce maps with rotated "ghost" copies of walls and thick, smeared surfaces. The per-window optimizer gives every trajectory node a fixed, loose rotation freedom (`*_1_sigma_deg` = 0.1–0.15° per node, independent of time step). That lets single nodes, especially the last nodes of a window that have almost no points, spin away. Via the hard-fixed origin of the next window, one such spike becomes a permanent heading error. Separately, the gyro bias is never removed on handheld data: the startup check sees hand motion (3–10 °/s spread, while sensor noise is only about 1 °/s per sample) and falls back to zero bias. Running from a caving start without standing still is normal, so the bias has to be estimated while moving.

Evidence (dataset 43, IMU timestamps repaired, same settings otherwise):

| run | node-to-node yaw jitter (median / p99) | end-of-window spikes > 3° | occupied 3 cm cells (Z section) |
|---|---|---|---|
| `lio_result_fixed_ts` (sigma 0.1/0.1/0.15°) | 0.343° / 4.13° | 9 | 39,307 |
| `lio_result_fixed_ts_sigma002` (sigma 0.02°) | 0.039° / 0.24° | 0 | 15,945 |

Tightening the sigma removes the ghosts. However, a fixed per-node sigma is the wrong model: node spacing varies from about 0.1 to 4 ms. A tighter prior also puts more weight on the biased IMU increments, leaving a steady LiDAR-vs-gyro yaw gap of about 1.4 °/s on dataset 43 and about 6 °/s on dataset 42.

## What Changes

- Rotation prior between consecutive nodes is scaled with the time step: `sigma_i = max(k * ARW * sqrt(dt_i), sigma_floor)`, per axis, instead of a constant per-node sigma.
- A 3-axis gyro bias (rad/s, body frame) is added to the unknowns of each window's optimization, entering the motion-model rotation residual as `-b * dt_i`.
- The bias estimate and its covariance are carried from window to window as a prior with random-walk growth, capped in magnitude, and applied to the next window's initial trajectory guess.
- The hand-off between windows is made robust, so that a poorly constrained last node can no longer inject a permanent heading error into the next window.
- Per-window bias estimates and their uncertainty are written to the processing-results output.
- Results become reproducible: new NDT buckets get an initialised normal vector (previously uninitialised memory decided whether 1–2 point buckets were used), and `useMultithread` defaults to false (parallel float summation is order-dependent). The parallel code is kept for a later rework.
- New tunable parameters (TOML-loadable): gyro ARW per axis, inflation factor `k`, sigma floor, bias random-walk, initial bias and its sigma, bias magnitude cap, and a switch to enable bias estimation.
- `motion_model_correction_om/fi/ka` is superseded by the estimated bias for new configurations. It keeps its existing meaning (a constant per-edge offset, usually 0) for backward compatibility and is not used to seed the bias.
- **BREAKING (results)**: with the new defaults, trajectories and maps differ from previous runs using the same TOML. Old behaviour stays reachable by disabling bias estimation and setting `k`/floor to reproduce the old constant sigma.

## Capabilities

### New Capabilities
- `lidar-odometry-motion-model`: How the per-window LiDAR odometry optimizer constrains consecutive trajectory nodes with the IMU. Covers the time-scaled rotation prior, online gyro bias estimation carried across windows, the window hand-off, and the reporting of bias estimates.

### Modified Capabilities
<!-- none: no existing specs in openspec/specs/ -->

## Impact

- `app/lidar_odometry_utils_optimizers.cpp`: `optimize_lidar_odometry` (state size `6n` becomes `6n+3`, motion-model block, bias prior, solution update and size check), `compute_step_2` window loop (bias/covariance carry-over, initial guess, hand-off).
- `app/lidar_odometry_utils.h`: `LidarOdometryParams` new fields; `WorkerData` stores per-window bias and covariance.
- `app/toml_io.h`: new entries in `POINTERS` and `CATEGORIES`.
- `app/default_settings.toml`, `settings/lidar-vertical.toml`: new parameters with defaults.
- `app/lidar_odometry.cpp`: `save_result` writes bias per window to the results JSON. `calculate_trajectory` is unchanged apart from making its `static` `first`/`last_ts` locals non-static, so that repeated runs in the GUI start clean.
- `app/lidar_odometry_gui.cpp`: default params and, optionally, GUI controls for the new parameters.
- Unchanged: Consistency/Consistency2 (they use the optimized trajectory as prior, not the IMU), the `use_robust_and_accurate_lidar_odometry` path (not supported with bias estimation in this change), and the core library.
- Depends on correct IMU timestamps. dLidar has been fixed, and existing recordings 42 and 43 are repaired.
