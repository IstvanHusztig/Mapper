# Tasks

## 1. Evaluation tooling and baseline

- [x] 1.1 Add `tools/eval_odometry.py` (pure Python, no numpy) that, for a dataset folder and one or more result folders, reports: node-to-node yaw jitter (median/p99), per-window LiDAR-vs-gyro yaw difference, end-of-window yaw spikes (> 3°), final LiDAR vs gyro yaw, and occupied 3/5/10 cm cells in a Z section from an ASCII export. Verify by reproducing the numbers for `raw data/43` `lio_result_fixed_ts` (jitter 0.343°, 9 spikes) and `lio_result_fixed_ts_sigma002` (0.039°, 0 spikes).
- [x] 1.2 Document in `tools/README.md` how to export `scan_lio_0.laz` to ASCII with CloudCompare CLI and run the script. Verify the documented commands run as written on dataset 43.
- [x] 1.3 Record baseline results for datasets 42 and 43 (repaired timestamps) with current code and `settings/lidar-vertical.toml` (sigma 0.1/0.1/0.15) and with rotation sigmas 0.02. Verify both output folders exist and the metrics are written to `openspec/changes/gyro-bias-estimation/baseline.md`.

## 2. Parameters and TOML

- [x] 2.1 Add the new fields from design D5 to `LidarOdometryParams` with documented defaults, and add `gyro_bias` / `gyro_bias_cov` to `WorkerData`. Verify the project builds (`cmake --build build --config Release --target lidar_odometry_step_1`).
- [x] 2.2 Register all new fields in `TomlIO::POINTERS` and in `CATEGORIES` (new section `gyro_bias_estimation`; sigma model keys in `motion_model_uncertainty`). Add them to `app/default_settings.toml` and `settings/lidar-vertical.toml`. Verify that an old TOML (e.g. `raw data/43/lio_result_1/HDMapping_params_*.toml`) loads with defaults and that a saved TOML round-trips all new values.
- [x] 2.3 Make `first` / `last_ts` in `calculate_trajectory` non-static (design D6), initialise `normal_vector` of new NDT buckets in `update_rgd` / `update_rgd_spherical_coordinates` (design D7), and set `useMultithread = false` by default (struct, `app/default_settings.toml`, `settings/lidar-vertical.toml`). Verify that two single-threaded CLI runs on dataset 43 with all new switches off give byte-identical `trajectory_lio_0.csv` and `scan_lio_0.laz`.

## 3. Time-scaled rotation prior

- [x] 3.1 Pass per-edge `dt` into the motion-model block of `optimize_lidar_odometry` and compute per-edge, per-axis rotation weights from `max(k*ARW*sqrt(dt), floor)` when `use_time_scaled_rotation_sigma` is on, keeping the Cauchy factor. Verify with the switch off that the output is unchanged (task 2.3 check).
- [x] 3.2 Tune default `k` (and the bias random walk `q`) on datasets 42 and 43, and record the sweep in `baseline.md`. Verify with `tools/eval_odometry.py` against the spec scenario "Map quality at least matches the tuned constant sigma" (jitter ≤ 0.03°, 0 end spikes > 3°, 3 cm cells not above the sigma-0.02 baseline on both datasets).

## 4. Gyro bias estimation

- [x] 4.1 Extend the state to `6n+3`: bias columns in the motion-model block via `relative_pose_obs_eq_tait_bryan_wc_case1_jacobian_simplified_1` (pose/bias cross terms, bias/bias block, RHS), LM damping on all entries, `h_x` size check `6n+3`, and the bias update. Verify on dataset 43 that the run completes with no "big jump" messages and that bias values are printed per window.
- [x] 4.2 Add the bias prior (design D3): carry `b` and `P` across windows in `compute_step_2`, grow `P` by `q²·T_window`, compute `P` from the factorization (3 solves), apply the clamp with a warning, and start from b = 0 with sigma `gyro_bias_initial_sigma_deg_s` (`motion_model_correction` stays a separate per-edge offset, see design D3). Verify that a window with the bias forced beyond `gyro_bias_max_deg_s` logs the clamp warning.
- [x] 4.3 Subtract the carried bias from the IMU relative rotations when building each window's initial guess (`new_trajectory`). Verify on dataset 43 that the per-window LiDAR-vs-gyro difference (bias-corrected gyro) has a lower mean than the task 3.2 run.
- [x] 4.4 Write the per-window `t_start`, bias (°/s) and 1-sigma (°/s) to the processing-results JSON in `save_result`. Verify that the JSON of a dataset 43 run holds one entry per window, in time order.
- [x] 4.5 Validate against the spec on 42/43 with bias estimation enabled: z-bias after 5 s has the sign of the whole-run offset and at least 1/3 of its magnitude, no clamp warning, and 3 cm cells at most 5 % above the bias-off run. Record the numbers in `baseline.md`. The static-recording scenario stays open until the recording exists.

## 5. Robust window hand-off

- [x] 5.1 Implement tail re-propagation (design D4): after each window, replace the last `handoff_tail_nodes` nodes by propagating from node `n-1-m` with bias-corrected IMU rotations and the optimized velocity, compute `prev_velocity` outside the tail, and take the next origin from the re-propagated last node. Verify that `handoff_tail_nodes = 0` disables it completely: with all three switches off, the run is byte-identical to `det_off_A` (built before the tail code existed).
- [x] 5.2 Verify on datasets 42 and 43 with defaults that no window has an end-of-window yaw spike > 3° (`tools/eval_odometry.py`), and record the results in `baseline.md`.

## 6. Integration

- [x] 6.1 Run the full default configuration on datasets 42 and 43 and render top-view Z-section comparisons against the baselines (old sigma, sigma 0.02). Verify that all spec scenario thresholds hold and attach the images and metrics to `baseline.md`.
- [x] 6.2 Verify that the rollback settings (all three switches off / tail 0) give metrics on dataset 43 within the pre-change run-to-run spread (reference `lio_result_fixed_ts_sigma002` and its rerun `check_a_old_rerun`), and that two single-threaded default runs are byte-identical.
- [x] 6.3 Update `CLAUDE.md` (Parameters section) with the new TOML section and switches. Verify the listed keys match `TomlIO::CATEGORIES`.

## 7. Static-recording validation and final defaults

- [x] 7.1 Measure the gyro bias, ARW and bias stability on the static recording `raw data/43/static`, and compare the bias estimate for q = 0.5 and q = 0.05 (as recorded and with evenly re-stamped IMU timestamps). Verify that the numbers are recorded in `baseline.md` and design "Tuning result".
- [x] 7.2 Set the defaults to q = 0.05, bias estimation on and ARW 0.028 / 0.042 / 0.057 (struct, `app/default_settings.toml`, `settings/lidar-vertical.toml`). Verify with default runs on 43 (twice, byte-identical), 42 and the static recording that the scenarios "Bias matches a static recording", "Bias on handheld data", "Bias estimate is stable", "Map quality with bias estimation" and "Map quality at least matches the tuned constant sigma" hold.
