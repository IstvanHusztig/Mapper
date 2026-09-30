# Design

## Context

See proposal.md (Why) for motivation and the measured evidence.

Current state of the per-window optimization (`optimize_lidar_odometry`, `app/lidar_odometry_utils_optimizers.cpp`):

- The unknowns are `6n` Tait-Bryan pose parameters for the `n = threshold_nr_poses` (360) nodes of one window, where a node is one IMU sample (about 1.1 ms apart after the timestamp repair). They are solved by Gauss-Newton/LM for `nr_iter` iterations with a dense `AtPAndt` that is later sparsified.
- Motion-model block (:830-918): for each edge `(i, i+1)`, the relative pose of the current estimate is compared with the relative pose of `intermediate_trajectory_motion_model`, which holds the IMU orientation chain plus the constant-velocity translation. Weights are constant per edge (`1/sigma²` from `*_1_sigma_deg` / `*_1_sigma_m`), with Cauchy weighting on the rotation terms. `motion_model_correction` adds a constant per-edge offset.
- Node 0 is fixed to the previous window's last node with sigma 1e-6 (:974-995).
- The window loop in `compute_step_2` (:1199-1270) builds each window's initial guess. It chains the IMU relative rotations from the previous window's last node and adds `prev_velocity * (t - t0)`, where `prev_velocity` comes from the previous window's last 20 nodes.
- In `compute_step_1` (`app/lidar_odometry.cpp:509-522`), a window only takes points with `t < t_last`, so the last node of every window has no points and the nodes just before it have very few. These poorly constrained tail nodes are where the heading spikes occur. The tail node is then copied into the next window as a hard-fixed origin.
- The generated observation equations already provide the 6x12 Jacobian `relative_pose_obs_eq_tait_bryan_wc_case1_jacobian_simplified_1` (`3rdparty/observation_equations/.../relative_pose_tait_bryan_wc_jacobian.h:351`), which is needed for the pose-bias cross terms.
- Gyro noise measured on dataset 43 (893 Hz, sample-to-sample differences, MAD): x 0.54, y 1.22, z 1.53 °/s per sample, which gives ARW of about 0.018 / 0.041 / 0.051 °/√s.

## Goals / Non-Goals

**Goals:**
- A rotation prior whose strength reflects how much the gyro can actually drift over each edge's `dt`.
- A bias estimate that converges while moving, with no stationary start required, and that stays stable through short feature-poor stretches.
- No single poorly constrained node can rotate the rest of the trajectory.
- All new behaviour is switchable from TOML, and the old behaviour is reproducible.

**Non-Goals:**
- Accelerometer bias, IMU preintegration of translation, or full continuous-time trajectory models.
- Loop closure or global pose-graph changes.
- Changing `calculate_trajectory`'s AHRS (Fusion) beyond removing the `static` state.
- Supporting bias estimation in the `use_robust_and_accurate_lidar_odometry` path (it keeps the old behaviour).
- Re-deriving the generated observation-equation headers.

## Decisions

### D1. Time-scaled rotation prior with inflation and floor

Per edge and axis `a ∈ {om, fi, ka}`: `sigma_a,i = max(k * ARW_a * sqrt(dt_i), sigma_floor)`, with the weight `1/sigma²` replacing `w_motion_model_om/fi/ka`. The Cauchy factor stays as is.

- `k` covers model error that is not white gyro noise: timestamp jitter, vibration, and the small-angle bias model. Default k = 4, which gives about 0.007° per typical 1.1 ms edge on z. Tuned on datasets 42/43: k = 12 (about the 0.02° of test A) leaves more jitter; k = 2 and k = 1 cut jitter further but blur the map (3 cm cells +4 % / +12 % on 43), because the solution then follows the IMU too closely.
- `sigma_floor` (default 0.002°) keeps near-zero `dt` from producing unbounded weights.
- Translation sigmas are unchanged in this change.
- *Alternative: keep a constant per-node sigma and just lower it (test A).* This works on 43 but ties the model to the IMU rate and gives a 0.1 ms edge the same freedom as a 4 ms edge. Rejected as the final model, but kept as the reference result.

### D2. Bias as 3 extra unknowns per window, linear in the residual

The state becomes `[pose_0 … pose_{n-1} | b_om b_fi b_ka]` (size `6n+3`). The motion-model rotation measurement for edge `i` becomes `m_i - b * dt_i`, where `b` is in rad/s in the body frame of node `i`. The relative rotation is expressed in node `i`'s frame, so the derivative of the residual with respect to `b` is `dt_i · I₃` (small-angle). Assembly uses the relative-pose Jacobian `J_i` (6x12):
- pose/pose blocks: unchanged (`J_iᵀ W_i J_i`, already produced by the `_AtPA_simplified` helper);
- pose/bias blocks: `J_i[3:6]ᵀ · W_i,rot · dt_i` and its transpose;
- bias/bias block: `Σ dt_i² · W_i,rot`;
- right-hand side: `Σ dt_i · W_i,rot · r_i,rot`.

The LM damping applies to the `6n` pose entries only. `lm_factor` grows ×10 every 10 iterations (to about 5e10), which is negligible against the pose curvature (about 1e8 and above) but froze the bias (curvature about 1e3) after roughly 20 iterations. On dataset 42 the bias then crept to only −4 °/s after 40 windows instead of converging near −6 °/s. The bias stays well-posed through its prior (D3). After the solve, `b += Δb` and the node updates proceed as before. The `h_x.size() == 6n` check becomes `6n+3`.

- *Alternative: estimate yaw bias only.* Roll and pitch are pulled toward gravity by the AHRS, but with vertical mounting and tilting in caves the body z-axis is not reliably vertical. Three axes cost almost nothing extra. Rejected.
- *Alternative: a separate pass after step 2 (fit gyro vs. LiDAR rates, then rerun).* This doubles the runtime and cannot follow a drifting bias. Rejected.

### D3. Bias prior carried across windows (Kalman-style)

Each window adds the prior `b ~ N(b_prev, P_prev + q² · T_window · I)`, where `q` is the bias random-walk in rad/s/√s and `T_window` is the window duration. After the solve, the posterior covariance `P` is the 3x3 bias block of the inverse normal matrix. It is obtained with 3 solves against the existing Cholesky factor (unit vectors on the bias columns), which is cheap.
- First window: `b_prev = 0`, `P_prev = sigma_b0² · I` (default sigma_b0 = 10 °/s, effectively uninformative). `motion_model_correction` keeps its existing meaning: a constant per-edge offset added to the IMU increment (usually 0). Seeding the bias from it as well would count the same correction twice, so the two stay independent.
- The first window keeps the prior and does not estimate the bias. It is registered against the initial map, which `compute_step_2` builds from the first `threshold_initial_points` points without motion compensation, so its rotational misfit is not gyro bias. On dataset 43 that misfit drove the first-window bias to the ±20 °/s clamp and took about 10 windows to decay.
- Cap: if `|b|` on any axis exceeds `bias_max` (default 20 °/s), clamp it and log a warning.
- The bias from window `k` is subtracted from the IMU relative rotations when building window `k+1`'s initial guess (`new_trajectory` in `compute_step_2`), so the guess is already corrected.
- *Alternative: an independent bias per window with no prior.* One 0.4 s window gives a bias sigma of about 0.5 °/s at best, and in feature-poor windows it would be unconstrained and absorb registration error. Rejected.

### D4. Robust hand-off: re-propagate the window tail

After a window converges, the last `m` nodes (default `m = 30`, about 33 ms) are replaced by propagating from node `n-1-m` with the bias-corrected IMU relative rotations and the optimized velocity at node `n-1-m`. The next window's fixed origin is taken from this re-propagated last node. `prev_velocity` is computed from nodes that are not in the tail.
- This targets the observed failure directly: the tail nodes have almost no points, so their pose should come from the IMU, not from a handful of NDT matches.
- The origin stays hard-fixed (1e-6). Once the handed-off pose is trustworthy, a hard origin keeps windows continuous.
- *Alternative: a soft origin prior.* It only partly damps a bad hand-off, and a wrong origin still drags the next window. Rejected as the primary fix, but can be added later.
- *Alternative: overlapping windows (re-optimize the tail inside the next window).* This is the cleanest approach but changes `compute_step_1` partitioning, caching and the point-to-node indices. Deferred as a possible follow-up.

### D5. Parameters and reporting

New `LidarOdometryParams` fields, all registered in `TomlIO::POINTERS` and a new `CATEGORIES` section `gyro_bias_estimation` (plus `motion_model_uncertainty` for the sigma model):
`use_gyro_bias_estimation` (bool, default true; validated against the static recording), `use_time_scaled_rotation_sigma` (bool, default true), `gyro_arw_{x,y,z}_deg_sqrt_s`, `rotation_sigma_inflation_k`, `rotation_sigma_floor_deg`, `gyro_bias_rw_deg_s_sqrt_s`, `gyro_bias_initial_sigma_deg_s`, `gyro_bias_max_deg_s`, `handoff_tail_nodes`.
When `use_time_scaled_rotation_sigma = false`, the old constant `*_1_sigma_deg` weights are used. With both switches off, the result is identical to the current code (regression check).
`WorkerData` gains `gyro_bias` (Vector3d) and `gyro_bias_cov` (Matrix3d). `save_result` writes the per-window `t_start, b, sqrt(diag P)` to the processing-results JSON.

### D6. Small fix in `calculate_trajectory`

`static bool first` and `static double last_ts` become ordinary locals, so that a second run in the same GUI session does not inherit the previous run's `last_ts`.

### D7. Reproducibility: initialised NDT normals, single-threaded by default

Two runs of the pre-change binary on the same input differ by up to about 100 mm and 12°. Two causes:
- `NDT::Bucket::normal_vector` (`core/include/ndt.h`) is an uninitialised Eigen vector. `update_rgd` sets it only when a bucket reaches its 3rd point, but the viewpoint check in `optimize_lidar_odometry` reads it for every bucket (`ablation_study_use_view_point_and_normal_vectors`). So 1–2 point buckets were accepted or rejected depending on leftover memory. New buckets now get `normal_vector = 0`, so they always pass the check. The fix is in the app (`update_rgd*`), not the core header, to avoid a core rebuild.
- `useMultithread` sums the per-point normal-equation terms in parallel, so the float summation order varies. The parallel paths stay in the code for a later rework, but `useMultithread` defaults to false.

Consequence for regression checks: a byte comparison against the pre-change binary is impossible, because that binary is not reproducible. The regression criterion becomes metric equivalence within its run-to-run spread, plus byte-identical repeat runs of the new binary.
Remaining hazard: `real_time_threshold_seconds` stops a window's iterations on wall-clock time. If a window ever exceeds it, results depend on machine speed.

## Risks / Trade-offs

- [Feature-poor stretches let the bias absorb registration error] → random-walk prior (`q`), `bias_max` clamp, and per-window `P` reported so that bad stretches are visible.
- [Tight rotation prior makes the solution trust the IMU more, so an unmodelled IMU problem (timestamps, saturation) propagates] → keep the repaired-timestamp requirement explicit, and keep an escape hatch via the switches.
- [Small-angle bias Jacobian breaks down at large `b·dt`] → `b·dt` is below 0.05° per edge for any realistic bias (20 °/s × 2 ms). Negligible.
- [Dense `AtPAndt` grows to `(6n+3)²`] → about 0.3% more memory. Negligible.
- [Tail re-propagation discards LiDAR information in the last `m` nodes] → those nodes have about 0–20 points each today. `m` is tunable, and `m = 0` disables it.
- [The ARW defaults come from moving data] → sample-difference MAD is robust to motion at 893 Hz. They will be confirmed with the static recording (Open Questions).

## Migration Plan

- Defaults turn the new behaviour on. `settings/lidar-vertical.toml` and `app/default_settings.toml` get the new keys. TOML files without the keys load with the defaults (the existing TomlIO behaviour).
- Rollback: set `use_gyro_bias_estimation = false`, `use_time_scaled_rotation_sigma = false`, `handoff_tail_nodes = 0`. This reproduces the current code path.

## Tuning result (datasets 42/43 and static recording 43/static)

- Defaults: k = 4, bias random walk q = 0.05 °/s/√s, bias estimation on, ARW 0.028 / 0.042 / 0.057 °/√s (measured on the static recording).
- The map-sharpness gain over the old constant sigma comes from the time-scaled sigma (k = 4). Bias estimation is map-neutral (3 cm cells within about ±1 %) and improves agreement between the LiDAR heading and the gyro.
- Static recording (17 s at rest, 882 Hz): gyro mean x +0.033, y −0.008, z −0.612 °/s; 1 s block means stable to about 0.08 °/s.
- On dataset 43 (moving), the estimate with q = 0.05 averages z = −0.60 °/s, matching the static −0.61. The earlier "targets" (−1.4 on 43, −6.3 on 42), taken from the whole-run LiDAR-vs-gyro gap, were wrong: that gap mixes registration noise (±3–5 °/s per window) and scale effects with the bias. They had pushed q to 0.5, which made the estimate about ten times noisier (window-to-window change 1.5 °/s on the static recording vs 0.16 °/s with q = 0.05) without improving its mean.
- On the static recording itself, the estimate averages z = −0.32 °/s: within the ±0.5 °/s criterion, but low. With no motion, the LiDAR constrains the heading only through the static scene, and the per-edge rotation freedom absorbs part of the drift (see D1).

## Open Questions

- Whether `m = 30` is right for other environments. It is tunable and validated on 42/43.
- dLidar's fixed IMU timestamps are strictly increasing, but samples within a batch are still stamped about 0.02 ms apart, followed by a jump of about 1 ms (true spacing about 1.13 ms). This does not bias the heading, and the static test gave the same result with evenly re-stamped timestamps. It does push most edges to `rotation_sigma_floor_deg`. To be fixed in dLidar, not here.
