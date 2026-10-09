# Design

## Context

- **A window** is 360 nodes, 0.4 s at about 900 Hz. Each node is a pose with Tait-Bryan parameters. Its unknowns are the node poses, the gyro bias (3) and, when the accelerometer term is active, an acceleration offset (3). These are `ib` and `io` in `optimize_lidar_odometry`.
- **The accelerometer term** (`scan_to_map.cpp`) is built per triple of anchor nodes about `imu_acceleration_span_s` apart. Its residual is the second difference of the positions minus (mean of R·f − g − offset)·τ, with the rotations held fixed. It adds a tie of the first velocity to the previous window's final velocity.
- **Without the term**, a velocity-continuity prior acts between consecutive nodes (sigma `lidar_odometry_motion_model_acc_1_sigma_mps2`), plus the same tie to the previous velocity.
- **The previous velocity** is estimated from the previous window's last 20 nodes (`estimate_previous_velocity`) and used as a fixed value, with no covariance.
- **The gyro bias is already a carried state** with a covariance: its prior is grown by a random walk (`apply_gyro_bias_prior`) and estimated in the window. That pattern is reused.
- **Gravity** is calibrated outside degenerate stretches from the LiDAR trajectory (`observed_acceleration_offset`), and the estimated offset is folded into the gravity vector.

## Goals / Non-Goals

**Goals:**
- The accelerometer acts in every window, weighted by its real noise, so that blind stretches are bridged by the IMU without any switch.
- A tilt error is corrected in the attitude, not in the positions.
- Velocity and accelerometer bias are states with covariances, carried from window to window.
- Bit-identical behaviour with the setting off.

**Non-Goals:**
- Full preintegration factors between keyframes; the per-node structure stays.
- A separate gravity state. The bias absorbs a constant gravity error, as the offset does today.
- Changing the loop closure or the consistency pass.

## Decisions

### 1. Diagnose before building (task 1)
In well-constrained windows (no direction flagged), compare the accelerometer with the LiDAR trajectory's acceleration. Fit the residual as offset + tilt leakage, (R_err·g, linear in the attitude error) + noise, and log it. If tilt leakage dominates, decision 2 is the main lever. If a drifting offset dominates, decision 3 is. The noise level sets the sigma of decision 4.

### 2. Rotation Jacobian in the accelerometer term
The term's measurement depends on R_k through a_k = R_k f_k − g. Linearized: δa_k = −[R_k f_k]× δθ_k, with δθ_k the world rotation increment of node k. That is mapped to the node's Tait-Bryan increments with the same numerical E_k as in the removed experiment (finite differences at the node's attitude, independent of the angle convention).

The term then constrains rotations and positions together. A tilt that leaks gravity is cheaper to fix in the attitude, which the gyro and the LiDAR also constrain, than in hundreds of positions. This widens the band of the normal matrix: rotations of the anchor nodes enter the same rows. `pose_band_offset` already covers the anchors' 6-parameter blocks.

### 3. Velocity and bias as carried states
- **Velocity:** the start velocity v0 is no longer a fixed value from the previous poses, but an unknown with a Gaussian prior, as the gyro bias has:
  - mean: the previous window's final velocity;
  - covariance: the previous window's velocity covariance plus σ_a²·gap.
  - The previous window's final velocity and its covariance come from that window's solution, as a marginal of the normal matrix, as for the gyro and acceleration-offset covariances today (`covariance_solve`).
- **Accelerometer bias:** the existing offset `e`, carried with its covariance and grown by `imu_acceleration_offset_rw_mps2_sqrt_s`, as it is while the term is active today. It stays continuous instead of being reset per degenerate stretch.

### 4. Noise from the sensor
- **The weight of a term** over the span τ is 1 / (σ_eff² · τ), with σ_eff² = σ_d² / τ + σ_floor².
  - σ_d: the accelerometer noise density, measured on the static recording `raw data/43/static` (task 1.0): 0.0011 / 0.0013 / 0.0013 m/s²/√Hz for x / y / z;
  - σ_floor: an assumed floor for walking vibration and unmodelled lever-arm effects, set from the task-1 measurement, not fitted to single windows.
- **Settings:**
  - `imu_always_on` (bool, default false);
  - `imu_accel_noise_density_mps2_sqrt_hz`;
  - `imu_accel_floor_sigma_mps2`;
  - `imu_velocity_initial_sigma_mps`.
- The existing offset settings are reused.

### 5. Always on replaces the gate
With `imu_always_on`, `prepare_acc_term` enables the term in every window from the second on, and keeps the normal translation sigmas: the motion model is no longer loosened. It skips the degenerate gate, the 15 s limit and the gravity calibration. The offset is estimated continuously.

## Risks / Trade-offs

- **Gravity and bias are not separable without rotation dynamics.** Long straight walks leave them correlated, so the bias random walk must stay small. Measured in task 1.
- **The wider coupling of rotations and positions changes the conditioning** of the window problem. Watch the iterations and the delta in `log_window_diagnostics`.
- **A consumer IMU bridges only seconds to tens of seconds** of a blind passage. The goal is a soft, continuous contribution, not a replacement for the LiDAR.
- **One test recording:** the cave test the user plans after the next change is the real check. Settings must not be fitted closely to 44.
- **Cost:** the term already exists. Its rotation Jacobian adds work per anchor triple (about 36 per window), negligible next to the NDT loops.
