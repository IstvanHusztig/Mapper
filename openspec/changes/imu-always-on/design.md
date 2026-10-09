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

### 4. Noise from the sensor (Unitree L2)
- **The weight of a term** over the span τ is 1 / (σ_eff² · τ), with σ_eff² = σ_d² / τ + σ_floor².
  - σ_d: the accelerometer noise density, measured on the static recording `raw data/43/static` (task 1.0): 0.0011 / 0.0013 / 0.0013 m/s²/√Hz for x / y / z. The default is 0.0013, the largest axis.
  - σ_floor: a floor for walking vibration and unmodelled effects, set from the task-1 measurement, not fitted to single windows. Bounds:
    - below: the measured white noise, 0.023 to 0.028 m/s² per sample;
    - above: the data sheet's filter value `acc_cov` 0.1 m/s², which is 3 to 4 times the measured per-sample noise and so already includes a vibration margin.
  - The lever arm (1.7 cm in the settings; the data sheet says co-located) adds at most about 0.02 m/s² at walking rotation rates. It is covered by the floor and not modelled.
- **Bias random walk:** today's `imu_acceleration_offset_rw_mps2_sqrt_s` = 0.05 is a guess. The data sheet gives 0.0001 without units and an actively heated IMU, which both point to a much smaller value. Task 3 tries 0.05, 0.01 and 0.001.
- **Timestamps, not rates:** the data sheet's 500 Hz output doesn't match the recordings (882 to 892 Hz, bursts, gaps such as 607 samples in one second of 44). The span τ and the noise per term come from the timestamps of the anchor nodes. A term whose span is stretched by a gap beyond twice `imu_acceleration_span_s` is dropped.
- **Settings:**
  - `imu_always_on` (bool, default false);
  - `imu_accel_noise_density_mps2_sqrt_hz` (0.0013);
  - `imu_accel_floor_sigma_mps2`;
  - `imu_velocity_initial_sigma_mps`;
  - `imu_zero_velocity_sigma_mps` (decision 6).
- The existing offset settings are reused, with the random walk set by task 3.

### 5. Always on replaces the gate
With `imu_always_on`, `prepare_acc_term` enables the term in every window from the second on, and keeps the normal translation sigmas: the motion model is no longer loosened. It skips the degenerate gate, the 15 s limit and the gravity calibration. The offset is estimated continuously.

### 6. Zero velocity at rest
The data sheet asks for 3 to 5 s at rest at start-up, and a cave surveyor stops often. A window at rest is the one moment the velocity is known exactly, which pins the velocity and, through it, the bias.
- **Detection:** the existing zero-velocity detection (`is_window_stationary` on a 50 ms average, thresholds `zero_velocity_acc_threshold_mps2` and `zero_velocity_gyro_threshold_deg_s`). Today it only sets the initial guess, behind `use_zero_velocity_detection` (off). With `imu_always_on` it runs regardless of that setting.
- **Effect:** the window's velocity unknowns get a zero-velocity measurement with `imu_zero_velocity_sigma_mps`. The LiDAR terms stay, so a wrong detection is outvoted where the geometry is good.
- **First window:** if it is at rest, its velocity prior is zero with `imu_zero_velocity_sigma_mps`, and the bias starts from its mean specific force; otherwise the prior has `imu_velocity_initial_sigma_mps`.
- **Recording 44 has no rest at the start** (gyro up to 0.15 rad/s and a specific-force spread of 0.02 g in the first 3 s, against 0.0025 g at rest), so the change is judged without it there. Task 1.1 counts the windows at rest in 44. The user docs will ask for a rest period at the start of each recording.

## Risks / Trade-offs

- **Gravity and bias are not separable without rotation dynamics.** Long straight walks leave them correlated, so the bias random walk must stay small, as the heated IMU allows. Measured in task 1; rests (decision 6) help separate them.
- **The data sheet's noise values are filter tuning, not measurements.** They are used only as an upper bound; the weights come from `43/static`.
- **The wider coupling of rotations and positions changes the conditioning** of the window problem. Watch the iterations and the delta in `log_window_diagnostics`.
- **A consumer IMU bridges only seconds to tens of seconds** of a blind passage. The goal is a soft, continuous contribution, not a replacement for the LiDAR.
- **One test recording:** the cave test the user plans after the next change is the real check. Settings must not be fitted closely to 44.
- **Cost:** the term already exists. Its rotation Jacobian adds work per anchor triple (about 36 per window), negligible next to the NDT loops.
