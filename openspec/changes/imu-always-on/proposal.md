# Proposal

## Why

dMapper is meant for surveying caves hundreds of metres long, and a cave has many passages where the LiDAR cannot see motion along the axis. In such a stretch something other than the LiDAR has to say how far the scanner moved. Today two things do it, and both are weak.

**A velocity-continuity prior.** The window keeps roughly the previous window's final velocity (white-noise acceleration, 1 m/s² sigma).

**An accelerometer term that is switched on only after a window the LiDAR found blind** (`imu_acceleration_only_when_degenerate`, information ratio below 0.3, at most 15 s). That on/off switch is what tipped on rounding and drew the ghost wall of recording 44 (change `odometry-stall-fix`). With the switch on more often, the term drifted by metres:
- in about 28 % of the windows with exact cell statistics;
- in a third of the windows with a continuous weight.

The term itself has three weaknesses:
- **It treats the node rotations as fixed** (`scan_to_map.cpp`: "the node rotations are taken as fixed here"). The gravity it subtracts is therefore only as good as the attitude: a 1° tilt error leaves 0.17 m/s² of gravity in the measured acceleration, as large as walking acceleration (`docs/degeneracy-information.html`). The optimizer can only explain that leak by moving the positions.
- **The velocity is not a state.** Each window re-derives it from the previous window's poses, with no uncertainty carried over.
- **Its weights are guesses, not the sensor's:** 0.5 m/s² per term and a bias random walk of 0.05 m/s²/√s, both far above what the Unitree L2 shows (below).

The LiDAR-inertial systems that hold up in tunnels and caves keep the IMU in the solution all the time, with velocity and accelerometer bias as estimated states and noise taken from the IMU, not switched on and off. This is the literature survey of `odometry-stall-fix` (`openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md`). The figures below are what the papers report, not our measurements.
- **Wildcat**, Ramezani et al. 2022, CSIRO, the family used for handheld cave mapping: 0.34 to 0.42 m in tunnels, against 0.92 to 1.69 m for LIO-SAM.
- LIO-SAM, FAST-LIO2, CT-ICP and Super Odometry work the same way.

Only the method comes from the literature. Every sensor value in this change is from the user's Unitree L2: its data sheet and its own built-in IMU, measured at rest (below).

The two other improvements tried in the meantime were measured and dropped (changes `odometry-stall-fix` and `hessian-degeneracy-detection`). This is the remaining structural step, next to robust loop-closure gating.

## The sensor: Unitree L2 and its built-in IMU

The user's scanner is a Unitree L2, and the IMU is the 6-axis IMU inside it; no other sensor is assumed. The table is the user's data sheet (2026-10-09) and what each value means here.

| Data sheet | Value | Consequence for this change |
|---|---|---|
| Range accuracy | ≤ 2 cm (resolution 4.5 mm) | Matches the 2 cm point noise used so far. |
| Range | 30 m at 90 % reflectivity, 15 m at 10 %; blind zone 0.05 m | Dark, wet rock reflects little. In large chambers the far walls drop out, so more windows are partly blind and the IMU carries more. |
| Scan | 360° × 90° (96° NEGA), non-repetitive, 64,000 points/s, 5.55 Hz rotation | A 0.4 s window spans about 2.2 rotations, so coverage within a window doesn't limit it. |
| IMU rates | 1000 Hz sampling, 500 Hz output | The recordings carry 882 to 892 samples/s, with timestamps in bursts, and gaps: recording 44 has only 607 samples in its third second. The term must use the timestamps, never a nominal rate. |
| Noise for filters | `acc_cov` 0.1 m/s², `gyr_cov` 0.1 rad/s | Unitree's values for its FAST-LIO / Point-LIO configuration: filter tuning, not a measured noise density. They are 3 to 4 times the measured per-sample white noise (below), so they include a margin for vibration. They bound the floor sigma from above. |
| Bias random walk | `b_acc_cov`, `b_gyr_cov` 0.0001 | Units are not stated (variance or sigma). With the active heating, the bias is meant to stay stable. Read as a variance it is 0.01 m/s²/√s, read as a sigma 0.0001 m/s²/√s: 5 to 500 times smaller than today's 0.05. Both readings are tried (task 3). |
| Heating | Active internal heating for bias control | Supports a slowly varying accelerometer bias, carried as a state over the whole run. |
| Extrinsics | Rotation identity, translation 0 (co-located) | No alignment state is needed. dMapper's settings keep a lever arm of 1.7 cm (`lidar_imu_lever_arm_*`, from the Unitree SDK). Its centripetal effect is about 0.017 m/s² at 1 rad/s, below the noise floor, so the term needs no lever-arm correction. |
| Start-up | 3 to 5 s at rest for gravity alignment and bias | Recording 44 doesn't have one: in its first 3 s the gyro reaches 0.15 rad/s and the specific force spreads by 0.02 g, against 0.0025 g at rest. The change must work without a rest period, and use one when it is there (below). |

**Measured on the L2's built-in IMU at rest**, the user's static recording `raw data/43/static` (17 s, 882 Hz, task 1.0; same device and file format as recording 44), the same recording the gyro ARW was measured on in `gyro-bias-estimation`:
- accelerometer white noise: 0.023 to 0.028 m/s² per sample; noise density x 0.0011, y 0.0013, z 0.0013 m/s²/√Hz (110 / 131 / 135 µg/√Hz);
- gyro: ARW 0.028 / 0.042 / 0.057 °/√s (the shipped values), about 0.03 rad/s per sample;
- spread of the 1 s means: x 0.064, y 0.007, z 0.005 m/s². The 17 s are too short for a bias-instability figure.

## What Changes

- **Measure first:** why the accelerometer term drifts when it is on often. Per window, compare the measured acceleration with the second derivative of the LiDAR trajectory in well-constrained stretches. Split the misfit into gravity leakage from tilt (correlated with the attitude error), an offset (bias), and noise. Also count the IMU gaps and the windows at rest. Report it on the full 44. This decides the weights and the order of the following steps.
- **Rotation in the accelerometer term:** the gravity-compensated acceleration R·f − g depends on each node's rotation. The term gets its Jacobian with respect to the node rotations, so a tilt error is corrected in the attitude, where the gyro and the LiDAR also act, instead of being absorbed by the positions.
- **Velocity and accelerometer bias as carried states:**
  - the window's start velocity becomes an unknown with a prior: the previous window's estimate and its covariance, grown by the acceleration noise over the gap;
  - the accelerometer bias, the existing acceleration offset, is carried with its covariance like the gyro bias (already done) and grows by a small random walk, as the heated IMU allows;
  - the window's final velocity and its covariance are handed to the next window.
- **IMU noise from the sensor, not from tuning:** the accelerometer sigma per term comes from the measured noise density and the span (σ_d / √Δt), plus a floor for walking vibration. The floor and the bias random walk are each tried at a few values between the measurement and the data-sheet filter values, not fitted to one recording.
- **Zero velocity at rest:** when a window is at rest (the existing zero-velocity detection, today used only for the initial guess), the velocity state gets a zero-velocity measurement. This uses the 3 to 5 s start-up rest the data sheet asks for, and every stop of the surveyor. The first window's velocity prior is zero with a tight sigma when the recording starts at rest, and `imu_velocity_initial_sigma_mps` otherwise.
- **Always on:** with the new setting the accelerometer term is active in every window. The degenerate-stretch gate (`imu_acceleration_only_when_degenerate`, the 15 s limit, the loosened translation sigma) is then not used. The gravity calibration from the LiDAR trajectory is replaced by the estimated bias.
- **Judged on the full 44** (short test, precise build only, until the final check with both builds):
  - against the verified `44_baseline`;
  - the loop-closure corrections, as a drift measure: less correction means less drift;
  - `eval_odometry.py`;
  - the time cost;
  - and, if adopted, your visual check for ghost walls.
- New baselines and docs if the shipped settings change, including a note in the user docs to start each cave recording with 3 to 5 s at rest. If it doesn't help, the code is removed again, following the no-dead-code rule.

**Not in this change:**
- full IMU preintegration with rotation, velocity and position deltas between keyframes (the per-node structure stays);
- loop-closure gating (the next change);
- removing the three switched-off options of `odometry-stall-fix` (separate, with your agreement);
- a gyro noise change: the shipped gyro ARW already comes from `43/static`.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: a new requirement, "Accelerometer always in the solution", alongside the gyro-bias requirements.

## Impact

- **Code:**
  - `src/domain/scan_to_map.cpp`: the accelerometer term, its rotation Jacobian, the velocity unknowns, the zero-velocity measurement;
  - `src/domain/optimizer_options.h`;
  - `src/application/run_odometry.cpp`: carrying velocity and bias with covariances, as the gyro bias is carried today; the rest flag of `predict_window_poses` passed to the optimizer;
  - `src/domain/window.h`, if the velocity state is stored per window;
  - settings in `odometry_config.h`, `toml_io.h` and the shipped TOMLs (golden files regenerated in their own commit).
- **Tests:** synthetic windows in `tests/scan_to_map_test.cpp`:
  - a corridor walk at constant velocity with a blind axis must keep its speed from the IMU;
  - a tilted start must be corrected in the attitude, not the position;
  - a window at rest must not drift, and a blind window at rest must stay put;
  - the gyro-bias tests must stay as they are.
- **Results:** change when the setting is on (new baselines if adopted); bit-identical with it off.
- **Debug build:** Release only, unless a task adds configuration-dependent code (`CLAUDE.md` rule).
