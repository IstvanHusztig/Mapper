# Proposal

## Why

dMapper is meant for surveying caves hundreds of metres long, and a cave has many passages where the LiDAR cannot see motion along the axis. In such a stretch something other than the LiDAR has to say how far the scanner moved. Today two things do it, and both are weak.

**A velocity-continuity prior.** The window keeps roughly the previous window's final velocity (white-noise acceleration, 1 m/s² sigma).

**An accelerometer term that is switched on only after a window the LiDAR found blind** (`imu_acceleration_only_when_degenerate`, information ratio below 0.3, at most 15 s). That on/off switch is what tipped on rounding and drew the ghost wall of recording 44 (change `odometry-stall-fix`). With the switch on more often, the term drifted by metres:
- in about 28 % of the windows with exact cell statistics;
- in a third of the windows with a continuous weight.

The term itself has two weaknesses:
- **It treats the node rotations as fixed** (`scan_to_map.cpp`: "the node rotations are taken as fixed here"). The gravity it subtracts is therefore only as good as the attitude: a 1° tilt error leaves 0.17 m/s² of gravity in the measured acceleration, as large as walking acceleration (`docs/degeneracy-information.html`). The optimizer can only explain that leak by moving the positions.
- **The velocity is not a state.** Each window re-derives it from the previous window's poses, with no uncertainty carried over.

The LiDAR-inertial systems that hold up in tunnels and caves keep the IMU in the solution all the time, with velocity and accelerometer bias as estimated states and noise taken from the IMU, not switched on and off. This is the literature survey of `odometry-stall-fix` (`openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md`). The figures below are what the papers report, not our measurements.
- **Wildcat**, Ramezani et al. 2022, CSIRO, the family used for handheld cave mapping: 0.34 to 0.42 m in tunnels, against 0.92 to 1.69 m for LIO-SAM.
- LIO-SAM, FAST-LIO2, CT-ICP and Super Odometry work the same way.

The two other improvements tried in the meantime were measured and dropped (changes `odometry-stall-fix` and `hessian-degeneracy-detection`). This is the remaining structural step, next to robust loop-closure gating.

## What Changes

- **Measure first:** why the accelerometer term drifts when it is on often. Per window, compare the measured acceleration with the second derivative of the LiDAR trajectory in well-constrained stretches. Split the misfit into gravity leakage from tilt (correlated with the attitude error), an offset (bias), and noise. Report it on the full 44. This decides the weights and the order of the following steps.
- **Rotation in the accelerometer term:** the gravity-compensated acceleration R·f − g depends on each node's rotation. The term gets its Jacobian with respect to the node rotations, so a tilt error is corrected in the attitude, where the gyro and the LiDAR also act, instead of being absorbed by the positions.
- **Velocity and accelerometer bias as carried states:**
  - the window's start velocity becomes an unknown with a prior: the previous window's estimate and its covariance, grown by the acceleration noise over the gap;
  - the accelerometer bias, the existing acceleration offset, is carried with its covariance like the gyro bias (already done) and grows by a random walk;
  - the window's final velocity and its covariance are handed to the next window.
- **IMU noise from the sensor, not from tuning:** the accelerometer sigma per term from a noise density and the span (σ_a / √Δt), plus a floor for walking vibration.
  - The sensor model is not known: the LAZ header and the CSV don't name it.
  - The IMU runs at about 892 Hz, so it isn't the Livox Mid-360's internal 200 Hz IMU.
  - The constants are therefore measured from the user's own sensor: noise density and bias instability (Allan variance) from a static recording of a few minutes, the scanner standing still.
  - Until there is one, the upper bound from recording 44 is used. Its stillest second, hand-held at t ≈ 1.5 s, gives an accelerometer spread of about 0.02 g per sample, about 0.009 m/s²/√Hz, tremor included.
- **Always on:** with the new setting the accelerometer term is active in every window. The degenerate-stretch gate (`imu_acceleration_only_when_degenerate`, the 15 s limit, the loosened translation sigma) is then not used. The gravity calibration from the LiDAR trajectory is replaced by the estimated bias.
- **Judged on the full 44** (short test, precise build only, until the final check with both builds):
  - against the verified `44_baseline`;
  - the loop-closure corrections, as a drift measure: less correction means less drift;
  - `eval_odometry.py`;
  - the time cost;
  - and, if adopted, your visual check for ghost walls.
- New baselines and docs if the shipped settings change. If it doesn't help, the code is removed again, following the no-dead-code rule.

**Not in this change:**
- full IMU preintegration with rotation, velocity and position deltas between keyframes (the per-node structure stays);
- loop-closure gating (the next change);
- removing the three switched-off options of `odometry-stall-fix` (separate, with your agreement).

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: a new requirement, "Accelerometer always in the solution", alongside the gyro-bias requirements.

## Impact

- **Code:**
  - `src/domain/scan_to_map.cpp`: the accelerometer term, its rotation Jacobian, the velocity unknowns;
  - `src/domain/optimizer_options.h`;
  - `src/application/run_odometry.cpp`: carrying velocity and bias with covariances, as the gyro bias is carried today;
  - `src/domain/window.h`, if the velocity state is stored per window;
  - settings in `odometry_config.h`, `toml_io.h` and the shipped TOMLs (golden files regenerated in their own commit).
- **Tests:** synthetic windows in `tests/scan_to_map_test.cpp`:
  - a corridor walk at constant velocity with a blind axis must keep its speed from the IMU;
  - a tilted start must be corrected in the attitude, not the position;
  - a stationary window must not drift;
  - the gyro-bias tests must stay as they are.
- **Results:** change when the setting is on (new baselines if adopted); bit-identical with it off.
- **Debug build:** Release only, unless a task adds configuration-dependent code (`CLAUDE.md` rule).
