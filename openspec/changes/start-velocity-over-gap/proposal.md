# Proposal

## Why

The window loop hands each window's end velocity to the next window as the mean of its start-velocity prior. Between the
two windows lies a gap (2 ms in the synthetic fixtures), over which the sensor accelerates. `prepare_velocity_state`
grows the prior's *covariance* over that gap (F P Fᵀ + Q, exact to 1e-16 in the harness) but leaves its *mean* at the
carried end velocity, so the mean lags the true start velocity by a·gap. Measured in `consistency-harness`
(`tests/unit_cases_inertial_test.cpp`, marker K3): -2.000 mm/s at 1 m/s² and -4.000 mm/s at 2 m/s², zero at rest. The
claimed prior sigma is about 5 mm/s, so the lag is about 0.4 sigma, one-sided, in every accelerating window. This is
finding F2a of the audit, and step 3 of the order the user decided (`consistency-harness` task 8.2).

## What Changes

- The start-velocity prior's mean is advanced over the gap by the acceleration at the hand-over, so it equals the true
  start velocity under constant acceleration (K3 oracle: within 0.1 mm/s).
- The covariance algebra is unchanged (it is already exact); with `imu_carry_joint_covariance` the joint path moves the
  velocity by the offset over the gap already and gets the same advance for the mean.
- The red marker `unit case K3 marker` loses `doctest::should_fail` and becomes a passing test.
- Numbers change (the prior's mean moves by up to a few mm/s per accelerating window), so the golden baselines are
  measured first and re-recorded only after the user's decision.

## Capabilities

### New Capabilities

### Modified Capabilities
- `processing-pipeline`: the start-velocity prior of a window is the carried end velocity advanced over the gap.

## Impact

- `src/application/run_odometry.cpp::prepare_velocity_state` (and the acceleration it reads); inertial region
  (`docs/regions/inertial.md`, F2a).
- Tests: `tests/unit_cases_inertial_test.cpp` (K3 marker flips); the synthetic loop's end-velocity readings should be
  re-measured.
- Results: every run with `imu_always_on` changes slightly. `44_short`, 44 and the Livox runs are no longer bit-identical
  to their manifests; a before/after on them, the sweep contract (`verify-robustness`) and the user's map look come before
  any baseline change.
- Not in scope: the tail re-propagation (F2b, its own change `tail-repropagation-acceleration`), the fixed first pose
  (step 6), the last node's unused accelerometer sample (to be settled in the design, see there).
