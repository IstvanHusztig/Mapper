# Design

> 2026-11-25: layer 2 of honest-inertial-carry (`imu_carry_joint_covariance`, the joint path named below) was removed from the code; only the plain path remains.

## Context

`prepare_velocity_state` (`src/application/run_odometry.cpp:136`) takes `mean = carry.velocity_mean` when a carried
velocity is valid and below `max_prior_velocity_mps`, with `gap = window start time − carry.velocity_time_s`. The
covariance goes through `velocity_prior_cov_over_gap` (or `joint_prior_cov_over_gap` with the joint switch), which
propagates it over `gap` with the accelerometer noise (density, floor); the mean gets no counterpart. At 500 Hz the gap
is one sample interval (2 ms), so the lag a·gap is 2 mm/s per m/s². The audit also noted that the last node's
accelerometer sample is used by no term (`accelerometer_term.cpp`): that sample is exactly the acceleration across this
gap, which suggests the source of the advance.

## Goals / Non-Goals

**Goals:**
- The mean equals the true start velocity under constant acceleration (K3, 0.1 mm/s).
- Nothing changes at rest, at constant velocity, or without a carried velocity.

**Non-Goals:**
- No change to the covariance, to the offset fold or to which windows carry a velocity.
- No tail re-propagation (F2b), no first-pose change (step 6).

## Decisions

1. **Source of the acceleration (open, to settle before coding):** (a) the accelerometer sample across the gap (the
   unused last-node sample of the previous window, or the first of this one) in world frame minus gravity and the carried
   offset; (b) the mean acceleration over the previous window's end span. Recommendation (a): it is the physical term the
   gap integrates, and its noise over 2 ms (about 0.06 mm/s) is far below the 2 to 4 mm/s being corrected. The floor and
   a body-fixed bias are the risk to check: a 0.1 m/s² misfit is 0.2 mm/s, still small.
2. **Where:** inside `prepare_velocity_state`, so the mean and the covariance are prepared in one place and the joint path
   shares it.
3. **Guard:** the advance is skipped when the gap is not positive or exceeds a few sample intervals (a data gap), where an
   acceleration sample says nothing about the interval.
4. **Order of work:** red marker first (exists), implementation, then replays: the per-term table of a few windows and the
   normalised start-velocity move (which should shrink slightly), then `44_short`, 44 and the Livox runs as a measurement
   (not bit-identical by design), the sweep contract on Livox 4 if the maps differ, and the user's look at the maps before
   baselines are re-recorded.

## Risks / Trade-offs

- A change of about 0.4 sigma in the prior mean in every accelerating window may move maps: the whole-run effect is not
  known and must be measured, not assumed small. The known fragility of Livox 4 (a 5 % change moved results by metres)
  means one run cannot judge it.
- If the world-frame acceleration at the hand-over is biased (gravity leak from the tilt, F5), the advance copies that
  bias into the prior. At 2 ms it is bounded by gap × bias.
- Precision comes before speed: no cost to report beyond one vector operation per window.
