# Design

## Context

See proposal.md (Why). `calculate_trajectory` (`src/application/lidar_odometry.cpp:119-182`) takes the first 1000
samples, rejects them when the summed per-axis variance exceeds 5.0 (°/s)², else uses the per-axis median as the bias
and subtracts it from every sample before Fusion's AHRS. The windows' `imu_angles` come from that AHRS, so an accepted
bias is already in them; the odometry's online estimate (`apply_gyro_bias_prior`, `run_odometry.cpp:1059-1064`,
initial sigma `gyro_bias_initial_sigma_deg_s` = 10 via `bias_sigma0_rad`, `:95`) then estimates only the remaining bias.

Measured: static recording (43/static, lying still) total variance 5.0–5.14 per 1000 samples, medians stable to
~0.2 °/s (z −0.58 … −0.75); 44's first 2.2 s 12.1 / 13.8 (hand-held), medians z −0.64 / −1.44 °/s; 44 at 2.2–3.8 s
42.6 (moving off). Measured ARW 0.028 / 0.042 / 0.057 °/√s, IMU ~890 Hz → per-sample sigma 0.8 / 1.25 / 1.7 °/s.

## Goals / Non-Goals

**Goals:** accept a hand-held stand-still, reject a moving start, and start the online estimate from it.

**Non-Goals:** a backward pass that re-optimizes the first windows with the later estimate (larger change; this fix
should remove most of the start error; revisit if the start room stays turned); the gyro fault of 44 at 253 s (C38).

## Decisions

1. **Tests instead of a variance limit.** A hand-held stand-still is noisy but zero-mean; motion moves the median or the
   gravity direction. So: (a) the four quarters' medians agree within 3.0 °/s per axis (a turn shifts them); (b) the
   gravity direction turns by at most 2° between the first and last quarter (a tilt); (c) each axis' scatter is within
   3× the sensor's per-sample sigma (gross shaking). Measured on the first second: hand-held starts (44, s1, s2, 45f)
   quarter spread ≤ 1.74 °/s, tilt ≤ 0.68°, scatter ≤ 1.91×; 44 moving off at 2.2–3.2 s 6.25 °/s, 3.07°, 4.88× (fails all
   three); lying still (static) 0.13–0.18 °/s, 0.26°. Hand-held tremor shifts the quarters by up to ~1.7 °/s, so a
   tighter spread limit would reject 44 again. Defaults chosen from these numbers;
   all settings in `[madgwick_filter]` next to `use_remove_imu_bias_from_first_stationary_scan`.
2. **Length in seconds**, `imu_startup_bias_window_s` = 1.0 (≈ the old 1000 samples at 890 Hz), from the timestamps,
   so other IMU rates work.
3. **A steady yaw rate at the start can't be told from a bias by the IMU alone** (all quarters agree, gravity steady).
   The bias clamp `gyro_bias_max_deg_s` caps it, and the online estimate (sigma below) lets the LiDAR correct it. The
   log shows the estimate so a wrong start is visible.
4. **Online estimate after an accepted start-up bias**: sigma `gyro_bias_startup_sigma_deg_s` = 0.5 °/s (the spread of
   the static recording's medians plus margin) instead of 10, so window 1 can't jump to −3.3 °/s on a 0.4 s window.
   Rejected start-up bias: unchanged (10).
5. **Pure function for the test** (`startup_gyro_bias(samples, rate_noise, options) → {accepted, bias, reason}`) in
   the application layer next to `calculate_trajectory`, so it is unit-tested on synthetic samples without Fusion.

## Risks / Trade-offs

- [A slow constant turn at the start is accepted as bias] → decision 3; the clamp and the small-but-not-zero online
  sigma; logged.
- [44's numbers change everywhere] → judged by the start room's angle on the plan and `map_quality.py` on s0/s1/s2
  beyond their spread; new baselines only after the user's yes.
- [Thresholds tuned on one sensor] → they scale with the measured ARW (settings), and the tests are about motion, not
  about this building.
