# Proposal

## Why

The first seconds of a recording are mapped with a wrong heading, and whatever is seen only then stays turned. On
recording 44 the room scanned during the start-up stand-still (the lower-left flat's wall the user marked) sits off the
house plan: `plan_fit.py` turns the start room (0–15 s) by +0.26 … +1.29° against the whole floor in all seven results
of `map-precision-leaks` (always the same sign; odometry only +1.29°, loop closure reduces it, consistency and the
gyro term don't change it).

Cause, measured: `calculate_trajectory` estimates the gyro bias from the first 1000 IMU samples only if their total
variance is below a fixed 5.0 (°/s)². The L2's gyro noise alone is at that limit (5.0–5.14 on the static recording,
sensor lying still), a hand-held stand-still is 12–14, so 44's estimate was rejected ("Excessive movement detected
(12.079). Bias set to 0.") and no bias was removed. The odometry's own estimate starts at 0 with a 10 °/s sigma and
needs three windows to settle (z: 0 → −3.28 → −2.55 → −0.80 … −1.0 °/s): the first ~1.2 s run with a bias error of
1–2.3 °/s, about 1–2° of false yaw. Caves start hand-held too, so every survey's start is affected.

## What Changes

- **Start-up stand-still test from the sensor's own noise**: the bias is accepted when the stretch shows no real
  motion, judged by tests that a hand-held stand-still passes and a moving start fails (median of sub-blocks agreeing,
  gravity direction steady, rate scatter within a multiple of the sensor's measured noise), instead of the fixed
  variance limit. Its length and limits become settings. The bias is still the median and still removed from every
  sample, the first seconds included.
- **The odometry's bias estimate starts from it**: when the start-up bias was accepted, the online estimate starts at
  zero residual with a small sigma (new setting) instead of 10 °/s, so the first windows don't overshoot.
- A log line that says which test passed or failed, with its numbers.

## Capabilities

### New Capabilities

### Modified Capabilities
- `lidar-odometry-motion-model`: a requirement for the start-up gyro bias (when it is accepted, what it is applied to,
  how the online estimate starts from it).

## Impact

- `src/application/lidar_odometry.cpp` (`calculate_trajectory`'s start-up bias), `src/application/run_odometry.cpp`
  (initial bias sigma), `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`, settings files, golden TOML
  files; `tests/orientation_test.cpp`.
- Changes the numbers of 44 (its estimate is now accepted): new 44 and 44_short baselines if accepted.
