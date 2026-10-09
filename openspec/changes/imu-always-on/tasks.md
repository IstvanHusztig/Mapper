# Tasks

**Checks on every code commit:**
- the Release build of `dMapper` and `lidar_odometry_tests` passes, and `ctest` passes;
- with `imu_always_on` off, `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`.

**Tests are kept short** (the user's wish): the full 44 with the precise build only, compared with the verified `raw data/44_baseline` (`tools/trajectory_seams.py`), the loop-closure corrections, `eval_odometry.py` and the time. Both builds are compared only once, for the candidate to ship.

Close the GUI before building. Never run two jobs on the same input folder at once. Run background jobs from copies of the binaries.

## 1. Diagnose

- [x] 1.0 The IMU's own noise, from the static recording `raw data/43/static` (17.2 s at rest, 15,160 samples, 882 Hz average; the timestamps come in bursts, so the rate is samples per duration). It is the recording `gyro-bias-estimation` measured the gyro on (ARW 0.028 / 0.042 / 0.057 °/√s, the shipped values).

  Accelerometer, white noise from sample differences (MAD, robust):

  | Axis | Per sample | Noise density | Spread of 1 s means |
  |---|---|---|---|
  | x | 0.0226 m/s² | 0.00107 m/s²/√Hz (110 µg/√Hz) | 0.064 m/s² |
  | y | 0.0269 m/s² | 0.00128 m/s²/√Hz (131 µg/√Hz) | 0.007 m/s² |
  | z | 0.0277 m/s² | 0.00132 m/s²/√Hz (135 µg/√Hz) | 0.005 m/s² |

  - **Defaults:** `imu_accel_noise_density_mps2_sqrt_hz` = 0.0013, the largest axis.
  - **Bias random walk:** stays at the existing 0.05 m/s²/√s, provisional, because 17 s is too short for bias instability. The larger x drift may be a touch during the recording.
  - **Mistake recorded:** an earlier draft of this task asked for a new static recording, although this one existed and had been used before (2026-09-30).

- [ ] 1.1 Behind `log_window_diagnostics`, log per window how the accelerometer fits the LiDAR trajectory in windows where no direction is degenerate (ratio above 0.3). For each anchor triple, compare the trajectory's second difference with R·f − g, and fit the residual of the window as offset + tilt leakage (the part explained by a small attitude error, [g]× δθ) + rest. Log the three sizes.
  - One full-44 run (precise). Report their distributions, the size of the tilt that would explain the leakage, and the noise level after both are removed.
  - Decide from it: the order of tasks 2.1 and 2.2, and `imu_accel_floor_sigma_mps2`. Record here. Results unchanged; `44_short` IDENTICAL. Commit.

## 2. Build

- [ ] 2.1 Rotation Jacobian in the accelerometer term (design, decision 2): the term depends on the anchor nodes' rotations through R·f − g, mapped to Tait-Bryan increments numerically.
  - Behind `imu_always_on` only; the existing term stays bit-identical with it off.
  - Test: a stationary synthetic window in the room, started 1 deg tilted, reduces the tilt and keeps the positions within a few millimetres. Commit.
- [ ] 2.2 Velocity and accelerometer bias as carried states (design, decision 3).
  - The start velocity becomes an unknown with a prior. The final velocity and its covariance (a marginal of the window's normal matrix) and the bias are carried to the next window.
  - Noise from `imu_accel_noise_density_mps2_sqrt_hz` and `imu_accel_floor_sigma_mps2` (decision 4).
  - With `imu_always_on` the term is active in every window after the first, without the degenerate gate, the 15 s limit, the loosened translation sigmas and the gravity calibration (decision 5).
  - Settings in `TomlIO` and the shipped TOMLs (off), golden files in their own commit.
  - Tests:
    - a synthetic corridor walk at 1 m/s whose axis the LiDAR can't see keeps about 0.4 m per window with the IMU showing no acceleration;
    - a stationary window doesn't drift;
    - the existing gyro-bias tests are unchanged.
  - `ctest`; `44_short` IDENTICAL with it off. Commit.

## 3. Judge

- [ ] 3.1 Full 44, precise build, with `imu_always_on` and a few values of the floor sigma around the task-1 measurement, each reported with:
  - the difference from `44_baseline`;
  - the loop-closure corrections and the number of loops;
  - `eval_odometry.py` and the time.

  If a variant is better (smaller loop-closure corrections at the same or better metrics, no new deviation from the baseline beyond what the loop corrections explain), run it once with the `/fp:fast` build as well (the difference between the builds must be at most today's 0.31 m), and the user checks the cloud. **If no variant helps, remove the code again** (restore the files, `44_short` and the full 44 identical with the manifests), record the measurements, and archive as rejected. Commit.

## 4. Ship (only if adopted)

- [ ] 4.1 Shipped settings, new baselines (old folders renamed with a date suffix, both manifests rewritten), and docs:
  - `docs/degeneracy-information.html` and the Hungarian copy (the gate replaced);
  - the settings tables of `docs/index.html` and `docs/hu/index.html`;
  - `CLAUDE.md` (the IMU paragraph).

  Sync the delta spec, archive, push. Memory notes.
