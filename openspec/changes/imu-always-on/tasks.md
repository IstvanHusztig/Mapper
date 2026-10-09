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

- [x] 1.1 Behind `log_window_diagnostics`, log per window how the accelerometer fits the LiDAR trajectory in windows where no direction is degenerate (ratio above 0.3). For each anchor triple, compare the trajectory's second difference with R·f − g, and fit the residual of the window as offset + tilt leakage (the part explained by a small attitude error, [g]× δθ) + rest. Log the three sizes.
  - Also log per window the IMU gaps (longest sample spacing, terms that a gap would drop) and whether the zero-velocity detection finds it at rest.
  - One full-44 run (precise). Report their distributions, the size of the tilt that would explain the leakage, the noise level after both are removed (against the bounds of design decision 4: 0.023 to 0.028 m/s² measured, 0.1 m/s² data-sheet filter value), the gaps and the windows at rest.
  - Decide from it: the order of tasks 2.1 and 2.2, `imu_accel_floor_sigma_mps2`, and whether the bias drifts faster than the heated IMU suggests. Record here. Results unchanged; `44_short` IDENTICAL. Commit.

  **Result (2026-10-09, full 44, precise build; results bit-identical with the manifest, only the parameters TOML differs):** `log_accelerometer_fit` logs per window the misfit IMU − trajectory at anchors 50 ms apart, as its mean in the world and body frame, the rms around it, the longest IMU sample spacing and the rest flag. Over the 580 well-constrained windows with the term off:
  - **Offset and tilt are small.** 10 s means of the misfit are mostly below 0.1 m/s² per axis (a few up to 0.25). As a tilt, that is at most 0.6°, typically 0.2 to 0.3°. Neither the world nor the body frame keeps it more constant, so it can't be split into a bias and a tilt from this.
  - **The rest dominates: 1.37 m/s² rms (median) at 50 ms**, 200 times the IMU's own noise at that span (0.006 m/s²) and above the data-sheet filter value (0.1). The per-window means alternate (lag-1 correlation −0.6 vertical): the walking surge at about 2 Hz against 0.4 s windows. Today's trajectory is constant velocity within a window (the translation motion model, see 2.4), so it misses the walk: about ±8 mm at 2 Hz.
  - **IMU gaps:** one window has a sample spacing above 50 ms, none other above 20 ms. **At rest:** no window of 44 (it doesn't start at rest).
  - **Decisions:** 2.2 is the main lever; 2.1 is built with it (the sign and size checked by a test), but the tilt it could correct is small and node 0 of each window is held by the fix-origin prior, so 3.1 measures whether it adds anything. The floor stays at the data-sheet 0.1 m/s² (the measured rest is motion the trajectory misses, not sensor noise); 3.1 also tries a larger floor. The bias random walk is tried smaller, as planned.

## 2. Build

- [x] 2.1 Rotation Jacobian in the accelerometer term (design, decision 2): the term depends on the anchor nodes' rotations through R·f − g, mapped to Tait-Bryan increments numerically.
  - Behind `imu_always_on` only; the existing term stays bit-identical with it off.
  - Test: a stationary synthetic window in the room, started 1 deg tilted, reduces the tilt and keeps the positions within a few millimetres. Commit.
  - **Done differently:** node 0 of a window is held by the fix-origin prior, so a tilt of the whole window can't change inside it. The test lets the roll drift from 0 to 1° over the window, without LiDAR points and with a loose rotation prior (within a window the gyro is far stronger than the accelerometer's tilt information): 1° → 0.06°, positions 0.001 mm.
- [x] 2.2 Velocity and accelerometer bias as carried states (design, decision 3).
  - The start velocity becomes an unknown with a prior. The final velocity and its covariance (a marginal of the window's normal matrix) and the bias are carried to the next window.
  - Noise from `imu_accel_noise_density_mps2_sqrt_hz` and `imu_accel_floor_sigma_mps2` (decision 4), span from the timestamps; terms stretched by a gap beyond twice `imu_acceleration_span_s` dropped.
  - With `imu_always_on` the term is active in every window after the first, without the degenerate gate, the 15 s limit, the loosened translation sigmas and the gravity calibration (decision 5).
  - Settings in `TomlIO` and the shipped TOMLs (off), golden files in their own commit.
  - Tests:
    - a synthetic corridor walk at 1 m/s whose axis the LiDAR can't see keeps about 0.4 m per window with the IMU showing no acceleration;
    - a stationary window doesn't drift;
    - the existing gyro-bias tests are unchanged;
    - a window with an IMU gap drops the stretched terms and is still solved.
  - `ctest`; `44_short` IDENTICAL with it off. Commit.
  - **Measured in the tests:** a walk (2 m/s² surge at 2 Hz) along a blind corridor: end error 80 mm → 0.3 mm, end velocity 1.0021 m/s (true 1.0013); a gap of 60 ms: the stretched terms dropped, 0.8 mm. A constant acceleration over a whole window is taken in part by the offset e (it looks like a bias); a walk's surge is not. The end velocity is tied to the last two anchors (the last node alone isn't in the anchor chain and gave it its noise).
- [x] 2.3 Zero velocity at rest (design, decision 6): with `imu_always_on`, the zero-velocity detection runs in every window, and a window at rest gets a zero-velocity measurement on its velocity unknowns (`imu_zero_velocity_sigma_mps`); a first window at rest starts from zero velocity and its mean specific force.
  - Test: a blind corridor window at rest, with a previous velocity of 0.5 m/s, stays within a few millimetres.
  - `ctest`; `44_short` IDENTICAL with it off. Commit.
  - **Measured:** 1.0 mm with the zero-velocity measurement, 4.9 mm without. A window at rest is also predicted without motion (the zero-velocity prediction is on with `imu_always_on`).
- [x] 2.4 The guess follows the accelerometer (design, decision 7, found while testing 2.2): `integrate_window_translations` predicts the window's translations from the start velocity prior with the term's acceleration model, so the translation motion model agrees with the accelerometer.
  - Test: exact for a constant acceleration with uneven node spacing (`window_steps_test`).
  - `ctest`; `44_short` IDENTICAL with it off (only the parameters TOML differs: the new keys). Commit.

## 3. Judge

- [ ] 3.1 Full 44, precise build, with `imu_always_on`. At most five runs: the floor sigma at the data-sheet bound 0.1 m/s² and larger (task 1), the bias random walk at 0.05 (today) and the two readings of the Unitree L2 value, 0.01 and 0.0001 m/s²/√s, and once without the rotation coupling (2.1) to see whether it adds anything. Each run reported with:
  - the difference from `44_baseline`;
  - the loop-closure corrections and the number of loops;
  - `eval_odometry.py` and the time.

  If a variant is better (smaller loop-closure corrections at the same or better metrics, no new deviation from the baseline beyond what the loop corrections explain), run it once with the `/fp:fast` build as well (the difference between the builds must be at most today's 0.31 m), and the user checks the cloud. **If no variant helps, remove the code again** (restore the files, `44_short` and the full 44 identical with the manifests), record the measurements, and archive as rejected. Commit.

## 4. Ship (only if adopted)

- [ ] 4.1 Shipped settings, new baselines (old folders renamed with a date suffix, both manifests rewritten), and docs:
  - `docs/degeneracy-information.html` and the Hungarian copy (the gate replaced);
  - the settings tables of `docs/index.html` and `docs/hu/index.html`;
  - `CLAUDE.md` (the IMU paragraph);
  - a recording note in `docs/index.html` and `docs/hu/index.html`: start each recording with 3 to 5 s at rest (Unitree L2 data sheet), and stop briefly now and then in long blind passages.

  Sync the delta spec, archive, push. Memory notes.
