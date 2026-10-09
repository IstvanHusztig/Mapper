# Why the research best practices did not help (2026-10-09)

The user's question after task 3.1's first runs: why do methods that work in the literature (LIO-SAM, FAST-LIO2, Wildcat, probabilistic degeneracy detection) make dMapper worse on recording 44?

## 1. Short answer

**The Unitree L2 recordings' IMU timestamps are 60–80 ms behind the LiDAR's (recording 44; 20–40 ms in recording 43).** The literature assumes a hardware-synchronized IMU (as on the Livox and Ouster sensors those systems were built on) or a calibrated time offset. At walking frequency (1.2–2 Hz), 70 ms is 30–50° of phase, so the accelerometer and the LiDAR disagree by about two thirds of the walking acceleration. Every method that leans more on the IMU at sub-second scale fights the LiDAR. The shipped pipeline happens to be immune: its translation prior is constant velocity (no IMU), and its rotations are dominated by the LiDAR. The one research method that worked, the degeneracy projection (Zhang 2016), doesn't increase IMU reliance.

Three structural reasons make it worse, and would matter even with synchronized clocks:
- a window's start pose is frozen (fix-origin prior), so no later evidence can correct it;
- the offset estimated in a 0.4 s window is folded into gravity at once, so it chases short-term misfit;
- the HDMapping motion model is a stiff prior around a guess (decision 7).

## 2. What was tried

| Change | Research basis | Effect on the IMU's role | Result on 44 |
|---|---|---|---|
| Degeneracy projection (`odometry-stall-fix`, shipped) | Zhang 2016, solution remapping | none: removes LiDAR information along weak directions; the constant-velocity prior decides there | ghost wall gone; builds 1.03 → 0.31 m apart |
| Continuous accelerometer weight (`odometry-stall-fix` variant) | X-ICP / soft weighting | accelerometer active more often | 4.4 m off |
| Exact NDT cell statistics with the accelerometer term | — | accelerometer active in ~28 % of windows | 2.8–8 m off |
| Hessian signal-to-noise degeneracy test (`hessian-degeneracy-detection`) | Hatleskog & Alexis 2025 | rotations handed to the gyro in 139 windows, translations to the motion model in 110 | builds 7.45 m apart, 6–10 m from the baseline, 3/8 loops; heading closer to the gyro |
| … rotation only | same | rotations handed to the gyro | 8.29 m off, 11 loops instead of 21; heading closer to the gyro |
| IMU always on, v1 (bias random walk 0.05) | LIO-SAM, FAST-LIO2, Wildcat | accelerometer in every window | 8.8 m off, 12 loops, largest correction 1.69 m (0.80 shipped), route 572 m (550) |
| … v2 (0.01, Unitree value read as a variance) | same | same | 1.2 m off, 18 loops, 1.35 m |
| … v3 (0.0001, read as a sigma) | same | same | 22 m off, 10 loops, 5.44 m, route 641 m |

Every failure increased the reliance on the IMU: the accelerometer for translation, or the gyro for rotation. The success didn't.

## 3. Measurements (verified baseline of 44, world-frame trajectory, one node per IMU sample)

Script: `imu_vs_traj.py` (scratch, pure Python). The accelerometer is rotated into the world with the trajectory's rotations; gravity comes from the whole recording.

### 3.1 The IMU is 60–80 ms behind the LiDAR

Three independent estimates, all pointing the same way:
- **Cross-spectrum phase slope** (0.5–4 Hz, coherence > 0.5): x 66, y 74, z 70 ms.
- **Time-domain cross-correlation** (0.25–2 Hz): x 53, y 80, z 84 ms (weak correlation because of the trajectory's high-frequency jitter, 3.4).
- **Prediction test.** Predict the trajectory 0.4 s ahead from the accelerometer with a time shift; the median error is smallest at +70 to +105 ms, and the −70 ms control is worse than no shift:

  | Shift of the accelerometer | −70 ms | 0 | +35 | +70 | +105 | +140 |
  |---|---|---|---|---|---|---|
  | vertical error after 0.4 s | 5.3 cm | 4.0 | 3.5 | 3.2 | 3.1 | 3.9 |
  | horizontal error after 0.4 s | 7.2 cm | 6.4 | 5.9 | 5.7 | 5.8 | 6.2 |

- **Over the recording** (60 s segments, phase slope x / y / z): 0–60 s 74/68/78, 60–120 s 79/68/74, 120–180 s 63/66/67, 180–240 s 63/65/62, 220–280 s 47/44/59 ms. So it's roughly constant, perhaps shrinking slowly.
- **Recording 43** (a short shake test): 17–39 ms, same direction (time-domain correlation 0.81 vertically). The offset differs between recordings, so it's a latency of the recording chain, not a fixed property of the sensor.
- **The IMU stamps themselves are regular:** 99.9 % at 1.12 ms (889.7 Hz; spacing p1–p99 1.110–1.131 ms in 44), and `timestampUnix − timestamp` is exactly constant. They look like one clock; whether the LiDAR points use the same one is the open question for the recorder.

### 3.2 What the misfit is not

- **Not walking motion the trajectory misses** (task 1.1's first reading was wrong): vertically, the trajectory's acceleration is coherent with the accelerometer at 0.3–2 Hz (coherence 0.86–0.94, gain 0.90–0.96). The trajectory follows the walk.
- **Not an accelerometer bias:** with a bias, the smallest random walk would win. Instead v3 (0.0001) is worst, v2 (0.01) best, v1 (0.05) between: a loose offset chases the misfit and corrupts gravity, a frozen one pushes it into the trajectory. The misfit is structural.
- **Not window seams:** the velocity change across 22 ms is the same at every node of a window, seam or not.

### 3.3 What the accelerometer can add at best

Prediction error of the trajectory from p(t₀), v(t₀), median:

| Horizon | constant velocity, horizontal | accelerometer (no shift / +70 ms) | constant velocity, vertical | accelerometer (no shift / +70 ms) |
|---|---|---|---|---|
| 0.4 s | 6.9 cm | 6.4 / 5.7 | 4.4 cm | 4.0 / 3.2 |
| 1 s | 20 cm | 17 / 17 | 10 cm | 10 / 8.0 |
| 2 s | 52 cm | 42 / 42 | 21 cm | 19 / 15 |
| 5 s | 2.30 m | 1.75 / 1.80 | 0.56 m | 0.48 / 0.37 |

A walking person is predicted well by constant velocity over a fraction of a second, so even a synchronized accelerometer adds centimetres there. Its value grows over seconds of blind passage, which is exactly where its errors integrate. The papers' gains come largely from what dMapper already has: one pose per IMU sample (de-skewing) and gyro rotations.

### 3.4 The trajectory has millimetre jitter at 10–50 Hz

The trajectory's acceleration spectrum rises with frequency (about 60 m/s² rms at 15–30 Hz horizontally, the IMU 0.1 m/s²): node-to-node jitter of a few millimetres, the same at every node of a window. A likely source is the L2's non-repetitive pattern: each 1.12 ms node sees a small patch of the scene, so its constraint is anisotropic and changes from node to node. It is below the 2 cm range noise and doesn't explain the failures, but it hides the IMU comparison above 2 Hz.

## 4. Why each method failed

1. **Time synchronization.** Literature LIO either has a shared clock or a calibrated offset (FAST-LIO2 exposes a LiDAR-to-IMU time offset setting; continuous-time calibration tools estimate it). dMapper had neither. With the IMU 70 ms late:
   - **accelerometer:** about 2 sin(π f d) ≈ 0.7 of the walking acceleration disagrees with the LiDAR, 0.5–1 m/s². That is task 1.1's 1.37 m/s² rms misfit and v1's ±0.48 m/s² offset jumps;
   - **gyro:** handing rotation to the gyro (Hessian attenuation) de-skews points with the attitude of 70 ms earlier. Heading then agrees better with the gyro, positions drift by metres.
2. **Weights from a static recording.** The accelerometer noise (0.0013 m/s²/√Hz, `43/static`) and the floor (0.1 m/s²) are right for the sensor at rest, but the timing error acts like about 1 m/s² of noise at walking frequency. The term was trusted about ten times too much.
3. **Window architecture.** FAST-LIO2 (iterated EKF), LIO-SAM (smoothing) and Wildcat (sliding window over seconds) estimate velocity, bias and gravity over long horizons and correct past states. dMapper's 0.4 s windows freeze their start (fix-origin 1e-6), and each window's offset estimate is folded into gravity at once. Misfit at the window scale therefore becomes a gravity error for the next window.
4. **The HDMapping motion model** ties every step to the guess's step with σ 0.1 mm, a velocity prior on every node. It holds a window's mean velocity at the guess's within 4.7 mm/s (decision 7). Before decision 7 it cancelled the accelerometer in blind directions; with it, the guess carries the IMU's errors (including the timing) as a strong prior.
5. **The incumbent is co-adapted.** The shipped settings were tuned on these recordings, so they work around the flaw: constant velocity, strong LiDAR rotation correction, an accelerometer gated to rare windows. Any method that relies more on the IMU loses against that tuning.
6. **For the Hessian test specifically:** NDT's default cell statistics (normals from a cell's first points) aren't the point-to-plane noise model the method assumes, and its near-switch decisions made the two builds 7.45 m apart.

## 5. Causal test: the IMU re-stamped by −80 ms

Copies of 44 with the IMU's `timestamp` and `timestampUnix` minus 80 ms (72 samples before zero dropped; 70 ms and 1 ms as controls), full runs, precise build. No window came near `real_time_threshold_seconds` (slowest 2.8 s), so machine load changed nothing.

| Run (shipped settings unless noted) | Loops / largest correction | Max diff from baseline | Route | 3 cm cells |
|---|---|---|---|---|
| baseline (as recorded) | 21 / 0.80 m | 0 | 550 m | 23,791 |
| IMU re-stamped 1 ms (null test) | 21 / 0.89 m | 0.80 m | 552 m | 23,474 |
| IMU re-stamped 70 ms | 14 / 0.30 m | 4.08 m | 563 m | 23,883 |
| IMU re-stamped 80 ms | 6 / 3.00 m | 4.06 m | 583 m | 24,359 |
| v3 always-on, as recorded | 10 / 5.44 m | 22.0 m | 641 m | 24,478 |
| v3 always-on, IMU re-stamped 80 ms | 18 / 2.09 m | 1.0 m | 554 m | 25,091 |

- **The offset is in the data:** after re-stamping, the residual is 0–9 ms (vertical +5 to +9 ms).
- **The gyro shows it too.** The baseline trajectory's per-window yaw agrees best with the gyro shifted by −80 ms:

  | Gyro shift | 0 | −20 | −40 | −60 | −80 | −100 | −120 | −160 ms |
  |---|---|---|---|---|---|---|---|---|
  | window yaw difference | 1.38 | 1.14 | 0.97 | 0.90 | 0.86 | 0.88 | 0.99 | 1.30° |

- **Not chaos:** a physically meaningless 1 ms change keeps loop closure intact, so single runs on 44 are meaningful at this scale.
- **The accelerometer fails because of the timing:** the same code goes from 22 m off to 1.0 m off when only the timestamps are corrected. On corrected data it beats the shipped settings (18 loops against 6).
- **The shipped settings are tuned around the lag** (the user's hypothesis). They get worse step by step as the timing is corrected (70 ms: 14 loops, 80 ms: 6), while the LiDAR geometry gets cleaner: information ratios rise (windows 153–165: 0.61–0.80 against 0.50–0.64), and the passage at windows 170–182 is no longer degenerate. The candidates are the rotation sigma inflation k = 4 (lets the LiDAR bend a late gyro), the gyro-bias settings (absorbed part of the lag) and the accelerometer gate (sigma 0.5 m/s², about what an 80 ms lag needs).
- **Not done:** on corrected data nothing beats the original baseline yet, and the map sharpness (cells) isn't better. That needs retuning on synchronized data, preferably a new recording from the fixed recorder, checked on a second recording.

## 6. What follows

- Fix the time stamping in the dLidar recorder (prompt in `dlidar-timestamp-prompt.md`), or estimate the offset per recording in dMapper (a pre-pass with this analysis, then the run with corrected IMU times).
- Then re-test `imu_always_on` (branch `imu-always-on`, not merged) and the Hessian rotation-only variant on synchronized data.
- Keep the shipped settings meanwhile.
