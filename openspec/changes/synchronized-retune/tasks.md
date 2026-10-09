# Tasks

**Runs:** full corrected 44, precise Release build, from copies of the binaries. Up to three runs in parallel on copies of the input folder (no window may come near `real_time_threshold_seconds`; checked per run). The copies are deleted at the end. Each run is reported with loops and corrections, route, the difference from `44_baseline`, `eval_odometry.py` and the time.

## 1. Rotation and accelerometer (batch 1)

- [x] 1.1 `rotation_sigma_inflation_k` 2 and 1 (shipped 4), and the accelerometer term off (`use_imu_acceleration_term = false`). Record here.

  **Result (2026-10-09).** On the corrected 44, a single run can't pick settings: the noise of one run is larger than every effect.
  - **Noise test:** each candidate re-run with the IMU shifted by +1 and +2 ms, a physically meaningless change:

    | Setting | Loops | Largest correction | 3 cm cells | Window yaw diff |
    |---|---|---|---|---|
    | shipped (k = 4) | 11 / 21 / 10 | 5.56 / 0.79 / 6.29 m | 24,485 / 24,239 / 24,409 | 0.87 / 0.84 / 0.78° |
    | k = 1 | 20 / 15 / 11 | 0.69 / 3.89 / 5.07 m | 24,464 / 26,684 / 23,957 | 0.78 / 0.75 / 0.75° |
    | accelerometer off | 18 / 11 / 13 | 4.10 / 6.49 / 5.05 m | 21,658 / 24,441 / 24,226 | 0.85 / 0.81 / 0.83° |

  - **Single runs, no shift:** k = 2: 13 loops, 4.89 m; k = 0.5: 14 loops, 8.75 m; k = 0.25: 18 loops, 2.01 m, 27,166 cells; k = 1 with the accelerometer off: 16 loops, 2.98 m, 21,459 cells.
  - **New synchronized recording 45f** (dLidar fix, residual offset x −1.4 / z +1.9 ms): shipped, k = 1, accelerometer off and both are equal within the noise (17–18 loops, 0.59–0.75 m, cells within ±1 %).
  - **Conclusion:** no candidate is better than the shipped settings beyond the noise. k = 1 only agrees slightly better with the gyro (0.75–0.78° against 0.78–0.87°). The two apparent wins, k = 1 on drift and the accelerometer off on sharpness, were single lucky runs.
  - **The real finding:** on the corrected 44, loop closure is chaotic. The largest correction ranges from 0.8 to 6.3 m for 1–2 ms changes, and most runs leave the baseline at the same place, t ≈ 74–78 s (the two-solution place noted in `hessian-degeneracy-detection`). From now on, settings are judged by at least three perturbed runs (median) and on 45f. That place in 44 is the precision problem to look at.

## 2. Follow-up (batch 2)

- [ ] 2.1 From batch 1's best: the accelerometer gate's sigma (0.2 m/s²) or the degenerate translation sigma, and the gyro-bias random walk (0.002 °/s/√s), as batch 1 suggests. Record here.

## 3. Ship

- [ ] 3.1 The chosen settings once with the `/fp:fast` build (the difference between the builds at most a few decimetres, as today).
- [ ] 3.2 New `settings/lidar-vertical.toml`, new baselines (the old folders renamed with a date suffix, manifests rewritten), the settings tables in `docs/index.html` and `docs/hu/index.html`. Archive, push, memory notes.

## 4. Re-check what was decided on the lagging data (after the retune, each its own change or task)

Everything below was measured or rejected on recordings whose IMU lagged the LiDAR by 50 to 80 ms, with settings tuned around that lag. A "worse" there may be "better" now, and a "better" may have been a compensation. Each item is re-run on the retuned settings and the corrected 44 (and on the user's new synchronized scans, as soon as there are some), with the same measures as in task 1.

- [ ] 4.1 **NDT cell statistics.** Exact cell statistics (`use_exact_ndt_bucket_statistics`) against the default "previous" statistics. Measured on lagging data in `odometry-stall-fix`: 2.8 and 8 m off. Also the in-plane regularization of planar cells (`ndt_plane_in_plane_sigma_m`, 8.8 m off) and the archived `ndt-bucket-statistics` change (rejected 2026-10-03). The question is whether the cell statistics decide ghost walls and drift, or whether the lag did. The NDT map is built from poses that the lagging gyro and the lag-compensating rotation weights shaped.
- **The spot in 44 (found 2026-10-09):** the bad runs are stalls in blind passages. Run A (shipped, no shift) and run B (+1 ms) were compared odometry-only with `log_window_diagnostics`:
  - they separate gradually (1 cm at window 52, 5 cm at 120), then sharply in the passage at windows 171–186 (t ≈ 69–76 s, weak direction ≈ (0.96, 0.26, 0));
  - **in A, window 172 reverses the motion** (+0.07 m against a guess of −0.12). Its information ratio was 0.237, just above the projection's 0.2, so it wasn't projected. The carried speed then stays near zero for 15 windows, about 6 m of walking lost, and the solver doesn't converge (step 1e-5 to 1e-4 after 100 iterations);
  - **B** had a ratio of 0.128 in that window (projected), walked on at 0.4 m per window and converged.

  A stall detector (trajectory speed < 0.25 m/s over 1 s for ≥ 1.2 s while the IMU's step rhythm shows walking) finds:
  - a stall at 70–73 s in most runs with bad loop closure, none in any run with good loop closure, and one stall at 125 s (the original ghost-wall passage);
  - in 45f, the same short stalls in every variant, probably real turns on the spot.

  Widening the projection (0.15/0.3, 0.2/0.4) didn't stabilise 44 (8–14 loops). Cause: in a blind passage the speed is unobservable (the LiDAR is blind along the axis, the accelerometer sees only changes), so one wrong window kills the carried speed. The principled fix is 4.3: velocity as a carried state that only the accelerometer may change. Judge it by stalls and loops over three perturbed runs and on 45f.
- [ ] 4.2 **Degeneracy projection** (`degenerate_projection_ratio` 0.1 / `_full` 0.2): the thresholds were chosen for the ghost wall of 44 on lagging data. Check the thresholds and the information-ratio gate (0.3) again.
- [x] 4.3 **Accelerometer on its own:** the gate, its sigma and the continuous weight variant (measured harmful, 4.4 m off), then IMU always on (branch `imu-always-on`; on corrected data v3 was 1.0 m off against 22 m).
  **Done 2026-10-09:** IMU always on shipped (archived `imu-always-on`): no stalls in three perturbed 44 runs, 45f largest loop correction 0.68 -> 0.22 m. The gate stays only for `imu_always_on = false`. Open: a `/fp:fast` build loses a window at the top of the 44 stairwell (t ≈ 86 s, 6 m apart at the end); see that change's task 3.1.
- [ ] 4.4 **Hessian signal-to-noise degeneracy test and its rotation-only variant** (rejected 2026-10-09, in git history at the implementation commit named in the archived change): the test handed rotations to the gyro, which was 80 ms late.
- [ ] 4.5 **The three switched-off options of `odometry-stall-fix`** (`ndt_plane_in_plane_sigma_m`, `imu_degenerate_weight_ratio_full` with its smoothing, `imu_acceleration_offset_freeze_blind`): decide after 4.1 and 4.3 whether to remove them.
- [x] 4.6 **Loop-closure gating** (chi-square, GNC, PCM): last, because the corrections it judges depend on all of the above.
  **Done 2026-10-09 (a first, simple gating):** archived `loop-closure-gating`: retry the next partner after a rejection, drop the worst edge while its residual exceeds 0.15 m / 2° (iterative, like a one-at-a-time PCM). Ghost walls 51–62 -> 19–25 cm horizontally on 44. Chi-square/GNC can follow if a recording shows a false loop that this misses.
- [ ] 4.7 **Gyro and ARW constants:** the gyro noise values came from a static recording and don't depend on timing, but `rotation_sigma_floor_deg`, `ahrs_gain` and the turn weighting were tuned on lagging data too.
