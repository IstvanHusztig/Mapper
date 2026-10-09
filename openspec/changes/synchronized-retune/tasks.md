# Tasks

**Runs:** full corrected 44, precise Release build, from copies of the binaries. Up to three runs in parallel on copies of the input folder (no window may come near `real_time_threshold_seconds`; checked per run). The copies are deleted at the end. Each run is reported with loops and corrections, route, the difference from `44_baseline`, `eval_odometry.py` and the time.

## 1. Rotation and accelerometer (batch 1)

- [ ] 1.1 `rotation_sigma_inflation_k` 2 and 1 (shipped 4), and the accelerometer term off (`use_imu_acceleration_term = false`). Record here.

## 2. Follow-up (batch 2)

- [ ] 2.1 From batch 1's best: the accelerometer gate's sigma (0.2 m/s²) or the degenerate translation sigma, and the gyro-bias random walk (0.002 °/s/√s), as batch 1 suggests. Record here.

## 3. Ship

- [ ] 3.1 The chosen settings once with the `/fp:fast` build (the difference between the builds at most a few decimetres, as today).
- [ ] 3.2 New `settings/lidar-vertical.toml`, new baselines (the old folders renamed with a date suffix, manifests rewritten), the settings tables in `docs/index.html` and `docs/hu/index.html`. Archive, push, memory notes.

## 4. Re-check what was decided on the lagging data (after the retune, each its own change or task)

Everything below was measured or rejected on recordings whose IMU lagged the LiDAR by 50 to 80 ms, with settings tuned around that lag. A "worse" there may be "better" now, and a "better" may have been a compensation. Each item is re-run on the retuned settings and the corrected 44 (and on the user's new synchronized scans, as soon as there are some), with the same measures as in task 1.

- [ ] 4.1 **NDT cell statistics.** Exact cell statistics (`use_exact_ndt_bucket_statistics`) against the default "previous" statistics. Measured on lagging data in `odometry-stall-fix`: 2.8 and 8 m off. Also the in-plane regularization of planar cells (`ndt_plane_in_plane_sigma_m`, 8.8 m off) and the archived `ndt-bucket-statistics` change (rejected 2026-10-03). The question is whether the cell statistics decide ghost walls and drift, or whether the lag did. The NDT map is built from poses that the lagging gyro and the lag-compensating rotation weights shaped.
- [ ] 4.2 **Degeneracy projection** (`degenerate_projection_ratio` 0.1 / `_full` 0.2): the thresholds were chosen for the ghost wall of 44 on lagging data. Check the thresholds and the information-ratio gate (0.3) again.
- [ ] 4.3 **Accelerometer on its own:** the gate, its sigma and the continuous weight variant (measured harmful, 4.4 m off), then IMU always on (branch `imu-always-on`; on corrected data v3 was 1.0 m off against 22 m).
- [ ] 4.4 **Hessian signal-to-noise degeneracy test and its rotation-only variant** (rejected 2026-10-09, in git history at the implementation commit named in the archived change): the test handed rotations to the gyro, which was 80 ms late.
- [ ] 4.5 **The three switched-off options of `odometry-stall-fix`** (`ndt_plane_in_plane_sigma_m`, `imu_degenerate_weight_ratio_full` with its smoothing, `imu_acceleration_offset_freeze_blind`): decide after 4.1 and 4.3 whether to remove them.
- [ ] 4.6 **Loop-closure gating** (chi-square, GNC, PCM): last, because the corrections it judges depend on all of the above.
- [ ] 4.7 **Gyro and ARW constants:** the gyro noise values came from a static recording and don't depend on timing, but `rotation_sigma_floor_deg`, `ahrs_gain` and the turn weighting were tuned on lagging data too.
