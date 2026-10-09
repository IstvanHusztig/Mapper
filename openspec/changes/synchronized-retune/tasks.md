# Tasks

**Runs:** full corrected 44, precise Release build, from copies of the binaries. Up to three runs in parallel on copies of the input folder (no window may come near `real_time_threshold_seconds`; checked per run). The copies are deleted at the end. Each run is reported with loops and corrections, route, the difference from `44_baseline`, `eval_odometry.py` and the time.

## 1. Rotation and accelerometer (batch 1)

- [ ] 1.1 `rotation_sigma_inflation_k` 2 and 1 (shipped 4), and the accelerometer term off (`use_imu_acceleration_term = false`). Record here.

## 2. Follow-up (batch 2)

- [ ] 2.1 From batch 1's best: the accelerometer gate's sigma (0.2 m/s²) or the degenerate translation sigma, and the gyro-bias random walk (0.002 °/s/√s), as batch 1 suggests. Record here.

## 3. Ship

- [ ] 3.1 The chosen settings once with the `/fp:fast` build (the difference between the builds at most a few decimetres, as today).
- [ ] 3.2 New `settings/lidar-vertical.toml`, new baselines (the old folders renamed with a date suffix, manifests rewritten), the settings tables in `docs/index.html` and `docs/hu/index.html`. Archive, push, memory notes.
