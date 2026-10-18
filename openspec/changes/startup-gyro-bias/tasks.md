# Tasks

## 1. Start-up bias

- [x] 1.1 `startup_gyro_bias` (design 5) with the three tests and settings `imu_startup_bias_window_s` (1.0), `imu_startup_bias_max_spread_deg_s` (3.0), `imu_startup_bias_max_tilt_deg` (2.0), `imu_startup_bias_max_noise_ratio` (3.0) in `[madgwick_filter]`; `calculate_trajectory` uses it and logs the estimate or the failed test with its value. Verify: unit tests on synthetic samples — still + white noise accepted with the right bias; hand-held-like noise (2× sigma) accepted; a rate step, a tilt and gross shaking rejected with the right reason; the existing orientation tests pass.
- [x] 1.2 `gyro_bias_startup_sigma_deg_s` (0.5): the online estimate's initial sigma when the start-up bias was accepted. Verify: the first windows' online bias in 2.1 doesn't overshoot (prepare_run is file-local; checked on the runs instead of a unit test).
- [x] 1.3 Settings files and golden TOML files (own commit). Verify: `ctest` in Release.
- [x] 1.4 Static recording check: the gyro samples of `unsynced_archive/43/static` through `startup_gyro_bias` (a small test harness or Python port of the same tests; dMapper is not run on the unsynced data). Verify: accepted, within 0.2 °/s of the median per axis.

## 2. Evaluation on recording 44 (one background script)

- [ ] 2.1 44_short and 44, s1, s2 with the new start-up bias: log line (accepted, bias), first windows' online bias, `plan_fit.py` start-room rotation, `map_quality.py`, `wall_precision.py`, against the shipped results. Verify: table in `results.md`; the spec's scenarios checked (start room smaller in absolute value on each run; first-window overshoot ≤ 0.5 °/s).
- [ ] 2.2 Review of the conclusions (Sonnet), the start-room overlay before/after, then ask the user whether to ship. Verify: the user has answered.

## 3. Ship (only after the user's yes)

- [ ] 3.1 Debug build + `ctest`; new 44 and 44_short baselines in their own commit; tracker entry; push; master-reviewer. Verify: `compare_baseline.py` IDENTICAL on the new manifests.
