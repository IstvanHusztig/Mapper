# Proposal

## Why

Recording 44's IMU lagged its LiDAR by 79–52 ms (dLidar timing defect; corrected on 2026-10-09 with `tools/restamp_imu.py`, see `openspec/changes/imu-always-on/analysis.md` on branch `imu-always-on`). The shipped settings were tuned on that lagging data, and they compensate for it:
- the rotation sigma inflation k = 4 lets the LiDAR bend a late gyro;
- the gyro-bias settings absorbed part of the lag;
- the accelerometer gate is loose enough (0.5 m/s²) for a late accelerometer.

On the corrected data they are clearly worse: the new baseline has 11 loop closures and a largest correction of 5.56 m, against 21 loops and 0.80 m on the lagging data. The settings have to be retuned for synchronized data before any research-based improvement can be judged again.

The scanner is broken, so recording 44 is the only synchronized recording. Every choice therefore has to be justified by the sensor or the method as well, not only by the score on 44.

## What Changes

- **Retune the settings that compensated for the lag**, one family at a time, on the full corrected 44 (precise build):
  - the rotation sigma inflation `rotation_sigma_inflation_k`;
  - the accelerometer gate (`use_imu_acceleration_term`, `imu_acceleration_sigma_mps2`, `imu_degenerate_translation_sigma_m`);
  - the gyro bias (`gyro_bias_rw_deg_s_sqrt_s`, `handoff_tail_nodes`).
- **Judge each candidate** against the corrected baseline by:
  - the loop-closure count and corrections (drift);
  - the route length;
  - the difference from the baseline (`trajectory_seams.py`);
  - `eval_odometry.py`, whose window yaw difference is meaningful again now that the gyro is synchronized;
  - the residual time offset (`imu_time_offset.py`).
- **Ship the result** as the new `settings/l2-vertical.toml` with new baselines, and check the candidate once with the `/fp:fast` build.
- **Not in this change:** the research-based improvements. Their redo follows in its own changes: the degeneracy projection re-check, IMU always on (branch `imu-always-on`), the Hessian degeneracy test, and loop-closure gating.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None (settings only).

## Impact

- `settings/l2-vertical.toml`, and possibly `settings/default_settings.toml`.
- New baselines (folders and manifests); docs that quote the settings (`docs/index.html`, `docs/hu/index.html`).
- No code change, so Release only (`CLAUDE.md` rule).
