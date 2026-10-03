# Design

## Context

In `compute_step_2` (`app/lidar_odometry_utils_optimizers.cpp`), each window after the first estimates `prev_velocity`. It is the position change over the previous window's last 20 nodes divided by their time span: about 22 ms at the L2's 893 Hz IMU. That estimate is used whenever the acc prior, IMU translation or the acceleration term is on (all of these are on in `settings/lidar-vertical.toml`). If `prev_velocity.norm() > 10.0`, the velocity is set to zero and `use_prev_velocity` to false.

That one flag affects three things in the window:
- the constant-velocity prediction of its starting poses;
- the velocity-continuity term (`VelocityPrior`);
- the IMU acceleration term (`VelocityPrior` together with `prev_velocity_age_s`).

**Speeds on the 44 baseline.** Measured on the final trajectory of `raw data/44_baseline` (after loop closure and consistency, so only an approximation of the window poses):
- over 1 s spans, the speed never goes above 1.2 m/s;
- over 20-sample spans it goes above 5 m/s in 407 of 248,120 spans (0.16%), up to 11.2 m/s;
- on `44_short`: 91 of 25,160 spans (0.36%), up to 8.0 m/s.

These are short wiggles of the trajectory: steps of up to 16 mm between nodes 1.1 ms apart, in bursts of about 0.1 s. They aren't real motion.

## Goals / Non-Goals

**Goals:**
- Replace the hard-coded 10.0 with a setting, keeping the same behaviour when the setting is 10.0.
- Make the dropped velocity visible in the console with its speed and the limit.

**Non-Goals:**
- Changing how `prev_velocity` is estimated, for example over a longer span. That would be a separate change if the wiggles above matter.
- Clamping the velocity instead of dropping it.
- Validating the value. A value of 0 or below drops every velocity prior; the docs say so.
- The `mean_shift.norm() > 1.0` check just below it, which is a different limit on a different quantity.

## Decisions

1. **The setting is a member of `OdometryConfig`, `double max_prior_velocity_mps = 10.0;`, next to `lidar_odometry_motion_model_acc_1_sigma_mps2`.**
   - It goes into `TomlIO::POINTERS` and the `motion_model_uncertainty` entry of `CATEGORIES`, next to the acc sigma, the term it guards.
   - `compute_step_2` reads `params.max_prior_velocity_mps` where it has `10.0` today.
   - *Alternative:* a member of `VelocityPrior` or `MotionModelOptions`. Rejected: the check runs in `compute_step_2` before the `VelocityPrior` is built, and `optimize_lidar_odometry` never sees a rejected velocity.
2. **Drop, as today, rather than clamp.** A velocity above the limit is the result of a bad or wiggling window, so its direction can't be trusted either. Keeping the drop also means the setting at 10.0 reproduces today's results bit for bit.
3. **Console message.** `log_info(logger) << "previous window velocity " << speed << " m/s > max_prior_velocity_mps " << limit << ": not used as prior"` replaces the fixed `!!!prev_velocity.norm() > 10.0!!!`. It is logged at info level as before, so the number of drops in a run can be counted in the CLI output.
4. **Shipped files.**
   - `settings/lidar-vertical.toml` sets 5.0, the user's limit for walking in a cave.
   - `app/default_settings.toml`, the GUI start-up file, sets 10.0, so the GUI defaults behave as before and the key is visible to whoever edits that file.
5. **Golden TOML files.** Both gain the key. They are regenerated with `LO_UPDATE_GOLDEN=1` in their own commit, after the code commit that adds the setting. `default_params.golden.toml` gets 10.0 and `lidar_vertical.golden.toml` gets 5.0.

## Risks / Trade-offs

- **The limit at 5 m/s will probably trigger on 44, and it may on `44_short`.** The 20-sample span is short enough that wiggles exceed 5 m/s although the walk is at about 1 m/s. A window whose previous velocity is dropped starts from a zero-velocity guess without a velocity prior, which is wrong for someone walking. It is less wrong than seeding it with an 8 m/s wiggle. → The console message counts the drops. Whether 5 m/s helps or hurts is visible in the `44_short` result and in the user's next runs. A longer span for the estimate would be a separate change.
- **The full 44 is not run in this change** (user decision, 2026-10-03). → The `44.manifest.json` entry for the parameters TOML is updated to the new hash, which is shared with `44_short`, because the paths are normalized. The other 44 entries stay as they are. If the limit changes the 44 result, the next change that runs the full 44 will see DIFFERENT. That change first refreshes `raw data/44_baseline` and `44.manifest.json` from master, as was done on 2026-10-03.
- **`44_short` and its manifest.** The parameters TOML always differs, because it has the new key.
  - If every other file is identical, the limit didn't trigger. Only the manifest's parameters entry is updated.
  - If other files differ, the run becomes the new `44_short` reference: `raw data/44_short_baseline` and `tools/baseline/44_short.manifest.json` are refreshed from it. The number of drops (console messages) is reported.

## Migration Plan

Old TOML files load with the default of 10.0 and run as before. Rollback means reverting the commits, or setting 10.0 in `settings/lidar-vertical.toml`.
