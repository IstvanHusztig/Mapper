# Design

## Context

`compute_step_2` in `app/run_odometry.cpp` (since 6a) has this shape. Line numbers are of today's file.

| Lines | Part | Inputs → outputs |
|---|---|---|
| 14–39 | place window 0 at `state.m_g`; seed the map with the initial points | windows[0], state → windows[0], state.buckets |
| 41–90 | run setup: option structs, bias random walk, `have_acc`, `gravity_norm`, `acc_gravity`, gating flags, counters | params, windows[0] → ~20 locals |
| 99–131 | previous window's velocity over its last 20 nodes, its age, the `max_prior_velocity_mps` drop | windows[i-1], windows[i] → use, velocity, age |
| 133–213 | initial guess: chain from the previous end, then IMU integration / ZUPT / constant velocity / mean shift | windows[i-2..i], velocity, flags → windows[i].poses, motion_model_poses, stationary |
| 215–228 | gyro-bias prior and its removal from the guess | carried bias → rotation_options, windows[i] |
| 233–257 | turn detection | windows[i] → inflation_k, turn counter |
| 259–300 | accelerometer-term gating, window sigmas, `VelocityPrior`, `MotionModelTranslationImuOptions` | carry, windows[i] → per-window options |
| 305–388 | robust mode (`optimize_sf2`, `optimize_rigid_sf`) | windows[i], points, state.buckets_indoor → windows[i] |
| 390–439 | LM iterations with `real_time_threshold_seconds`, and the window log | → windows[i], delta, iterations |
| 441–488 | bias log and carry; degeneracy and gravity calibration; offset folding | → carry |
| 490–519 | tail hand-off | windows[i], tail nodes, bias update → windows[i].poses |
| 521–549 | progress, preview every 100 windows, NaN check | |
| 551–564 | re-chain every later window from the new end of window i | windows[i..] |
| 566–622 | points into the map; sliding-window reset and decimation; consecutive distance | state |
| 625–647 | `motion_model_poses = poses`, summary log, trajectory length | windows, state |

**Constraints:**
- `/fp:fast` with whole-program optimization. Extracting a block into a function can change inlining and thus the last bits of the results (see 6a). Every extraction is checked bit for bit on `44_short`.
- Robust mode is off in `settings/lidar-vertical.toml`, so its extraction is only build-checked.
- The console output is not compared by the baseline. It still stays the same, message for message.

## Goals / Non-Goals

**Goals:**
- The loop body reads as a list of named steps.
- The state carried between windows is one struct, and the per-run settings are another.
- The pure steps are tested on synthetic windows.
- Results stay bit-identical.

**Non-Goals:**
- Any behaviour change, including the longer velocity span. That stays a separate change; `estimate_previous_velocity` takes the span as a parameter, called with 20.
- The quadratic re-chaining of all later windows after every window (lines 551–564). It runs about 690 × 690 / 2 window chains on 44. It is kept as is, and only noted.
- Moving `update_rgd`, `decimate` or the map into a domain type (later steps).
- A synthetic test of `optimize_lidar_odometry` (6c).

## Decisions

1. **Two homes for the steps.**
   - **`app/window_steps.{h,cpp}`:** steps that only need `Window`, Eigen, `imu_kinematics` and plain parameters. These are the testable ones.
   - **`app/run_odometry.cpp`, in an anonymous namespace:** steps that need `OdometryState`, the NDT map, `PointStore`, `PreviewSink` or the `Logger`.
   - *Alternative:* everything in one file. Rejected: the tests would need `lidar_odometry_utils.h` and `core`.
2. **Pure steps don't log.** Each one returns what the caller needs to log.
   - `estimate_previous_velocity` returns `{bool use; Eigen::Vector3d velocity; double age_s; bool dropped; double speed}`, and the loop logs the drop message.
   - `predict_window_poses` returns whether the window is stationary, and the loop logs it and counts it.
   - Messages and their order stay the same.
3. **`OdometryRunSetup`** (built once by `prepare_run(windows, params, logger)`) holds everything fixed for a run:
   - the option structs (rotation, scan-to-map, motion model);
   - `bias_rw_rad`, `bias_sigma0_rad`;
   - `have_acc`, `use_imu_pred`, `use_zupt`, `use_acc_prior`, `use_acc_term`, `acc_gated`;
   - `gravity_norm` and the initial `acc_gravity`.

   `prepare_run` also logs the two "disabled" messages, as today.
4. **`WindowLoopCarry`** holds what passes from window to window:
   - `bias` (`GyroBiasState`), `acc_gravity`, `acc_offset_cov`;
   - `previous_window_degenerate`, `degenerate_time_s`;
   - `acc_distance`, `points_global`;
   - the counters `stationary_windows`, `turn_windows`, `acc_term_windows`.

   Each step takes the parts it changes by reference. The struct doesn't hide which step changes what.
5. **`place_window_after(Window &w, const Eigen::Affine3d &start)`** replaces the three identical chaining loops (window 0 from `m_g`, and every later window from its predecessor's end). It uses the same expression, `m_last = m_last * (tmp[k-1].inverse() * tmp[k])`.
6. **`compute_step_2` → `optimize_windows`**, renamed in the last code commit, with `tools/rename_identifiers.py` (`-r compute_step_2=optimize_windows`) over `app/` and in the docs. The signature is unchanged otherwise.
7. **One extraction per commit, each with the step checks.** If an extraction breaks `44_short` bit-identity, stop and report which one and how large the difference is (same rule as 6a).
8. **The test file `tests/window_steps_test.cpp`** builds windows in code:
   - nodes every 10 ms;
   - known motion: constant velocity, a constant turn rate, at rest;
   - accelerometer readings that match that motion.

## Risks / Trade-offs

- **[A different inlining after an extraction breaks bit-identity]** → One extraction per commit; decision 7.
- **[Moving a block changes the order of floating-point operations]** → Blocks are moved as they are. Expressions are not regrouped. Local names may change only where a parameter replaces a local.
- **[Robust mode is untested by any run]** → The same limitation as 6a. Its block moves as one piece.
- **[`WindowLoopCarry` becomes a grab bag]** → It only holds today's carried locals, and each field has a comment saying which step updates it.

## Migration Plan

Nothing to migrate. To roll back, revert the commits.
