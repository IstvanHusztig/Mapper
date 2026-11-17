# Tasks

**Step checks, on every commit:**
- the Release build;
- the unit tests;
- the characterization guard, byte-identical with the reference recorded in task 2.1;
- the loop guard, byte-identical with its reference recorded in task 2.1;
- the replays of the 44_short capture (20–40 s, 21 windows) and the Livox 3 capture (100–130 s, 84 windows), every
  window identical;
- 44_short against its manifest.

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the code. Result: plan section "Stage 5: one
  place for the weights", 2026-11-17 (26770fd); the review's findings worked in.
- [x] 1.2 The user's decisions on the seven points at the end of the plan section. Verify: the plan's heading reads
  "Decisions (user, <date>)" with what was decided. Result: 2026-11-17, all seven as recommended (3b2d146).

## 2. Guards and clean-up

- [x] 2.1 Record the characterization reference and the loop guard's reference from the code at the change's start,
  twice each. Verify: the two recordings are identical; the characterization reference equals stage 4's (the code is
  unchanged since). Result: recorded twice at 81583e0, both guards identical with stage 4's references.
- [x] 2.2 Remove the no-op reset of `x/y/z_sigma_m` in `prepare_acc_term` (`run_odometry.cpp:102-105`); `CLAUDE.md`
  without the degenerate-window sigma raise. Verify: step checks. Result: `AccTerm::motion_model` (only ever a copy of
  `setup.motion_model`) removed with the reset; its users read `setup.motion_model`. Step checks identical.

## 3. The noise model (pure refactors)

- [x] 3.1 `src/domain/window_noise_model.{h,cpp}`: `WindowNoiseModel` with the layout, the fixed values and the NDT
  parameters per grid; `build_window_noise_model` from the options, the window and the carried priors; the rotation
  sigma function (max(k·ARW·√dt, floor)). Not used yet. Tests (`tests/window_noise_model_test.cpp`): each weight
  equals its formula; the first window, a window at rest and the path without the accelerometer term have the terms'
  conditions; every weight is bit-identical with the one the term computes today. Verify: step checks.
  Result: `MotionEdgeWeights` / `motion_edge_weights` moved into `window_noise_model` unchanged (the edges' sigma via
  `time_scaled_rotation_sigma`); the accelerometer terms are listed in the model (kind, nodes, tau, weight) in the order
  they are added; 5 test cases; step checks identical. The priors' growth formulas move with task 3.3.
- [x] 3.2 `prepare_window` builds the model; `WindowProblem` takes it (const) and stops rebuilding the layout per
  `linearize`. The motion edges (`motion_edge_weights`, the bias coupling) and the velocity continuity and first-step
  prior take their weights from the model. Verify: step checks.
  Result: step checks identical, and the per-term table of the 44_short replay identical with a reference recorded
  before this task (checked on every commit from here). The table builds its own model until task 3.6. Replay time
  measured back to back: 28.9-34.5 s against 31.7-33.3 s before (the first, slower readings were the machine).
- [x] 3.3 The accelerometer term, the zero velocity, the weak end-velocity tie (named constant) and the gyro-bias,
  start-velocity and acceleration-offset priors take their weights from the model. Verify: step checks.
  Result: the accelerometer term walks the model's list of terms (one `shape_of`, the three adders gone); the
  priors' growth formulas (`random_walk_prior_cov`, `velocity_prior_cov_over_gap`) moved into `window_noise_model` and are
  called by `apply_gyro_bias_prior`, `prepare_acc_term` and `prepare_velocity_state`, with a test. Step checks and the
  per-term table identical.
- [x] 3.4 The origin fix from the model (the degree quirk kept, commented; recorded in `docs/code-review-tracker.md`);
  the NDT term's parameters per grid from the model, applied in today's order. Verify: step checks.
  Result: `fix_origin` reads `noise.origin` (the quirk is the existing tracker entry C37, its location updated); each
  `NdtGrid` carries the model's `NdtNoise` (planarity, range weight and cap, Huber, scale), the projection reads its ratios
  from it (`ndt_noise(scan_to_map)` builds them). Step checks and the per-term table identical; the Livox 3 replay covers
  the Huber cutoff and the information scale.
- [x] 3.5 `ndt_window_cost` reads the model's per-grid range weight and cap. Verify: step checks; the weak-axis
  diagnostic lines on 44_short with `log_weak_axis_diagnostics` before and after, the differences listed (results
  identical).
  Result: the 26 diagnostic lines of the 44_short replay are identical before and after: both shipped settings have
  `ablation_study_use_norm` and planarity off, so the weight is 1 in both grids and the cap never acts. The fix only
  changes the diagnostic with `use_norm` on (no range weight and cap in the outdoor grid, as in the term). Step checks and
  the per-term table identical.
- [x] 3.6 The per-term table: `WindowTermInputs` takes the model; guess and result share it. Verify: step checks; the
  table of the 44_short replay before and after the commit is identical.
  Result: `WindowTermInputs::velocity_prior` became `noise`; `record_window_terms` passes the window's model from
  `prepare_window` to both rows. The table (`window_terms.csv`, `window_features.csv`) identical with the reference recorded
  before task 3.2; step checks identical.
- [ ] 3.7 Consistency's time-scaled rotation sigma calls the model's sigma function. Verify: step checks (the 44_short
  run has `use_consistency` on in the shipped settings, so its manifest covers consistency).

## 4. The log

- [ ] 4.1 `log_window_noise` (`OdometryConfig`, `TomlIO::POINTERS` and the `[lidar_odometry_debug_info]` category,
  the three settings files, `false`); the `window noise <n>:` line in `report_window`; the `noise budget` line at the
  start of the odometry; the model's values as `window_features.csv` columns. Verify: step checks; 44_short with
  `log_window_noise` on has 70 lines and is identical except the params TOML; the `noise budget` line in every run and
  replay.
- [ ] 4.2 Golden TOMLs (`LO_UPDATE_GOLDEN=1`) in their own commit. Verify: unit tests.

## 5. Docs and manifests

- [ ] 5.1 Docs: `CLAUDE.md` (the noise model, the setting), `docs/index.html` (the debug setting; the Fundamentals'
  "Sigma, weight, information" entry points to the model and its log line), `tools/README.md` (the new feature
  columns), the plan's progress. Verify: the text matches the code.
- [ ] 5.2 Manifests (44, 44_short, Livox 3): the params TOML entry, in their own commit. Verify: the stage end runs
  compare IDENTICAL.

## 6. End of stage

- [ ] 6.1 Full 44 and Livox 3 against their manifests with the shipped settings (sequential, in the background).
  Verify: identical except the params TOML before task 5.2, IDENTICAL after it; the odometry time reported against
  stage 4's (1214 s and 697 s).
- [ ] 6.2 Debug build and `ctest`. Verify: pass.
