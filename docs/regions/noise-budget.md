# Region: noise-budget

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The noise-budget region owns every weight of a window's problem and the readings that show what the weights do. One
function, `build_window_noise_model`, computes all fixed weights of a window once, before its first iteration; the
terms only read them. The per-term table, the `window noise` and `noise budget` log lines and the weak-axis and heading
diagnostics show how the weights balance. It owns the weights' formulas, not the terms.

## Contract

**Takes**
- The run's options (orchestrator: `get_scan_to_map_options`, `get_motion_model_options`,
  `get_motion_model_rotation_options` in `optimizer_options.cpp`) and this window's: the rotation options with the
  bias prior and the turn window's k (`prepare_window`), `MotionModelTranslationImuOptions` with the carried
  velocity and offset priors and the rest flag (inertial: `prepare_acc_term`, `prepare_velocity_state`), the
  `VelocityPrior`, and the window's timestamps.
- The layout `WindowUnknowns` from the solver's `layout_window_unknowns` (which unknowns and terms exist, the anchors).

**Gives**
- `WindowNoiseModel` (`src/domain/window_noise_model.h`), built in `prepare_window` (`run_odometry.cpp:1036`; the
  tests' single iteration `optimize_lidar_odometry` builds its own, `scan_to_map.cpp:102-103`), read
  `const` by `WindowProblem`, every term (`add_*` of inertial, `fix_origin` of the solver, the NDT term's `NdtNoise` of
  lidar-matching), the per-term table and `ndt_window_cost`.
- Formula helpers others call: `time_scaled_rotation_sigma` (also consistency's gyro term, `consistency.cpp:93`, which
  multiplies by its own scale), `random_walk_prior_cov` (`apply_gyro_bias_prior`, `prepare_acc_term`),
  `velocity_prior_cov_over_gap` (`prepare_velocity_state`). These callers are the inertial region's: a change of one
  of these formulas changes the priors it carries from window to window, and its tests move with it.
- The table and records to the `WindowTermSink` port (`src/application/window_term_sink.h`); the CSV adapter writes them.

**Invariants others rely on**
- **Bit-identity.** Every weight uses the expression, operand order and condition the term used before stage 5. The
  weight enters each term where it entered before (`w * jaᵀ * r`, `prior_inf * delta`, the generated edge code), never
  folded into a Jacobian. Changing an expression changes bits; that is a numeric change, not a refactor.
- **Fixed for the window.** The values depend only on settings, timestamps and carried state; none changes during the
  iterations. The posteriors (`take_covariance`) go to the next window, never into this window's priors.
- **Conditions kept:** velocity continuity only without the accelerometer term (dt clamped at 1e-4 s); edge dt clamped
  at 0; no bias weights in the first window; accelerometer terms only with `imu_always_on` from the second window,
  a term stretched by an IMU gap longer than 2 × `imu_acceleration_span_s` is left out; the edges' x/y/z weight × 
  `imu_position_information_scale` only while the accelerometer term is active.
- **Formulas.** Edge rotation σ = max(k · ARW · √dt, floor) with time scaling, else the constant σ; accelerometer weight
  1 / ((density² / τ + floor²) τ²); priors: carried + I·rw²·T (bias, offset), carried + I·(density²·gap + floor²·gap²)
  (start velocity); zero velocity 1/σ²; weak end tie `weak_end_velocity_tie_weight = 1` (σ 1 m/s).
- **NDT rules stay in the term**, with their parameters here (`NdtNoise`): per point, in this order, cell information
  (from the map), planarity, range weight and its cap of 10 (fine grid only), Huber weight, `ndt_information_scale`;
  then the degeneracy projection. Separate multiplications, because folding them changes bits.
- **Readings only observe.** The table, the CSV files and the diagnostics change no result (`window capture` test "the
  run is the same with log_window_terms on").
- `log_weak_axis_diagnostics` also reads the inertial region's state: the start-velocity prior's covariance
  (`acc_term.options.velocity_prior_cov`) and `window.velocity`.

## Owns

- `src/domain/window_noise_model.{h,cpp}`: `WindowNoiseModel`, `MotionEdgeWeights` / `motion_edge_weights`,
  `AccelerometerTermSpan`, `NdtNoise` / `ndt_noise`, `build_window_noise_model`, the helpers above,
  `window_noise_sigmas`, `log_window_noise` (the `window noise <n>:` line), `log_noise_budget` (the `noise budget` line).
- `src/domain/window_term_table.{h,cpp}`: the twelve motions (shift, stretch, curve, turn × along, sideways, vertical),
  weight w = mᵀHm and pull mᵀb / w per term row, the `total` and `damping` rows, `window_motion_change`.
- `src/application/window_term_sink.h`: `WindowTermRecord`, the `WindowTermSink` port.
- `src/infrastructure/window_term_csv.{h,cpp}`: `FolderWindowTermSink` (`window_terms.csv`, `window_features.csv`).
- In `src/application/run_odometry.cpp`: `record_window_terms` (the `window terms N:` line and the record),
  `log_weak_axis_diagnostics`, `log_heading_diagnostics` (behind `log_window_diagnostics`): the diagnostics of the
  balance between terms. A diagnostic of one term's fit is the term's region's (`log_accelerometer_fit`: inertial).
- Tests: `tests/window_noise_model_test.cpp`; the table cases of `tests/scan_to_map_test.cpp` (shared).

Settings that are weights (shipped L2 / Livox values): `lidar_odometry_motion_model_{x,y,z}_1_sigma_m` 1e-4 m;
`imu_position_information_scale` 1 / 0.1; `ndt_information_scale` 1 / 0.1; `ndt_huber_cutoff` 0 (off) / 3;
`rotation_sigma_inflation_k` 7.0898154 (`_turn` off); `gyro_arw_{x,y,z}_deg_sqrt_s` 0.028 / 0.042 / 0.057 (the L2's,
from `raw data/43/static`, also in the Livox file); `rotation_sigma_floor_deg` 0.0035449077;
`imu_accel_noise_density_mps2_sqrt_hz` 0.0013; `imu_accel_floor_sigma_mps2` 0.1; `imu_acceleration_offset_rw_mps2_sqrt_s`
0.0001; `gyro_bias_rw_deg_s_sqrt_s` 0.005; `gyro_bias_initial_sigma_deg_s` 10; `imu_zero_velocity_sigma_mps` 0.01;
`imu_velocity_initial_sigma_mps` 1.0; the origin-fix sigmas. The settings themselves are the orchestrator's
(`settings/`): a weight idea is tried on a copy.

## How to check your work

- Unit tests (Release): `out/build/x64-Release/bin/lidar_odometry_tests.exe -tc="window noise model*,window term table*"`.
  `window_noise_model_test.cpp`: each weight equals its formula (edges with floor and turn k; velocity continuity only
  without the accelerometer term; accelerometer spans and weights; what exists in the first window, at rest, without
  timestamps; the same values the terms put into the normal equations; prior growth by random walk and gap). Table
  cases in `scan_to_map_test.cpp`: the rows add up to the window's complete equations; NDT shows a sideways offset; the
  accelerometer's curve pull; the gyro edges' turn pull; a corridor held across, not along. `window_capture_test.cpp`:
  the run is the same with the table on, and the files hold it.
- Replays: `python tools/run_replay.py <capture> <settings with log_window_terms / log_window_noise = true>`; the
  replay writes `<settings name>_window_terms.csv` and `_window_features.csv` next to the capture. Then
  `python tools/step_checks.py` (the per-term table of the 44_short replay is one of its references).
- Readings that are this region's own:
  - per-term table (`python tools/window_terms.py <csv> [--window N] [--from S --to S] [--state guess|result]`): the
    hold (1/√weight) and the share of the weight of every row, per motion and direction; `ndt_unprojected` against
    `ndt_indoor` + `ndt_outdoor` (what the projection takes away); cost per residual against 1 where the weight is a
    noise model (not for NDT);
  - `window noise N:` line (`log_window_noise`): edge translation σ, edge rotation σ min/median/max per axis, velocity
    continuity σ, accelerometer σ range, start-velocity, bias and offset prior σ, zero-velocity σ, NDT scale and Huber
    cutoff; the same 28 `noise_*` columns in `window_features.csv`; the run's `noise budget` line;
  - window verdict: the LiDAR's weight share along travel (`lidar_share_along`, with `log_window_terms`); the velocity
    chi² is read together with the inertial region;
  - `log_window_diagnostics`: `window diagnostics N: weak axis ...` (stiffness of NDT, velocity prior and motion model
    against a velocity offset along the weak axis, NDT cost profile), `window diagnostics N: heading ...` (yaw
    correction, NDT yaw stiffness), and, the inertial region's, `accelerometer fit N:` (IMU − trajectory at 50 ms anchors, world and body mean, rms).
- Skills: `verify-robustness` before any weight is called better (perturbation spread), `verify-run` on every run.
  Table timing: about 9 % more replay time with `log_window_terms` on.

## Known flaws

- **C34** With `imu_always_on` the edges' translation rows tie every node to a guess integrated from the start-velocity
  prior with σ 1e-4 m per edge, holding a window's velocity to ~5 mm/s (derived) whatever the velocity's carried
  covariance: the accelerometer and the velocity prior are counted again. Today: the weight in
  `window_noise_model.cpp::motion_edge_weights` (× `imu_position_information_scale`, 0.1 on the Livox, 1 on the L2:
  σ 0.32 mm / 0.1 mm); the guess in `window_steps.cpp::integrate_window_translations` (inertial), called in
  `prepare_window`. The tracker's "loosened on the gated path" no longer applies (gated path removed 2026-11-12).
  Reported; not measured on a recording. Crosses to the inertial region. Measured on synthetic windows 2026-11-20
  (`tests/window_covariance_test.cpp`, no LiDAR, scale 0.1, 100 runs): with the `honest-inertial-carry` switches off the
  handed-on velocity along travel scatters 4.0 per dof against its claim (claimed sigma about half the scatter), the
  offset 1.45; switches on 1.07-1.12; near the 100-iteration cap, so non-convergence is not ruled out. Not pinned
  (an approximate factor).
- **C37** The origin fix's rotation weights are 1/σ_deg² applied to radians (1e12 instead of 3.3e15 for 1e-6°; 3283×
  weaker). Today: `window_noise_model.cpp::build_window_noise_model` (`m.origin`), read by `window_solver.cpp::fix_origin`;
  the `noise budget` line says "(used as radians, tracker C37)". Verified 2026-11-20 (K10, `tests/unit_cases_ndt_test.cpp`
  `unit case K10: the origin fix's rotation weight against 1 / sigma^2 in radians`): 1e12 against 3.283e15. Not pinned
  (the user's decision: harmless in size, both hold the first node; kept bit-identical in stage 5). Not in the order of
  `consistency-harness` 8.2.
- **F8** (reported by the audit of 2026-11-20, `experiments/consistency/research.md`; not measured by the harness): the
  accelerometer noise is modelled white only and the offset (random walk 0.01 in the candidate set) is a fast state; the
  handed-on 4.5 mm/s velocity sigma is set by the 0.1 m/s² floor; with layer 2, 78 % of the offset estimate's variance
  mirrors the velocity move and jitters gravity by 0.024 m/s² per window (Livox 3). A model limit.
- Fixed: **C5** (the rotation prior × Cauchy 1/π, D4 519b3e8, sigmas ×√π: k 4 → 7.0898, floor ×√π); the tracker still
  says "verified".

- **C39** (verified): `run_odometry.cpp::log_weak_axis_diagnostics` computes the motion model's stiffness from the
  x/y/z sigmas without `imu_position_information_scale`; with the Livox settings it reports 10× the edges' stiffness.
  A diagnostic only.

## To split in the first job

None: no function of this region is above 80 lines (`python tools/function_sizes.py src --min 80`, 2026-11-18).

## Experiments

- **2026-11-21, consistency harness: the claims against the scatter in the loop** (`consistency-harness` 6.1-6.3;
  `experiments/consistency/loop.md`; candidate settings, `ahrs_gain` 0.05, the world drawn from the settings' own noise
  claims, 20 seeds). With both `honest-inertial-carry` switches on, where the body moves the end velocity's claim is still
  smaller than its error: NEES per dof median 4.1 in seen windows and 6.6 in turns on R1 (means 11.7 and 32.7); on R2
  1.4-2.8; R3 mean 1.20 (off 1.51). Under the shipped Livox file (its own world, four values different, never paired) R1
  NEES median 3,844. The first loop runs had a world whose IMU errors were not the settings' (offset walk 5×, gyro bias
  walk 10× the claim): the world is built from the run's own `OdometryConfig` now (verification). Not concluded: a
  universal inflation factor (K8 depends on the geometry), or anything on a recording.
- **2026-11-20, consistency harness: the model world's claims** (`consistency-harness` 5.1-5.3, 5.6; tests
  `tests/terms_at_truth_test.cpp`, `tests/accelerometer_noise_test.cpp`, `tests/window_covariance_test.cpp`,
  `tests/unit_cases_ndt_test.cpp`, `tests/lidar_claim_test.cpp`; `experiments/consistency/fixtures.md` section 2, design
  D4; synthetic data, stage 8 candidate settings). *Accelerometer noise (F-b):* in the sample-aligned fixture the
  whitened terms have variance 0.997 and no correlation between terms with disjoint supports; with the Livox timing the
  triples sit about 5 % under 1 and the start term about 1.9, both equal to the analytic value for the actual sample
  supports (the model world's white noise at D = 0.01057 m/s²/√Hz matches the shipped weight at one span length, 11 ms,
  only). **F11** (new observation): the anchor rule (`window_equations.cpp:27`, solver's `layout_window_unknowns`,
  `>= span_s` on rounded timestamps) puts the anchors 12 ms apart in 89 % of steps for a 10 ms span at 500 Hz; the
  weights follow the actual span. *Rotation edges:* variance 1.009 / 1.021 / 1.009 of the assumed max(k ARW √dt, floor)
  (k 1, floor 0, gain 0, behind the filter's ramp). **F13** (new observation): the edges' float floor in motion is
  4.6e-7 rad, 1.6-1.9× the float32 storage bound (probably Fusion's float arithmetic), below 0.1 of the edge sigma.
  *Priors over the gap:* F P F^T + Q exact to 1e-16 and the gap noise 0.998 of D² gap (the mean is F2a, inertial).
  *A window's covariance (F-c):* 0.88-1.06 per dof in four sets of 277 runs, inside the 1 % intervals, in the fixture's
  own world (accelerometer floor 0 with a matched density, a = 0.8 m/s², F2a left out; 26 and 23 corridor runs excluded
  as not blind; the corridor sets at the iteration cap); in the corridor the sideways and vertical velocity reads
  0.64-0.85, below its interval (the claim larger than the scatter), cause not isolated. C34's set and C37 (K10): Known
  flaws. *NDT information scale:* for a pure NDT solve the shipped 0.1 divides K8's scatter ratio by 3.162 (room
  0.32-0.90); in a real window the other terms share the solve, so not exact (lidar-matching). Not concluded: that the
  candidate settings as they are hold these claims; anything on a recording.
- 2026-11-20 consistency-harness 3.1 (9a7d414, merged 527f18d): `WindowPreparedState` / `WindowHandedOnState` /
  `WindowHandOff` in the per-window record; `mean_rotation_rate` (from the prepared poses' attitude increments) and
  `start_world_acceleration` (first 30 ms) in `window_term_table`; 71 columns appended to `window_features.csv` (164 in
  all). Step checks: all identical except the features table; its old prefix on 44_short 20-40 s: 2046 tokens, 0
  different; `window_terms.csv` identical. The loop fills the values in task 3.3.

- 2026-11-19 Joint (start velocity, offset) prior for layer 2 of `honest-inertial-carry` (job J4a, ef95898): fields
  `joint_prior*` in `WindowNoiseModel`, helpers `joint_prior_cov` and `joint_prior_cov_over_gap` (transition
  F = [[I, -gap I], [0, I]], then today's growth). An offset-only chain of 15 gaps gives the coherent velocity sigma
  (sigma x N x gap); off: step checks identical. Open: should the offset's random walk during the gap feed the velocity
  block too (today it grows over the next window's duration only)?
- **2026-11-18** Livox gyro weight (stage 8, change `livox-measured-gyro-noise`): the gyro's measured noise (ARW 0.0094 /
  0.0224 / 0.0144 °/√s, k 1, floor 0.0002°) in place of the L2's values with k 7.09. Full Livox 4: windows bent > 3°
  96 → 0, largest speed 8.5 → 1.62 m/s, end height −78 → −0.7 m, largest bias estimate 5.1 → 0.58 °/s; Livox 3:
  largest bend 1.98 → 0.30°, start-to-end 0.08 → 0.07 m. k 1 alone (L2's ARW) still gives speed bursts to 5.5 m/s on
  the flyaway capture. The user's look: Livox 4 "a huge win" with "rotation drifts generally", Livox 3 "crisp as
  expected". Shipped in `livox-vertical.toml`. `experiments/stage8/`
- **2026-11-18** Gyro scale: with the measured gyro noise the yaw bias estimate follows the yaw rate (slope −0.0022 …
  −0.0032 °/s per °/s, R² 0.20–0.25 on Livox 4, 0.46–0.58 on Livox 3) and not the yaw acceleration (R² 0–0.02). A
  full Livox 4 run with the yaw rate × 1.004 brings the slope to about 0 (R² 0.00–0.10) and the run closes:
  start-to-end 3.73 → 0.77 m, loop corrections' rotation median 2.1 → 1.2°. The user's look: "even better". Not
  shipped (a scaled copy of the recording; needs a setting or an estimated scale). `experiments/stage8/`
- **2026-11-18** A prior with a sigma on node 0's rotation in place of the hard fix (experiment branch
  `region/lidar-matching/2026-11-18-rotation-prior-experiment`, 5869043, replays): σ 0.02° with the scale correction
  narrows the yaw bias estimate's range (passage −0.23..+0.37 → −0.10..+0.22 °/s) and, with the in-plane part removed,
  turns the end-stretch runaway (3.3 / 4.2 m/s) into one burst of 2.4 m/s (mean 0.12 m/s while the user stands). But
  σ 0.02° on the stairs capture runs away to 4.6–5.0 m/s after node 0 pitch turns of 0.5–0.85° per window: the NDT's
  claimed rotation hold (about 0.01°) outweighs the prior. σ 0.005° changes little, σ 0.08° bends the passage by 1.3°.
  One replay per variant; other sizes on the stairs not tried. `experiments/regions/lidar-matching/2026-11-18-rotation-prior/`.
- 2026-11-18 Stage 5 (`window-noise-model`): one model per window, pure refactor. 44, Livox 3, 44_short IDENTICAL
  (4b48184); 44_short with `log_window_noise`: 70 `window noise` lines, 28 `noise_*` columns; Debug ctest pass.
  `ndt_window_cost` now follows the term's per-grid rules; the 26 weak-axis lines of the 44_short replay are unchanged
  (use_norm and planarity off in both shipped settings). `openspec/changes/archive/2026-11-18-window-noise-model/`.
- 2026-11-17 Verdict limits kept by the user after full runs of 44, Livox 3, Livox 4: the velocity chi² of the weak
  windows triggers on all three (Livox 3 median 35; Livox 4 passage median 1120 against 230 outside). Plan progress,
  "Verdict limits decided".
- 2026-11-17 First reading of the table (Livox 3 100–130 s, 84 windows): the LiDAR carries 90 % of the stretch-along
  weight (median); the gyro 0.13 % of the heading-bend weight, its hold 0.242° = 0.057 °/√s × 7.09 × √0.358 s (0.034°
  without k), NDT ~800:1; velocity prior cost per residual median 8.65 (max 65.4), its σ along travel 4.6 mm/s; edge
  positions hold stretch at 4.21 mm; projection keeps 14–15 % of NDT in windows 313–314. `docs/window-terms.html`.
- 2026-11-17 Stage 3 (`per-term-window-log`): table on +7–9 % time; Livox 3 replay 84/84 identical with it on.
  `openspec/changes/archive/2026-11-17-per-term-window-log/`.
- 2026-11-16 Huber 3 + NDT information ×0.1 (α) + position information ×0.1 (β) ported from Codex's side build as
  settings; with the dynamic AHRS gain removed, run G is the user's "best so far" Livox 3 map; `livox-vertical.toml`
  carries the three. Memory note `livox-state-2026-11-16.md`.
- 2026-11-12/13 Phase 0b, NDT information multipliers 1 / 0.3 / 0.1 / 0.03 / 0.01: Livox 2 at 0.3 and 0.1 local visual
  progress (user: 0.1 "no visible double walls"); Livox 1 at 0.1 severe deformation, ~31.24 m largest trajectory
  difference from control. `openspec/changes/ndt-information-probe/results.md`.
- 2026-11-12 β 0.1 with α 0.1: Livox 2 "looks really good" (user), Livox 1 doubled walls (user). Same change,
  `motion-prior-results.md`.
- 2026-11-10 Yaw hold in a window: gyro ~34 per deg² against NDT 7e3–2e5 per deg². Sweep of k on livox/1 (bends > 3° /
  largest): 7.09 16 / 18.0°; 4 6 / 13.7°; 2.257 6 / 16.4° (broken); 1 0 / 2.4°, but small perturbations still move it
  > 1 m. livox/2 at k = 1: the break at 307 s gone, bends > 3° in 0–300 s 64 → 15. Memory note
  `fast-turn-fragility-state.md`.
- 2026-10-17 D4 (C5): the Cauchy factor removed, sigmas ×√π. 44_short moved up to 3.1 cm (the dropped (1+x²) factor,
  ~1e-6 of the weight, amplified); six-cut gallery heading mean −4.52° vs −4.31° (range −4.71…−4.02): accepted. Memory
  note `fix-plan-state.md`.
- 2026-10-09 Synchronized retune on 44 (three runs, IMU +0/+1/+2 ms): k 4 vs 1 vs accelerometer off equal within run
  noise; loop closure chaotic (largest correction 0.8–6.3 m). `openspec/changes/synchronized-retune/tasks.md` 1.1.
- 2026-10-09 Accelerometer weights: density 0.0013 m/s²/√Hz measured on `raw data/43/static`; floor 0.1 kept (the 50 ms
  misfit is 1.37 m/s² rms of walking motion); floor 0.3 similar; offset random walk 0.01 lost half the loops in one run,
  0.0001 kept. `openspec/changes/archive/2026-10-09-imu-always-on/tasks.md` 1.0, 1.1, 3.1.
- 2026-09-30 Time-scaled rotation σ introduced: on dataset 43 node yaw jitter 0.343 / 4.13° (constant 0.1°) against
  0.039 / 0.24° (0.02°). `openspec/changes/archive/2026-09-30-gyro-bias-estimation/proposal.md`.

## Ideas rejected

- **Rescue weights for the stairwell window 213** (re-solve with inflation k 1, a 0.02 m/s² accelerometer floor):
  no help; removed (`2026-10-09-imu-always-on/tasks.md` 3.1).
- **Offset random walk 0.01 m/s²/√s**: one 44 run lost half its loops; 0.0001 kept (same source).
- **NDT information scale 0.1 as a global value**: fails the Livox 1 cross-check (phase 0b, `results.md`); it stays a
  Livox setting, not a default.
- **Soft range weight w = 1/(1+(d/d₀)²)** for far points (d₀ 4/6/8 m on 44 cuts): gallery heading −4.91/−4.93/−4.53°,
  no gain; removed (2026-10-17, memory note `fix-plan-state.md`).
- **The dynamic AHRS gain switch** (not a window weight, recorded for its interplay): with Huber 3 and the two 0.1
  scales it gave loop-closure corrections up to 1.48 m (0.33 m without); removed 2026-11-16 (tracker C3).

## Open questions

- Velocity chi²: is the start-velocity prior too tight (its σ along travel ~4.6 mm/s on Livox 3), or is process noise
  between windows missing? Stage 5 put the covariance in one place so this can be measured as its own change.
- C34: what do looser edge translation σ under `imu_always_on` do (44_short: result − guess, start-velocity chi²,
  `trajectory_seams.py`)?
- (Answered for the Livox 2026-11-18: its file carries the measured ARW with k 1, see Experiments. Open: the L2's k.)
- The rotation model: each window's node 0 is hard-fixed (`fix_origin` adds diagonal weight, no anchor) and the edges
  follow the gyro, so the attitude is corrected only through the gyro bias estimate, which swings by ±0.4 °/s while
  walking. A fixed-sigma prior on node 0 is not safe as tried (Experiments). To design: a prior whose size follows
  what the previous window knew, or a tilt correction from gravity the LiDAR cannot override; a gyro scale state.
- Formerly: the Livox ran with the L2's ARW and k 7.09; the Livox gyro at rest is 13–30× better (memory note
  `livox-state-2026-11-15.md`), the yaw edge σ ~17× its own noise. A sensor-measured gyro weight is open; k = 1 removes
  bends on livox/1 but not the sensitivity to perturbations.
- `log_weak_axis_diagnostics` computes the motion model's stiffness from `mm.x_sigma_m` without
  `imu_position_information_scale`, so on the Livox (scale 0.1) it reports 10× the stiffness the edges have (found
  while writing this file; now tracker row C39).
- Accelerometer density on the Livox: 0.0013 is the L2's; Livox measured 0.003–0.011 (memory note
  `fast-turn-fragility-state.md`).
- At the guess the accelerometer row is ~3000× its result cost (Livox 3 point 8): read only result rows until the
  start value is settled (inertial).

## See also

- [Regions](README.md)
- [The per-term table](../window-terms.html)
- [Degeneracy information](../degeneracy-information.html)
- [The code-split plan](../code-split-and-region-agents-plan.md) (stages 3 and 5)
- [Code-review tracker](../code-review-tracker.md)
- [tools/README.md](../../tools/README.md) (`window_terms.py`, the `noise_*` columns)
- [All documents](../README.md)
