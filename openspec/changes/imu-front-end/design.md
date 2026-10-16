## Context

- **Test bed:** the exact 95 s cuts `raw data/44c95b_s0..s5`, odometry only (`odo_master.toml`: `lidar-vertical` with loop closure and consistency off, window diagnostics on). The master references are `44c95bo_master_s0..s5`.
  - gal−A: M = −4.31, range [−4.71, −4.02].
  - A−B: 0.89–1.00.
  - gal−B: −3.16…−3.82 on s0–s2.
- **Full runs:** `44_s0..s5` and `45f` with `settings/lidar-vertical.toml`, against master's `44_g3b2bs6_s0..s5` (`mq_g3b2bs6.txt`) and `45f_cand`.
- **IMU references** (`hand_eye_gyro.py`, master odometry-only full map `44_c10o_s0`):
  - gyro bias +0.61 / +0.46 / −0.82 °/s (body, ± 0.05);
  - yaw scale −0.58 ± 0.25 % (halves −0.19 / −1.23 %);
  - misalignment ≈ 0.6° about x and y.
- **F/E's V5** (finer cells) is parked. D is measured on master's cells.

## Fixes

- **D3 (C6), bias measurement:**
  - `apply_gyro_bias_prior` removes the carried bias b₀ from the guess and from `motion_model_poses`.
  - `optimize_lidar_odometry` then subtracts the full estimate b again: the measurement becomes raw − (b₀ + b) dt, and the estimate settles near half the true bias.
  - Fix: subtract (b − b₀) dt (b₀ = `rotation_options.bias_prior_mean`). Jacobian and prior unchanged.
  - Red test first: at least three chained synthetic windows with a known gyro bias and a carried b₀ ≠ 0 (with b₀ = 0 the bug is invisible). Today the estimate settles near half the bias; after the fix within 5 % of it.
  - The tail re-propagation (`repropagate_window_tail`, db = b − b₀) already assumes the corrected meaning; the carry (full b) and the clamp stay valid.
- **D4 (C5), prior weight:**
  - Remove `cauchy(·, 1)` from the three rotation weights.
  - Multiply by √π, in the shipped settings and the code defaults: `rotation_sigma_inflation_k` (4.0 → 7.0898), `rotation_sigma_floor_deg` (0.002 → 0.003545), the constant `lidar_odometry_motion_model_{om,fi,ka}_1_sigma_deg`, and `rotation_sigma_inflation_k_turn` when it is > 0. The effective prior is then unchanged up to the (1 + x²) factor.
  - Everything that states the old values is updated too: `docs/index.html`, `docs/hu/index.html` (k = 4.0, "not below about 2"), `openspec/specs/lidar-odometry-motion-model/spec.md` (k = 4; a delta spec in this change), the fixtures in `tests/scan_to_map_test.cpp`, and the job-tmp experiment TOMLs when they are used again.
  - Golden TOML files regenerated in their own commit.
  - Expected: within 5 mm / 0.01° of master on `44_short` and the cuts. Otherwise investigate before going on.
- **D1 (C2), AHRS start-up:**
  - The adapter calls `FusionAhrsUpdate(…, FUSION_VECTOR_ZERO, dt)`, i.e. `FusionAhrsUpdateNoMagnetometer` without its heading reset during initialisation.
  - The gain ramp (10 → 0.05 over 3 s, tilt following the accelerometer) stays.
  - Unit test: a constant yaw rate during the first 3 s gives the integrated yaw, not 0.
  - Side effect: the windows start at ≈ 0.9 s, inside the heading lock, so D1 rotates the whole world frame. Absolute A / B, the "≤ 5 m off master" check and `trajectory_seams.py` are only valid after a yaw alignment at the start; differences (gal−A, gal−B, A−B) stay valid. Expected: ≈ no change of A−B (A's halves inside and after the initialisation already agree).
- **D2 (C3), dynamic gain (a probe first):**
  - First measured: the fraction of samples the ±0.1 g gate sets to gain 0 on 44 and 45f (a single-sample gate probably fires often while walking).
  - Variant a: `set_gain` goes through `FusionAhrsSetSettings` with the adapter's own settings copy (not `ahrs_.settings`, which holds converted rejection fields). After the initialisation, Fusion's `rampedGain` follows the gain. Unit test: after the initialisation, gain 0 leaves the tilt unchanged under a tilted accelerometer. Readings plus a tilt reading (floor normal against the AHRS gravity).
  - Variant b: the dynamic gain removed (bit-identical, since it has no effect today); only with the user's decision (a feature that misses its purpose is not removed without asking).
  - Neither touches the initialisation ramp.
- **D5 (C15), hand-off:**
  - The windows stay disjoint; `place_window_after` advances the pinned first node by the IMU's own motion between the previous window's last sample and this window's first sample (relative pose of the two IMU poses), instead of giving it the previous sample's pose.
  - The points between the previous window's last node and this window's first node (≈ 1.1 ms, today in no window) go to the previous window's last node.
  - Chosen over shared boundary samples, which would duplicate a node at every boundary: the consistency pass flattens all window poses and its smoothness term (σx 1 mm) assumes uniform spacing, `save_result` would write duplicate rows, the window count changes, and points exactly at ts[0] would be dropped.
  - Red tests first: chained synthetic windows under a constant yaw rate (today they lose 1/N of the rotation; after the fix below 1e-6 rad), and a `compute_step_1` fixture for the points between windows (no test calls it today).
  - Debug build and `ctest` for D5 (index and container changes); loop closure and consistency on `44_short` after it (the test bed is odometry only). Old snapshots stay valid (the window grid is unchanged); the windows' timestamps are unchanged.
  - Windows with ≤ 1000 points still leave a hole (logged as a silent event); out of scope.

## Probes and diagnostics (not shipped by themselves)

- **P1, prior strength** (after D3 + D4 are in): `rotation_sigma_inflation_k` on the test bed s0–s2 at the effective weight ×1 (the shipped value), ×π (k 4.0, the weight configured before the Cauchy factor) and ×π² (k = 4.0 / √π = 2.257).
  - The floor `rotation_sigma_floor_deg` is scaled with k, so the dose reaches the short-dt edges too.
  - It answers whether a stiffer gyro resists the gallery turn, and at what cost to consistency.
  - Adoption only through change I (the cave retune), or by the user's decision with the numbers.
- **P2, gyro scale and misalignment:** `hand_eye_gyro.py` on master's odometry-only 44 s0–s5 and 45f.
  - If the yaw scale is consistently beyond ±0.5 % with an SD below 0.2 %, a separate feature change (gyro scale estimation) is proposed.

## Rule (written 2026-10-16, before any run of this change)

- **Readings per run** (test bed): gal−A, gal−B, A−B (start split), and the start's absolute A and B.
  - Plus the regression slopes of the gyro residual (`yaw_resid.py`, reference bias from the hand-eye fit) over 0–10, 10–30, 30–71 and 71–85 s.
  - Plus the bias estimate at 10 / 30 / 85 s.
  - Sanity as in F/E: 0 big jumps, every window 100 iterations, the route on a 1 s grid within 10 % of master's, and at most 5 m off master at the end. Unusable otherwise.
- **Per fix** (s0–s2, against master on the same input). These are correctness fixes, so a gain is not required:
  - **accepted** if no reading is worse beyond the noise floor: for the angles, mean outside master's s0–s5 range and at least 0.5° from M on the worse side;
  - **correct-but-unmasks-compensation** if a reading is worse: kept in the code, the compensating settings are named, the user decides.
  - **Predictions** checked before acceptance; a missed prediction is investigated first:
    - D3: the z bias estimate's mean over 30–85 s within 0.15 °/s of the hand-eye z bias (−0.82 °/s); x and y are not comparable (Fusion's tilt feedback absorbs much of them; the hand-eye fit uses the raw gyro), unless fitted against the AHRS increments;
    - D4: ≈ identical to master;
    - D1: the windows in 0–3 s get yaw increments; A−B ≈ unchanged;
    - D5: the tests go green, |Δ gal−A| ≤ 0.5° on 44.
  - **Reference per fix:** the previous accepted state (the chain D4 → D3 → D1 → D2 → D5), so a worse fix doesn't contaminate the later ones; paired per-input deltas are reported and summed for the cumulative step (the 0.5° criterion can't see fixes expected to move ≤ 0.3°).
  - **"Worse" per reading:** gal−A and gal−B further from 0; A−B larger; jitter p99 higher; sanity failures. Diagnostic only: the residual slopes, the bias estimate, absolute A / B.
- **Cumulative** (all accepted fixes, s0–s5 cuts), then the full validation (44 s0–s5 loop-closed, `map_quality.py --floors`, paired against master; 45f):
  - global geometry not worse beyond master's spread;
  - consistency and sharpness each worse by less than half master's spread, and no spread wider by more than half (as in F/E 2a);
  - 45f: loops ≥ 90 % of master's, no runaway (route on a 1 s grid);
  - runtime and peak memory reported.
- **Order:** D4 (≈ no-op) → D3 → D1 → D2 → D5, each on its own commit with a red / green test where testable. `44_short` new baseline after each accepted numeric fix (own commit); the full-44 manifest and the dated folder renames once at the end. P1 / P2 after the cumulative test-bed run. `docs/PIPELINE.md` updated (hand-off, AHRS start). The user's visual check before the push.
