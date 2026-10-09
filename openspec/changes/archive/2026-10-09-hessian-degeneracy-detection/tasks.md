# Tasks

**Checks on every code commit:**
- the Release build of `dMapper` and `lidar_odometry_tests` passes, and `ctest` passes;
- with the new setting off, `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`.

Judging a setting means the full 44 with both builds:
- the precise Release build, and the `/fp:fast` build of the same code (the scratch worktree with `/fp:precise` replaced, built outside the presets);
- compared with `tools/trajectory_seams.py` (between the builds, and against `raw data/44_baseline`, which is checked free of ghost walls);
- `eval_odometry.py` and the loop-closure corrections.

Close the GUI before building. Never run two jobs on the same input folder at once. Run background jobs from copies of the binaries.

## 1. Derive and measure (results unchanged)

- [x] 1.1 Derive, for dMapper's point-to-distribution NDT term, the information that noise alone adds to a direction (λ_noise).
  - Base it on Hatleskog & Alexis 2025 (arXiv:2410.10784; their code is at github.com/ntnu-arl/drpm).
  - Noise sources: the point noise from the sensor data sheet, and the uncertainty of the cell's mean and covariance from its point count and spread.
  - Write the derivation and the chosen constants into `design.md`.
  - If it doesn't carry over soundly to NDT, decide for the X-ICP count test (design, decision 2) and write down why.

  Result: derived. See `design.md`, section "Derivation (task 1.1)".
  - NDT is decomposed into point-to-plane terms; only the normal term of planar cells is used, with unit weights.
  - Rigid window motion about the window's mean node position; noise as in the paper, Eqs. 7, 8 and 34, with the cell's point count and in-plane spreads.
  - Translation and rotation are separated by Schur complements, and the test is the paper's Eq. 20.
  - Computed once per window, in the first iteration (as LIO-SAM decides degeneracy), and held.
  - Constants: σp = `degenerate_point_sigma_m` 0.02 m (Livox Mid-360 class assumed; the sensor is a Unitree L2, also ≤ 2 cm), s = 10.
  - The count test is not needed. The point-sigma setting is a fifth key next to the four of the delta spec; the spec is amended.
- [x] 1.2 Compute the window's rigid-motion information and its translation and rotation Schur complements, and the per-direction p (design, decisions 1 and 2), as a separate, tested helper.
  - Tests: synthetic room, nothing degenerate; synthetic corridor, translation along the axis degenerate; a round synthetic tunnel, rotation about the axis degenerate; noise added to the corridor's cells doesn't make the axis look constrained; invariance under moving the world origin.
  - Log per window behind `log_window_diagnostics`: the three translation and three rotation p values with their directions, next to today's ratio.
  - Verify: `ctest`; `44_short` IDENTICAL. Commit.

  Result:
  - **Helper:** `src/domain/degeneracy.{h,cpp}` with `PlaneObservation`, `normal_rotation_covariance` (Eq. 34), `is_planar_cell` and `detect_degeneracy`.
  - **Cells:** they store their eigen-decomposition (`frame_spreads`, `frame_vectors`) when the test or its diagnostics are on (`refresh_cell_inverse`, `update_rgd(..., store_frame)`).
  - **Optimizer:** `optimize_lidar_odometry` collects the planar observations per node in point order and runs the test once per window. The result is held in `ScanToMapWorkspace::degeneracy`, and the window loop marks each window's first call.
  - **Log:** a `window degeneracy N` line behind `log_window_diagnostics`.
  - **Settings:** `degenerate_snr_detection` (false), `degenerate_snr_factor` (10), `degenerate_point_sigma_m` (0.02); golden files in their own commit.
  - **Tests** (`tests/degeneracy_test.cpp`):
    - room: all six p = 1;
    - corridor: p along the axis < 0.01, the rest 1;
    - round tunnel: translation along the axis and rotation about it < 0.01;
    - corridor with 2 deg normal noise: p along the axis still about 1e-217;
    - shifted by (1000, -500, 20) m: the same p.
  - A clearly constrained direction is set to exactly 1 above 1 − 1e-6 (z > 4.75); the room showed 1 − 1e-10.
  - **Checks:** `ctest` passes; `44_short` identical data.
- [x] 1.3 Full 44 with the diagnostics (precise build). Report p along the blind passage at t = 123 to 126 s (windows 303 to 311), at the two-solution place around t = 74 s, and in the first room, and compare with today's ratio. Choose `degenerate_snr_factor` (start at the paper's 10) and the constants from this, without fitting to single windows. Record the choice here.


  Result: full 44, precise build, `log_window_diagnostics` on (results unchanged); p per direction, s = 10, σp = 0.02 m.
  - **First room** (windows 1 to 11, ratio 0.31 to 0.47): all translation and rotation p = 1, except window 1's rotation at 0.98.
  - **Area around t = 74 s** (windows 178 to 191): translation along x at 0.65 to 0.88 in four windows, otherwise 1.
  - **Blind passage:** translation along x (direction 0.95 to 0.99 x) at **0.45, 0.018, 0.019 in windows 304 to 306**. From window 307 it is back to 1, with information 300 to 470, while the ratio stays 0.27 to 0.34. The transition is steep (window 306 at information 255 gives 0.02, window 307 at 357 gives 1): soft in form, nearly a switch in practice.
  - **Rotation:** one axis per window often below 1 (windows 298 to 302: 0.37 to 0.75).
  - **Over the whole run:** translation p < 0.5 in 110 of 690 windows (< 0.99 in 178); rotation p < 0.5 in 139 (< 0.99 in 328). The ratio projection shipped today acts in 16 windows (ratio < 0.2; none below 0.1).

  Choice for task 2: s = 10 and σp = 0.02 m as derived, not fitted to windows. Task 3 also judges s = 5, which flags fewer directions. The frequent rotation flags are the main risk: the gyro and the motion model then decide those rotations.
## 2. Attenuation

- [x] 2.1 Scale the NDT node blocks by p per direction, translation and rotation (design, decision 3), behind `degenerate_snr_detection` and `degenerate_snr_factor`.
  - If both detectors are on, use the signal-to-noise test and log a warning.
  - Settings in `TomlIO` and the shipped TOMLs, off; golden TOML files regenerated in their own commit.
  - Tests: the scenarios of the delta spec (corridor, room bit-identical, tunnel rotation, noise, off by default), and multithreaded equals single-threaded.
  - Verify: `ctest`; `44_short` IDENTICAL with it off. Commit.


  Result:
  - **Implementation:** with `degenerate_snr_detection` the node blocks are buffered like the ratio projection's. Each node's block becomes Mᵀ B M, with M = diag(P_t, E⁻¹ P_r E), the p values of the window's verdict, and E taken numerically per node.
  - **Precedence:** the ratio path is unchanged and uses the same expressions as before when only translation is scaled. Both settings on gives a warning per run, and the noise-aware test is used.
  - **Optimizer tests** (σp = 0.005 m there, the synthetic scenes' jitter: with 2 cm on exact points the sparse synthetic room looked degenerate, p down to 4e-30):
    - corridor: x p ≈ 1e-138, the change along the axis 0.012 mm, the error across 5.8 mm, multithreaded equals single-threaded;
    - room: all p = 1, bit-identical with the test on or off;
    - round tunnel: translation along the axis and rotation about it p ≈ 1e-240 and 1e-141. The roll changes by 2.5e-5 deg against the guess (0.086 deg without the test, the false in-plane stiffness), and the error across the axis is 0.23 mm (0.22 mm without).
  - **Checks:** `ctest` passes; `44_short` identical data with the test off.
## 3. Judge on the data

- [ ] 3.1 Full 44, both builds, with the signal-to-noise test on and the ratio projection off. Variants are the noise constant and s around the derived values, each reported with:
  - the difference between the builds;
  - the difference from `44_baseline`;
  - loop-closure corrections, metrics and time cost.

  The target is the builds within a few centimetres, at least clearly below today's 0.31 m. Pick the shipped settings by that. The user checks the cloud for ghost walls (first room, t ≈ 74 s area). If no variant beats today's projection, keep the projection shipped and record why. Commit.


  **Paused by the user on 2026-10-09 ("it doesn't help"); this task is not done.** Measured so far (full 44):
  - **Shipped settings, precise build:** identical data to `tools/baseline/44.manifest.json` (only the parameter file differs, by the new keys), so the ratio projection is unchanged by this change's code.
  - **SNR test at s = 10** (ratio off), both builds:
    - the builds differ by 7.45 m (shipped projection: 0.31 m);
    - against `44_baseline`, 9.60 m for fast and 6.78 m for precise;
    - loop closure accepted 3 / 8 loops, with corrections of 6.40 / 5.19 m;
    - route 605 / 639 m (about 550 m shipped);
    - processing time 714 / 672 s (shipped 587 s, so 15 to 20 % slower, over the 5 % budget).
  - **What improved is rotation consistency:** window yaw difference 0.91 to 1.00 (1.38 shipped), spikes above 3 deg 1 (5), final yaw about 6 deg from the gyro (10.5 deg). Scaling down translation and rotation in about a fifth of the windows lets the motion model drift by metres.
  - The s = 5 pair was stopped before it was compared.
  - **Outcome for now:** the shipped settings stay as they are (ratio projection 0.1 / 0.2). The detector, the attenuation and the diagnostics stay in the code, off by default, and tested.
  - **Possible resumption:** use the test for rotation only and keep the ratio projection for translation; measure the cost first.
## 4. Baselines and docs

- [ ] 4.1 If the shipped settings change: new baselines (old folders renamed with a date suffix, both manifests rewritten from the final code).
  - Docs: `docs/degeneracy-information.html` and the Hungarian copy, the settings tables of `docs/index.html` and `docs/hu/index.html`, `docs/PERFORMANCE.md` if the time changes, `CLAUDE.md` if it describes the detector.
  - Memory notes. Commit, archive (sync the delta spec), push.

## Outcome (2026-10-09): rejected, code removed

- **Rotation-only variant**, tried at the user's request:
  - full 44, precise build only (short test);
  - SNR test on rotation, ratio projection on translation;
  - 8.29 m from `44_baseline` (above 50 cm from t = 18.9 s);
  - loop closure 11 loops instead of 21;
  - 651 s instead of 587 s.
  - Better heading consistency again: window yaw difference 0.98, final yaw 3 deg from the gyro. Not adopted.
- **The code is removed.** At the user's request, the detector, the attenuation, the settings and the tests are not kept switched off. Every file outside `openspec/` is restored to the proposal commit `b8258af`; `ctest` passes and `44_short` is IDENTICAL with the manifest. The implementation is in git history (commits up to `4f1d937`) if it is ever wanted again.
- **The delta spec is not synced:** the main spec keeps the ratio projection.

