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

- [ ] 1.1 Derive, for dMapper's point-to-distribution NDT term, the information that noise alone adds to a direction (λ_noise).
  - Base it on Hatleskog & Alexis 2025 (arXiv:2410.10784; their code is at github.com/ntnu-arl/drpm).
  - Noise sources: the point noise from the sensor data sheet, and the uncertainty of the cell's mean and covariance from its point count and spread.
  - Write the derivation and the chosen constants into `design.md`.
  - If it doesn't carry over soundly to NDT, decide for the X-ICP count test (design, decision 2) and write down why.
- [ ] 1.2 Compute the window's rigid-motion information and its translation and rotation Schur complements, and the per-direction p (design, decisions 1 and 2), as a separate, tested helper.
  - Tests: synthetic room, nothing degenerate; synthetic corridor, translation along the axis degenerate; a round synthetic tunnel, rotation about the axis degenerate; noise added to the corridor's cells doesn't make the axis look constrained; invariance under moving the world origin.
  - Log per window behind `log_window_diagnostics`: the three translation and three rotation p values with their directions, next to today's ratio.
  - Verify: `ctest`; `44_short` IDENTICAL. Commit.
- [ ] 1.3 Full 44 with the diagnostics (precise build). Report p along the blind passage at t = 123 to 126 s (windows 303 to 311), at the two-solution place around t = 74 s, and in the first room, and compare with today's ratio. Choose `degenerate_snr_factor` (start at the paper's 10) and the constants from this, without fitting to single windows. Record the choice here.

## 2. Attenuation

- [ ] 2.1 Scale the NDT node blocks by p per direction, translation and rotation (design, decision 3), behind `degenerate_snr_detection` and `degenerate_snr_factor`.
  - If both detectors are on, use the signal-to-noise test and log a warning.
  - Settings in `TomlIO` and the shipped TOMLs, off; golden TOML files regenerated in their own commit.
  - Tests: the scenarios of the delta spec (corridor, room bit-identical, tunnel rotation, noise, off by default), and multithreaded equals single-threaded.
  - Verify: `ctest`; `44_short` IDENTICAL with it off. Commit.

## 3. Judge on the data

- [ ] 3.1 Full 44, both builds, with the signal-to-noise test on and the ratio projection off. Variants are the noise constant and s around the derived values, each reported with:
  - the difference between the builds;
  - the difference from `44_baseline`;
  - loop-closure corrections, metrics and time cost.

  The target is the builds within a few centimetres, at least clearly below today's 0.31 m. Pick the shipped settings by that. The user checks the cloud for ghost walls (first room, t ≈ 74 s area). If no variant beats today's projection, keep the projection shipped and record why. Commit.

## 4. Baselines and docs

- [ ] 4.1 If the shipped settings change: new baselines (old folders renamed with a date suffix, both manifests rewritten from the final code).
  - Docs: `docs/degeneracy-information.html` and the Hungarian copy, the settings tables of `docs/index.html` and `docs/hu/index.html`, `docs/PERFORMANCE.md` if the time changes, `CLAUDE.md` if it describes the detector.
  - Memory notes. Commit, archive (sync the delta spec), push.
