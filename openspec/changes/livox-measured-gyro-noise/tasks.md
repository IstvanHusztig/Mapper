# Tasks

## 1. Settings

- [x] 1.1 `settings/livox-vertical.toml`: `gyro_arw_x/y/z_deg_sqrt_s` 0.0094 / 0.0224 / 0.0144,
  `rotation_sigma_inflation_k` 1.0, `rotation_sigma_floor_deg` 0.0002; the "TODO: measure" comment replaced by the
  source (`raw data/livox/4`, 2–20 s at rest, 2026-11-15) and the reason for k = 1.
  Result: fb9e80c. The values equal those of the runs the user looked at (`experiments/stage8/full_gyro_livox{3,4}.toml`).
- [x] 1.2 The golden TOML `tests/data/livox_vertical.golden.toml` regenerated (`LO_UPDATE_GOLDEN=1`), in its own
  commit. Verify: Release build, all unit tests pass.
  Result: golden file in its own commit (five value lines differ); Release build, 163 test cases pass (one
  `should_fail` case fails as expected). Debug not run: a settings change cannot depend on the configuration.

## 2. Reference results

- [ ] 2.1 Full Livox 4 and Livox 3 runs with the committed file (Release, one after the other, through the gate).
  Verify: `tools/trajectory_seams.py` against `experiments/stage8/full_gyro_livox4` / `full_gyro_livox3` reports
  0 difference; the run check gives bend and speed CLEAR.
- [ ] 2.2 Old folders `raw data/livox/4_baseline` renamed `4_baseline_2026-11-18`; the new outputs moved to
  `raw data/livox/4_baseline` and `3_baseline`; manifests written with `compare_baseline.py --write-manifest
  --dataset livox4|livox3 --params settings/livox-vertical.toml`, in their own commit.
- [ ] 2.3 Unchanged elsewhere: `tools/step_checks.py` ALL HOLD (its Livox 3 capture has its own settings file) and
  44_short IDENTICAL.

## 3. Docs

- [x] 3.1 `docs/index.html` parameter table: the gyro rows say the values shown are the L2's and that the Livox file
  uses its measured noise with k = 1 (general terms, no recording names).
  Result: the three gyro rows of the parameter table and the troubleshooting row; no recording names.
- [x] 3.2 `docs/window-terms.html`: the example's settings named as the former values (inflation 7.09), numbers kept.
  Result: the example's settings row names the former values and what the new ones change (the gyro rows' hold).
- [x] 3.3 `docs/regions/noise-budget.md` (Experiments, Open questions: the k and ARW question answered for the Livox)
  and `docs/regions/inertial.md` where they cite the old values; `livox-sensor` notes in memory.
  Result: noise-budget Experiments and Open questions updated; `inertial.md` cites no gyro values; memory updated.

## 4. Close

- [ ] 4.1 The spec delta synced on archive (the user is asked); push only when the user asks.
