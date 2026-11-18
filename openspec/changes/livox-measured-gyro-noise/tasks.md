# Tasks

## 1. Settings

- [ ] 1.1 `settings/livox-vertical.toml`: `gyro_arw_x/y/z_deg_sqrt_s` 0.0094 / 0.0224 / 0.0144,
  `rotation_sigma_inflation_k` 1.0, `rotation_sigma_floor_deg` 0.0002; the "TODO: measure" comment replaced by the
  source (`raw data/livox/4`, 2–20 s at rest, 2026-11-15) and the reason for k = 1.
- [ ] 1.2 The golden TOML `tests/data/livox_vertical.golden.toml` regenerated (`LO_UPDATE_GOLDEN=1`), in its own
  commit. Verify: Release build, all unit tests pass.

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

- [ ] 3.1 `docs/index.html` parameter table: the gyro rows say the values shown are the L2's and that the Livox file
  uses its measured noise with k = 1 (general terms, no recording names).
- [ ] 3.2 `docs/window-terms.html`: the example's settings named as the former values (inflation 7.09), numbers kept.
- [ ] 3.3 `docs/regions/noise-budget.md` (Experiments, Open questions: the k and ARW question answered for the Livox)
  and `docs/regions/inertial.md` where they cite the old values; `livox-sensor` notes in memory.

## 4. Close

- [ ] 4.1 The spec delta synced on archive (the user is asked); push only when the user asks.
