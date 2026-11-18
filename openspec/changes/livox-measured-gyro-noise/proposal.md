# Proposal

## Why

`settings/livox-vertical.toml` still carries the Unitree L2's gyro noise (ARW 0.028 / 0.042 / 0.057 °/√s) times an
inflation of 7.09, with a "TODO: measure" note. The Livox gyro measured at rest is 13–30 times quieter. With the L2
values the LiDAR decides every window's rotation: on Livox 4 windows are bent up to 14.7° against the gyro, the gyro
bias estimate runs to 5.1 °/s and the run flies away on a staircase (−78 m). Stage 8 of
`docs/code-split-and-region-agents-plan.md` (change `livox4-passage-and-flyaway`) measured the alternative in replays
and full runs, and the user looked at both maps (2026-11-18): Livox 4 "a step in the right direction, far from perfect
but overall a huge win"; Livox 3 "crisp as expected".

## What Changes

- `settings/livox-vertical.toml`: the gyro values measured on the Livox at rest (`raw data/livox/4`, 2–20 s):
  `gyro_arw_x/y/z_deg_sqrt_s` 0.0094 / 0.0224 / 0.0144, `rotation_sigma_inflation_k` 1.0,
  `rotation_sigma_floor_deg` 0.0002; the TODO comment replaced by where the values come from.
- The golden TOML of the Livox file (`tests/data/livox_vertical.golden.toml`), in its own commit.
- New reference results for Livox 3 and Livox 4: the manifests `tools/baseline/livox3.manifest.json` and
  `livox4.manifest.json` and the folders `raw data/livox/3_baseline` (new) and `4_baseline` (old ones kept with a date
  suffix), in their own commit.
- Docs: the guide's parameter table names the L2 values as the L2's and says the Livox file uses its measured ones; the
  per-term table page's example reading keeps its numbers (made with the old values) and says so; the regions' notes.
- `settings/l2-vertical.toml`, the defaults and all code are unchanged. The L2's own gyro is faulty and keeps its
  tuned inflation.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `sensor-settings`: the requirement "One shipped settings file per supported sensor" gains that the Livox file's gyro
  noise is the sensor's measured value, not the L2's.

## Impact

- Results of every run with `settings/livox-vertical.toml` change (intended). No other settings file, no code.
- Known limits, not fixed here: the rotation still drifts on Livox 4 (the user's look), the passage stall stays (6 m of
  about 10 m), the gyro bias estimate swings while walking. These are stage 8's open questions.
