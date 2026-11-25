# Proposal

## Why

`settings/livox-vertical.toml` cuts the points at 40 m, at loading (`filter_threshold_xy_outer`) and in the odometry and
loop closure (`max_distance_lidar`). That value is the Unitree L2's range, kept when the file was copied; the Livox
Mid-360S reaches 100 m (the user, 2026-11-25). On the corridor recording `raw data/livox/5` the far wall was seen at
40.7 and 42.9 m during the two turns and discarded.

## What Changes

- `filter_threshold_xy_outer` and `max_distance_lidar` become 100.0 in `settings/livox-vertical.toml`, with the reason in
  their comments. The L2 file and the built-in defaults do not change.
- The Livox results change wherever a return lies beyond 40 m: measured on Livox 3 (its baseline), Livox 4 and Livox 5
  before the new Livox 3 manifest is set.

## Capabilities

### New Capabilities

### Modified Capabilities

## Impact

- `settings/livox-vertical.toml`; `tests/data/livox_vertical.golden.toml`; the settings-hash pin of the Livox file in
  `tests/odometry_snapshot_test.cpp` (snapshots and captures made with the old file are refused with the new one: that
  is the meaning of the change); `tools/baseline/livox3.manifest.json` and the baseline folder, in their own commit.
- Runs: Livox 5 (7 min), the Livox 3 end run (20 min), Livox 4 (35 min), in the background, one at a time.
