## Why

The points of a recording are sorted by timestamp with `std::sort`, which is not stable, and many Unitree L2 points share a timestamp. The order of points with equal timestamps then depends on the whole array, so the same data gives a different order, a different voxel decimation (`decimate` keeps the first point of each voxel after a second unstable sort, C14), a different map and a different trajectory depending on how much recording follows: recording 44 cut to its first 95 s with `cut_recording` differs from the full run by more than 5 cm from 1.7 s on and by 0.5 m at 88 s (2026-10-15). This makes a short test bed for odometry changes (change F/E) unusable as an exact copy, and it is a source of run-to-run fragility between near-identical inputs, the noise every map-quality verdict has to beat.

## What Changes

- Points with equal timestamps keep their order in the recording (`std::stable_sort` in the loader).
- `decimate` keeps the first point of each voxel in that order (stable sort by cell index); the consistency pass's per-cell point order is stable as well.
- Numbers change (a different representative point per voxel): new baselines in their own commit.

## Capabilities

### New Capabilities

### Modified Capabilities
- `processing-pipeline`: the result for the windows a recording cut contains no longer depends on the data after the cut.

## Impact

- `src/application/lidar_odometry.cpp` (load), `src/domain/ndt_map.cpp` (`decimate`), `src/application/consistency.cpp` (per-cell order).
- Unit test for `decimate`; 44 and 44_short baselines.
