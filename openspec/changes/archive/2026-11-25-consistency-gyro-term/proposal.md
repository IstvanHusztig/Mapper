# Proposal

## Why

The consistency pass makes the map less precise where it matters. In `map-precision-leaks` (findings F2, tracker C1):
- it raises the trajectory's node-to-node rotation about 4× (0.041° → 0.167° per IMU sample on 44; the gyro shows 0.024°);
- on 44_short, points it never fitted get 1.85 mm thicker walls while the points it fitted get 2.5 mm thinner: it
  overfits each node's few points (~39 per node, about three nearly free degrees of freedom per node) instead of
  following the motion.

Its only constraint between nodes is a second-difference smoothness with a 0.1° rotation sigma, about 1000× looser
than the real angular acceleration, and no tie to the gyro. A rotation term between consecutive nodes from the gyro,
with the sigma the odometry already uses, made the never-fitted points 0.87 mm thinner than odometry only in a scratch
experiment. That was one 28 s recording at half point density, so it must be built properly and judged on the full 44
and its repeat runs. Sharpness measured on all points rewards overfitting, so the judge needs points the fit never saw:
a hold-out mode.

## What Changes

- **Gyro rotation term in the consistency pass** (variant 1, `Consistency`): between each pair of consecutive nodes, the
  relative rotation measured by the IMU orientation increments minus the window's estimated gyro bias, with the sigma
  `s × max(k × ARW × √dt, floor)` of the odometry's rotation prior (k, ARW, floor from the existing settings) and the
  scale `s` as a new setting. A window whose IMU rotation disagrees with its odometry result by more than a gate gets
  no gyro edges (recording 44 has a stretch where the gyro is wrong, C38). New settings in `[consistency]`, off by
  default; on in `settings/l2-vertical.toml` only if the evaluation accepts it.
- **Hold-out evaluation mode**: a setting that leaves the points of every other time block of a given length out of
  the odometry's, the loop closure's and the consistency pass's maps and point terms. All points are still saved, so
  the never-fitted points can be measured. Off by default (0); for evaluation only.
- **`tools/holdout_sharpness.py`**: wall thickness on planar spots of the held-out points only, paired between results,
  with an interval that includes the repeat-run spread.
- **`tools/wall_precision.py` fixes** from the post-push review: a pair-matched noise floor, 3D incidence, plan
  distance reported by sides, direction-independent length, no crash without the gyro-bias JSON entries, `--cell` /
  `--sigma`.
- If accepted: new 44 and 44_short baselines in their own commit; tracker C1 marked fixed.

## Capabilities

### New Capabilities

### Modified Capabilities
- `processing-pipeline`: adds the consistency pass's gyro rotation term and the hold-out evaluation mode (two new
  requirements); the processing sequence itself is unchanged.

## Impact

- `src/application/consistency.{h,cpp}` (gyro edges), `src/application/run_odometry.cpp`, `src/domain/scan_to_map.cpp`,
  `src/application/loop_closure.cpp` (hold-out), `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`
  (settings), `settings/l2-vertical.toml`, `settings/default_settings.toml`, golden TOML test files.
- Tests: `tests/consistency_test.cpp` (gyro term on the synthetic loop recording), a hold-out unit test.
- Tools: new `tools/holdout_sharpness.py`; `tools/wall_precision.py`; `tools/README.md`.
- With both settings off, results stay bit-identical (44_short and 44 manifests). Consistency time about +10 %.
