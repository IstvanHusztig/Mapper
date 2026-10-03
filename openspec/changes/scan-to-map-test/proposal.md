# Proposal

## Why

The scan-to-map optimizer `optimize_lidar_odometry` (`app/scan_to_map.cpp`) is the core of dMapper, and it is the one algorithm that has no test of its own. The only check today is the golden baseline: a 3.5- or 30-minute run on recorded data. The baseline says *whether* a result changed, not *whether it is right*, and it can't separate the optimizer from the rest of the pipeline.

What the Goals in `docs/ARCHITECTURE.md` call a testable core needs the optimizer to run on synthetic data with a known answer. That would let the next changes be checked against ground truth, not only against the old numbers, for example the longer velocity span and the later layering in step 7.

This is roadmap step 6c, the last of the three changes the user split step 6 into on 2026-10-03.

## What Changes

- **A synthetic scene and window in the tests** (`tests/scan_to_map_test.cpp`):
  - a closed box room (floor, ceiling, four walls) sampled as points, from which both NDT reference maps are built with the app's own `update_rgd`, as `seed_map` does;
  - a window moving through the room, with nodes every 10 ms; each node sees part of the scene, given in its own frame from the true pose.
- **Tests of `optimize_lidar_odometry` against the known truth**, run like `run_lm_iterations` does (the damping grows ×10 every 10 iterations):
  - **a perfect guess stays put:** the poses barely move, and the update falls toward zero;
  - **a rotation drift is corrected:** the guess turns with a constant yaw-rate error, as a gyro bias does; the optimizer brings the poses back close to the truth;
  - **the gyro bias is estimated:** with bias estimation on, the estimated bias matches the injected yaw-rate error;
  - **degeneracy is detected:** in a corridor (no end walls), the translation information the optimizer reports is weak along the corridor and strong across it; in the closed room it is strong in all three directions. `optimize_windows` uses this information to detect degenerate windows.
- **The options are set in the test**, at the values of `settings/lidar-vertical.toml`, so tuning the shipped TOML doesn't break the tests.
- **No link with `core`.** The roadmap entry says "the tests then link `core`". They don't need to: `scan_to_map.cpp` uses only the NDT types from `core`'s headers and `get_rgd_index`, which `app/lidar_odometry_utils.cpp` defines (checked with `dumpbin` on the Debug objects). The test target compiles `app/scan_to_map.cpp` and `app/lidar_odometry_utils.cpp` directly, like the other app sources it tests, and avoids `core`'s link-order constraint.
- **Docs.** `CLAUDE.md` gets the new tested sources. `docs/ARCHITECTURE.md` marks 6c done and corrects the "link `core`" note.

**Unchanged:** no app code changes, so the `dMapper` results stay bit-identical.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: adds a requirement that the scan-to-map optimizer is tested on synthetic data with a known true trajectory, including the gyro-bias estimate and the degeneracy information.

## Impact

- **Code:**
  - `tests/scan_to_map_test.cpp` (new);
  - `tests/CMakeLists.txt`: the new test file, plus `app/scan_to_map.cpp` and `app/lidar_odometry_utils.cpp`.
- **App:** none. `44_short` must stay IDENTICAL with its manifest, and the full 44 is not needed, because `dMapper` isn't rebuilt from changed sources.
- **Test time:** the new tests should take a few seconds in Release. Debug is slower, because unoptimized Eigen is.
- **Risk:** the tests check what the optimizer *should* do. If one fails, it may show a real weakness of the optimizer, for example a bias that isn't recovered. That is reported to the user, not hidden by a looser test.
