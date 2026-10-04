# Proposal

## Why

The reference map the odometry registers against is a grid of NDT cells ("buckets"), each with a mean, a covariance and a surface normal. `update_rgd` (`app/lidar_odometry_utils.cpp`) builds them point by point, and the synthetic scan-to-map test (`scan-to-map-test`, 2026-10-04) showed that it computes all three wrongly:

- **The mean is the cell's first point** and is never updated. On a synthetic room this alone moved a perfect guess by about 1 mm (1.5 mm, against 0.5 mm with true centroids).
- **The normal is fixed once, from the first 3 points** plus the initial isotropic 3 cm covariance, and never updated. When those points lie roughly on a line, which is common because points arrive along the scan rings, the normal can point anywhere. On a synthetic corridor, 60 % of the cells had a normal more than 30° off the surface (12 % with the points in random order).
- **The covariance skips points.** After the third point, a point updates the covariance only if it passes the view-point test against that (possibly wrong) normal, and the update formula measures its spread from the first point, not the mean.

The wrong normals do the most damage:
- they make the degeneracy information unreliable (on the corridor it was strongest along the corridor, the opposite of the truth), and that information decides when the accelerometer term is switched on;
- they make the view-point check skip good cells (`use_view_point_and_normal_vectors`, on by default).

The same function feeds the odometry map, loop closure and the consistency pass. `consistency.cpp` already builds its own cells correctly (centroid, full covariance, normal from the eigenvectors), so the fix follows code the project already trusts.

## What Changes

- **`update_rgd` keeps proper running statistics per cell:**
  - the **mean** is the centroid of the cell's points, updated with each point (Welford);
  - the **covariance** is the sample covariance about that mean, also running, with the existing 3 cm isotropic covariance kept as a weak prior worth one point, so that one- and two-point cells behave as today;
  - the **normal** is recomputed from the covariance, as the eigenvector of the smallest eigenvalue, for every cell the call touched, once at the end of the call, and flipped towards the viewport as today.
- **The view-point gate is kept, with the current normal.** Points from the far side of a thin wall still don't mix into the near side's cell, but the decision now uses a normal that has seen all the cell's points.
- **A switch, `use_exact_ndt_bucket_statistics`** (bool, in `[lidar_odometry_control]`). `true` uses the new statistics and `false` the old ones, unchanged. It allows A/B runs with one binary and gives a fallback. The default becomes `true` in `app/default_settings.toml`, `settings/lidar-vertical.toml` and `OdometryConfig`, once the evaluation is accepted.
- **The tests follow:**
  - the corridor test in `tests/scan_to_map_test.cpp` should pass with the new statistics, so its `doctest::should_fail` marker comes off;
  - the room limits are tightened to what the new statistics give, measured;
  - new unit tests for `update_rgd` check the mean, the covariance and the normal against the exact values on synthetic planes inserted in line order.
- **Evaluation before acceptance.** This changes every run's results, so it is judged, not just checked:
  - `tools/eval_odometry.py` on `44` old against new: map sharpness (occupied 3/5/10 cm cells), yaw jitter, window differences, end spikes, final yaw;
  - the degeneracy log: how many windows count as degenerate, and their weakest directions;
  - whether loop closure still closes the same loops;
  - the run time.

  The user decides with these numbers. Only then are the defaults switched and new baselines written, in their own commit.

## Capabilities

### New Capabilities

- `ndt-reference-map`: how the NDT reference-map cells are built from points. Their mean, covariance and normal describe all the points in the cell, independent of insertion order, with a switch back to the previous statistics.

### Modified Capabilities

None. The unit-tests requirement for the scan-to-map optimizer already covers the corridor case, which becomes a normal passing test.

## Impact

- **Code:**
  - `app/lidar_odometry_utils.{h,cpp}`: `update_rgd` and the new setting;
  - `app/toml_io.h`: the new key;
  - the callers of `update_rgd` (`run_odometry.cpp`, `loop_closure.cpp`, `consistency.cpp`) pass the switch.
  - `NDT::Bucket` (`core/include/ndt.h`) already has `mean`, `cov`, `number_of_points` and `normal_vector`, so no `core` change is needed if the running sums fit them (design).
- **Not touched:** `update_rgd_spherical_coordinates`, which only robust mode uses (off in `lidar-vertical.toml`), and `core`'s own NDT code.
- **Results:** every run changes. New baselines for `44_short` and `44` only after the user accepts the evaluation.
- **Tests:** new `update_rgd` tests; the 6c corridor test turns into a normal test; the room limits tighten.
- **Docs:**
  - `docs/degeneracy-information.html`: the weakness is fixed, with the new corridor numbers;
  - `docs/index.html`: the Known-limitations entry and the new setting;
  - `docs/PERFORMANCE.md`: the run time;
  - `CLAUDE.md`.
