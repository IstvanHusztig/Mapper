# Design

## Context

`update_rgd(rgd_params, buckets, points_global, viewport)` (`app/lidar_odometry_utils.cpp`) adds points to a `NDTBucketMapType` (a hash map from the cell index `get_rgd_index` to `NDT::Bucket`: `mean`, `cov`, `number_of_points`, `normal_vector`, `index_begin`, `index_end`). Today it works like this:

| Point number in the cell | What happens |
|---|---|
| 1 | `mean` = the point, `cov` = 0.03² · I, `normal_vector` = 0 |
| 2 | `cov` = cov·(n−1)/n + d·dᵀ·(n−1)/n², with `d = mean − point`; `mean` is not updated |
| 3 | the same update, then `normal_vector` = col(1) × col(2) of the eigenvectors, flipped towards `viewport` |
| more than 3 | the same `cov` update, but only if `normal · (viewport − mean) ≥ 0`; the normal is never recomputed |

**Callers:**
- `seed_map` and `add_window_to_map` in `app/run_odometry.cpp`: indoor and outdoor grids. The viewport is the map origin or the window's first pose.
- `run_loop_closure` (`app/loop_closure.cpp`).
- the consistency pass (`app/consistency.cpp`, line 218).

**Readers:** the scan-to-map optimizer (`scan_to_map.cpp`) and robust mode use the mean, the inverse covariance and the normal. The cell is rejected if any entry of `cov⁻¹` exceeds 1e5, or if the normal faces away from the node. The normals also feed the degeneracy information.

`consistency.cpp` (around line 610) builds its own cells correctly, as a reference: the centroid, the covariance about it, and the normal as col(1) × col(2) of the eigenvectors (the eigenvector of the smallest eigenvalue).

## Goals / Non-Goals

**Goals:**
- Mean, covariance and normal that describe all of a cell's points, independent of insertion order.
- The previous statistics, bit-identical, behind a switch.
- An evaluation on 44 that the user can accept or reject.

**Non-Goals:**
- `update_rgd_spherical_coordinates`, used only by robust mode, which is off.
- `core`'s own NDT code, and changing `NDT::Bucket`.
- Changing the cell size, the 1e5 rejection threshold or the view-point rule itself.
- Retuning `imu_degenerate_information_ratio` (0.3). If the ratios shift, that is reported, and retuning is a follow-up.

## Decisions

1. **Running statistics in the existing fields.** Each new point x in a cell with n points:
   - `mean ← mean + (x − mean) / (n + 1)` (Welford);
   - `cov` holds the covariance of the points plus the prior worth one point:
     `cov_{n+1} = (n · cov_n + (x − mean_old)(x − mean_new)ᵀ) / (n + 1)`, starting from `cov_1 = 0.03² · I`.

     This keeps one- and two-point cells exactly as today: a 3 cm sphere, usable as a point-to-point term. For many points the prior fades as 1/n. For example, on a flat wall with 1 cm noise and 50 points, it adds 0.03²/50 = 1.8e-5 m² to the normal variance of 1e-4 m².
   - Alternative: store the sum of outer products (M2) and divide when the cell is read. Rejected, because the readers use `cov` directly, and `NDT::Bucket` has no spare field.

2. **The normal is recomputed at the end of each call, for touched cells only.**
   - `update_rgd` collects the indices of the cells it changed, and after the loop computes each one's normal: `SelfAdjointEigenSolver` on `cov`, eigenvector `col(0)`, flipped towards `viewport`.
   - Cells with fewer than 3 points keep a zero normal, as today. The readers treat a zero normal as "no normal": the information sum skips it, and the view-point test passes.
   - **Ambiguous normals (added during 2.2):** a cell also gets a zero normal when its two smallest eigenvalues differ by less than a factor of 3 (λ1 < 3·λ0). This happens when a cell's points lie on one line (a scan ring through a cell corner) or on one cross-section of a corner: the normal is then undefined, and the eigensolver returns an arbitrary direction in the plane across the line. On the synthetic corridor, 327 of 27,056 indoor cells had such a normal more than 30° off the faces, and they alone kept the information along the corridor at 23.9 against 1073 across (45×). With the rule it is 1.7 against 987 (570×). A flat cell is far from the threshold: with 9 points on a 15 cm cell and 5 mm noise, λ1/λ0 is about 19. Such a cell still serves as a point-to-distribution term; only its normal is left out.
   - Computing the normal per point instead would cost an eigen-solve for every point. With one solve per touched cell per call, the cost is about the number of cells a window touches, a few thousand solves per window. It is measured (decision 7).

3. **The view-point gate uses the current normal.** A point is not added to a cell whose normal (from the end of an earlier call) faces away from this call's viewport. Within one call all points share the viewport, so both sides of a thin wall can't enter one cell in the same call. The first call that creates a cell has no normal yet, so its points are always added, as today.

4. **The switch: `use_exact_ndt_bucket_statistics`,** in `[lidar_odometry_control]`.
   - It is passed to `update_rgd` as a parameter, not read from a global. The callers get it from `OdometryConfig`, and loop closure and consistency get it through their `params`.
   - `false` runs the previous code, kept verbatim in its own function `update_rgd_legacy`.
   - **Defaults during implementation:** the C++ default and `default_settings.toml` start at `false`, with `lidar-vertical.toml` also `false`, so every commit before the evaluation keeps today's results.
   - **After acceptance:** both switch to `true` in their own commit, with the new baselines.

5. **Checking the switch-off path.** Adding a parameter and a branch can move the `/fp:fast` build's last bits (see `window-loop-steps`). So with the switch off:
   - the precise build must stay bit-identical with a precise reference from master;
   - the fast build must be within `--tolerance 1e-6` of `raw data/44_short_baseline`.

6. **Unit tests for `update_rgd`,** in a new `tests/ndt_map_test.cpp` with synthetic planes, both settings:
   - exact statistics: mean within 1 mm of the centroid, normal within 5° of the plane for cells with 10 or more points, both in line order and in random order; the two orders agree within 1e-9;
   - previous statistics: the documented quirks (first-point mean, a fixed normal) are reproduced, so the legacy path is pinned;
   - a thin wall seen from two viewports in two calls: the second side doesn't enter the first side's cells.

   In `tests/scan_to_map_test.cpp`, the options pick exact statistics. The corridor test loses `should_fail`, and the room limits tighten to measured values with margin.

7. **The evaluation** (the user decides on it):
   - run 44 with the switch off (the current baseline) and on, from a scratchpad copy of `lidar-vertical.toml`;
   - export both clouds with CloudCompare (`tools/README.md`), then run `tools/eval_odometry.py "raw data/44" <off> <on> --png …`;
   - from the logs, count the windows marked degenerate and the windows with the accelerometer term active, and summarize the weakest directions;
   - count the loop-closure edges accepted;
   - compare the odometry time.

   The results go in a table in this change, and the user then decides: accept (switch the defaults, write the new baselines), reject (keep the switch off and the code for later), or investigate.

## Risks / Trade-offs

- **[More cells pass the view-point check, so the optimizer sees more and different constraints]** The behaviour can change more than the 1 mm suggests, for example in stairwells, where the degeneracy gating matters. → The evaluation looks at the whole trajectory and the map sharpness, not just one number, and the user decides.
- **[`imu_degenerate_information_ratio = 0.3` was tuned on the wrong normals]** With correct normals, rooms may give higher ratios and corridors lower ones, so different windows may count as degenerate. → The counts and directions are reported in the evaluation. Retuning is a follow-up, not part of this change.
- **[Run time]** One eigen-solve per touched cell per call. → It is measured, and if it is noticeable, solves can be skipped for cells whose point count grew by less than some fraction since their last solve.
- **[Loop closure and consistency change too]** They call `update_rgd`. → Covered by the 44 evaluation, which includes both. The loop-closure edge count is compared.
