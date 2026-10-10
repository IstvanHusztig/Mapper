# Proposal

## Why

Each NDT point term pulls a point towards its cell's mean with the cell's full inverse covariance. With the default cell statistics, the mean is the cell's first point and never moves. In a planar cell (wall, floor) the term is stiff along the normal, which is right. But it also pulls the point within the plane towards that arbitrary first point. Two consequences, both measured on 44:
- **Cause 1 of the stairwell window (synchronized-retune 4.8):** a sub-millimetre pose difference moves points across cell borders, new cells get other first-point means, and the next window matches a different map. That is the `/fp:fast` runaway at window 213, and a likely source of 44's run-to-run chaos, where a 1–2 ms IMU shift changes loops by metres.
- **False constraint along blind axes:** in a corridor the in-plane pull holds the window back towards the wall cells' means. That is what the degeneracy projection was built to fight.

A point-to-plane term (residual n · (p − m)) removes the in-plane pull, so where on the surface the mean lies no longer matters. It is the research report's top-ranked recommendation (`archive/2026-10-08-odometry-stall-fix/research.md`, item 1), and the only form of it not yet tried. The fixes tried so far changed the mean instead: settling (candidate 1), exact statistics (4.1). Both were rejected.

**Known counter-evidence:** the in-plane variance raised to 0.3 m (`ndt_plane_in_plane_sigma_m`, nearly point-to-plane) was rejected twice. On synchronized data with IMU always on it lost half the loops and ran away once (synchronized-retune 4.1). So this change starts with a go/no-go measurement and only continues if point-to-plane behaves differently. Two things may make it differ: a clean rank-1 normal term with the cell's sensor noise instead of a variance floor, and the LM step check with the unmatched-point cost, which didn't exist when the in-plane sigma was tested.

## What Changes

- **Stage 1 (go/no-go):** a setting `ndt_point_to_plane` (off by default). Clearly planar cells (eigenvalue ratio test) contribute w · (n · (p − m))² / σₙ² with n from the cell's covariance and σₙ² its smallest eigenvalue (floored at the sensor noise). All other cells keep the point-to-distribution term. Measured on the evaluation set, with and without the LM step check.
- **Stage 2, only if stage 1 passes:** the degeneracy projection thresholds re-checked, settings shipped, new baselines, docs.
- **If stage 1 fails:** the setting is removed again (no dead code), and `ndt_plane_in_plane_sigma_m` is removed with it, closing synchronized-retune 4.5 for that option. The result goes into synchronized-retune 4.8 / 4.1.

## Capabilities

### New Capabilities

### Modified Capabilities
- `lidar-odometry-motion-model`: a requirement for point-to-plane NDT terms in planar cells (off by default, results unchanged when off).

## Impact

- `src/domain/scan_to_map.cpp` (the two NDT point lambdas and the cost of `ScanToMapCost`), `src/domain/ndt_map.h` (cell normal / normal variance alongside the stored inverse), `src/domain/optimizer_options.{h,cpp}`, `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h` (+ golden TOML files).
- `tests/scan_to_map_test.cpp`: room and corridor cases with the setting on; `tests/ndt_map_test.cpp` for the cell normal.
- When switched on, the odometry's numbers change everywhere: new baselines (stage 2 only). Cost: about the same per point (a rank-1 term is cheaper than a 3×3); time measured and reported.
- Interacts with the degeneracy projection (`degenerate_projection_ratio`) and with synchronized-retune's candidate 2. Ordered after `robust-loop-closure`, so the loop closure judging it is the final one.
