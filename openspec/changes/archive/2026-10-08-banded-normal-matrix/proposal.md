# Proposal

## Why

After `deterministic-multithread`, the largest phase of the odometry is no longer the point loops. The profile of the full 44 (`docs/PERFORMANCE.md`, fix 7) shows, per Levenberg–Marquardt iteration:

| Phase | ms | Share of odometry |
|---|---|---|
| setup: allocate and zero the dense normal matrix | 4.09 | 26.6 % |
| the rest of the iteration, mostly freeing it | 1.71 | 11.2 % |
| assembly: fix-origin products and `pose_band_to_sparse` | 1.30 | 8.5 % |

That is about 46 % of the odometry spent on one 2166×2166 dense matrix (37 MB for a 360-node window) that is zeroed, filled in a narrow band, read back through a band-only conversion, and freed, 100 times per window. Only the band is ever read.

The change replaces it with a band-only store that receives exactly the same additions in the same order, so the result stays bit-identical. It is the second change of the performance plan (`docs/PERFORMANCE.md`, "Planned order").

## What Changes

- **A banded normal matrix** (`BandedNormalMatrix`, domain, standard library and Eigen only) replaces the dense `AtPAndt` in `optimize_lidar_odometry`. It stores the pose–pose band (half-width from the window's acceleration-term span, 17 by default) and the rows and columns of the extra unknowns (gyro bias, acceleration offset). Every `+=` of the current code goes to the same element, in the same order. Its conversion to a sparse matrix produces the same entries as `pose_band_to_sparse(dense)` today.
- **The fix-origin term** (6 diagonal elements) is added to the band directly instead of through three small sparse products. It is the same single addition of the same two values.
- **`pose_band_to_sparse`** moves from the domain into the tests, where it stays as the dense reference the new type is compared with.
- **Possibly, decided by a measurement in this change:** reusing the Cholesky symbolic analysis between iterations (a per-window workspace passed to `optimize_lidar_odometry`), guarded by an equality check of the sparsity pattern so the permutation, and therefore the result, cannot change. Only if the two solves are still a large share after the first part.

**Unchanged:** the numbers. With the setting-free defaults, precise `44_short` must be IDENTICAL with the precise reference and the fast build IDENTICAL or within 1e-6, and the full 44 must match its manifest.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. A pure performance change; no behaviour changes (`skip_specs: true`).

## Impact

- **Code:** `src/domain/scan_to_map.cpp` (the accumulation, the assembly, the up-front band computation), `src/domain/normal_equations.{h,cpp}` (the new type; `pose_band_to_sparse` leaves), possibly `src/domain/scan_to_map.h` and `src/application/run_odometry.cpp` (the workspace).
- **Tests:** `tests/normal_equations_test.cpp` (new type against the dense reference, bit for bit).
- **Docs:** `docs/PERFORMANCE.md` (a new fix entry), `docs/ARCHITECTURE.md` only if a decision changes.
- **Debug build:** this change adds asserts for out-of-band writes, so the Debug build and its `ctest` run for it, as `CLAUDE.md` asks for configuration-dependent code.
- **Results:** bit-identical; the params TOML does not change, so no manifest hash changes.
