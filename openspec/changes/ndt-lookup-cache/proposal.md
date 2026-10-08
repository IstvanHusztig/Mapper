# Proposal

## Why

After `consistency-speedup` the full 44 runs in 647 s, 517 s of it the odometry. In the odometry profile of 2026-10-05 the two NDT point loops (indoor and outdoor grid, `optimize_lidar_odometry`) were 47 % of the time; after multithreading and the banded matrix they are about a quarter. Reading the loops (`src/domain/scan_to_map.cpp`, `hessian_fun_indoor` and the outdoor one) shows what every point does in every Levenberg–Marquardt iteration (about 69,000 iterations on 44, each over all points of a window):

1. `ndt_cell_index` (three divisions, three truncations to `int16_t`, a key);
2. `unordered_map::find` on `NDTBucketMapType = std::unordered_map<uint64_t, NdtBucket>`: hash, bucket array, then a pointer chase into a node;
3. **`this_bucket.cov.inverse()`, a 3×3 inverse of a matrix that does not change while the map is fixed**, so it is recomputed for every point that hits the cell, in every iteration, and then the two threshold tests;
4. the normal-vector test (if on), `pose_tait_bryan_from_affine_matrix`, the generated AtPA and AtPB code.

The map is built once per window (`update_rgd`) and then only read during the iterations. Items 2 and 3 are therefore the candidates: the inverse (and its threshold tests) depends on the cell only, and the hash lookup is a node-based map used read-only.

`Consistency` (`point_term`) does the same `cov.inverse()` per point and per pass.

This is a reading of the code, so the change **measures first**, as `stage-timing` and `consistency-speedup` did.

## What Changes

- **Measure** (task 1): split the NDT loop time into lookup (cell index and `find`), inverse and thresholds, and the term computation, by a micro-benchmark on a recorded window (not by timers inside the per-point loop, which would disturb it). Report the shares on `44_short`. If lookup and inverse together are below about 15 % of the loop time, tasks 2 and 3 are marked skipped with the numbers.
- **Precomputed cell data** (task 2): after `update_rgd`, each cell gets its inverse covariance and a flag "usable" (both threshold tests passed), computed once with the same expression as today (`cov.inverse()`, same thresholds), so the values are the same bits. The loops read them instead of inverting. The data lives with the map, is filled where the map is finished, and is refreshed whenever a cell changes (`update_rgd` adds to existing cells in later windows). Results bit-identical. The same helper serves `Consistency`'s `point_term`.
- **Faster lookup** (task 3, only if the measurement says the `find` matters): a flat open-addressing table for the read-only phase, or a changed map type where nothing depends on iteration order (to be checked by listing every user of the map first). Lookups return the same cell, so results stay bit-identical.
- **Check and docs** (task 4): full 44 in the background with the shipped settings, bit-identical with the manifests (`44_short` on every code commit); a fix entry in `docs/PERFORMANCE.md`, the performance pages (English and Hungarian), run times in `CLAUDE.md` and `tools/README.md` if they change.

**Not in this change:** caching the matched cell per point across iterations (the poses move in every iteration, so a point can fall in another cell and the result would change); the robust-mode paths (`use_robust_and_accurate_lidar_odometry` is off in the shipped settings); `io-and-copies`; loop closure.

## Capabilities

### Modified Capabilities

None. A performance change that must not change a bit (`skip_specs: true`). `design.md` is skipped because the approach fits in this proposal.

## Impact

- **Code:** `src/domain/ndt_types.h`, `ndt_map.{h,cpp}` (cell data and its refresh), `src/domain/scan_to_map.cpp` (the two loops), `src/application/consistency.cpp` (`point_term`); tests in `tests/ndt_map_test.cpp` and `tests/scan_to_map_test.cpp` (cached inverse equals `cov.inverse()` bit for bit; a changed cell is refreshed).
- **Results:** bit-identical. `44_short` IDENTICAL with its manifest on every code commit, the full 44 once; no new baselines.
- **Docs:** `docs/PERFORMANCE.md`, `docs/performance.html`, `docs/hu/performance.html`.
- **Debug build:** Release only for task 2 (same arithmetic, same data). Task 3 changes a container read by several threads, so it adds the Debug build and `ctest` (`CLAUDE.md` rule).
