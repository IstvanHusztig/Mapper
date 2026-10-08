# Tasks

**Checks on every code commit:** the Release build of `dMapper` and `lidar_odometry_tests` passes and `ctest` passes; `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json` (about 1.5 min). The full 44 once, in task 4.

Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes.

## 1. Measure

- [x] 1.1 Measure what the NDT loops of `optimize_lidar_odometry` spend on the cell index and `find`, on `cov.inverse()` with the two threshold tests, and on the rest, for a recorded window of `44_short` (a throw-away benchmark in the scratchpad or a test-only helper, not timers inside the per-point loop). Report the shares and the per-iteration times next to the 2026-10-05 profile in `docs/PERFORMANCE.md`. If lookup plus inverse are below about 15 % of the loop time, mark 2.1 and 3.1 skipped with the numbers and go to 4.1. Also list every user of `NDTBucketMapType` that iterates the map, to decide whether 3.1 is possible at all.

  Result (throw-away benchmark, serial, Release, 600,000 points of the synthetic room / corridor against the indoor map with 8,255 / 27,056 cells; milliseconds for all points, best of 5; the real maps are larger, so the `find` is a lower bound): transform of the point 7.2; cell index + `find` +15 (22.1 / 26.2 in total); `cov.inverse()` + thresholds +25 (47.1 / 52.0 in total); `pose_tait_bryan_from_affine_matrix` per point 17; with the generated AtPA and AtPB 87-90 in total (so about 72 for the two terms). Per hit, in all about 136 ms: the terms 53 %, inverse and thresholds 19 %, pose conversion 12 %, `index` + `find` 11 %, transform 5 %. Lookup plus inverse are about 30 % of the loop, far above the 15 % bar: **2.1 goes ahead.** **New finding:** the pose conversion (`pose_s`, trigonometry) depends on the node's pose only, yet is done per point; computing it once per node and iteration is bit-identical and worth about 12 %, so it is added to 2.1. The `find` is about 11 %: 3.1 is worth about 2-3 % of the odometry at best; decide after 2.1 is measured. Users of the maps that iterate them: `Consistency2` (`NDTBucketMapType2`, not the odometry map) and the robust mode (`robust_lidar_odometry.cpp`, off in the shipped settings); the odometry path only does `find`/`emplace`/`at`, so a different lookup structure for the read-only phase is possible.

## 2. Precomputed cell data

- [x] 2.1 Compute each cell's inverse covariance and "usable" flag once, after the map is finished for a window (and again for cells `update_rgd` changed), with the same expression and thresholds as today, and the node poses converted once per node and iteration (`pose_tait_bryan_from_affine_matrix`, found in 1.1); use them in the indoor and outdoor loops and in `Consistency::point_term`. Tests: the cached inverse equals `cov.inverse()` bit for bit, a changed cell is refreshed, an optimizer run with and without the cache gives the same result bit for bit. Verify: `ctest`; `44_short` IDENTICAL; report the loop times per iteration. Commit.

  Result: `NdtBucket` carries `cov_inverse`, `inverse_computed` and `inverse_usable`; `update_rgd` (exact and previous statistics) refreshes the cells it added or changed once at the end (`refresh_cell_inverse`), and `ndt_cell_inverse` returns the stored inverse or inverts on the spot for a cell that never went through `update_rgd` (same expression, same thresholds). The indoor and outdoor loops and the point loop of `Consistency` use it, and the node's Tait-Bryan pose is converted once per node (`Consistency`: once per run of points of one node). New tests: every cell's stored inverse equals `cov.inverse()` bit for bit after one and after two `update_rgd` calls, in both modes (the second call changes and adds cells); an optimizer run with and without the stored inverse is bit-identical (room and corridor, 1 and 30 iterations). `ctest` passes (Release); `44_short` IDENTICAL with the manifest. `44_short` loop times (`log_stage_timing`, 7000 calls each): indoor 1.905 → 1.694 ms (−11 %), outdoor 0.957 → 0.836 ms (−13 %); odometry 51.9 → 49.8 s (−4 %). The gain is smaller than the serial benchmark of 1.1 suggested (the loops run on 16 threads and are partly memory-bound).

## 3. Faster lookup (only if 1.1 shows the `find` to matter)

- [x] 3.1 (skipped, see below) Replace the read-only lookup by a flat open-addressing table (or change the map type if no code depends on iteration order). Tests: lookups equal `unordered_map::find` for hits and misses on a random set of cells. Verify: Release and Debug builds, `ctest`, `44_short` IDENTICAL; report the loop times. Commit.

  Skipped. After 2.1 the two NDT loops are 17.8 s of the 49.8 s odometry on `44_short`, and `index + find` was about 11 % of their serial work in 1.1, so a flat table could save about 2 s (4 % of the odometry) at best, in exchange for a changed container in code that several threads read. Not worth the risk now; to be reconsidered with `io-and-copies` if the profile points to it.

## 4. Full run and docs

- [x] 4.1 Full 44 in the background with the shipped settings and `log_stage_timing` on a scratchpad copy of the settings, from a copy of the binaries. Verify: IDENTICAL with `tools/baseline/44.manifest.json`; report odometry and whole-run times. Commit.

  Result: full 44 from the copied binaries with `log_stage_timing` on: differs from `tools/baseline/44.manifest.json` only in the parameter file (the timing setting), every data file identical. Odometry 516.7 → 490.5 s, processing time 627.6 → 600.6 s, wall-clock 622 s; NDT loops 1.613 ms (indoor) and 0.810 ms (outdoor) per iteration; consistency 45.0 s (unchanged). No new baselines.
- [x] 4.2 Docs: `docs/PERFORMANCE.md` (fix entry with the table and the checks, the plan), `docs/performance.html` and `docs/hu/performance.html`, run times in `CLAUDE.md` and `tools/README.md` if they change. Verify: the numbers match the 1.1, 2.1, 3.1 and 4.1 reports. Commit, archive, push.

  Result: `docs/PERFORMANCE.md` fix 12, both performance pages (cards, a sixth chart bar, step table, "what is left"), full-44 run time in `CLAUDE.md` and `tools/README.md` (11 → 10 min).
