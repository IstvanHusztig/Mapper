# Tasks

**Checks on every code commit:** the Release build of `dMapper` and `lidar_odometry_tests` passes and `ctest` passes; `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json` (about 1.5 min). The full 44 once, in task 4.

Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes.

## 1. Measure

- [ ] 1.1 Measure what the NDT loops of `optimize_lidar_odometry` spend on the cell index and `find`, on `cov.inverse()` with the two threshold tests, and on the rest, for a recorded window of `44_short` (a throw-away benchmark in the scratchpad or a test-only helper, not timers inside the per-point loop). Report the shares and the per-iteration times next to the 2026-10-05 profile in `docs/PERFORMANCE.md`. If lookup plus inverse are below about 15 % of the loop time, mark 2.1 and 3.1 skipped with the numbers and go to 4.1. Also list every user of `NDTBucketMapType` that iterates the map, to decide whether 3.1 is possible at all.

## 2. Precomputed cell data

- [ ] 2.1 Compute each cell's inverse covariance and "usable" flag once, after the map is finished for a window (and again for cells `update_rgd` changed), with the same expression and thresholds as today; use them in the indoor and outdoor loops and in `Consistency::point_term`. Tests: the cached inverse equals `cov.inverse()` bit for bit, a changed cell is refreshed, an optimizer run with and without the cache gives the same result bit for bit. Verify: `ctest`; `44_short` IDENTICAL; report the loop times per iteration. Commit.

## 3. Faster lookup (only if 1.1 shows the `find` to matter)

- [ ] 3.1 Replace the read-only lookup by a flat open-addressing table (or change the map type if no code depends on iteration order). Tests: lookups equal `unordered_map::find` for hits and misses on a random set of cells. Verify: Release and Debug builds, `ctest`, `44_short` IDENTICAL; report the loop times. Commit.

## 4. Full run and docs

- [ ] 4.1 Full 44 in the background with the shipped settings and `log_stage_timing` on a scratchpad copy of the settings, from a copy of the binaries. Verify: IDENTICAL with `tools/baseline/44.manifest.json`; report odometry and whole-run times. Commit.
- [ ] 4.2 Docs: `docs/PERFORMANCE.md` (fix entry with the table and the checks, the plan), `docs/performance.html` and `docs/hu/performance.html`, run times in `CLAUDE.md` and `tools/README.md` if they change. Verify: the numbers match the 1.1, 2.1, 3.1 and 4.1 reports. Commit, archive, push.
