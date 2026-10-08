# Tasks

**Checks on every code commit:**
- the Release build of `dMapper` and `lidar_odometry_tests` passes, and `ctest` passes (Debug too, for tasks 2.x, see below);
- precise `44_short` is IDENTICAL with the precise reference;
- fast `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or else within `--tolerance 1e-6` of `raw data/44_short_baseline`.

Debug: per `CLAUDE.md`, the Debug build and its `ctest` run for the tasks that add asserts (2.1, 3.x), not for the rest.
Close the GUI before building. Never run two jobs on the same input folder at once. Pass input paths with backslashes.

## 1. The type

- [x] 1.1 Add `BandedNormalMatrix` to `src/domain/normal_equations.{h,cpp}` (design decisions 1, 2, 4) with `add`, `add_block6`, a read accessor and `to_sparse()`. Move `pose_band_to_sparse` into `tests/normal_equations_test.cpp` as the dense reference. Tests: for random additions in a fixed order (diagonal blocks, edges, bias coupling in both triangles, offset coupling, a term wider than 17, an out-of-band write that the reference drops), `to_sparse()` equals `pose_band_to_sparse(dense)` entry for entry and bit for bit, with the same pattern. Verify: Release build, `ctest`. Commit.

  Result: `BandedNormalMatrix` in `normal_equations.{h,cpp}`, three test cases (56 checks) that compare it with the dense matrix and `pose_band_to_sparse` bit for bit, pattern included: random diagonal blocks, edges, both-triangle couplings with 0, 3 and 6 extra unknowns, bands of 17 and 113, cancelling additions that leave an exact zero, and a band wider than the matrix. Release build and `ctest` pass.

  Two deviations from the text above, both small:
  - A write outside the band is dropped and counted (`dropped_writes()`) instead of asserting inside the type, so a unit test can check the dropping. The assert is placed at the call site in 2.1, where the band is chosen.
  - `pose_band_to_sparse` stays in the domain for now, because `optimize_lidar_odometry` still calls it; it moves to the test in 2.1, so every commit builds.

## 2. Use it

- [x] 2.1 In `optimize_lidar_odometry`, compute the anchors and the band up front (decision 3) and replace `AtPAndt` by the new type: the NDT loops use `add_block6`, the other terms `add`; the fix-origin term goes to the band (decision 5) and the triplet lists and sparse products go. Verify: the step checks (Release and Debug, both builds with `ctest`); precise and fast `44_short`; the synthetic scan-to-map tests give exactly the numbers they gave before (their values are in the test comments). Report the phases with `log_stage_timing` on `44_short`. Commit.

  Result:
  - `optimize_lidar_odometry` computes the anchors and the band first, builds a `BandedNormalMatrix`, and assembles `AtPA` from `to_sparse()`. The fix-origin term is added to the band and the triplet lists and three sparse products are gone. `pose_band_to_sparse` moved into `tests/normal_equations_test.cpp`. An `assert` at the call site checks that no write fell outside the band.
  - Release and Debug builds and `ctest` pass. Precise and fast `44_short` are IDENTICAL with the precise reference and the manifest (11 files, no manifest change: the params TOML is untouched).
  - Phases on `44_short` (fast build, `log_stage_timing`; before = after `deterministic-multithread`): setup 4.08 → 0.12 ms per iteration; "rest of the iteration" 12.7 s → 0.8 s; assembly 1.30 → 1.00 ms; odometry 108.8 → 72.2 s (−34 %); whole run 144 → about 107 s. The two Cholesky solves are now 47 % of the odometry (2.23 + 2.58 ms per iteration), which makes 3.x worth measuring.
  - A test finding: the bit-for-bit comparison with a dense reference failed in Release (about 1e-12 on 1,200 of 25,000 elements) while it passed in Debug. In isolation both sides equal a plain sequential sum, so `/fp:fast` reordered the test's own dense reference sum, not the type. `tests/normal_equations_test.cpp` is therefore compiled with `/fp:precise` (`tests/CMakeLists.txt`); production code is checked by the precise `44_short` build as before. The cancelling-to-zero case is a separate test on pristine elements.
- [x] 2.2 Full 44 in the background with `log_stage_timing` on. Verify: IDENTICAL with `tools/baseline/44.manifest.json` apart from the params TOML (the timing line), or within `--tolerance 1e-6` of `raw data/44_baseline`. Report the profile against the numbers in the proposal. Commit the result in the tasks file.

  Result (2026-10-08): 1137 s run. The fast build's output is no longer bit-identical with the manifest (31 of 35 files differ, `44_short` fast and precise are still IDENTICAL), but it is within tolerance of `raw data/44_baseline`: the largest difference is 1e-9 (trajectories; results JSON 8e-14), the params TOML differs only in settings lines (`log_stage_timing`). This is the documented `/fp:fast` effect on the long run when the shape of a function changes. A precise full 44 was not run. The 44 baselines (folder and manifest) are refreshed once at the end of this change, from a clean run with `log_stage_timing` off.

  Profile, full 44 (before = after `deterministic-multithread`): odometry 1059.6 → 688.7 s (−35 %), whole run 1467 → 1137 s (−22 %). Per iteration: setup 4.09 → 0.17 ms, assembly 1.30 → 1.11 ms, motion terms 1.64 → 1.45 ms; the rest of the iteration 118.2 s → 3.7 s in total. Left: NDT loops 27 % of the odometry, the covariance solve 20 % (2.00 ms), the damped solve 23.5 % (2.34 ms), motion terms 14.5 %, assembly 11 %. Loop closure 56 s, consistency 355 s, save 24 s are unchanged.

## 3. Cholesky symbolic reuse (only if the measurement says so)

- [x] 3.1 Measure the share of `analyzePattern` in the two solves on `44_short` (design decision 7). If the saving is below about 5 % of the odometry, mark 3.2 skipped with the numbers and stop. Commit the note.

  Result (2026-10-08, a throwaway instrumented build, not committed; `44_short`, odometry 72.1 s): `analyzePattern` 11.04 s in the 6,900 covariance solves and 10.57 s in the 7,000 damped solves, together 21.6 s = 30 % of the odometry. `factorize` takes 3.36 s + 3.33 s, and the solve of the damped system 1.04 s. So the symbolic analysis is three times the numeric work; the saving from reusing it is up to about 30 % of the odometry, far above the 5 % bar. 3.2 goes ahead. Both matrices very likely share one pattern (every diagonal is present), but each solver keeps and checks its own pattern.
- [x] 3.2 (skipped, see the result) `ScanToMapWorkspace` with the two solvers and the stored patterns, created per window in `run_lm_iterations`, passed to `optimize_lidar_odometry` (default `nullptr` for the tests), re-analysing when the pattern differs. Verify: the step checks with Debug; precise and fast `44_short`; a unit test that a changed pattern triggers re-analysis and gives the same result as a fresh solver. Then the full 44 as in 2.2. Commit.

  Result: skipped, after being implemented and tested. A `PatternCachedCholesky` (a `SimplicialCholesky` that keeps the symbolic analysis while the pattern compares equal) with a per-run `ScanToMapWorkspace`, 3 new tests, gave bit-identical results (including the synthetic scan-to-map window) but never hit: on `44_short` it analysed 6,900 times in 6,900 covariance solves and 7,000 in 7,000 damped solves. The stored entries wobble by about ±100 of 33,000 between iterations (about 930 of 2160 columns differ by a few entries; entries that are exactly zero in one iteration and tiny in the next), so the pattern is never the same twice. Reuse would need a fixed superset pattern, which changes the elimination order and so the numbers: a result-changing change of its own (estimated saving up to 30 % of the odometry), not part of this bit-identical one. The code was reverted; nothing of it is committed. The numbers are in `docs/PERFORMANCE.md`, fix 8.

## 4. Docs

- [x] 4.1 `docs/PERFORMANCE.md`: a new fix entry (problem, fix, measured gain on `44_short` and 44, checks) with its summary row, the profile table updated, and the planned order (step 2 done). `docs/ARCHITECTURE.md` and `CLAUDE.md` only if something they say changes. Verify: the numbers match the 2.1, 2.2 and 3.x reports. Commit.

  Result: `PERFORMANCE.md` has fix 8 (problem, fix, table, checks), its summary row, the Cholesky finding, and the planned order updated. `ARCHITECTURE.md` and `CLAUDE.md` unchanged.

  Baselines: the clean full 44 (settings as shipped, `log_stage_timing` off) ran in 1112 s (odometry 675 s, processing time 1087 s). It is within 1e-9 of the old baseline for every data file (the params TOML differs only in settings lines). `raw data/44_baseline` is now this output (the old folder is `44_baseline_2026-10-08`) and `tools/baseline/44.manifest.json` is written from it (commit `2ca78b3`); the params hash is the one of the `44_short` manifest. `44_short` baselines are unchanged: fast and precise are still bit-identical.
