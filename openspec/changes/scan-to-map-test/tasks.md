# Tasks

**Checks on every commit:**
- both presets build `dMapper` and `lidar_odometry_tests`;
- `ctest` passes;
- report the new tests' run time in Release and Debug.

There is no app change, so `44_short` is checked once, in 3.1 (design decision 7). Close the GUI before building.

## 1. Test target and fixtures

- [x] 1.1 Add `app/scan_to_map.cpp` and `app/lidar_odometry_utils.cpp` to `tests/CMakeLists.txt`, with a comment that the tests compile them directly and don't link `core`. Add an empty `tests/scan_to_map_test.cpp`. Verify: both presets build `lidar_odometry_tests` without link errors, and `ctest` passes. If a symbol is missing, pause and report it; don't link `core`. Commit.
- [x] 1.2 Fixtures in `tests/scan_to_map_test.cpp` (design decisions 1–4):
  - the box-room and corridor scenes, with a fixed-seed jitter;
  - building the indoor and outdoor maps with `update_rgd`;
  - the synthetic window from a list of true poses: points per node in the node frame, timestamps;
  - the options from a default `OdometryConfig` with the `lidar-vertical.toml` values;
  - the iteration helper;
  - a `NullLogger`.

  Add a first test that checks the fixtures themselves: every scene point lies within 1 cm of a face, both maps hold buckets, and each node has 40–80 points in range. Verify: build, `ctest`. Commit.

## 2. Optimizer tests

- [x] 2.1 **Perfect guess:** in the closed room, a guess equal to the truth stays within 3 mm and 0.1° on every node after 30 iterations, and the last `delta` is below 1e-3. These limits were changed on 2026-10-04 (decision 6, outcome), from 1 mm / 0.01° / 1e-6. Measured: 1.5 mm, 0.046°, delta 1.5e-4. Verify: `ctest`. Commit.
- [x] 2.2 **Rotation drift without bias estimation:** the guess's yaw grows by 0.5 °/s · t (decision 5), with `estimate_bias = false`. After 30 iterations, the heading error at the last node is at most 10 % of the guess's, and the positions are within 3 mm of the truth. This was changed on 2026-10-04 from "no worse than the guess". Measured: heading 0.245° → 0.016°, positions 1.8 mm. Verify: `ctest`, with the before/after errors logged by `MESSAGE`. Commit.
- [x] 2.3 **Gyro bias estimation:** the same guess with `estimate_bias = true`. The estimated vertical-axis bias is within 20 % of the injected 0.5 °/s, and its sign follows the convention of the relative-pose term (decision 5). The other two axes stay below 0.05 °/s. Verify: `ctest`. Pause if the target or the sign is off. Commit. Measured: 0.469 °/s about the vertical (sign as injected); 0.045 and −0.031 °/s on the other axes.
- [ ] 2.4 **Iterations needed** (decision 8): give the iteration helper an iteration count and a per-iteration callback. In the two drift scenarios (2.2 and 2.3), run 100 iterations and report with `MESSAGE`:
  - N_final;
  - N_fast;
  - the error against the truth at N_fast and at 100;
  - the time per iteration.

  Assert only N_final < 100. Verify: `ctest` passes, and the numbers are written in this task. Commit.
- [ ] 2.5 **Degeneracy information:** pass `lidar_translation_information` for a window in the corridor and in the closed room. Corridor: the eigenvalue along x is at most 1 % of the smallest of the other two. Room: the smallest eigenvalue is more than 1 % of the largest. Verify: `ctest`. Commit.

## 3. Check and docs

- [ ] 3.1 Build `dMapper` in Release and run `44_short`. Verify it is IDENTICAL with `tools/baseline/44_short.manifest.json` (decision 7: no app source changed). Report the result.
- [ ] 3.2 Update the docs:
  - `CLAUDE.md`, Testing: add `app/scan_to_map.cpp` and `app/lidar_odometry_utils.cpp` to the compiled sources, and mention the synthetic scan-to-map tests;
  - `docs/ARCHITECTURE.md`: 6c ✅ with the date, and correct "the tests then link `core`" to "the tests compile `scan_to_map.cpp` and `lidar_odometry_utils.cpp` directly; no `core` needed";
  - `docs/ARCHITECTURE.md`, status line: steps 0–6 done;
  - `docs/PERFORMANCE.md`: the measured iteration counts from 2.4, next to `nr_iter` and `convergence_delta_threshold`, with the caveat and the suggested real-data follow-up.

  Verify: `grep` finds no "tests then link" in `docs/` or `CLAUDE.md`. Run `openspec validate scan-to-map-test --strict`. Commit.
