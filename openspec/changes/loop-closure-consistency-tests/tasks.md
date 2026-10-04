# Tasks

**Checks on every commit** (design decision 8):
- both presets build `lidar_odometry_tests`, and `ctest` passes;
- `git diff --stat <change start> -- src core` stays empty: the product code doesn't change, so no `44_short` or 44 runs are needed.

## 1. Fixture

- [x] 1.1 Move the scene helpers of `tests/scan_to_map_test.cpp` (faces, face sampling, pose errors) into `tests/synthetic_recording.{h,cpp}` without changing them, and use them from there. Add `tests/memory_point_store.h` (decision 2), the DLL copy for the test target (decision 7), and the new files to `tests/CMakeLists.txt`. Verify: both presets, `ctest`; the 6c `MESSAGE` lines print the same numbers as before the move (diff the output of `lidar_odometry_tests -tc="scan-to-map*" -s`). Commit. Result: `NullLogger`, `Face`, `Scene`, `box_scene`, `face_point`, `face_area`, `sample_scene`, `pose`, `PoseError` and `max_error` moved verbatim into `namespace synthetic`. All 7 6c messages print the same numbers as in `ndt-bucket-statistics` 2.2 (0.366667 mm … corridor 1.72194 / 986.754). Both presets build and `ctest` passes (Release 4 s, Debug 56 s), and `src/` and `core/` are unchanged.
- [x] 1.2 Add the loop scene, the two-lap truth and the windows with their observations (decision 1), plus a fixture test:
  - every point lies on a face within 1 cm;
  - each point's timestamp maps to its own node by consistency's rule;
  - each window holds between 2,000 and 6,000 points;
  - the two laps pass each segment's place twice, more than 10 s apart.

  Report the recording's size, and the fixture's Release and Debug build times. Verify: both presets, `ctest`. Commit.

  Result: two laps make 44 windows, 880 nodes and 132,000 points (3,000 per window); the lap takes 22.28 s. Building it takes 20 ms in Release and 0.8 s in Debug. The fixture test checks:
  - points within 5.0 mm of their face;
  - 0 timestamp mismatches;
  - steps of 0.1 m and turns of at most 0.05 rad per node (the path is continuous where straights meet arcs);
  - every first-lap node has a second-lap node within 6.6 cm.

  Changed from the design: 2 m/s and 150 points per node instead of 1.5 m/s and 200, to keep Debug short (its `ctest` already took 56 s). Both presets build and `ctest` passes (Debug 60 s); `src/` and `core/` are unchanged.

## 2. Tests

- [ ] 2.1 Add `tests/loop_closure_test.cpp` with the three loop-closure scenarios (decisions 3–5). First run them with logging only and record the numbers here: accepted loops, end error before and after, largest change on the truth case, Release and Debug run time. Then set the limits (decision 6). Verify: both presets, `ctest`. If the drift case misses the spec (error not halved, or no loop accepted), mark it `should_fail` with the cause, record it, and pause. Commit.
- [ ] 2.2 Add `tests/consistency_test.cpp` with the seam and truth scenarios, for both `Consistency` and `Consistency2` (through `run_consistency`). First run with logging only and record, per variant: mean and largest pose error before and after, occupied 3 cm voxels before and after, largest change on the truth case, Release and Debug run time. Then set the limits (decision 6). Verify: both presets, `ctest`. If a variant misses the spec, mark it `should_fail` with the cause, record it, and pause. Commit.

## 3. Docs

- [ ] 3.1 Update:
  - `CLAUDE.md` Testing: the new test files, and the fixture shared with 6c;
  - `docs/ARCHITECTURE.md`: step 8 done, with the findings;
  - `docs/index.html`: any finding, where loop closure and consistency are described.

  Verify: `git diff --stat <change start> -- src core` is empty, and `openspec validate loop-closure-consistency-tests --strict` passes. Commit.
