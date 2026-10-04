# Design

## Context

What the two algorithms read and assume (from the code, 2026-10-04):

**`run_loop_closure(windows, params, point_store, logger)`:**
- It cuts the windows into segments of `loop_closure_segment_windows` windows. Each segment's points are loaded from `PointStore` (`PointSet::WindowIntermediate`) and placed with `windows[w].poses[p.index_pose]`. Points outside `min_distance_lidar`…`max_distance_lidar` are skipped, and the rest are decimated to 0.1 m voxels.
- **A revisit** pairs a segment with an earlier one that is at least `loop_closure_min_time_gap_s` before it and within `loop_closure_search_radius_m` horizontally and `loop_closure_max_vertical_offset_m` vertically. Both clouds need at least 500 points.
- **Registration:** a coarse yaw/translation search, then `ndt_refine`. The match is accepted at an overlap of at least `loop_closure_min_fitness` with a correction of at most `loop_closure_max_correction_m + 1` m and 15°.
- **Pose graph** over the segment poses, with the correction interpolated in time onto every node; `motion_model_poses` is set to the result. It returns the number of accepted loops.

**`run_consistency(windows, params, point_store, logger)`** (in `src/application/lidar_odometry.cpp`) runs `num_constistency_iter` passes of `Consistency`, or of `Consistency2` when `use_mutliple_gaussian` is set. Each pass:
- concatenates all nodes and loads all `WindowIntermediate` points;
- **re-assigns each point to a node by its timestamp:** the last node whose timestamp is below the point's;
- builds an NDT map from all points per 1,000,000-point chunk, only when a chunk has more than 10,000 points;
- solves one Gauss–Newton step with point-to-cell terms, a smoothness term (in each node's frame: weight 1e6 along x, 1e8 along y and z, 0.1° in rotation) and the first node fixed;
- skips points closer than 1 m (`Consistency`) or than `consistency_min_distance` (`Consistency2`).

The tests (`tests/CMakeLists.txt`) link `lo_infrastructure` and, with it, the inner layers. `scan_to_map_test.cpp` has a closed-room scene, face sampling and pose-error helpers, but in an anonymous namespace.

## Goals / Non-Goals

**Goals:**
- Each algorithm is called through its real entry point (`run_loop_closure`, `run_consistency`) on data shaped like a real run's, and judged against the truth.
- The numbers are measured and written down, so that a later change that moves them is noticed.
- Runs in seconds in Release, without recorded data or a display.

**Non-Goals:**
- Fixing anything the tests find: a finding becomes an expected failure with its cause, plus a note in the docs. A fix is its own change.
- Testing the private helpers of `loop_closure.cpp` (`coarse_search`, `ndt_refine`, `optimize_pose_graph`) one by one. They stay file-local; the end-to-end scenarios cover them.
- Changing `src/` or `core/`.

## Decisions

1. **A shared fixture, `tests/synthetic_recording.{h,cpp}`.**
   - **Scene:** a rectangular floor with a central block, so the trajectory drives a loop through four corridors, closed by outer walls, floor and ceiling. Boxes and pillars along the walls give each corridor structure along its length; without them a straight corridor is degenerate (the 6c finding).
   - **Truth:** about 1.5 m/s forward along the body x axis, turning at the corners. The smoothness term of `Consistency` assumes forward motion along x.
   - **Run:** two laps, nodes every 50 ms, windows of 1 s.
   - **Observations:** each node sees about 200 points sampled on the faces within `min_distance`…`max_distance`, with no occlusion and ±5 mm noise along the face normal, from fixed seeds. They are stored in the node frame with `index_pose` = the node's index in its window and `timestamp` = node time + half a node interval, so that consistency's timestamp lookup finds the same node.
   - **Size:** about 40 s of recording, roughly 160,000 points; it is reported in the first test run.

   Alternative: reuse the 6c room. Rejected, because a single room has no loop to close, and loop closure needs segments far apart in time.

   The scene helpers that 6c already has (faces, sampling, pose errors) move into the fixture, and `scan_to_map_test.cpp` uses them from there. That is a pure move; its numbers must stay identical.

2. **An in-memory point store, `tests/memory_point_store.h`:** a map from `(PointSet, index)` to points, implementing the `PointStore` port. The fixture fills `WindowIntermediate`.

3. **Disturbances, applied to the windows' `poses` and `motion_model_poses`, never to the stored points:**
   - **Loop-closure drift:** from the start of lap 2, a translation growing linearly to 0.5 m and a yaw drift to 2° at the end, as accumulated odometry drift looks.
   - **Consistency seams:** each window after the first gets a constant rigid offset (uniform ±3 cm, ±0.2°, fixed seed). Window 0 stays exact, because `Consistency` holds the first node.
   - **The truth case:** poses equal the truth.
   - **No revisit:** only the first lap is stored, so no two segments are `loop_closure_min_time_gap_s` apart.

4. **Options set in the tests:**
   - **Loop closure:** `loop_closure_segment_windows = 4` and `loop_closure_min_time_gap_s = 10`, to fit the short recording, with the radius, fitness and sigmas of `settings/lidar-vertical.toml`.
   - **Consistency:** `num_constistency_iter = 3`, the indoor grid 0.15 m and `consistency_min_distance = 0.4`, as shipped.

5. **Metrics, against the truth:**
   - **pose error:** the largest and the mean position and rotation difference over all nodes, and the end-of-run error;
   - **map sharpness:** the number of occupied 3 cm voxels of all points placed with the poses, the same measure `tools/eval_odometry.py` uses;
   - **loop closure:** also the return value (number of accepted loops).

   Each test prints its numbers with `MESSAGE`, as 6c does.

6. **Limits come from a first measurement.** Each scenario is first run with logging only. The limits are then set to the spec's expectation where that holds, tightened to the measured value with margin, and the measurement is written in a comment with its date. A variant that misses the spec's expectation gets `doctest::should_fail` and a comment that names the cause, and implementation pauses to report it, as in 6c.

7. **The test target copies its runtime DLLs** (`$<TARGET_RUNTIME_DLLS:lidar_odometry_tests>`), as `dMapper` does. Today the tests only run because `dMapper` has copied TBB into the same `bin/`.

8. **Loop closure is opt-in** (decided by the user during task 2.1). Each loop-closure run takes 15–27 s in Release, and one case didn't finish in 15 minutes in Debug, so the scenarios are marked `doctest::skip()`. `ctest` and a plain test run skip them. `lidar_odometry_tests -tc="loop closure*" --no-skip` runs them, in either build, to debug the loop closure. They keep their checks, so a request run still says whether the loop closure meets them.

9. **Checks:**
   - both presets build `lidar_odometry_tests`, and `ctest` passes;
   - `git diff --stat -- src core` is empty for the whole change, so no `44_short` / 44 runs are needed: the product code doesn't change.

## Risks / Trade-offs

- **[Debug run time]** `Consistency` builds and solves a system over every node (about 800 nodes × 6 unknowns) and an NDT map of all points. Unoptimized Eigen is slow. → Measure both presets in the first run. If Debug takes more than about a minute, shrink the fixture (fewer points per node), not the scenarios, and record the reason.
- **[No occlusion]** Synthetic points can be "seen" through the central block. Neither algorithm models visibility, but loop closure's overlap score may come out higher than on real data. → It is noted in the fixture; the tests judge the trajectory against the truth, not the overlap value.
- **[Expectations the code may not meet]** For example, the consistency pass may not reduce the error of rotation-only seams, or `Consistency2`'s time segments may be too long for a 40 s recording. → Such a case becomes an expected failure that names the cause, and it is reported (decision 6). The decision about a fix stays with the user.
- **[Fixture changes move 6c's numbers]** The shared helpers come from `scan_to_map_test.cpp`. → The move is pure, and the 6c messages must print the same numbers before and after.
