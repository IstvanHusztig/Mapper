# Tasks

**Checks on every commit** (design decision 9):
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

- [x] 2.1 Add `tests/loop_closure_test.cpp` with the three loop-closure scenarios (decisions 3–5), skipped in the regular run (decision 8). First run them with logging only and record the numbers here: accepted loops, end error before and after, largest change on the truth case, Release and Debug run time. Then set the limits (decision 6). Verify: both presets, `ctest`. If the drift case misses the spec (error not halved, or no loop accepted), mark it `should_fail` with the cause, record it, and pause. Commit.

  **Result (2026-10-04).** The run was paused after measuring, and the user decided: no loop-closure unit tests in the regular run, but the scenarios stay in the executable for debugging (design decision 8). All three cases are `doctest::skip()` and run with `-tc="loop closure*" --no-skip`. Drift and no revisit carry their checks; the truth case is `should_fail` with its cause. The plain run shows 46 passed and 3 skipped; with `--no-skip`, all 3 pass (one expected failure). Measurements: Fixture: 150 points per node within 15 m. Options: shipped values, with segments of 4 windows and a 10 s time gap.

  | Case | Loops accepted | Result | Release time |
  |---|---|---|---|
  | Drift on lap 2 (0.5 m, 2°) | 5 | end error 568 → 113 mm, mean 112 → 35 mm (spec: halved, met) | 26.6 s |
  | Truth | 5 | largest change **31 mm** / 0.095° (spec: 5 mm and 0.1°, **missed**) | 27.0 s |
  | No revisit (16 windows) | 0 | poses bit-identical (met) | 13 ms |

  Debug: one loop-closure case did not finish in 15 minutes.

  Experiments, none of them committed:
  - **The truth case's 31 mm comes from registration noise.** Each already-aligned revisit returns a "correction" of 1–3 cm and up to 0.2°. The pose graph trusts loop edges to `loop_closure_edge_sigma_m = 0.05` and spreads them onto the nodes. The exact NDT statistics don't change it (31.8 mm). Denser points help a little: 150 points within 8 m give 25 mm; 100 points within 8 m give 46 mm. With 50 points per node, no loop is accepted at all.
  - **A false loop:** with points only within 6 m, loop closure accepted a wrong registration and moved the true trajectory by 1.5 m (6 loops accepted). The overlap threshold alone doesn't catch it.
  - **Run time:** a 2 m search range (`loop_closure_max_correction_m`, shipped 6 m) gives 16 s instead of 26 s. The rest is the fixed fine stage of `coarse_search` (3,773 overlap evaluations over the 0.2 m voxels of the source).
- [x] 2.2 Add `tests/consistency_test.cpp` with the seam and truth scenarios, for both `Consistency` and `Consistency2` (through `run_consistency`). First run with logging only and record, per variant: mean and largest pose error before and after, occupied 3 cm voxels before and after, largest change on the truth case, Release and Debug run time. Then set the limits (decision 6). Verify: both presets, `ctest`. If a variant misses the spec, mark it `should_fail` with the cause, record it, and pause. Commit.

  **Paused (2026-10-04) after the first measurement.** Both truth cases moved the true trajectory at the corner windows, against the fixture's instant turn-rate step (0 to 57°/s in one 50 ms node): `Consistency` 8.3 mm / 0.23°, `Consistency2` 27 mm / 3.2°. `Consistency2` didn't reduce the seams' rotation error (0.207° → 0.217°). The user decided: ramp the fixture's turn rate realistically, re-measure, then set limits, with remaining misses as expected failures with their cause.

  **Fixture change:** each corner's curvature now ramps linearly from 0 to 1/(2 m) over 1 m (0.5 s at 2 m/s) and back (clothoid transitions). The straights stay on the corridor centrelines, and the lap is 44.4 m (22.20 s). The fixture test also checks that the turn changes by at most 0.005 rad from node to node. Loop closure, re-measured with `--no-skip`, is unchanged: drift 575 → 112 mm with 5 loops (20.6 s), truth 32 mm / 0.09° (still expected to fail), no revisit 0 loops.

  **Result (2026-10-04)**, 3 passes; seams of ±3 cm / ±0.2° per window:

  | Variant | Seams: mean error | Seams: 3 cm voxels (truth 125,898) | Truth: largest change (mean) | Release |
  |---|---|---|---|---|
  | `Consistency` | 27.9 mm / 0.207° → 19.5 mm / 0.147° | 127,956 → 126,563 | **6.4 mm** / 0.075° (2.1 mm) | 1.0 s |
  | `Consistency2` | 27.9 mm / 0.207° → 14.6 mm / 0.125° | 127,956 → 126,324 | **16 mm / 0.46°** (5.2 mm) | 0.6 s |

  - **Seams** (met by both): the checks are mean position and rotation error at most 0.8 × before, and the voxels' excess over the truth at most halved.
  - **`Consistency` truth case** (`should_fail`): misses 5 mm by 1.4 mm. The change is spread evenly over all windows (2–6 mm, 0.03–0.07° each), which is the noise of fitting 150 points per node to 0.15 m cells, not a pull at the corners any more.
  - **`Consistency2` truth case** (`should_fail`): the rotation changes sit in the corner windows (0.2–0.46°, against about 0.05° on the straights). Its smoothness term holds the change of turn rate to 0.01° per node, and the ramp changes the turn by 0.29° per node. With a 1 s ramp (tried, not kept) the corner windows drop to 0.18°, and an even 5–12 mm / 0.1° remains in every window.

  Both presets build and `ctest` passes (Release 7 s; Debug 132 s, of which the four consistency cases take about 70 s, at the design's one-minute bound, so the fixture stays). `src/` and `core/` are unchanged.

## 3. Docs

- [x] 3.1 Update:
  - `CLAUDE.md` Testing: the new test files, and the fixture shared with 6c;
  - `docs/ARCHITECTURE.md`: step 8 done, with the findings;
  - `docs/index.html`: any finding, where loop closure and consistency are described.

  Verify: `git diff --stat <change start> -- src core` is empty, and `openspec validate loop-closure-consistency-tests --strict` passes. Commit. Result: `CLAUDE.md` Testing names the fixture, the in-memory store, both test files, the `--no-skip` run and the `should_fail` convention; `docs/ARCHITECTURE.md` marks step 8 done with the findings; `docs/index.html` adds two known limitations (loop closure and consistency both move a correct trajectory, and the false loop at 6 m range). The change validates strictly, and `src/` and `core/` are unchanged.
