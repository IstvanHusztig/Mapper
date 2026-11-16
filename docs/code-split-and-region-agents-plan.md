# Code split and region agents ("divide et impera"): plan

Status: **agreed 2026-11-16** (decisions below). Branch `honest-degeneracy-measure`, nothing pushed.

## Why

Livox 4 has two faults left, both inside the per-window optimizer: the passage stall (100–125 s, about 5 m of a true
10 m) and a flyaway (200–280 s). Today each look at them costs a full 6-minute run, and the optimizer is one function
of 1064 lines (`optimize_lidar_odometry`) in which nine terms add into one matrix. The log can't say which term holds a
window back or pushes it away, and no term can be tested alone.

This plan makes four things possible:

1. **Rerun one bad stretch in seconds** (window capture and replay).
2. **See what each term does to a window** (per-term log: cost, weight and pull, per direction).
3. **Test and change one term without reading the others** (one function per term).
4. **Give each part of the code an owner with a small context** (region agents).

Livox 4 is parked until stage 7 and is then the first job for the regions (stage 8).

## Rules that apply to every stage

- A refactor commit changes no number. After every commit: unit tests, the replay check (seconds, from stage 1 on) and
  `44_short` against its manifest (1.5 min). At the end of every stage: the full 44 and Livox 3 against their manifests.
- A change that alters numbers never shares a commit with a refactor, is measured first, and its verdict is the user's.
- Release builds. A stage that touches the parallel loops, lifetimes or asserts (stages 1, 2, 4) also runs the Debug
  build and `ctest` once at its end.
- Runs are sequential (`real_time_threshold_seconds` makes parallel runs timing-dependent).
- No dead code: nothing stays behind a switch that is off without a decision.

## Stage 0: preparation

| Step | What | Why |
|---|---|---|
| 0.1 | Decide the four switches that are off (decision 1 below). | Each one adds branches to the NDT term; the split is smaller without them. |
| 0.2 | Record the build's identity in the results JSON: git commit, "uncommitted changes" flag, compile definitions. | Today's lesson: the good Livox 3 map could not be traced to its code from its output folder. |
| 0.3 | `tools/function_sizes.py`: lists every function above a length limit. | Makes "huge functions" a number that can be tracked (table in stage 6). |

## Stage 1: window capture and replay

**Capture.** A debug setting (`[lidar_odometry_debug_info]`, so it stays outside the snapshot hash) names a time range.
When the window loop reaches the range, it writes one file with everything the loop needs to continue from there:

- the settings in use, and the per-run setup the loop derives from the first window (`OdometryRunSetup`: whether
  accelerometer data exists, the gravity norm);
- the loop's state at the first window of the range: all of `OdometryState` (both NDT maps with every cell field bit
  for bit, including the stored inverses; `m_g`; the lengths; the reference points) and all of `WindowLoopCarry`;
- the two windows before the range (the velocity estimate and the pose prediction read them), the windows of the range
  as they are at that moment, and the range's points;
- per window, what the run decided and produced: the number of iterations and whether the time limit stopped it, and
  the final poses.

**Replay.** A CLI mode (like `--from-snapshot`) loads the file and runs the same loop over the range. It forces the
recorded iteration counts, so the wall clock can't change the result, and compares the poses with the recorded ones
bit for bit. With a second settings file it runs the range with changed settings instead: the first screening of an
idea in seconds.

**Limit.** A replay with changed settings starts from the captured map. It answers "what does this change do in this
stretch", not "what does it do to the run". A full run confirms.

**Where the code goes.** A small port in the application layer (`WindowCaptureSink`), its file adapter in the
infrastructure next to `odometry_snapshot_file`, the replay as a use case. The window loop only calls the port.

**Refactors needed first (each bit-identical, own commits).**

- `WindowLoopCarry`, `OdometryRunSetup` and the accelerometer-term struct are private to `run_odometry.cpp`. They move
  to a header so the port can name them.
- `optimize_windows` can only start at window 0, and the first window's special steps (placing it, seeding the map,
  the rest check) are inside the loop. The loop is split into "start a run" and "process windows first..last from a
  given state", which the full run and the replay both call.

**Checked by the reviewer, no problem found.**

- The NDT maps are `unordered_map`s, whose iteration order can differ after loading. No code on the odometry path
  iterates over them (only lookups by key; `update_rgd` sorts its index lists).
- `ScanToMapWorkspace` (the Cholesky analyses kept from window to window) affects speed only, so a replay may start
  with an empty one.
- The time limit is the only wall-clock dependence in the loop; recording the last iteration is enough.

**Open risk.** File size: the maps and map points of one start state, plus the windows' points. Expected tens to a few
hundred MB per range; measured in the first capture.

**Done when:** a captured range of `44_short` and one of Livox 3 replay bit-identically, and a unit test captures and
replays the synthetic loop recording.

## Stage 2: split `optimize_lidar_odometry` (pure refactor)

Today's function, by its line ranges, and the function each part becomes:

| Lines today | Part | Becomes |
|---|---|---|
| 159–220 | Layout of the unknowns, anchors, band of the matrix | `layout_window_unknowns` → `WindowUnknowns` |
| 222–637 | NDT point terms: matching, Jacobian, Huber and scale, per-node parallel sum, degeneracy projection, translation information | `add_ndt_terms`, one function per sub-step, returning `NdtTermResult` |
| 640–835 | Motion-model edges (rotation and position together), gyro-bias coupling and prior | `add_motion_edge_terms`, `add_gyro_bias_prior` |
| 836–890 | Velocity continuity | `add_velocity_continuity` |
| 892–1047 | Accelerometer term | `add_accelerometer_terms` |
| 1048–1081 | Cost, origin fix | `window_cost`, `fix_origin` |
| 1082–1171 | Solve, covariance of the extra unknowns | `solve_window_step` |
| 1172–1210 | New poses, big-jump rejection | `apply_window_step` |

- Each term is a free function `add_<term>(problem, normal_equations)`. `optimize_lidar_odometry` stays as a conductor
  of about 100 lines that calls them in today's order. No class hierarchy of terms: there is one optimizer, and a list
  of functions is enough until a second user exists.
- **Bit-identity rule:** every matrix entry receives the same additions in the same order as today. The terms are
  moved, never reordered, and the per-node parallel sum keeps its structure.
- **Rotation and position edges stay one function.** One generated call builds each edge's 12×12 block from all six
  weights, so two functions would round differently. The gyro-bias coupling sits in the same per-edge loop and stays
  there as a sub-function. The per-term log (stage 3) still reports rotation and position separately: it computes its
  own figures and doesn't have to match the matrix bit for bit.
- **The solve depends on NDT results.** The covariance of the extra unknowns reads the NDT blocks after projection and
  the normal-direction information; the cost reads the matched and unmatched counts. `add_ndt_terms` returns them in
  one struct (`NdtTermResult`) instead of the solver reaching into NDT internals.
- **The indoor and outdoor point functions are not copies.** They differ in the distance limit, in the weight (indoor:
  planarity × norm, capped at 10; outdoor: planarity only) and in the normal information (indoor only, so the
  degeneracy measure sees indoor points only). They are merged into one function with a per-grid policy as the last
  commit of the stage, and only if the bits stay; otherwise they stay two.
- The phase timers cut across the new functions and are passed in.
- One term per commit, the simplest first (origin fix, velocity continuity, gyro-bias prior, motion model,
  accelerometer, NDT last). Files: one per term group in `src/domain/` (`ndt_term`, `motion_model_terms`,
  `accelerometer_term`, `window_solver`).
- Each term gets a unit test on a synthetic window: its residual is zero at the true state, and its analytic Jacobian
  matches a numeric one.

## Stage 3: per-term log

With the terms separate, a diagnostic wrapper records per window and per term (NDT, gyro rotation, position prior,
velocity continuity, accelerometer, bias and offset priors, velocity prior):

- the cost before and after the solve, and the number of residuals;
- the weight (information) on the window's translation along three directions: along travel, sideways, vertical;
- the pull along the same directions, in metres: where this term alone would move the window.

One line per window in the log, and the same as CSV rows (also the per-window features for the future learned tuner).
The log only observes: results are identical with it on or off.

This is the table that answers the Livox 4 questions: in the passage, which term outweighs the others along the travel
direction; at 200 s, which term starts the flyaway.

## Stage 4: solver and window verdict

- **Solver.** The iteration loop (`run_lm_iterations`, now in `run_odometry.cpp`), the damping rule and the step
  acceptance move into one unit that knows nothing about the terms: it asks for the normal equations and a cost, and
  returns a step. Pure refactor.
- **Window verdict.** After the iterations and before the map update, one function judges the window from what is
  already known: bend against the gyro, speed, per-term cost against its expected value, rejected steps, degeneracy.
  In this stage it only logs its verdict.
- Not in this stage (each a measured change of its own, later): acting on the verdict (a suspect window stays out of
  the map or is solved again), and the known flaw that every window runs all 100 iterations.

## Stage 5: one place for the weights

Today the weights come from many places: sigmas in the options, the time-scaled rotation sigma, the accelerometer
noise formula, the two 0.1 scales, the NDT cell covariances, the sigma raise for degenerate windows in
`optimize_windows`. One function builds a window's complete noise model from the settings and the window; the terms
receive weights and never compute them. Pure refactor. With stage 3 this is the "noise budget": every weight in one
struct, and its effect in one log line.

## Stage 6: the other huge functions

Measured today (lines):

| Tier | Function | Lines | When |
|---|---|---|---|
| 1 | `optimize_lidar_odometry` | 1064 | stage 2 |
| 1 | `optimize_windows` (179), `run_lm_iterations` (about 115) | | stage 4 |
| 2 | `Consistency` (311) | | before the back-end region's first job |
| 4 | `Consistency2` (483): only with `use_mutliple_gaussian`, off in both sensor settings | | decision 4: remove, or split with tier 2 |
| 2 | `run_loop_closure` (336), `optimize_loop_pose_graph` (134) | | same |
| 3 | `compute_step_1` (180), `calculate_trajectory` (143), `load_imu` (110) | | before the front-end region's first job |
| 4 | `optimize_sf2` (275), `optimize_rigid_sf` (272) | | decision 4: remove, or split with tier 2 |
| 5 | `exportLaz` (185), `save_result` (146), `TomlIO::LoadParametersFromTomlFile` (116), `processing_results` (110), `pair_recording_files` (100), GUI and viewer functions (96–130) | | when next changed |

Method for each: name the steps the function already has, move each into a function with one job (SOLID, single
responsibility), bit-identical, with a test for each step that has maths in it. Target: no function above about 80
lines in tiers 1–3.

Tiers 2 and 3 are split when their region is about to work, not all up front: the split is cheapest when the region's
owner needs to read the code anyway, and Livox 4 doesn't wait for them.

## Stage 7: region agents

**Mechanism.** One agent definition per region in `.claude/agents/region-<name>.md` (like the existing
`master-reviewer`), and one knowledge file per region in `docs/regions/<name>.md`. The agent starts cold each time, so
the knowledge file is its memory: the region's contract, files, known flaws, experiments done and ideas rejected. The
agent reads it first and updates it last. It is in git and readable by the user.

| Region | Owns | Contract |
|---|---|---|
| front-end | recording loading, AHRS (`calculate_trajectory`), window cutting, `imu_kinematics` | Windows with correct times, IMU samples and first poses |
| map | `ndt_map`, map update and sliding rebuild | Cell statistics; what enters the map and when |
| lidar-matching | `ndt_term` | A point's residual, Jacobian and weight against a cell; degeneracy |
| inertial | motion-model terms, accelerometer term, gyro bias, velocity carry, `window_steps` | The inertial terms and what passes from window to window |
| solver | normal equations, solve, iteration loop, window verdict | A step from terms; the verdict |
| back-end | loop closure, consistency | Corrections after the odometry |
| noise-budget | the weights' one place, the per-term log | The balance between the terms; owns no term |
| verification | `tools/`, the verify skills, baselines, replay | Checks the others' results; never edits `src/` |

**How a problem is handled.**

1. The orchestrator (the main session) captures the bad stretch and reads the per-term log: which term, so which region.
2. That region's agent gets the question, the capture file and its knowledge file. It works by replay and returns a
   finding with its evidence, a proposed change, and what it could not measure.
3. The verification agent runs the checks on the proposal (replay, tests, baselines, verify skills). A Sonnet reviewer
   checks the conclusion.
4. The numbers go to the user, who decides. Full runs and the map verdict stay as today.

**Rules.** One region edits at a time, in the one working tree. A change to a shared type (`Window`, the options) goes
through the orchestrator. A region never changes another region's files; it reports what it needs.

**Models.** Region owners: Opus, medium. Verification and the reviewer: Sonnet. Design questions stay with the main
session on Fable, high.

## Stage 8: resume Livox 4

Capture 95–125 s (passage) and 195–285 s (flyaway) from one full run, replay with the per-term log, name the term,
hand it to its region.

## Decisions (user, 2026-11-16)

1. **Switches that are off:** `degenerate_projection_all_point_normals`, `carried_covariance_normal_information_only`
   and `ndt_unmatched_point_cost` are removed in stage 0 (with the `all_points_*` cell fields and their map-update
   code). `lm_step_check` and its cost code stay: the per-term log and the window verdict need a cost.
2. **Order:** the per-term log comes after the term split (stage 3).
3. **Regions:** the eight listed in stage 7.
4. **Code the sensor settings never reach:** robust mode (`optimize_sf2`, `optimize_rigid_sf`,
   `use_robust_and_accurate_lidar_odometry`) and `Consistency2` (`use_mutliple_gaussian`) are removed in stage 0.
