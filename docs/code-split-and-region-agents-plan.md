# Code split and region agents ("divide et impera"): plan

<!-- docs-nav -->
> [dMapper docs](README.md) › Code and architecture · [All documents](README.md) · [User guide](index.html)

Status: **agreed 2026-11-16** (decisions below). Branch `honest-degeneracy-measure`, nothing pushed.

Progress:
- Stage 0 done 2026-11-16 (43748c7 .. c365fdd): three switches, robust mode and Consistency2 removed (retired settings
  still load), build identity in the results JSON, `tools/function_sizes.py`; 44, 44_short and Livox 3 bit-identical.
- Stage 1 done 2026-11-16 (46c56b2 .. 133f63a): `process_windows`, capture (`capture_windows_from_s` / `_to_s`),
  `dMapper --replay`; replays identical: 44_short 21 of 21 windows in 35 s, Livox 3 84 of 84 in 93 s.
- Stage 2 done 2026-11-16 (0710807 .. addd8e4): `optimize_lidar_odometry` 1064 -> 83 lines, the terms in `ndt_term`,
  `motion_model_terms`, `accelerometer_term`, `window_solver`; 44 and Livox 3 bit-identical, Debug `ctest` passes.
- Stage 3 designed 2026-11-17 (below); implemented 2026-11-17 (OpenSpec `per-term-window-log`, 0e4d42a .. ): the four
  splits bit-identical, `window_term_table`, setting `log_window_terms`, the CSV files, `tools/window_terms.py`. Livox 3
  replay with the setting on: 84 of 84 windows identical.
  Archived 2026-11-17 (b234b6a); the first reading of the table is `docs/window-terms-livox3.html`.
- Stage 4 designed 2026-11-17 (below), decisions taken, OpenSpec `solver-and-window-verdict`; implemented 2026-11-17
  (4ed2f46 .. ): loop guard, `lm_solver` + `WindowProblem`, `process_windows` in four steps, the window verdict and its
  summary; every commit identical on the guards, replays and 44_short. End runs identical (manifests 4896eeb), Debug
  ctest pass.
- Stage 5 designed 2026-11-17 (below), decisions taken, OpenSpec `window-noise-model`; implemented 2026-11-17
  (fd63f71 .. ): `WindowNoiseModel` built once per window, every term reads its weights from it, the per-term table and
  the weak-axis diagnostic too, the priors' growth formulas in one place, `noise budget` / `log_window_noise`; every
  commit identical on the guards, replays, 44_short and the per-term table.
- Stage 6 designed 2026-11-18 (below), decisions taken, OpenSpec `split-window-prediction`; implemented 2026-11-18
  (3c0fdda .. ): one rest test (`window_rests`), `predict_window_poses` in four steps with the chaining through
  `place_window_after`; every commit identical (`tools/function_sizes.py` now also counts functions in namespaces and
  headers). No tier-1 function above 80 lines; the rest
  waits for its region (table in the stage 6 design).
- **Verdict limits decided (user, 2026-11-17)**, after the readings of full runs of 44, Livox 3 and Livox 4 (task 6.1 of
  `solver-and-window-verdict`): all agreed limits stay (bent 3°, bent severely 10°, run trigger above 1 % bent; fast
  2.5 m/s; bias high 0.5 °/s; velocity chi² of the weak windows median 5 / p95 20). Three recordings, one map judged
  good by eye. Notes:
  - Bent and fast mark Livox 4's flyaway (76 of 85 bent and all 124 fast windows lie in 200–280 s); neither sees the
    passage (100–125 s: 2 bent, 0 fast). Livox 3: largest bend 1.82°, largest speed 1.28 m/s. 2.5 m/s is a handheld
    assumption, not a measured limit.
  - Bias high says almost nothing on the L2 (634 of 689 windows of 44: the faulty gyro, and no start-up bias is
    subtracted on a handheld start). On Livox 4 it starts at 143 s and 397 of its 615 windows lie outside the bad
    stretches; a wrong estimate carried on, real rotation errors or temperature drift are all open. A run trigger on
    the bias trend or the share of high windows is an alternative for later.
  - The velocity chi² triggers on all three, also on the good Livox 3 (weak windows median 35), so on these recordings
    it doesn't separate good from bad: a question about the noise model (the velocity prior's covariance, or missing
    process noise between windows) for stage 5, not a reason to loosen the limit. Its rise in the Livox 4 passage
    (median 1120 against 230 outside) is a hint from one recording.
  - No limit is tested on a failure every flag catches; the passage is caught only by the chi².

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

- a hash of the odometry settings in use, and the run's first window, from which the replay derives the per-run setup
  again (`OdometryRunSetup`: whether accelerometer data exists, the gravity norm);
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

Design of 2026-11-16, from the function as it stands after stage 0 (`src/domain/scan_to_map.cpp`, lines 147–1121, 975
lines), reviewed against the code. It replaces the first sketch of this stage.

### What the function does today, in order

| Lines | Part | What it reads beyond the window |
|---|---|---|
| 157–219 | Layout: which unknowns exist (poses; gyro bias; acceleration offset, start and end velocity), their column indices, the accelerometer anchors, the band of the matrix; the matrix and right-hand side | rotation and accelerometer options |
| 175–193 | The motion-model weights from the sigmas, and the position scale (used only by the edges) | motion model, accelerometer options |
| 220–536 | NDT point terms, indoor then outdoor grid: per node in parallel, each point's block is summed into its node's block (or straight into the matrix when the projection is off), with the normal information (indoor only) and the cost counts | points, both maps, scan-to-map options |
| 545–579 | Degeneracy projection of the node blocks into the matrix | scan-to-map options |
| 581–587 | The window's translation information (only when the caller asks for it) | |
| 589–772 | Motion-model edges between consecutive nodes: rotation and position in one generated 12×12 block, and inside the same loop the gyro-bias coupling | motion model, rotation options |
| 774–784 | Gyro-bias prior | rotation options |
| 786–840 | Velocity continuity (only in windows without the accelerometer term), with the tie to the previous window's velocity | motion model, velocity prior |
| 846–994 | Accelerometer term: the acceleration terms over the anchors, start and end velocity terms, start-velocity prior, zero velocity at rest, a weak end-velocity tie when its term was dropped, offset prior | accelerometer options, acceleration offset |
| 998–1008 | Cost: NDT cost and counts summed over the nodes, plus the motion cost | |
| 1011–1023 | Origin fix; band to sparse matrix | motion model |
| 1028–1052 | Covariance of the extra unknowns (undamped matrix); writes the bias, offset and end-velocity covariances | |
| 1055–1056 | Return here when the caller only wanted the cost and the covariances (`apply_step = false`); `delta` stays untouched | |
| 1059–1079 | Damping, solve | LM factor |
| 1082–1120 | New poses, big-jump rejection, update of bias / offset / velocities, delta | rotation options (bias clamp) |

Since stage 0 removed the normal-only carried covariance, nothing after line 587 reads an NDT-internal value except the
cost block: the NDT part hands on only the cost counts and the translation information.

### Two things that carry bits across the terms

1. **The matrix and the right-hand side.** Every entry must receive the same additions in the same order. The terms are
   called in today's order and keep their loops; nothing is reordered. The parts are strictly sequential today, except
   the gyro-bias coupling, which sits inside the edge loop and stays there.
2. **The motion cost is one running sum.** `motion_cost` is added to in this order: edges, bias prior, velocity
   continuity and its tie, each acceleration term, start-velocity prior, zero velocity at rest, weak end tie, offset
   prior. Summing per term and adding the totals afterwards would round differently, and with `lm_step_check` on a
   different cost can flip a step decision. So the terms add to one shared accumulator in today's order. The
   accumulator also keeps a per-term subtotal beside the running sum; stage 3 reads the subtotals, the step check keeps
   reading the running sum.

### The types the terms share (`src/domain/window_equations.h`)

- **`WindowUnknowns`**: `n_nodes`, `n_pose_params`, `n_params`, the flags `have_dt`, `time_scaled_sigma`,
  `estimate_bias`, `acc_term`, the columns `ib`, `io`, `iv`, `ie`, the `anchors` and `pose_band_offset`. Built by
  `layout_window_unknowns(window, rotation_options, translation_imu_options)`.
- **`WindowEquations`**: the banded matrix, the right-hand side, and the cost accumulator (running sum and per-term
  subtotals). Terms write only through it.
- **`NdtSums`**: what the point loops leave for the conductor: the node blocks and right-hand sides (when the projection
  is on), the per-node normal information, and the per-node cost and matched / unmatched counts.
- Each term function takes exactly the inputs it reads (the window, the layout, its own options, the acceleration
  offset where it reads it) and the `WindowEquations`. No "everything" struct: a term's signature shows what it depends
  on, and its test builds only that.

### The functions

| File (`src/domain/`) | Function | Lines today |
|---|---|---|
| `window_equations.{h,cpp}` | `layout_window_unknowns` | 162–174, 195–214 |
| `ndt_term.{h,cpp}` | `sum_ndt_point_terms`: both grids, parallel per node. With the projection on it fills the node blocks of `NdtSums`; with it off it adds each point straight into the matrix, as today | 220–536 |
| | `add_projected_ndt_blocks` | 545–579 |
| | `window_translation_information` | 581–587 |
| `motion_model_terms.{h,cpp}` | `add_motion_edge_terms`: the weights, rotation, position and bias coupling in one loop, as today | 175–193, 589–772 |
| | `add_gyro_bias_prior` | 774–784 |
| | `add_velocity_continuity` | 786–840 |
| `accelerometer_term.{h,cpp}` | `add_acceleration_terms` (anchors, start and end velocity terms; returns whether the end velocity was tied) | 846–951 |
| | `add_velocity_priors` (start prior, zero velocity at rest, weak end tie) | 953–985 |
| | `add_acceleration_offset_prior` | 987–993 |
| `window_solver.{h,cpp}` | `fix_origin` | 1011–1019 |
| | `extra_unknowns_covariance` (returns the bias, offset and end-velocity covariances when the solve succeeds) | 1028–1052 |
| | `solve_window_step` (damping, solve) | 1059–1079 |
| | `apply_window_step` (new poses, big-jump rejection, updates, delta) | 1082–1120 |
| `scan_to_map.cpp` | `optimize_lidar_odometry`: the conductor, about 100 lines: the calls in today's order, the cost block, the return for `apply_step = false`, and the phase timers around the calls | |

The two NDT point lambdas stay two functions at first. They differ in the distance limit, the grid, the weight and the
normal information:

| | Indoor | Outdoor |
|---|---|---|
| Weight | planarity × range | planarity |
| Cap at 10 | yes | no |
| Applied when | planarity or range weight is on | planarity is on |
| Normal information | added | not added |

One function with these four as a per-grid policy performs the same operations (for the outdoor grid the range factor is
1.0, and × 1.0 is exact). The merge is the last commit of the stage and stays only if the guard below shows no change.

The timers stay the phases they are; the projection is still timed with the motion terms.

### The guard: which branches the runs don't reach

The replays and baselines only exercise the shipped settings. Not reached by any run:

- the cost code and the cost-only call (`lm_step_check` is off);
- planarity and range weights (both off);
- the path without the projection, and without the outdoor grid or the view-point test (both always on);
- constant rotation sigmas, no bias estimation in a later window, windows without timestamps;
- the translation information output (only with `log_window_diagnostics`);
- rare data-dependent paths: an accelerometer term dropped by an IMU gap, the weak end-velocity tie, the bias clamp, a
  failed solve, a big jump.

So the first commit of the stage adds a **characterization check**: a skipped doctest case that runs
`optimize_lidar_odometry` on the synthetic room and corridor windows over a matrix of these options (and constructs the
rare cases: an IMU gap, a bias at the clamp, a guess more than 1 m off) and writes every output (poses, bias,
velocities, offset, covariances, translation information, cost fields, delta, the return value) as hexadecimal doubles
to a file, plus a script that compares two such files. It is recorded once from today's code; every commit of stages 2
to 5 must reproduce it exactly. It is a development guard, not a regression test of values: the file is not committed,
because the bits may differ between compilers and between Release and Debug.

### Checks per commit

| Check | Covers | Time |
|---|---|---|
| Characterization file identical | every option branch, on synthetic windows | seconds |
| Unit tests | | 15 s |
| Replay of the 44_short capture (21 windows) and the Livox 3 capture (84 windows) | the shipped paths on real data, Huber and scales included | 35 s + 93 s |
| `44_short` against its manifest | the whole run, first window included | 2 min |

End of stage: the full 44 and Livox 3 against their manifests, Debug build and `ctest`.

### Commit order

1. The characterization check and its script.
2. `WindowUnknowns` and `WindowEquations` introduced; the function body uses them, nothing moves yet.
3. `fix_origin`, `extra_unknowns_covariance`, `solve_window_step`, `apply_window_step`.
4. `add_gyro_bias_prior`, `add_velocity_continuity`.
5. `add_motion_edge_terms`.
6. The three accelerometer functions.
7. `sum_ndt_point_terms`, `add_projected_ndt_blocks`, `window_translation_information`.
8. The merge of the two NDT point functions (kept only if the guard is unchanged).

### A test per term

Each move commit adds a test of the moved term on a synthetic window. For the terms with a cost it is the same test:
**the right-hand side is minus half the gradient of the term's cost.** The test moves each unknown the term touches by
±h, reads the term's cost from the accumulator, and compares the numeric gradient with the right-hand side the term
wrote. One check covers the residual, the Jacobian, the weight and their signs together, without exposing the Jacobians.
Where the residual is linear in the unknowns (velocity continuity, the priors, the translation part of the accelerometer
term) the test also compares the matrix with half the numeric second derivative.

The relation holds for the NDT points, the edges with their bias coupling, the bias prior, the velocity continuity, the
acceleration terms, the start-velocity prior, the zero velocity at rest and the offset prior. Special cases:

- **NDT points:** tested with the projection off; the step stays inside the cells (a point that changes cell makes the
  cost jump). The Huber weight is covered: its cost 2c√s − c² has the derivative c/√s, which is the weight.
- **Projection:** its own test: with every direction kept the blocks pass unchanged; with one direction removed a block
  B becomes M B M and its right-hand side M g.
- **Weak end-velocity tie:** it treats the start velocity as fixed, so the gradient is compared for the end velocity
  only.
- **Origin fix:** no cost and no right-hand side; the test checks the six weights on the first node's diagonal.
- **Acceleration terms:** the rotation part of the Jacobian is itself a central difference, so the comparison has a
  tolerance.

These tests are also the base of stage 3: the per-term cost, weight and pull come from the same accumulator.

## Stage 3: per-term log

Design of 2026-11-17, from the code after stage 2 (`addd8e4`), reviewed against the code. It replaces the first
sketch of this stage. The user's decisions are at its end.

With the terms separate, the window loop records per window and per term its cost, how strongly it holds the window
and where it pulls it. This is the table that answers the Livox 4 questions: in the passage, which term outweighs the
others along the travel direction; at 200 s, which term starts the flyaway. The log only observes: results are
identical with it on or off.

### Why "translation of the window" is not enough

The sketch asked for each term's weight on the window's translation. Read from the code, two terms would then show
nothing at all:

- The gyro rows of the edges only see the rotation between two nodes. Moving nodes doesn't change them.
- The accelerometer terms only see a change of velocity. Their coefficients sum to zero (`1/dt1 − (1/dt1 + 1/dt2) +
  1/dt2`), so a window that moves as a whole, or travels faster at constant speed, costs them nothing.

And a third fact: the origin fix (sigma 1e-6 in both sensor settings, weight 1e12) pins the first node of every window,
so a window cannot move as a whole at all. What the solver decides is how the window continues from its first node.

So the table measures four motions of a window. Each has one amplitude: what it does to the last node.

| Motion | What moves (τ = time since the first node / window duration, 0 … 1) | Amplitude | Who can resist it |
|---|---|---|---|
| shift | every node by the same vector | metres | NDT, origin fix |
| stretch | node k by a·τ; start and end velocity by a / duration | metres at the last node | NDT, edge positions, velocity prior, zero velocity, velocity continuity |
| curve | node k by a·τ²; end velocity by 2a / duration | metres at the last node | NDT, edge positions, accelerometer, zero velocity, weak end-velocity tie, velocity continuity |
| turn | node k's attitude by the rotation θ·τ, positions unchanged | degrees at the last node | NDT, edge rotations (gyro), edge positions, accelerometer |

The velocities move with the nodes as the motion's own derivative, so the accelerometer terms see a stretch as what it
is: no change of acceleration. A window without the accelerometer term has no velocity unknowns; there the motions
move the nodes only. The gyro-bias coupling of the edges, the bias prior and the offset prior act on the bias and the
offset, which none of the four motions moves: the table shows their cost, not a weight.

The turn is a rotation in the world frame, turned into changes of (om, fi, ka) with the Jacobian the accelerometer
term already uses (`tait_bryan_rotation_jacobian`). That Jacobian has no inverse at fi = ±90°; within 3° of it the
window's turn columns stay empty and the log says so.

The stall in the passage is a stretch question (the window travels too short), the flyaway a stretch and curve
question, a heading bend against the gyro a turn question. Shift stays in the table although the origin fix always
wins it: NDT's pull there says how far off the map thinks the window sits as a whole, which is the previous window's
error.

### Weight, hold and pull

Each motion is taken along three directions: **along travel** (the horizontal direction from the result's first to
its last node), **sideways** (horizontal, across it) and **vertical** (world z); for turn these are the rotation axes
(roll, pitch, yaw). A window that moves less than 1 cm horizontally keeps the previous window's directions.

For a term with normal matrix H and right-hand side b (as the term function writes them), and a motion along a
direction as the vector m in the space of the unknowns:

- **weight** w = mᵀ H m (1/m², or 1/deg² for turn): how strongly the term resists that motion;
- **hold** = 1/√w: the same as a sigma (the term alone keeps this motion to ± hold). The log prints the hold;
- **pull** p = mᵀ b / w (m or deg): how far along this motion the term's own minimum lies from the present state, with
  everything else held. Empty when the weight is zero.

The numbers of one window combine: the sum of w·p over the terms, divided by the sum of w, is the step all terms
together would take along that motion, without damping. At the result this is the **remaining step**: near zero when
the terms balance (then the heaviest term decided the result), and large when the iterations ended before the balance
was reached. The damping is why that can happen: the factor starts at 5, grows tenfold every 10 iterations and reaches
5e9 after iteration 90, far above most terms' weights, so the late iterations hardly move a window. (The loop ends
earlier only when the update falls below 1e-12 or the window takes more than 10 s.) The `damping` row shows the last
factor as a weight, so the two cases can be told apart.

Limits, to be kept in mind when reading it:

1. "Everything else held": a pull along one motion ignores that the solver also changes the other unknowns (attitude,
   bias, offset, the other motions) at the same time.
2. One linear step: NDT's pull is for the cells its points match now.
3. Twelve numbers don't describe every change of a window (a wiggle inside it, for example).

To see what the limits cost, the feature file also holds what the window actually did between guess and result,
expressed in the same twelve numbers.

### Rows of the table

| Row | From | In the total |
|---|---|---|
| `ndt_indoor`, `ndt_outdoor` | the point terms of each grid, as solved: Huber weight, information scale, degeneracy projection (for both grids the projection the solver uses, from the indoor grid's normals) | yes |
| `ndt_unprojected` | both grids before the projection: what the projection took away | no (information only) |
| `edge_position` | the x/y/z rows of the consecutive-node edges | yes |
| `edge_rotation` | their rotation rows (gyro); cost with the bias coupling | yes |
| `gyro_bias_prior` | | yes (cost only: it acts on the bias) |
| `velocity_continuity` | with the tie to the previous window's velocity; windows without the accelerometer term | yes |
| `accelerometer` | the acceleration terms and the start and end velocity terms | yes |
| `velocity_prior` | start velocity against the previous window's end velocity | yes |
| `zero_velocity` | windows at rest | yes |
| `end_velocity_tie` | the weak tie when the end velocity term was dropped | yes |
| `offset_prior` | | yes (cost only) |
| `origin_fix` | weight only, it has no right-hand side (its rotation sigmas are named `_deg` but act as radians; the table converts) | yes |
| `total` | the sum of the rows above; its pull is the remaining step | |
| `damping` | the last iteration's factor on every pose parameter (at the guess: the initial factor); weight only | no (information only) |

Per row: cost, number of residuals, and weight and pull for the 4 motions × 3 directions. Each row exists twice per
window: at the **guess** (the state the first iteration starts from: what each term wanted) and at the **result** (the
balance).

Cost / n is the term's cost per residual measured with its own weight. It is about 1 only where that weight is a noise
model (gyro rows, accelerometer, the carried priors). It is not for NDT (cell scatter, Huber cost, the 0.1 scale), the
scaled edge positions or the weak tie (sigma 1 m/s by choice). So it is read against the term's own usual value over a
run, which is what the window verdict of stage 4 will do.

### How it is computed without changing a result

`window_term_table` (new, `src/domain/window_term_table.{h,cpp}`) takes the same inputs as `optimize_lidar_odometry`,
all `const`, and returns the table. For each row it calls the row's term function on an empty `WindowEquations` and
reduces the matrix and right-hand side that term wrote onto the twelve motion vectors. The real iterations are not
touched: the matrix they solve receives the same additions in the same order as today.

The window loop calls it twice, outside `run_lm_iterations` (whose clock the time limit reads) and outside the span
the logged window time measures:

- at the guess, after `motion_model_poses` is set (the edges read it);
- at the result, directly after `run_lm_iterations` and before the steps that change the window again (the bias and
  offset carry, the tail re-propagation).

Both calls get the window's own options (`acc_term.motion_model`, `window_rotation_options`, `acc_term.options`, the
present acceleration offset), no solver workspace and no timing target. The result is a state the iterations never
evaluated, and the NDT sum throws on a non-finite distance (Huber on): the table call catches that, logs a warning and
leaves the window's rows empty, so the log can't stop a run the iterations would finish.

Cost: two extra assemblies per window against up to 100 iterations, expected a few percent of the odometry time with
the setting on; measured at the end of the stage.

### Splits needed first (pure refactors, each its own commit)

| Function | Lines | Split into | The table needs |
|---|---|---|---|
| `sum_ndt_point_terms` | 251 | today's local `Grid` struct and `point_term` lambda become `NdtGrid` and a function (one point against one grid); the points-by-node index; one grid's loop over the nodes; the conductor. The maps become `const` | one grid at a time, into node blocks whatever the projection setting |
| `add_projected_ndt_blocks` | 46 | the projection matrix as its own function | the solver's projection (from the indoor normals) for each grid's row |
| `add_motion_edge_terms` | 216 | `MotionEdgeWeights` built by its own function (position weights with the scale, rotation weights per edge); the edge's measurement; the edge block; the edge cost; the bias coupling. The term receives the weights (a first step of stage 5) | position rows and rotation rows apart: the same function with the other weights zero |
| `add_acceleration_terms` | 116 | the per-node samples; one term (today's lambda); the three callers | only the 80-line limit |
| `add_velocity_priors` | 43 | start prior, zero velocity, weak end tie: three functions called in today's order | three rows |

`WindowEquations` gets a residual counter beside `motion_cost` (an integer, so no bit of the solve changes). The
per-term subtotals stage 2 planned for the cost were not built and are not needed: a term on an empty
`WindowEquations` leaves its own cost there (the edges with `with_cost` set; NDT's cost is in `NdtSums`). The weak
end-velocity tie needs to know whether the end velocity term was dropped, so the table evaluates the accelerometer row
first.

### Output

- **Setting:** `[lidar_odometry_debug_info] log_window_terms` (off by default). A capture and a snapshot store only a
  hash of the odometry settings, and the hash leaves out every key listed under this section in `TomlIO::CATEGORIES`:
  the key must be in that list as well as in `POINTERS`, or every existing capture and snapshot is refused. The three
  golden TOMLs and the params TOML entry of the manifests change with it.
- **Log:** one line per window: the travel direction; cost / n of every term at guess and result; and for stretch
  along travel, curve along travel, curve vertical and turn about vertical the three strongest holds, the largest pull
  and the remaining step.
- **`window_terms.csv`:** one row per window, state (guess / result) and term, with all weights and pulls.
- **`window_features.csv`:** one row per window for the future learned tuner: what `log_window_diagnostics` prints
  today as columns (time, duration, iterations, delta, guess and result move, velocities, information ratio and weak
  axis, rest, turn window), the last damping factor, matched and unmatched points, gyro bias and acceleration offset,
  the travel direction, and the window's actual change from guess to result as the twelve motion numbers.
- **Where the files go:** a port `WindowTermSink` in the application layer (like `WindowCaptureSink`) and a CSV adapter
  in the infrastructure. A CLI run writes them to the output folder; `dMapper --replay` writes them next to the
  capture, named after the params file. The GUI gets the log line only.
- **`tools/window_terms.py`** (pure Python): prints the table of one window, or per term the summary of a range.

### Tests

- The rows' sum equals the matrix and right-hand side of the real assembly, reduced the same way, within a tolerance
  (the sums are formed in another order), so the table can't silently miss a term.
- Corridor: along the corridor NDT's hold is far weaker than across, and the edge positions hold the stretch.
- A guess moved 5 cm sideways in the room: NDT's shift pull sideways is −5 cm; the same for a stretched guess.
- A trajectory with a constant acceleration against a guess without it: the accelerometer's curve pull is the missing
  end displacement.
- Poses turned against the motion model by a heading ramp: the edge rotations' turn pull about vertical is minus the ramp.
- The synthetic recording through `optimize_windows` with the setting on and off: the poses are the same bits.
- The CSV adapter writes the expected columns and rows.

### Checks and commit order

Per commit as in stage 2: the characterization file (recorded again from `addd8e4` first; it is not in git), unit
tests, both replays, `44_short`. End of stage: the full 44 and Livox 3 against their manifests with the setting off;
`44_short` and the Livox 3 replay with the setting **on** (poses identical, only the params TOML differs); Debug build
and `ctest`, because the NDT split touches the parallel loop.

1. The four splits (NDT, edges, accelerometer, velocity priors) and the residual counter.
2. `window_term_table` with its tests.
3. The setting, the port, the calls in the window loop, the log line.
4. The CSV adapter, CLI and replay wiring, `tools/window_terms.py`.
5. Golden TOMLs; manifests (params TOML); each its own commit.
6. Docs (`CLAUDE.md`, `tools/README.md`, this plan's progress).

Not in this stage: any judgement on a window (stage 4), any change of a weight (measured changes, later).

### Decisions (user, 2026-11-17: all as recommended)

1. **Four motions instead of translation only** (without curve and turn the accelerometer and gyro rows would be
   empty).
2. **Both states, guess and result** (the guess shows what each term wanted, the result who won).
3. **NDT rows per grid, plus the unprojected row** (the passage question is partly "what did the projection remove").
4. **Split `add_acceleration_terms` and `add_velocity_priors` in this stage** (the files are open and the guard is at
   hand).
5. **`log_window_diagnostics`' weak-axis and heading lines** measure by finite differences part of what the table
   gives: kept through stage 8, then decided.

## Stage 4: solver and window verdict

Design of 2026-11-17, from the code after stage 3 (`b234b6a`), reviewed against the code. It replaces the first
sketch of this stage. The decisions for the user are at its end.

Three parts: the iteration loop becomes a unit that knows nothing about the terms; the window loop's body is cut into
named steps; and after each window a verdict records what is already known about it. The first two are pure refactors.
The verdict only observes and logs: no window is treated differently because of it.

### Where the loop is today

| Piece | Where | What it knows |
|---|---|---|
| The iteration loop (`run_lm_iterations`, 118 lines): damping schedule, convergence, time limit, the forced iteration count of a replay, `lm_step_check` (take a step back when the cost rose), the record for `log_iteration_convergence` | `src/application/run_odometry.cpp` | the window, the maps, the accelerometer term, the settings |
| One iteration (`optimize_lidar_odometry`, 83 lines): the terms, then the origin fix, the covariance of the extra unknowns, the damped solve, the step; or, for the cost-only call, everything up to the covariance | `src/domain/scan_to_map.cpp` | everything |
| The solve itself (`extra_unknowns_covariance`, `solve_window_step`, `apply_window_step`) | `src/domain/window_solver.cpp` | the layout of a window's unknowns |
| The window loop's body (`process_windows`, 177 lines) | `src/application/run_odometry.cpp` | everything |

The loop and the solve are tied to the window in every line, so the solver can't be tested without a map, and the
known flaw of the loop (every window runs all its iterations) has no home of its own.

### Part 1: the solver unit

`src/domain/lm_solver.{h,cpp}`: Levenberg-Marquardt over any problem that can do four things. It never sees a term, a
pose or a map.

| The solver asks the problem to | Today |
|---|---|
| **linearize**: give the undamped normal matrix, the right-hand side, how many leading unknowns are damped, and (when asked) the cost, at its present state | the term calls, the origin fix and the band-to-sparse conversion in `optimize_lidar_odometry` |
| **take the covariance** of its undamped unknowns (the solver computes it from the undamped matrix) | `extra_unknowns_covariance` and the lines that store the bias, offset and end-velocity covariances |
| **apply a step** and say whether it took it, and how large it was | `apply_window_step` (big-jump rule, bias clamp, delta) |
| **save and restore** its state | `LmState`, `capture` and `restore` in `run_lm_iterations` |

The window's side of this is `WindowProblem` (`src/domain/window_problem.{h,cpp}`): it holds the references the terms
read (points, window, maps, the five option structs, the velocity prior, the acceleration offset) and implements the
four operations with the functions that exist today.

The solver has two functions:

- `lm_iteration`: one iteration, in today's order: linearize, covariance (from the undamped matrix, before the damping
  is added to it), damp, solve, apply. With "no step" it ends after the covariance and leaves the update untouched:
  that is today's cost-only call, which still writes the covariances into the window. `optimize_lidar_odometry` stays
  as a thin function that builds a `WindowProblem` and calls it, so the characterization guard and the term tests keep
  running the same single call in both modes.
- `lm_solve`: today's loop of `run_lm_iterations`, statement for statement: the factor from `next_lm_factor`, the stop
  at `convergence_delta_threshold`, the last iteration (forced in a replay), and `lm_step_check` with its final
  cost-only call. It returns what the loop did (replacing the file-local `WindowOptimization`), and **why it stopped**
  (converged, iteration limit, time limit, forced), which no log shows today.

Details that keep it bit-identical and clean:

- **The clock stays out of the domain.** The time limit becomes a callback the application passes in (`out_of_time`),
  which starts its clock where the loop does today and is asked at the same point: once per iteration, after the factor
  update, not after a rejected step. A replay passes none. The poses after each accepted iteration (for
  `log_iteration_convergence`) and the debug line when the factor changes go through callbacks too.
- **The saved state.** With `lm_step_check` the loop today saves the state before the call and then copies the
  covariances and the translation information computed during the call into it. That is the same as saving between
  "covariance" and "apply" (the step itself writes no covariance), which is how the solver does it. Without the step
  check nothing is saved, as today.
- **What is not part of the state.** The two Cholesky analyses (kept from window to window) and the counters of skipped
  steps and big-jump nodes stay in the workspace and are never restored: a rejected iteration still solved and applied
  a step, and its events still count.
- **What the solver is given besides the problem:** the workspace, the options, the callbacks, and the three phase
  timers it owns (covariance, solve, update); the problem times its own phases.

`run_lm_iterations` shrinks to: build the problem and the options, call `lm_solve`, hand back the result.

Kept exactly as it is (each would be a measured change of its own):

- the damping schedule, and that a rejection multiplies the factor by 10 on its own;
- a rejected iteration counts as an iteration, and skips the convergence test, the scheduled factor update and the time
  check;
- a forced last iteration switches off only the time check, not the convergence test;
- the loop ends only when the update falls below 1e-12 on an accepted iteration, at the iteration limit, or at the
  time limit (10 s per window in the shipped settings, so in practice every window runs all 100 iterations). An
  exception from the NDT sum (a non-finite distance with the Huber kernel) leaves it too, as today.

### The loop guard

The replays and baselines run with `lm_step_check` off, and the characterization guard calls single iterations. So no
check covers the loop's own logic with the step check on. The first commit of the stage adds a second skipped doctest
case next to the guard: the synthetic loop recording through `optimize_windows` with the step check off and on, and
with a low iteration limit and a convergence threshold that ends some windows early; every window's poses, velocity,
bias, iteration count and number of rejected steps as hexadecimal doubles into a file. Recorded before the refactor,
compared after every commit. The fixture must make the step check actually reject steps (a small initial damping
factor, a guess moved away from the truth): the recorded file has to show rejected steps and early ends, or the
fixture is changed until it does. The time limit can't be covered this way (it depends on the computer); its code
moves as one `if`.

### Part 2: the window loop's body in named steps

`process_windows` becomes a short loop over four steps, each a function under 80 lines, in today's order:

| Step | Holds today's |
|---|---|
| `prepare_window` | points load (with its timer), the pause point, previous velocity, the guess (`predict_window_poses`), the gyro-bias prior, the window's rotation options and turn flag, the accelerometer term, rest detection, the velocity state, the accelerometer-integrated guess, `motion_model_poses`, the copies of the guess. Returns one struct with what the later steps read |
| `solve_window` | `run_lm_iterations` (with its timer) and the warnings about skipped steps and big jumps |
| `report_window` | the window's elapsed time, the `log_*` calls, the per-term table, and the verdict (part 3). It reads the window before the tail is re-propagated, like today's logs |
| `finish_window` | bias and offset carry, tail re-propagation, preview, the distance check, the outcome for a capture, progress, placing the next window, the map update, the travelled distance. Returns false when the distance is not a number: the run is cancelled there and the later parts are skipped, as today |

The timer of the whole window stays in the loop around the four calls. The order inside the steps is today's; three
places depend on it: the window's rotation options are copied after the bias prior is written, the guess is copied
after the accelerometer integration, and the distance check comes after the tail re-propagation.

Pure refactor; the replays and the loop guard check it.

### Part 3: the window verdict

**What it is.** After the iterations and before the map update, `judge_window` (`src/domain/window_verdict.{h,cpp}`,
a pure function) turns what is known about the window into readings and flags. At the end of the run the flags are
counted into the run's triggers, each `CLEAR`, `TRIGGER` or `NOT MEASURED`. It follows the agreed evaluation contract
(`openspec/changes/sensor-neutral-yardstick/criteria-proposal.md`, 2026-11-12): these are investigation triggers,
not a statement about the map's accuracy, and evidence that is missing is `NOT MEASURED`, never clear.

**Flags with an agreed limit** (the same quantities and numbers as `tools/run_health.py`, without parsing the log):

| Flag of a window | Reading | The run's trigger (contract) |
|---|---|---|
| bent, bent severely | the vertical component of the rotation from the guess to the result at the last node (today's "yaw correction last"), above 3° and above 10° | more than 1 % of the windows bent; any window bent severely |
| fast | the norm of the window's end velocity, above 2.5 m/s | any fast window |
| bias high | the estimated gyro bias per axis (on top of the start-up bias), above 0.5 °/s | any such window |

**Which windows have a reading.** A window without the accelerometer term (the first one; all of them with
`imu_always_on` off) has no velocity state: its speed and its velocity chi² are `NOT MEASURED`. The same holds for the
bias without bias estimation. This is stricter than `run_health.py`, which reads the zeros the log prints for such a
window as measured; the contract's rule decides here.

**Readings without an agreed limit** (recorded, never a trigger in this stage):

| Reading | From | Why it is there |
|---|---|---|
| why the iterations stopped, iterations run, last damping factor, steps not applied, big-jump nodes, rejected steps | the solver's result | the stop cause is `NOT MEASURED` today |
| the start velocity's distance from its prior (chi², 3 residuals; the same number as the per-term table's velocity-prior cost) for every window with the accelerometer term | the velocity state and the prior | today only logged for windows with an information ratio below 0.45; the contract's limits (median 5, 95th percentile 20) are for a run, not a window |
| information ratio and weak axis | the window's translation information | context: a flag in weak geometry reads differently |
| tilt bend (roll, pitch) against the gyro | as the heading bend, the two horizontal components | the first table reading showed tilt changes as large as heading changes |
| with `log_window_terms`: cost per residual of every term, the largest remaining step, the LiDAR's share of the weight along travel | the per-term table | the stage 3 evidence, per window |

The translation information is collected in every run from now on (today only with `log_window_diagnostics`, or as a
by-product of the degeneracy projection). It adds one small sum per matched point and enters no matrix; its time is
measured. Without it the ratio would read 0 and every window would look weak.

**Run summary.** At the end of the odometry the loop logs, for each flag, how many windows have it, their share, the
first and the worst with its time, and the coverage (windows measured of all), and from them the run's triggers of the
table above. For the readings it logs the median, the 95th percentile and the largest value. For the velocity chi² it
logs the contract's trigger (median above 5 or 95th percentile above 20) over the windows with an information ratio
below 0.45, as today's log selects them, and the same statistics over all measured windows beside it. Percentiles are
interpolated linearly, as `run_health.py` does.

A replay logs the summary of the windows it replayed. A cancelled run logs the summary of the windows it finished and
says that it is incomplete.

**Output.**
- The run summary is logged in every run (a few lines, like today's "silent events").
- `log_window_verdict = true` (`[lidar_odometry_debug_info]`, off by default) adds one log line per window that has a
  flag, with all its readings. The line starts with `window verdict <n>:`, so `run_health.py` doesn't take it for one
  of its `window diagnostics` lines.
- With `log_window_terms`, the flags and readings are also columns of `window_features.csv`: the labels the future
  tuner needs next to the features.

**Limits of the new readings are not chosen here.** After the stage's runs, a table shows each reading's distribution
on 44, Livox 3 and Livox 4, and the user decides which reading gets a limit and what it is. The contract's rule
applies: limits are not picked by looking at which candidate they favour.

**It changes nothing.** `judge_window` takes values and returns a value. Its inputs exist already (the guess's poses
are kept by the loop today), except the tilt bend, which is a few rotations more per window. The summary lives in
`WindowLoopCarry` and is not written to a capture, like the timing and convergence fields, so capture files keep
their format.

### Tests

- **Solver on a small problem** (two unknowns, no map): a single iteration with and without a step; the factor grows tenfold after iterations 10, 20, ..., and at
  once after a refused step; it stops at the convergence threshold and reports why; a forced last iteration ends it
  there; `out_of_time` ends it; with the step check a step that raises the cost is taken back and the damping raised.
- **`lm_iteration` equals the old single call**: the characterization guard, unchanged.
- **The loop**: the loop guard, and the existing scan-to-map tests through the real `lm_solve` instead of their own
  copy of the loop (`run_iterations` in `tests/scan_to_map_test.cpp`), if their numbers stay the same.
- **Verdict**: each flag at, below and above its limit; missing evidence gives `NOT MEASURED`; a window without the
  accelerometer term has no chi²; the summary's counts, shares, coverage and triggers on a made-up list of windows
  (1 % bent is clear, more is a trigger).
- **Against the existing tool**: on `44_short` run with `log_window_diagnostics` and `log_window_verdict`, the
  summary is compared with `tools/run_health.py` on the same log. The tool reads values the log prints with three
  decimals (the chi² with one), so the comparison is: the bend, speed and bias counts are equal, except for windows
  whose reading lies within the printed precision of a limit, which are listed; the chi² median and 95th percentile
  of the weak windows agree within the printed precision. The windows the two treat differently on purpose (no
  accelerometer term; an information ratio that prints as 0.450) are listed, not counted as a mismatch.
- **No effect**: the synthetic loop recording gives the same poses with `log_window_verdict` on and off.

### Checks and commit order

Per commit: the Release build, unit tests, the characterization guard, the loop guard, both replays, `44_short`. End
of stage: the full 44 and Livox 3 against their manifests, the Debug build and `ctest`.

1. The loop guard, recorded twice from `b234b6a`.
2. `LmProblem`, `WindowProblem`, `lm_iteration`; `optimize_lidar_odometry` calls them.
3. `lm_solve`; `run_lm_iterations` calls it; the solver's own tests.
4. `process_windows` in four steps.
5. `judge_window` with its tests.
6. The verdict in the loop: the summary, the setting, the log line, the feature columns; golden TOMLs and manifests in
   their own commits.
7. The readings table of 44, Livox 3 and Livox 4 (one full run each with `log_window_verdict` and
   `log_window_terms`), shown to the user; the cross-check with `run_health.py`.
8. Docs.

Livox 4 stays parked: its run here is a reading of the verdict on known bad stretches (passage 100–125 s, flyaway
200–280 s), not work on them.

Not in this stage: acting on the verdict (a suspect window kept out of the map, or solved again); changing when the
iterations end; any limit for the new readings.

### Decisions (user, 2026-11-17: all as recommended)

1. **The solver behind a small interface** (linearize, covariance, apply, save and restore), with the window as its
   one implementation. Recommended: it is what makes the solver testable without a map and gives the iteration rules
   one home. The alternative, moving `run_lm_iterations` into the domain as it is, would leave it tied to the window.
2. **The verdict uses the agreed triggers and limits; new readings get no limit in this stage.** Recommended: the
   limits come after their distributions on three recordings are on the table.
3. **The run summary in every run; the per-window lines behind `log_window_verdict`.** Recommended: a run then always
   says how many windows are flagged, at almost no cost (the translation information in every run; measured).
4. **The name "window verdict" stays** although the contract says a trigger is not a verdict on the map; the code and
   the log say flag, `CLEAR` / `TRIGGER` / `NOT MEASURED`, and the docs say what it is not. Recommended, because later
   stages act on it. The alternative is to call it "window triggers" everywhere.
5. **A window without a reading is `NOT MEASURED`**, also where `run_health.py` reads the log's zeros as measured (the
   first window's speed, for example). Recommended: it is the contract's rule; the tool can follow later.
6. **One full Livox 4 run** for the readings table. Recommended: it is the only recording with known bad stretches.
7. **The scan-to-map tests use the real solver** instead of their copy of the loop. Recommended if their numbers stay
   the same; otherwise the copy stays and the difference is reported.

## Stage 5: one place for the weights

Today the weights come from many places: sigmas in the options, the time-scaled rotation sigma, the accelerometer
noise formula, the two 0.1 scales, the NDT cell covariances. One function builds a window's complete noise model from
the settings and the window; the terms receive weights and never compute them. Pure refactor. With stage 3 this is the
"noise budget": every weight in one struct, and its effect in one log line.

Designed 2026-11-17 (Opus, high), from an inventory of the code (every weight of the window problem, with its formula,
inputs and lifetime).

### Where the weights are today

Each `WindowProblem::linearize` (once per iteration) computes them again inside the terms, from the options, the
window's timestamps and the carried state:

| Weight | Where | Lifetime |
|---|---|---|
| Motion edges: translation 1/σ² (× `imu_position_information_scale` with the accelerometer term), rotation 1/σ² constant or time-scaled max(k·ARW·√dt, floor), k for turn windows | `motion_edge_weights` (`motion_model_terms.cpp:95-135`) | constant per window, recomputed per iteration |
| Velocity continuity σ = acc·½(dt₁+dt₂), and the first-step velocity prior σ = acc·dt (path without the accelerometer term); its dt clamped at 1e-4 s, the edges' dt at 0 | `motion_model_terms.cpp:35-68` | constant per window, per node triple, recomputed |
| Bias coupling of the motion edges: the edge's rotation weight with its dt | `add_edge_bias_coupling` (`motion_model_terms.cpp:207-230`) | per edge, from the edge weights |
| Gyro-bias prior: covariance grown by the random walk; information = inverse | grown in `apply_gyro_bias_prior`, inverted in `motion_model_terms.cpp:17` | per window; inverse per iteration |
| Accelerometer term 1/((density²/τ + floor²)·τ²) per term span τ | `accelerometer_term.cpp:124, 205-206` | constant per window, per term, recomputed |
| Start-velocity prior: carried covariance grown over the gap, or the initial / rest sigma; information = inverse | covariance in `run_odometry.cpp:146-162`, inverse in `accelerometer_term.cpp:219` | per window; inverse per iteration |
| Zero velocity 1/σ²; the weak end-velocity tie, weight 1.0 written in the code | `accelerometer_term.cpp:237, 249-260` | per iteration |
| Acceleration-offset prior: carried covariance grown by the random walk; information = inverse; its mean is always zero (`optimizer_options.h:84`) | `run_odometry.cpp:89, 115`, `accelerometer_term.cpp:281` | per window; inverse per iteration |
| Origin fix 1/σ² | `fix_origin` (`window_solver.cpp:9-16`) | per iteration |
| NDT: cell information (inverse cell covariance, in the map), planarity, range weight and its cap of 10 (fine grid only), Huber weight, `ndt_information_scale`, degeneracy projection | `ndt_term.cpp:106-333`, `ndt_map.h:20-38` | per point and per iteration; the Huber weight and the projection depend on the current poses |

The degenerate-window sigma raise this stage was planned around no longer exists; what is left of it is a reset of the
motion model's x/y/z sigmas to their configured values in `prepare_acc_term` (`run_odometry.cpp:102-105`), which changes
nothing. `CLAUDE.md` still describes the raise.

### The noise model

`WindowNoiseModel` (`src/domain/window_noise_model.{h,cpp}`) holds every weight of one window, in two kinds:

- **Values, fixed for the window**, computed once by `build_window_noise_model` in `prepare_window`, before the first
  iteration, with the same expressions in the same operand order as today, and under the same conditions: the model
  holds the window's layout (`layout_window_unknowns`: which unknowns and terms exist, from the timestamps and the
  options), built once, and computes a weight only where today's term would (the velocity continuity only without the
  accelerometer term, no bias weights in the first window, both dt clamps as they are):
  - the motion edges' weights, one per edge (translation and rotation, the scale and the turn-window k applied);
  - the velocity-continuity weights per node triple and the first-step prior's weight;
  - the gyro-bias prior (mean, covariance, information);
  - the accelerometer terms' weights, one per term, with their τ (from the timestamps and the anchors only);
  - the start-velocity prior and the acceleration-offset prior (mean, covariance, information);
  - the zero-velocity weight and the weak end-velocity tie (its 1.0 becomes a named constant);
  - the origin fix's six weights.
- **Rules applied per point and per iteration**, kept in the NDT term because they depend on the current poses or the
  map: planarity, the range weight and its cap per grid, the Huber cutoff, `ndt_information_scale`, the projection's
  ratios. The model holds their parameters (per grid); the term applies them in today's order (the generated product
  with the cell information, then the range weight, then the Huber weight, then the scale: separate multiplications,
  because folding them into one factor changes the bits). The cell information stays in the map: it changes with every
  map update.

The terms take the model (`const WindowNoiseModel&`) instead of options they compute weights from. `WindowProblem`,
the per-term table (`WindowTermInputs` gains the model; its guess and result share it, since the fixed weights depend on
the timestamps only) and `ndt_window_cost` (the weak-axis diagnostic) all read the same model. The current state (the
bias update, the poses) stays in the terms. Captures need no new format: a replay runs `process_windows`, so
`prepare_window` rebuilds the model from the carried state, also with other settings.

**Why it stays bit-identical.** Every fixed value depends only on settings, timestamps and carried state, which don't
change during the iterations (the posteriors written by `take_covariance` go to the next window, never into this
window's priors; rest detection and the turn decision happen before the first iteration). Computing the same expression
once instead of every iteration, or inverting the same matrix once, gives the same bits. The weight enters each term
where it enters today (`w * jaᵀ * r`, `prior_inf * delta`, the generated motion-edge code), never folded into a
Jacobian.

**It is a little faster**, not slower: the inverses and per-edge weights are computed once per window instead of 100
times. Not a goal; reported.

### The log line

`log_window_noise = true` (`[lidar_odometry_debug_info]`, outside the settings hash) adds one line per window,
`window noise <n>:` with the model as sigmas (σ = 1/√w): the motion edges' translation σ and rotation σ (smallest,
median, largest over the edges, per axis), the velocity-continuity σ, the accelerometer σ (range over its terms), the
start-velocity prior's σ (square roots of the covariance's eigenvalues), the bias and offset priors' σ, the zero-velocity
σ when at rest, and the NDT parameters (scale, Huber cutoff). With `log_window_terms` the same values are columns of
`window_features.csv` (the learned-tuner data). Every run logs one `noise budget` line at its start: the settings-derived
constants (σ before the time scaling, density, floor, random walks, scales). Results unchanged.

### Findings the refactor keeps as they are

1. **The origin fix's rotation weights use the sigma in degrees as if it were radians** (`window_solver.cpp:12-14`: no
   `DEG_TO_RAD`). With 1e-6 the weight is 1e12 instead of 3.3e15; both hold the first node practically fixed. Kept
   bit-identical, with a comment; fixing it changes numbers, so it goes to the code-review tracker.
2. **`ndt_window_cost` disagrees with the NDT term**: it applies `use_norm` and `min(w, 10)` to both grids
   (`scan_to_map.cpp:70-72`), the term `grid.range_weight` and `grid.cap`, set for the fine grid only
   (`ndt_term.cpp:14-26, 164-168`). It only feeds `log_weak_axis_diagnostics`. With the model it reads the term's per-grid
   rules, so the diagnostic agrees with the term; that diagnostic's log numbers change, the results don't.
3. **The no-op sigma reset** in `prepare_acc_term` is removed (no dead code), and `CLAUDE.md` loses the sigma raise.

### Tests

- `build_window_noise_model` on a synthetic window with known timestamps: each weight equals its formula, and equals
  what the terms compute today (checked before the terms switch, in commit 2) (time-scaled
  rotation σ with its floor, the turn window's k, the accelerometer σ for a given τ, the priors' growth by the random
  walk and the gap); a window at rest gets the zero-velocity weight; the first window has no accelerometer or velocity
  prior.
- The existing term tests (`tests/window_terms_test.cpp`) take their weights from a model built for their windows.
- The characterization guard, the loop guard and the replays carry the bit-identity.

### Checks and commit order

Every commit: build, tests, characterization guard, loop guard, the two replays (44_short 20–40 s, Livox 3 100–130 s),
44_short against its manifest. End of stage: full 44 and Livox 3 against their manifests (the params TOML gains
`log_window_noise`, new manifests in their own commit), Debug build and `ctest` (the model's vectors change lifetimes).

1. Remove the no-op sigma reset; `CLAUDE.md` without the sigma raise.
2. `WindowNoiseModel`, `build_window_noise_model` and its tests (not used yet).
3. The motion edges and the velocity continuity take the model.
4. The accelerometer term, the zero velocity and the three priors take the model.
5. The origin fix and the NDT term's parameters from the model; `ndt_window_cost` reads the same rules.
6. The per-term table reads the model.
7. `log_window_noise`, the `noise budget` line, the feature columns; golden TOMLs in their own commit.
8. Docs (`CLAUDE.md`, the guide's debug settings and the Fundamentals' sigma entry, `tools/README.md`), manifests.

### Decisions (user, 2026-11-17: all as recommended)

1. **Scope: the window problem only.** Loop closure and consistency keep their own weights; consistency's time-scaled
   rotation σ (`consistency.cpp:82-91`) calls the model's formula function, which returns the σ (max(k·ARW·√dt,
   floor)), and multiplies by its own scale as today: sharing the σ is bit-identical, sharing a weight would not be.
   Recommended.
2. **The per-point NDT rules stay in the term, with their parameters in the model** (not precomputed weights), because
   the Huber weight and the projection depend on the current poses and the cell information on the map. Recommended;
   the alternative, a per-iteration model, would mix the window's fixed noise with the solver's state.
3. **The log: per-window line behind `log_window_noise`, the same values as feature columns with `log_window_terms`, and
   one `noise budget` line in every run.** Recommended. Alternative: no new setting, the per-window values only in the
   CSV.
4. **The origin-fix quirk is kept and recorded** (finding 1); a later numeric change may fix it with a measurement.
   Recommended: it holds the first node either way.
5. **`ndt_window_cost` follows the term's per-grid rules** (finding 2): the weak-axis diagnostic's numbers change, no
   result does. Recommended: a diagnostic that disagrees with the term misleads.
6. **The velocity chi² question from stage 4 is not answered here**: the stage puts the velocity prior's covariance in
   one place and in the log line, so the question (prior too tight, or process noise missing) can be measured after it,
   as its own change. Recommended: a pure refactor changes no number.
7. **The model is built once per window in `prepare_window`, with the layout**, and `WindowProblem` takes it
   (const); `linearize` no longer rebuilds the layout. Recommended. Alternative: the problem builds the model on its
   first linearize (keeps today's call order, but the model then lives inside the solver).

**At the end of the plan (user, 2026-11-17):** the dMapper guide (`docs/index.html`, pipeline step 6 "Window
optimization" and the Fundamentals) is updated with how the optimizer works after stages 2-5: the term functions, the
solver unit and its stop reasons, the window verdict, the noise model and its log lines. Then the Hungarian pages.

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

### Stage 6 design (2026-11-18, Opus, high)

**Measured today** (every function above 80 lines in `src/`, with a rough brace-matching reader):

| Lines | Function | Region | Tier | When |
|---|---|---|---|---|
| 84 | `predict_window_poses` (`window_steps.cpp`) | inertial / window loop | 1 | done in stage 6 (30 lines, four steps) |
| 84 | `update_rgd_legacy` (`ndt_map.cpp`; the default cell statistics) | map | 2 | before the map region's first job |
| 336 | `run_loop_closure` | back-end | 2 | before the back-end region's first job |
| 311 | `Consistency` | back-end | 2 | same |
| 134 | `optimize_loop_pose_graph` | back-end | 2 | same |
| 180 | `compute_step_1` | front-end | 3 | before the front-end region's first job |
| 143 | `calculate_trajectory` | front-end | 3 | same |
| 110 | `load_imu` | front-end | 3 | same |
| 100 | `pair_recording_files` | front-end | 3 | same |
| 89 | `load_point_cloud` | front-end | 3 | same |
| 185, 146, 119, 117, 89, 85 | `exportLaz`, `save_result`, `TomlIO::LoadParametersFromTomlFile`, `FolderResultSink::processing_results`, `TomlIO::SaveParametersToTomlFile`, `TomlIO::check_keys` | I/O | 5 | when next changed |
| 119, 116, 106, 96 | `display`, `drawMiniCompassWithRuler`, `view_kbd_shortcuts`, `camMenu` | GUI | 5 | when next changed |
| 93 | `affine_matrix_from_pose_rodrigues` (`transformations.h`, HDMapping's, unchanged) | domain types | 5 | not split: one closed-form expression |

Checked by a reviewer against the code; the term files, `run_odometry.cpp` and the solver have no function above 80
lines (`process_windows` 15, `optimize_windows` 58, `optimize_lidar_odometry` 24). Tier 1 is done except `predict_window_poses`: the window loop, the terms, the solver, the verdict and the noise model are all
under 80 lines. Tier 4 went in stage 0.

**`predict_window_poses` in four steps** (pure refactor). It already has them:

1. `mean_shift_of(previous, before_previous)`: the mean shift per node between the two previous windows' last nodes, zero
   (and reported) above 1 m.
2. Chaining onto the previous window's end: today a copy of `place_window_after`'s loop (start at the previous end times
   `imu_motion_from_previous`, then each node's increment). It calls `place_window_after` instead: the same expressions in
   the same order, so the same bits (`inverse() * x` of two `Affine3d` is an `Affine3d` in both, and `place_window_after`
   copies the poses before it writes), and one copy of the chaining less. The translations are then overwritten in
   `window.poses` and `motion_model_poses` set from it, as today from the separate copy. An empty window (never produced:
   a window has `threshold_nr_poses` nodes) keeps today's behaviour, one node at the previous end, by a guard.
3. Rest detection: `window_rests(poses, timestamps, imu_acc, imu_angles, gravity_norm, acc_threshold, gyro_threshold)`
   (`window_steps`), the zero-velocity test on a 50 ms average with its 0.025 s half width as one named constant. The same
   call is written out three times today: in `predict_window_poses`, in `window_at_rest` (`run_odometry.cpp`, the
   accelerometer term's rest) and in `log_accelerometer_fit` (a diagnostic), each with its own copy of the constant. All
   three call the one function. It holds only the acceleration on the 50 ms average and the stationarity test: the size
   checks stay at the callers (they differ: `use_zupt` and the IMU sizes in the prediction, the window's sizes in
   `window_at_rest`) and so does the gyro threshold's conversion to rad/s, as today. The poses passed differ (the chained
   guess, the window later), which changes nothing: the test reads only the rotations. The prediction's other
   acceleration (the integration's own smoothing width) stays where it is.
4. `guess_window_translations(...)`: the three ways to the positions (accelerometer integration from the previous velocity
   or rest, constant velocity, mean shift), with the gap to the previous window.

Each step is under 40 lines and has a test: the mean shift and its drop at 1 m, the chaining equal to
`place_window_after`, rest on a still and a moving window, the three translation branches. The existing tests of
`predict_window_poses` (`tests/window_steps_test.cpp`, `tests/window_cut_test.cpp`) stay as they are.

**What the measurement leaves for later.** The table goes into each region's knowledge file in stage 7 (front-end, map,
back-end): a region splits its functions in its first job, as this plan says, before it changes them.

**A tool for the measurement:** `tools/function_sizes.py [src] [--min 80]` prints every function above the limit (it
exists since stage 0; a second tool added here by mistake, `function_lengths.py`, was removed on 2026-11-18 and its
reader, which also counts functions inside namespaces and in headers, moved into `function_sizes.py`). The verification region runs it; the region contracts of stage 7 can name it.

**Checks and commit order.** Every commit: Release build, unit tests, characterization guard, loop guard (it runs the
prediction through the window loop), the two replays, 44_short against its manifest, the per-term table of the 44_short
replay. End of stage: full 44 and Livox 3 against their manifests, IDENTICAL (no setting changes, so the manifests stay).
No Debug build: a pure split with no lifetime, threading or configuration-dependent change (CLAUDE.md, "Debug builds only
when they can matter").

1. `window_rests` and its named constant; the three callers use it; test.
2. `predict_window_poses` in its four steps, chaining through `place_window_after`; tests of the steps.
3. `tools/function_sizes.py`, `tools/README.md`; the plan's table and progress; `CLAUDE.md` where it names the window steps.

### Decisions (user, 2026-11-18: all as recommended)

1. **Split `predict_window_poses` now**, although it is only 4 lines over: it is the last tier-1 function above the limit,
   and its split removes a copy of the chaining and gives the rest test one home. Recommended. Alternative: leave it at
   84 lines and record it.
2. **One rest-detection function for the three callers** (prediction, the accelerometer term's rest, the fit
   diagnostic). Recommended: the same test with the same constant written three times can drift apart.
3. **Tiers 2 and 3 stay deferred** to their region's first job, and the measured table goes into the region knowledge
   files in stage 7. Recommended; `update_rgd_legacy` (map) joins tier 2, which the old table did not list.
4. **`tools/function_sizes.py`** in the repo for the verification region. Recommended. Alternative: keep the measurement
   as a one-off.
5. **No Debug build at the end of the stage.** Recommended (a pure split; the rule in CLAUDE.md).

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

## See also

- [Architecture](ARCHITECTURE.md)
- [The per-term table: Livox 3](window-terms-livox3.html)
- [Code-review tracker](code-review-tracker.md)
- [All documents](README.md)
