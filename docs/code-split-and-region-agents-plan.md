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

## See also

- [Architecture](ARCHITECTURE.md)
- [The per-term table: Livox 3](window-terms-livox3.html)
- [Code-review tracker](code-review-tracker.md)
- [All documents](README.md)
