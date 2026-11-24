# Region: back-end

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The corrections made after the odometry has finished every window: loop closure (cut the trajectory into segments, find
revisits, register them, solve a pose graph over the segments, drop false edges, spread the correction onto every node)
and the consistency pass (a whole-trajectory least-squares pass over all nodes with NDT point terms and a smoothness
term). It also owns the odometry snapshot, which lets both run again from a saved odometry in minutes instead of a full
run.

## Contract

**Takes** (from the orchestrator's `OdometryPipeline::post_process`, `src/application/odometry_pipeline.cpp:112`, which
runs loop closure only with `use_loop_closure` and consistency only with `use_consistency`; that gating is the
orchestrator's):
- `std::vector<Window> &windows` as the odometry left them: `poses` (world `Eigen::Affine3d`, one per node),
  `timestamps` (`first` = seconds), `imu_angles`, `gyro_bias.mean` (rad/s); the window count and node count do not change.
- `PointStore` with `PointSet::WindowIntermediate` per window: points in the node's frame, `index_pose` = node index in
  its window (loop closure places them with `windows[w].poses[p.index_pose]`, `loop_closure.cpp:878`); consistency
  recomputes each point's node from its timestamp against the whole trajectory (`consistency.cpp:169-177`).
- `const OdometryConfig &` (`[loop_closure]`, `[consistency]`, `holdout_block_s`, `in_out_params_indoor`,
  `min/max_distance_lidar`, `use_exact_ndt_bucket_statistics`), and a `Logger`.

**Gives:**
- `run_loop_closure` (`loop_closure.cpp:837`) returns the number of kept loop edges and, if any, rewrites every node's pose
  in place (`tr[n] = C * tr[n]`) and sets `motion_model_poses = poses` per window (`:1147-1168`). Returns 0 and leaves the
  windows untouched with fewer than 2 segments, no accepted loop, all edges dropped, or an implausible result
  (`loop_pose_graph_plausible`, `:697`).
- `Consistency` (`consistency.cpp:124`) writes the solved poses back into `windows[].poses` (`:431-432`); a failed
  Cholesky leaves them as they were (`:409`, `:414`). Its driver `run_consistency` (passes, timing lines) sits in the
  front-end's file `src/application/lidar_odometry.cpp:425`: it sets `motion_model_poses = poses` before each of the
  `num_constistency_iter` passes and shares one `ConsistencyData` between them.
- Snapshot: `OdometrySnapshot` (`src/domain/odometry_snapshot.h`: windows, `m_g`, `total_length_of_calculated_trajectory`,
  `consecutive_distance`); `write_odometry_snapshot` / `read_odometry_snapshot` (`src/infrastructure/odometry_snapshot_file.cpp`),
  called from `src/presentation/cli/cli.cpp:68,83`. The restore itself (`OdometryPipeline::restore_odometry`, window count
  and every window's timestamps must match) is the orchestrator's.

**What the others rely on:**
- Order: loop closure, then consistency; each stage is announced (`stage_started`) before it writes a pose, so the GUI
  stops reading windows (orchestrator side).
- Frames and units: world-frame poses; solvers work on Tait-Bryan parameters (`pose_tait_bryan_from_affine_matrix`);
  settings in metres and degrees, radians inside. Loop closure fixes the first segment's pose (diagonal 1e12,
  `:556`); consistency holds node 0 (diagonal 1e6, `consistency.cpp:269-270`), nothing else.
- Determinism: the same input gives the same bits. Consistency merges its parallel chunks in chunk order (`:383`) and
  adds smoothness terms in node order (`:256`); ties in `loop_edge_to_drop` keep the earlier edge (`:680`). Tests check
  repeatability bit for bit.
- Hold-out: both stages drop held-out points with `remove_held_out(..., holdout_block_s)` (`loop_closure.cpp:872`,
  `consistency.cpp:163`).
- Snapshot format `DMSNAP01`, version 1, machine byte order, FNV-1a settings hash over every `TomlIO::POINTERS` key except
  the categories `loop_closure`, `consistency`, `lidar_odometry_debug_info`, `paths`, `misc`, `version_info`
  (`odometry_snapshot_file.cpp:62`); keys in `added_later` (`:74-75`) are hashed only when not at their default. A new
  setting therefore counts as an odometry setting unless its category is on that list. `tools/snapshot_trajectory.py`
  (verification) reads this format: a format change breaks it. Window serialization is `binary_io::write_window`
  (orchestrator's `binary_io.h`, shared with the window capture): the snapshot holds exactly the `Window` members it
  writes. A new `Window` member (inertial, solver) is missing from a snapshot until the orchestrator extends
  `write_window`, and a `--from-snapshot` run would then work on stale data. `holdout_block_s` (category `evaluation`)
  is in `added_later`: it is hashed only when it is not at its default.

## Owns

- `src/application/loop_closure.h` / `.cpp`: `run_loop_closure` and its public steps (`path_at_windows`,
  `same_floor_revisits`, `revisit_tries`, `revisit_slots`, `loop_edge_information`, `loop_edge_region_sigmas`,
  `optimize_loop_pose_graph`, `loop_edge_residuals`, `loop_pose_graph_cost`, `loop_edge_to_drop`, `loop_edge_drop_ratio`,
  `loop_pose_graph_plausible`); file-local: voxel helpers, `crop_to_shared_footprint`, `information_ratio`,
  `coarse_search` (0.5 m then 0.2 m voxel overlap search), `ndt_refine` (0.5 / 0.25 m NDT, 15 iterations each).
- `src/application/consistency.h` / `.cpp`: `ConsistencyData` (points with nodes, chunks of 1e6 points, kept Cholesky
  analysis), `Consistency`, file-local `add_gyro_rotation_terms` (`consistency_gyro_rotation_term`, off in both
  shipped settings files).
- `src/domain/odometry_snapshot.h`, `src/infrastructure/odometry_snapshot_file.h` / `.cpp`.
- Tests: `tests/loop_closure_test.cpp`, `tests/consistency_test.cpp`, `tests/odometry_snapshot_test.cpp`.
- In the front-end's file `src/application/lidar_odometry.cpp`: `run_consistency` (the passes' driver), by a function
  entry in `owners.toml`.
- Not owned, though close: the `[loop_closure]` / `[consistency]` members of `OdometryConfig` and the settings files
  (orchestrator), `restore_odometry` and `--from-snapshot` (orchestrator).

## How to check your work

- **Unit tests** (Release `lidar_odometry_tests`, from `out/build/x64-Release/bin`):
  - `-tc="loop closure*"`: path walked, revisit rule, tries and slots, leave-one-out drop order, pose graph (drift removed,
    implausible result refused, false edge stands out, kernel cases), information edges (residual reproduces the
    registration error, cross-floor edge carries no height, shaft vs box region), plausibility independent of the
    distance from the origin. Three end-to-end cases on the synthetic two-lap recording are `doctest::skip()` (about
    20 s each): add `--no-skip -s`. "a trajectory that is already right stays" is `should_fail` (moves 32 mm / 0.09°).
  - `-tc="consistency*"`: gyro term keeps node-to-node rotation (and is repeatable), several chunks repeatable and reduce
    seams, offsets between windows reduced, and the `should_fail` truth case (moves the true trajectory up to 6.3 mm).
  - `-tc="odometry snapshot*"`: round trip bit for bit, refused for other odometry settings, damaged file, restore checks,
    post-processing after a restore equal to after the odometry.
- **Loop closure replay** (the way to test a loop rule): a CLI run (also `--from-snapshot`) with `capture_loop_closure = true` writes `loop_closure_capture.bin`; `dMapper --replay-loop-closure <capture> <params.toml> <output_folder>` runs `close_loops` on the recorded registrations (edge choice, pose graph, pruning, node correction) and writes the trajectory and its loop log, in seconds. With the capture's own settings every node must equal the run's bit for bit (exit 1 otherwise). The capture holds no points, so it only chooses among recorded tries: settings that ask for another try are named as missing and write nothing; odometry settings, `loop_closure_segment_windows`, `loop_closure_max_correction_m` and `loop_closure_cross_floor_search_m` are refused. Measure the replayed trajectory with `wall_pairs.py measure` and `trajectory_seams.py`; a rule that looks better is confirmed by a full run. `capture_loop_closure_max_tries` bounds the tries kept. Files: `loop_closure_capture.h`, `loop_closure_replay.{h,cpp}`, `loop_closure_capture_file.{h,cpp}`, `loop_closure_replay_cli.cpp`.
- **Replays do not reach this region**: `dMapper --replay` runs only the window loop. `python tools/step_checks.py` still
  covers it through the unit tests and 44_short against its manifest, but 44_short (30 s) hardly exercises loop
  closure; only the full 44 does (`tools/README.md`).
- **From a snapshot**: `dMapper <input> <params.toml> <output> --from-snapshot <odometry_snapshot.bin>` reruns loop
  closure, consistency and saving (44: 178 s against 1338 s for the full run, measured 2026-10-10). It loads the
  recording and uses its cache folder, so for the gate it is a full run: the orchestrator starts it, one at a time,
  and hands you the result folder and log. A gated command for it comes with this region's first job.
- **Its own log lines**: `loop candidate segments i -> j (...): overlap a -> b, correction .. m .. deg ... ACCEPTED|rejected`
  (with path and ratio, or the information ratio for `cross-floor`), `loop edge i -> j kept`, `loop segments i -> j dropped:
  residual ...`, `loop closure: loop edge residuals [m/deg]`, `loop closure: N loop(s) accepted, largest segment
  correction X m`, the implausible-result warning, `N of M cross-floor registrations accepted`, the cross-floor edge
  sigmas; `consistency gyro term: N edges; G of W windows left out`; with `log_stage_timing` the `timing consistency`
  lines and the `loop_closure` / `consistency` step times.
- **Not its readings**: the per-term table, the `window noise` line and the window verdict are odometry readings.
  `run_health.py`'s row "consistency" is the start-velocity chi², not this pass; its row "loop closure" (loops, largest
  correction) is this region's, descriptive only (no 1 m fail rule, agreed contract 2026-11-12).
- **Tools**: `trajectory_seams.py` (world positions; sees what loop closure moves), `map_quality.py --floors` (revisit
  heading/offset, jitter, railing, stairwell on 44 s0/s1/s2), `stairwell_check.py`, `spot_check.py`,
  `evaluate_floors.py`, `wall_precision.py` (`pair_diff_m`: two visits of one wall), `holdout_sharpness.py` (whether
  consistency only fits its own points), `snapshot_trajectory.py`, `wall_pairs.py` (two visits of a wall measured from poses, no cloud: the replay's yardstick), `loop_graph.py` (the pose graph in Python for the kept edges: prototype a weighting rule and read the leave-one-out table before any C++; `check` against a replay; no registration, no pruning). Skills: `verify-run`, then `verify-map`.

## Known flaws

- **C1** (verified): consistency adds pose jitter. One 6-DOF unknown per node, smoothness weights constant and not scaled
  by dt (`consistency.cpp:215`: x 1e6, y and z 1e8, rotation 1/(0.1°)²), no prior to the input poses, only node 0 held
  (`:269`), 10 passes (`num_constistency_iter`). Measured on 44: rotation per IMU sample p50 0.041° (odometry) → 0.167°
  (consistency), gyro 0.024°. The gyro rotation term (`add_gyro_rotation_terms`) is the candidate fix, off.
- **C24** (partly fixed): robust kernel off (`loop_closure_kernel_scale` 0, `odometry_config.h:169`, not set in the
  settings files); up to two edges per segment since G2b; coarse-search ties still go to the first candidate in loop
  order (strict `>` in `coarse_search`, `loop_closure.cpp:165`, `:189`). G4 (yaw consistency vote, then kernel 3.55)
  designed, not implemented, deferred to the cave retune.
- **C25** (partly fixed): each loop edge is pinned at its segment's first node (`Segment::pose`, `:855`) and the
  correction is interpolated by time between the segments' first nodes (`:1151-1163`); the last segment takes its own
  correction unchanged. Segments of 6 windows (2.4 s) since 2026-10-15.
- **C35** (reported): each consistency chunk of `ConsistencyData::chunk_points` = 1e6 points gets its own NDT map
  (`compute_chunk`, `consistency.cpp:286-370`), so visits in different chunks never share a term; a chunk of ≤ 10,000
  points (the trailing one) gets no NDT term (`:290`).
- Accepted as they are: C9 (shift and plausibility now measured as anchor and source-centre movement, `:697-715`,
  `:984-988`; the cross-floor 1.0 m limit is still a constant, `:994`), C22 (path rule instead of the 30 s gap), C23
  (same-floor edges weighted by registration information, fixed sigmas only as fallback; odometry edges by length
  rejected). C36 is moot (`Consistency2` removed 2026-11-16).
- Also hard-coded (not in the tracker): the same-floor acceptance limits `shift ≤ loop_closure_max_correction_m + 1.0`
  and `angle ≤ 15°` (`:998-999`).

## To split in the first job

From `python tools/function_sizes.py src --min 80` (2026-11-18):
- `run_loop_closure` (`src/application/loop_closure.cpp:837`): 336 lines.
- `Consistency` (`src/application/consistency.cpp:124`): 311 lines.
- `optimize_loop_pose_graph` (`src/application/loop_closure.cpp:476`): 134 lines.

## Experiments

- **2026-11-24, match check (held-out innovation), built, off by default** (`loop_closure_match_check`, needs the
  readings rule; refuses a quantile outside (0, 1) and the robust kernel). On the format 2 Livox 4 capture with both
  switches on: the same 10 edges pruned as the Python prototype, then 24 → 63 dropped (T 35.99, limit 22.46, 2 held out,
  innovation 2.08 m), 81 kept; largest T left 13.71 / 7.77 / 5.88 / 5.23; trajectory equal to the prototype's within 1 µm;
  loop closure step 1.12 s without the check, 6.71 s with it. On uniform steps the check rejects 12 edges the rule graph
  accepts (the four corridor-mouth edges among them). Frozen rule
  `experiments/consistency/livox4_lever/match_check/RULE.md`; change `loop-closure-weak-runs-and-match-check`.
- **2026-11-24, odometry steps from the windows' readings (blind-stretch rule), built, off by default**
  (`loop_closure_odometry_from_readings`). `Window::readings`, snapshot and capture format 2. On Livox 4 the C++ rule
  gives the prototype's 25 runs (stall, windows 482–506: T 8.95 s, S1 2.00, S2 4.63, sigma 5.04 m, shares
  0.16/0.24/0.24/0.24/0.12); with the rule on the pruning keeps 82 (the 79 of uniform steps plus 51 ⇒ 79, 71 → 81,
  75 → 82), and the solve equals `tools/loop_graph.py` with the same steps within 0.002 mm. Frozen rule
  `experiments/consistency/livox4_lever/odo_cov/RULE.md`; agent folder
  `experiments/regions/back-end/2026-11-24-odometry-step-covariance/`. One recording; not validated.
- **2026-11-16, Livox 3:** with the AHRS gain switch made effective, the odometry was unchanged but loop closure
  corrected up to 1.48 m (0.33 m without), route 355 vs 313 m; the switch was removed (front-end). Tracker C3.
- **2026-11-13, full Livox 2, post-processing from a pinned snapshot:** loop closure moved the trajectory up to 10.47 m
  (5.36 m in z), 54 edges kept after 19 drops; the dense route grew 333.2 m → 408.6 (consistency pass 1) → 674.2 (pass 5)
  → 892.2 m (pass 10) while the stride-50 route stayed 331–338 m; largest position change over the 10 passes 0.331 m,
  median 0.030 m. `openspec/changes/ndt-information-probe/full-livox2-postprocess-attribution.md`.
- **2026-10-18, consistency gyro term on 44 s0/s1/s2:** gated windows 15/14/24 of 690 (gyro fault windows 629–639
  gated); jitter p50 0.91 → 0.49°, p99 4.44 → 1.40°; held-out wall thickness +1.31 → −1.40 mm against consistency off;
  façade and revisits within the spread; railing spread 3.0 → 4.1 cm; consistency time +10–20 %.
  `openspec/changes/consistency-gyro-term/results.md`.
- **2026-10-18, post-processing ablation from one 44 snapshot:** visit-pair difference p50/p90 odometry 4.5/12.9 cm,
  loop closure only 2.9/6.4, consistency only 4.7/13.2, both 3.0/6.1; consistency helps only pairs < 10 s apart
  (2.2 → 1.3 cm). Hold-out on 44_short: consistency +1.85 mm on never-fitted points, −2.5 mm on fitted ones.
  `openspec/changes/map-precision-leaks/findings.md` F1, F2.
- **2026-10-15, G4 diagnosis:** of 411 kept same-floor edges (s0–s5) the weighted residual is p50 0.02, p95 0.07 (limit
  0.15); the start ↔ end edges into segments 112–114 (the fast spin at the end) disagree by 5–9° in yaw and stay under the
  limit; an offline yaw vote would remove one edge per input. `openspec/changes/archive/2026-10-15-loop-closure-fixes/design.md`.
- **2026-10-15, ablation at 2.4 s segments (s0–s5 from snapshots):** fixed sigmas (A) turned the start room (5 of 6
  outside ±0.3°); C* = path rule + leave-one-out drop + two slots + information edges: façade spread 0.88°, gallery
  out − back ≤ 0.11°, railing median 4.4 cm, stair median 9.5 cm (C10 8.5); loop closure 517–572 s per run (259 s at
  4.8 s); 2–4 gross false registrations per run, all dropped. Shipped (user). Same design.md.
- **2026-10-14, path rule (G1, C22):** gallery out/back ≤ 0.24°, single walls 6 of 6. Same design.md.
- **2026-10-11, information-weighted cross-floor edges:** stairwell floor −2/−1 on 44 s0/s1/s2 5.8/6.7/24.3 → 1.2/1.3/7.2 cm;
  per-edge sigmas: height free, horizontal 5–8 cm, turn and tilt 0.4–2.7°.
  `openspec/changes/floor-alignment-diagnosis/findings.md` 1.5.
- **2026-10-11, which cross-floor edges move the gallery:** 6-DoF edges at a round shaft impose its yaw (residuals 1–3°)
  and swing the gallery; keep-height fixed a 0.4–0.86 m vertical slip but not the gallery. Same findings.md 1.2–1.4.
- **2026-10-10, odometry snapshot:** reprocessing bit-identical with full runs; 44_short 18 s vs 179 s, 44 178 s vs
  1338 s. `openspec/changes/floor-alignment-diagnosis/tasks.md` 2.1–2.3.
- **2026-10-09, loop-closure gating:** retrying candidates and dropping the worst edge removed the false edge 6 → 49
  (3.05/3.21 m); loops 23/21/21, largest correction 1.72/1.86/1.92 m. `openspec/changes/archive/2026-10-09-loop-closure-gating/`.
- **2026-10-08, consistency speed-up:** 10 passes on 44 363.6 → 44.7 s, within 1e-10. `docs/PERFORMANCE.md` fix 11.
- **2026-10-04, synthetic tests:** loop closure end error 575 → 112 mm (20.6 s); consistency seams 27.9 → 19.5 mm.
  `openspec/changes/archive/2026-10-04-loop-closure-consistency-tests/`.

## Ideas rejected

- `loop_closure_min_time_gap_s` (30 s) for same-floor pairs: replaced by the path rule and removed (2026-10-15; admits
  standing still, rejects fast out-and-back). Design G1.
- G3a, odometry edges weighted by path walked and turned: no gain at 2.4 s, code removed (2026-10-15). Design ablation.
- Segments of 12 windows (4.8 s): a turn inside one rigid segment twisted the s2 gallery by 2.4° (tracker C25).
- Cross-floor point edges (X6), keep-height (X5) and `loop_closure_cross_floor_rotation_sigma_deg`: removed when the
  information-weighted edges shipped (2026-10-11, memory note `cross-floor-information-edges.md`).
- I1b, 2° rotation noise added to each cross-floor edge: s2's lowest floor fell back to 19.2 cm; removed. Findings 1.5.
- I2, same-floor edges weighted by information in the setup of 2026-10-11: rejected then (turned floor 0 about 3°,
  gallery −52 cm, memory note); same-floor information edges shipped later as G3b, with 2.4 s segments and two slots.
- `Consistency2` (multiple-Gaussian variant): removed 2026-11-16 (tracker C36).

## Open questions

- Ship the consistency gyro term? Task 4.4 of `consistency-gyro-term` waits for the user; two reviewer findings first:
  no gyro edge across window seams (`consistency.cpp:80`), and the gate is re-evaluated in every pass and can flip.
- The robust kernel on real data (`robust-loop-closure` task 2.4) was never run; G4a (yaw vote) waits for the cave data.
- `crop_to_shared_footprint` has no unit test (`robust-loop-closure` task 3.5).
- Why C* is 0.2–3.3 cm worse than C10 on the stairwell in 6 of 6 inputs (tracker C22).
- Cross-floor edges correct roll/pitch by up to 6.9° with no gate; a tilt gate or 4-DoF edges deferred (design G4).
- The dense-route growth through the consistency passes on Livox 2 (x smoothness weight 100× below y and z): a
  single-pass weight diagnostic was deferred.
- Loop closure cannot see where the odometry drifted (yaw disagreement, per-window bias): not in `Window` or the snapshot.
  (Since 2026-11-24 the information ratio, weak axis, rest test, placed move and duration are: `Window::readings`.)
- Blind-stretch rule: a run's time in the last segment has no odometry step and is not represented (logged "after the
  last step"); should the last segment get an end node? A window without any LiDAR information is loosened along one
  arbitrary axis only, and a window weak in two axes along one (code review 2026-11-24); the rule is frozen until the
  next recording has tested it.
- Match check: its time on a long recording is not measured (two graph solves per edge and round; the covariance form
  of design D3 if it matters); each test solve starts from the odometry poses, the prototype's from the full solution
  (equal to 0.01 in T on Livox 4); the test is conservative while the steps keep 0.1 m (median T 0.3 where the model
  expects 5.3).
- (Settled 2026-11-18: `run_consistency` is this region's, by a function entry in `owners.toml`, although it lies in
  the front-end's file; `--from-snapshot` runs are the orchestrator's until this region's first job.)

## See also

- [Regions](README.md)
- [Loop-closure research](../loop-closure-research.html)
- [Performance](../PERFORMANCE.md) (fix 11: faster consistency)
- [Code-review tracker](../code-review-tracker.md)
- [Code-split plan](../code-split-and-region-agents-plan.md) (stage 6 table)
- [All documents](../README.md)
