# Design

## Context

`run_loop_closure` (`src/application/loop_closure.cpp`) works in five steps: segments from the windows; revisit candidates
per segment (`revisit_tries` on the same floor, nearest-first across floors); for each try a registration (coarse voxel
search, then NDT refinement, with the overlap before and after and the refinement's normal matrix); the edges chosen from
the accepted tries (`revisit_slots`) and weighted (`loop_edge_information`); the pose graph (`optimize_loop_pose_graph`)
with residual pruning in a loop; the correction interpolated onto every node. Only the registrations read points, and
they are where the time goes; everything after them is arithmetic on a few hundred poses. Codex's diagnostic of
2026-11-22 (`experiments/consistency/doubles/codex/edge_diagnostic`, a patched worktree) re-solved the graph with single
edges removed and reproduced the native poses within nanometres, which shows the split is clean; it still registered
everything again and is not a tool of the repository.

Existing precedents: the window capture and replay (port in `src/application/window_capture.h`, file in
`src/infrastructure/window_capture_file.{h,cpp}`, the gate `tools/run_replay.py`, bit-for-bit with its own settings), the
odometry snapshot (`--from-snapshot`, refused when odometry settings differ), `log_stage_timing` (off by default,
`docs/PERFORMANCE.md`), the per-window time limit `real_time_threshold_seconds` and the replay's time-limit count.
Trajectories of all stages share the node times, so a return can be carried from one trajectory to another through its
node (the method of Codex's `geometry_data.place`, checked there to 0.09 mm against the native export).

## Goals / Non-Goals

**Goals:**
- A loop closure question answered in seconds on a saved odometry, and in minutes on a family of them.
- One wall measure that follows the same returns through every trajectory, without clouds.
- A Livox dataset short enough for routine use.
- A run that states its stage durations and whether load could have touched it.

**Non-Goals:**
- No change to any rule of the loop closure, to the odometry or to a shipped setting. No checkpointed odometry. No
  change of the rule about parallel runs (a measurement and a decision for the user come out of this change).

## Decisions

1. **What the capture holds:** the windows of the odometry (as the snapshot does), the segments, and one record per try
   in the order tried. Not the points. A replay therefore cannot register; it selects among the recorded tries with the
   replay's settings and fails loudly when a try is missing. Cross-floor candidates are tried "next one when the last was
   rejected", so a looser acceptance can stop earlier (fine) and a stricter one can ask for a try that was never made
   (reported as missing). An option to record more tries than the run used (all qualifying candidates up to a cap) makes
   captures useful for stricter rules; its extra registrations cost time only when the capture is switched on.
   *Alternative considered:* keeping the decimated segment points in the capture so the replay can register on demand:
   larger files and a second code path that must stay identical; deferred until a rule needs new registrations.
2. **One code path.** `run_loop_closure` is split at the registration: the first part produces the try records (and
   hands them to a capture port when one is set), the second part consumes records. The run and the replay call the same
   second part, so bit-for-bit equality with the capture's own settings is a property of the structure, and it is tested
   on the synthetic loop recording of `tests/loop_closure_test.cpp` (capture, replay, compare; plus a stricter setting
   and a missing try).
3. **Which settings a replay accepts:** the acceptance, slot, weighting, graph and pruning settings of `[loop_closure]`.
   Settings that change the segments or a registration are refused by name (a list in the code next to the settings
   hash's `added_later` list). Consistency does not run in a replay: it needs the points; the stage after loop closure
   is what the replay gives, and a kept rule is confirmed by a reprocessing from the snapshot.
4. **Replay output:** the trajectory files a result has (so every tool that reads a result folder's trajectory reads a
   replay), the loop log, and a small JSON with the edges used and pruned. No cloud, no session files.
5. **The wall-pair tool** (`tools/wall_pairs.py`, pure Python, numpy): `freeze` builds the set from a result and a
   `wall_precision` piece table (plan walls or all pieces, stated), `measure` takes any number of trajectories. The frame
   of each pair is the earlier visit's node pose, so a rigid change of the whole trajectory gives zero. A set chosen on
   one result favours that result's walls: the tool prints the set's origin with every table, and a comparison of rules
   uses a set frozen before the rules are run (as the sweep's frozen reference). Tested on synthetic walls in
   `tests/test_verification_tools.py` (a rigid move changes nothing; a shifted second visit shows its shift; a trajectory
   with other node times is refused).
6. **Short Livox dataset:** a cut of Livox 4 from the start (about 110 s, so that the segments of the loop edges seen on
   2026-11-22 are inside), made with `cut_recording`; before it becomes a baseline it is checked to contain accepted
   loop closures and the wall-pair behaviour of the full run's first part. It goes into `tools/end_runs.py`, not into the
   step checks (their 7 minutes stay). The manifest is recorded after the user's decision on the Livox baselines, which
   are open since the lever arm changed.
7. **Timing and time limit:** one log line per stage with its duration in every run (the detailed `log_stage_timing`
   output stays as it is), and one line with the number of windows stopped at the time limit (the count the replay
   already reports). `run_health.py` reads both. Then one measurement: the short Livox dataset alone and two copies side
   by side, compared bit for bit, with the counts; the user decides what follows for the rule in `CLAUDE.md`.
8. **Order:** 1 and 2 (the capture and replay, back-end region) and 5 (the tool, verification region or the orchestrator)
   in parallel; then 6; 7 is small and can go with either. `livox4-double-walls-cause` uses each as it lands: its tasks
   2.2, 2.3 and 3 are what the replay and the tool are for.

## Risks / Trade-offs

- Splitting `run_loop_closure` touches code whose results are golden: the 44 and 44_short baselines must stay
  bit-identical with the capture off, and the full 44 runs once at the end of the change.
- A capture without points limits replays to rules that select among recorded tries; rules that change the registration
  need reprocessing as today. The option to record extra tries widens this at a time cost paid once per odometry.
- The wall-pair numbers are agreement on a selected set, not accuracy; a rule tuned on one set can fit that set. The
  user's eye on a confirmed full result remains the check.
- A short dataset can lack the faults of the long route (the lower storeys, the stairwell): it is for speed and for
  refactors, not a replacement for the full runs.
- One-time costs: capturing the 16 existing odometries is one reprocessing each (about 15 minutes), one after another.
