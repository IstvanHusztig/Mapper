# Proposal

## Why

When the odometry loses travel in a stretch where the LiDAR does not hold the travel direction, the loop closure restores
the stretch's total, but not when inside the stretch the travel happened: the correction is attached to the first pose of
each 2.16 s segment and interpolated between those poses. On the Livox 4 recording before `imu_edges_follow_state` this drew
the stair enclosure wall 1.4-1.9 m too far along the passage (about 28,000 returns, the "ghost wall"). The odometry fix
(`imu_edges_follow_state`, shipped 07c0ae4) removed the loss there, and with it the ghost. It is the first line of defence.
Nothing stands behind it: on another recording or in a cave passage, a stall the odometry does not avoid would draw a
ghost again. On 2026-11-24 the user decided to keep Codex's fine-time loop closure ("road B",
`experiments/consistency/ghost_wall_review/codex/`) as that second guard. This change builds it.

## What Changes

- **Fine-time placement, a new last step of the loop closure.** For each weak run of the readings rule whose sigma is at
  least 1 m, the scans of every window in the run (and one segment on either side) are compared with the surfaces other
  visits saw, along the run's weak axis only, and a smooth profile of offsets is fitted, zero at both ends. The nodes inside
  move along the axis by that profile. Rotation, the position across the axis, and everything outside the stretch stay as
  the pose graph left them. A window without such surfaces follows its neighbours along the smooth profile (in Codex's
  prototype ten such windows move by up to 0.5 m); a run in which no window has any stays as it is. Frozen prototype: Codex's
  (`ghost_wall_review/codex/frozen-prototype.md`, `prototype.py`).
- **A rule for when loop edges and scans disagree.** In Codex's prototype the two corridor-mouth edges into segment 84 end
  up 1.0 m off. Most of that is the place where the edge is read: at the segment's first pose, 1.25-1.34 s before the scans
  that carry its information along the passage. The rule reads an edge at the times of its own information (its
  "time support"). An edge whose informative scans are contradicted there by more than the limit stops the guard for that
  run, and the run stays as the pose graph left it; an edge whose source scans have no surfaces across the passage is
  reported as unconfirmed and does not stop it.
- **Registrations keep the time of their information.** Each registration stores, per source window, how much of its
  translation information that window gave. Without the guard this is one more diagnostic: an edge that the graph satisfies
  only at its first pose is visible.
- **A run the scans cannot time is reported.** Run, time, axis and the remaining uncertainty are logged ("not timed"), for
  the passage walked once or without features across it.
- **Loop closure capture format 3:** the per-window information of each try, and the guard's score curves with the graph
  result and the set of eligible runs they belong to. A replay with the capture's settings gives the step bit for bit; a
  replay whose graph or eligible runs differ from the stored ones refuses the guard and names the cause. Variants of the
  rule are screened with the Python tool, which reads the points.
- **Prototype tool** `tools/loop_fine_time.py`: Codex's prototype rebuilt with operations that port to C++ one to one, and
  the edges' time-support residuals.
- Off by default (`loop_closure_fine_time_placement = false`); with it off every result is bit-identical. It needs
  `loop_closure_odometry_from_readings` and is refused without it. `settings/` changes only after the user's look at the maps.

Not in this change: time-support attachment of the edges inside the pose graph itself, per-window nodes in the pose graph,
dropping edges by their time-support residual, any change to the odometry, the hybrid form of a run's error (task 4.4 of
`loop-closure-weak-runs-and-match-check`).

## Capabilities

### New Capabilities

### Modified Capabilities
- `loop-closure`: an optional last step places the nodes of a blind stretch in time from the scans of other visits; loop
  edges are also read at the times of their information; the capture carries what a replay of that step needs.

## Impact

- `src/application/loop_closure.{h,cpp}` (the step, the per-window information in `ndt_refine` / `segment_points`, the
  report), a new port for the scene scores next to `LoopClosureRegistrations`, `loop_closure_capture.h` and
  `loop_closure_replay.{h,cpp}`, `src/infrastructure/loop_closure_capture_file.*` (format 3; formats 1 and 2 still load),
  `OdometryConfig` / `TomlIO` (two `[loop_closure]` settings, outside the odometry settings hash) and the golden TOML test
  files, `src/presentation/cli/loop_closure_replay_cli.cpp` (the replay's JSON and a per-window CSV).
- Tools: new `tools/loop_fine_time.py` with tests; `tools/loop_graph.py` (`read_capture` for format 3).
- Tests: `tests/loop_closure_test.cpp`, a truth case on the synthetic route R2 (out and back through a blind corridor).
- Docs: the guide (settings, loop closure), `docs/regions/back-end.md`, `CLAUDE.md`, `tools/README.md`.
- Runs: one full Livox 4 run and one full Livox 3 run with a capture (about 35 min each, in the background, one at a time);
  everything else by `--from-snapshot` and by replay. Recording 44's baselines are untouched (the guard is off, and the L2
  file does not use the readings rule).
- Time in a run: the reference surfaces around each eligible run are read from the point store again (a few seconds per
  run, expected; measured in task 3.6). The loop closure stage on Livox 4 takes about 630 s today.
