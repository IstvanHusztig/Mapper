# Proposal

## Why

Inside a blind stretch the loop closure's correction along the passage can run forward and back again. On the saved
Livox 4 stall (windows 482-506) it rises to +4.25 m at window 486, falls to +2.41 m at window 492 and rises again to
+5.80 m: for six windows the corrected path goes backward at about 1 m/s against the odometry. The fine-time placement
(shipped 2026-11-25) moves the windows whose scans see something across the passage, and lets the 14 windows without
such surfaces follow as a smooth offset to that correction, so the fall is still there (+3.25 to +1.01 m,
`fine_time/results.md` sections 3 and 6).

The cause is in what the fine-time rule assumes, not in the scans: it treats the pose graph's placement inside the
stretch as right up to a smooth offset. Inside a blind stretch that placement comes from loop edges that are attached at
their segment's first pose, 1.3 s before the scans that carry their information, and from edges whose information along
the passage comes from scans that see nothing across it (edge 71 -> 81: informed share 0.00, 2.0 m from the trajectory
where its information lies). Nothing there supports a correction that reverses.

The target is cave passages: long stretches without a surface across them, registered against an earlier visit. That is
where such edges are the rule and not the exception. The user decided on 2026-11-25 to treat this as its own change, and
a recording of a long underground corridor is planned to test it.

## What Changes

- **Straight gaps.** Inside the stretch of an eligible run, the correction along the run's axis becomes one line in
  time through the places the scans support: the two ends of the stretch (where the pose graph's correction is kept) and
  the middle of every window whose scans saw surfaces across the passage (where the fine-time offset is kept as it is
  today). Between two such places the correction is straight; the pose graph's own placement along the axis is no longer
  used there. Windows with surfaces keep exactly the offsets of today's rule. Positions across the axis, orientations and
  everything outside the stretch stay as the pose graph left them, as today.
- **A reading for every weak run: how far the correction goes back.** For each weak run of the readings rule the loop
  closure logs the largest fall of the correction along the run's axis against the run's own direction (0 for a
  correction that only grows), before and after the fine-time step. It needs no points, so a loop closure replay gives it
  for any settings, and it shows on a new recording whether blind stretches that the fine-time step does not treat (not
  eligible, or nothing to see across the passage) have the fault too.
- **The rule is prototyped and frozen first** (`tools/loop_fine_time.py`, on the saved stall capture, on today's
  Livox 4 capture and on the Livox 3 capture), against criteria stated in the design before any result is read. One
  alternative form is read beside it (the profile fitted to the total correction instead of to the offset), and two
  readings that change nothing: the pose graph solved without the along-axis information of the unconfirmed edges, and
  the straight line through a stretch with no scan evidence at all against what the scans say where they speak.
- **In dMapper behind a setting**, `loop_closure_fine_time_straight_gaps` (off by default, needs the fine-time
  placement). With it off every result is bit-identical. The score curves and the capture do not change (capture format
  4 stays): a replay gives the step bit for bit with either value.

Not in this change: any change to the pose graph itself (edges attached at the times of their information, or the
along-axis information of blind scans taken out of an edge: read here, built only if the readings ask for it); treating
a run in which no window has surfaces across the passage (it stays `not timed`; the reading above measures it); the
fine-time rule's own constants; dropping or reweighting an edge.

## Capabilities

### New Capabilities

### Modified Capabilities
- `loop-closure`: the fine-time placement can take the correction along the axis straight through the windows without
  surfaces; every weak run reports how far its correction goes back.

## Impact

- `tools/loop_fine_time.py` and `tests/test_loop_fine_time.py` (the rule's second form, the reading, a scene with a
  correction that reverses); a new frozen rule text beside `fine_time/RULE.md`
  (`experiments/consistency/livox4_lever/zigzag/RULE.md`).
- Later, after the user's go: `src/application/loop_fine_time.{h,cpp}` (the nodes' move, the reading),
  `src/application/loop_closure.{h,cpp}` (the report), `odometry_config.h` / `toml_io.h` (one `[loop_closure]`
  setting, outside the odometry settings hash), the replay's JSON and `loop_fine_time.csv`, `tests/loop_fine_time_test.cpp`,
  the golden TOML files; `tools/loop_graph.py` unchanged.
- Docs: the guide (loop closure step 5, the setting), `docs/regions/back-end.md`, `CLAUDE.md`, `tools/README.md`,
  `checks/instruments.toml` (the `loop_fine_time` section's planted fault).
- Runs: none for the prototype (it reads the existing captures and the recordings' point caches; the Livox 4 cache copy is
  in `fine_time/cache`). For the port: `--from-snapshot` runs of Livox 4 and Livox 3, and the new corridor recording
  with and without the setting. The Livox 3 baseline does not move while the setting is off.
- The check register: the new tests are additions; no existing assertion changes.
