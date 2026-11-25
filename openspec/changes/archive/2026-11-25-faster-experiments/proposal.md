# Proposal

## Why

Finding the cause of the Livox 4 double walls (`livox4-double-walls-cause`) now turns on loop closure, the odometry's map
horizon and the consistency pass, and each question there is answered by runs. Measured on the Livox 4 run of
2026-11-22 (35 minutes): loading 1.5 min, odometry 21.6 min, loop closure and consistency together about 11 min, saving
about 1 min. Around that:

- a loop-closure question needs a reprocessing from an odometry snapshot (13 to 18 minutes), and it has to be asked on a
  family of odometries, because one run cannot judge a rule (16 snapshots exist): 4 to 5 hours per rule;
- every result is judged from an exported cloud (300 MB, written and read again), with wall tools that find their own
  walls in each cloud, so two results are not measured on the same walls;
- Livox has no short dataset: the smallest complete Livox run is the full recording;
- runs go one after another, because a loaded machine can change a result through the per-window time limit, although
  a run does not say whether that limit was ever reached (a window takes about 1.3 s against a limit of 10 s here);
- the split between loop closure and consistency is not known, because stage timing is off.

The window capture and replay made the odometry's questions cheap (a stretch in 1.5 minutes instead of a run). The same
idea is missing after the odometry.

## What Changes

- **Loop closure capture and replay.** A run can write what its loop closure worked from: the segments and every
  registration it tried, with the result of each (transform, overlap before and after, the registration's information).
  A replay runs only the part after the registrations (choice of edges, pose graph, pruning, the correction of the
  nodes) with other loop closure settings, in seconds, and writes the trajectory. With the capture's own settings it
  must reproduce the run's loop closure bit for bit.
- **Wall pairs measured from poses.** A tool freezes a set of wall returns once (their positions relative to their own
  trajectory node) and then measures the gap between two visits of each wall for any trajectory of that recording:
  an odometry snapshot, a loop closure replay, a final result. Seconds per trajectory, the same walls for every result,
  no cloud.
- **A short Livox dataset.** The first part of Livox 4 as a cut recording (with revisits and loop closures inside), run
  in about a quarter of the time, with a baseline manifest and a place in the end-of-change runs.
- **A run says what its timing was and whether it could have depended on load:** stage durations, and how many windows
  stopped at the time limit. `tools/run_health.py` reports both. Whether runs may then go side by side is measured and put
  to the user; the rule is not changed here.
- No result of a normal run changes: the capture is off by default and outside the settings hash; the golden baselines
  stay bit-identical.

## Capabilities

### New Capabilities

### Modified Capabilities
- `loop-closure`: a run can capture its loop closure, and a replay re-solves it from the capture.
- `regression-baseline`: wall pairs measured from poses on a frozen set of returns; a short Livox reference dataset; a
  run's check reports stage durations and the time-limit count.

## Impact

- Code: `src/application/loop_closure.{h,cpp}` (back-end region: the registrations recorded through a port, the part after
  them callable on its own), a capture file in `src/infrastructure/`, the CLI argument in `src/presentation/cli`, the
  time-limit count and stage durations in the run's log; tests in `tests/` on the synthetic loop recording.
- Tools: a new pure-Python tool for the wall pairs, `tools/run_health.py`, `tools/run_replay.py` (the gate also for loop
  closure replays), `tools/end_runs.py`, `tools/README.md`; manifests under `tools/baseline/`.
- Docs: `docs/regions/back-end.md`, `docs/regions/verification.md`, the guide's chapter on the instruments, `CLAUDE.md`
  (Run, Testing).
- Not in this change: resuming the odometry from a checkpoint (later, when the work is back at the odometry: it is only
  valid for questions about what follows the checkpoint); any change to loop closure's rules (that is
  `livox4-double-walls-cause` and the fixes after it).
