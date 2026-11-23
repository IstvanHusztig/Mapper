# Proposal

## Why

The Livox 4 map has double walls: the same wall seen on two passes lands in two places. The user judges this the main
fault of the map, and finding its cause comes before every other fix (2026-11-22). The recording has no truth: everything
below is agreement between runs, stages and visits, not accuracy.

Two analyses were made on 2026-11-22: phase 1 of this change (`experiments/consistency/doubles-phase1.md`, reviewed) and an
independent one by Codex (`experiments/consistency/doubles/codex/analysis-codex.md`, with its scripts and clouds; its
numbers are not yet reproduced here, except the recording metadata). Together they change the picture of the diagnosis of
2026-11-19, which put the cause in the odometry's blind corridor stretches:

- **Loop closure both removes and makes doubles.** The same wall returns were followed through the stages (odometry, after
  loop closure, after consistency). In the reference run the odometry leaves gaps of up to 4.3 m between passes, and loop
  closure removes them (start storey: 307 of 708 visit pairs at or above 8 cm in the odometry, 65 in the final cloud). But
  walls seen twice in the first 105 s agree in the odometry (median 7 mm) and are pulled apart at loop closure: in the run
  with the first lever-arm vector 9 of 155 early pairs at or above 8 cm become 39, the largest from 0.4 mm to 48 cm.
- **One loop edge does most of that there.** Removing the edge between the segments at 45 s and 104 s with the other edges
  held brings all 94 measured gaps at or above 8 cm below it (six others rise); the edge aligns surfaces that all run in
  one direction, and the reference run had pruned the same edge. With the pruning repeated, other edges enter and 190
  gaps appear: the graph is fragile to which edges survive.
- **The odometry cannot see the early walls on the way back.** Its map is rebuilt once the path passes 200 m (at about
  250 s), which removes what was seen before about 119 s.
- **The consistency pass bends windows.** The path through all nodes grows from 292 m to 375 m, and it cannot tie visits
  that are far apart in time (627 of 708 pairs share none of its maps).
- **The lever arm set in 692fd91 was in the wrong axes.** The recorder turns the sensor's axes for the hand-held mount and
  writes the IMU position in the output axes into each recording (`timing*.json`); the setting is the negative of that,
  (+0.04412, -0.011, -0.02329) m. Corrected in a46bf85; the run with the first vector (`livox4_lever/physical`) stays as a
  record. No result with the corrected vector exists yet.
- **Shown to be small:** point times (exact against the packet schedule), jumps at window boundaries (under 1 mm of the
  first 5 cm between runs), a remaining gyro scale error. The corridor stretches and the carried state remain a
  contributor to the odometry's gaps, not the first cause of the doubles on the start storey.

## What Changes

- A new base: Livox 4 with the corrected lever arm and its odometry snapshot; the stage split and the wall measures
  repeated on it, with Codex's stage measurements reproduced on at least the largest doubles.
- Loop closure examined first: which edges make doubles, what each matched, why it was accepted and not pruned; candidate
  rules tested by reprocessing from snapshots (minutes each) on a family of existing odometries (the new base, the
  reference and the fourteen sweep members), so no rule is judged on one run.
- Then the 200 m map horizon of the odometry, then the consistency pass, each with its own measurement.
- The odometry's own gaps (corridor stretches, carried state, hand-off) after those.
- Numbers without a verdict; the user judges the maps. A fix that changes results is its own OpenSpec change after the
  user's decision.

## Capabilities

### New Capabilities

### Modified Capabilities

(none: an investigation, `skip_specs` like `livox4-passage-and-flyaway`. A diagnostic that changes no result may be added
on the orchestrator's judgment, with its delta spec then.)

## Impact

- Code read, not changed here: `src/application/loop_closure.cpp` (candidates, registration, information edges, residual
  pruning), `consistency.cpp`, the map rebuild in `run_odometry.cpp` (`sliding_window_trajectory_length_threshold`).
  Experiments in the back-end and map regions run on their own branches.
- Machine: one full run for the new base (started 2026-11-22 23:03); reprocessing from snapshots; one full run per map
  horizon value. One job at a time.
- Follow-up outside this change: dMapper should take the lever arm from the recording's metadata, so a settings file
  cannot carry the wrong axes again.
- Order of other work: the F2a and F2b changes (proposals written), the fixed-first-pose design, the rest test and
  F9 / C16 wait, unless this change shows one of them to be a cause.
