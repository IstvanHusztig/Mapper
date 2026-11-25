# Proposal

## Why

Livox 4 has two faults inside the per-window optimizer: the passage stall (a 10 m walk through a narrow passage at
100–110 s comes out as 6.5 m) and the flyaway (207–285 s: up to 8.5 m/s, 73 m down). They were parked on 2026-11-16
until a stretch could be rerun in seconds, each term could be read on its own and each part of the code had an owner
(stages 1–7 of `docs/code-split-and-region-agents-plan.md`). This is stage 8: use those tools on the two stretches.

## What Changes

- Two window captures of Livox 4 (passage 95–125 s, flyaway 195–285 s) from full runs with the shipped settings.
- A reading of both stretches (per-term table, window verdict, noise lines): which term moves the window away from the
  truth, and which term should have held it.
- Region jobs for the terms the readings point to (the first: lidar-matching, the NDT term's pull along the passage),
  each in its worktree, with replays through the gate, verified and reviewed.
- Replays of the captures with changed settings where a setting is the question (the gyro's weight).
- A report of the numbers for the user's decision. A fix that changes results is not part of this change: it becomes
  its own OpenSpec change after the user's decision, with full runs and the before and after clouds.
- A diagnostic that changes no result (new rows of the per-term table) may be merged in this change on the
  orchestrator's judgment; its requirement is then added here as a delta spec.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None for the investigation (`skip_specs`). If a diagnostic row of the per-term table is merged, the delta is added
then.

## Impact

- No shipped setting and no result changes in this change.
- Data: `C:/Users/User/Downloads/dLidar/experiments/stage8/` (captures, replays), region folders under
  `experiments/regions/`.
- Code: at most diagnostics that change no result (`window_term_table`, `ndt_term`), merged under the region rules.
- Knowledge files of the regions that take part, the code-review tracker where a finding is a defect.
