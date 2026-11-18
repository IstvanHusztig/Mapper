# Design

Stage 8 of `docs/code-split-and-region-agents-plan.md` ("capture 95–125 s and 195–285 s, replay with the per-term log,
name the term, hand it to its region"), run under the protocol of `docs/regions/README.md` ("From a question to a merged
change") and the evaluation contract (`openspec/changes/sensor-neutral-yardstick/criteria-proposal.md`): numbers, not
verdicts; the user judges the map by eye.

## Decisions

1. **Two captures, two full runs.** A run writes one capture range. Run 1 (passage, 95–125 s) has every per-window
   log on and its post-processing; run 2 (flyaway, 195–285 s) only captures (loop closure and consistency off: outside
   the odometry's settings hash). Both hold the gate as a full run.
2. **Read before any experiment.** The per-term table of a full Livox 4 run exists (stage 4,
   `experiments/stage4/livox4-readings`); the refactors since were bit-identical, so its numbers are today's. The
   reading names, per stretch: the term that moves the window, its hold and pull, and the term that gave way.
3. **A setting is tried by replay, a code question goes to its region.** A replay with other settings starts from the
   run's own map before the range and reports how far the windows move; it screens an idea and proves nothing about the
   map. A region agent works in its worktree and hands back a branch and numbers.
4. **What truth there is.** The passage: about 10 m in 100–110 s (the user's route on the plan); the stretch after it
   (110–124 s): about 12 m. The flyaway: a person walks, so speed above about 2.5 m/s and a steady fall are wrong. No
   other reference: every other reading is a trigger for a look, not a verdict.
5. **Stop at the numbers.** After the readings and the first experiments the user decides how to go on. A fix is its
   own change.
6. **Hard parts are flagged.** A question that the readings and replays of this change do not settle is listed in
   tasks.md under "Open, for a later look at higher effort" with what was tried (the user, 2026-11-18).

## Risks

- A replay with changed settings sees the run's map up to the range's start. In the flyaway capture that map already
  holds the shortened passage; a replay that behaves there says nothing about a run with the passage fixed.
- One recording. A setting that helps Livox 4 must be cross-checked on Livox 3 (the map the user judged good) and on 44
  before it is proposed.
