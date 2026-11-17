# Design

## Context

The full design is the subsection "Stage 6 design" of `docs/code-split-and-region-agents-plan.md` (2026-11-18): the
measured table of every function above 80 lines in `src/` with its region and stage, the four steps of
`predict_window_poses`, the shared rest test, the tool, the checks and the commit order. A Sonnet reviewer checked it
against the code at `6672aac`; its findings (the empty-window guard, the size checks and the threshold conversion staying
at the callers, the bit-identity of the chaining) are worked in. This file keeps the decisions and the risks.

The user took all five decisions of the plan as recommended (2026-11-18).

## Goals / Non-Goals

**Goals:**
- No function above about 80 lines in tier 1 (the window loop, its terms, the solver).
- One chaining (`place_window_after`) and one rest test.
- The measurement repeatable by a tool.
- Results bit-identical.

**Non-Goals:**
- No split of the front-end, map, back-end, I/O or GUI functions (their region's first job, or when next changed).
- No change to how rest is detected or how the guess is made.

## Decisions

1. **Split `predict_window_poses` now**, although only 4 lines over: it is the last tier-1 function above the limit, and
   the split removes a copy of the chaining.
2. **One rest test for the three callers.** `window_rests` holds the acceleration on the 50 ms average and the
   stationarity test only; the size checks (different at each caller) and the gyro threshold's conversion to rad/s stay
   where they are. The poses passed differ by caller (the chained guess, the window later), which changes nothing: the
   test reads only the rotations.
3. **Tiers 2 and 3 stay deferred**; the measured table goes into the plan now and into the region knowledge files in
   stage 7. `update_rgd_legacy` (map) joins tier 2.
4. **`tools/function_lengths.py`** in the repo (pure Python, a rough brace-matching reader).
5. **No Debug build at the end**: a pure split without lifetime, threading or configuration-dependent changes.

## Risks / Trade-offs

- [The chaining through `place_window_after` rounds differently] → the same expressions in the same order (`inverse() * x`
  of two `Affine3d` is an `Affine3d` in both); a test compares the two bit for bit; the guards and replays run the
  prediction in every window.
- [An empty window writes out of bounds in `place_window_after`] → a guard keeps today's behaviour (one node at the
  previous end); windows always have `threshold_nr_poses` nodes, so it never runs on real data.
- [The rest test changes a caller's result] → the same calls with the same arguments; the loop guard, the replays and
  44_short cover the prediction and the accelerometer term's rest; the fit diagnostic's lines are compared on the 44_short
  replay with `log_window_diagnostics`.
- [The length tool misreads a function] → it is a measurement aid, not a check that fails a build; its table was
  sampled against the code by the reviewer.
