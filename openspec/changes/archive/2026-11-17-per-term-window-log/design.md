# Design

## Context

The full design is the section "Stage 3: per-term log" of `docs/code-split-and-region-agents-plan.md` (2026-11-17):
why four motions, the definitions of weight, hold and pull, the rows, the splits, the output and the tests. A Sonnet
reviewer checked it against the code at `addd8e4`; its eleven findings are worked in. The user took all five decisions
as recommended. This file keeps the decisions and the risks.

## Goals / Non-Goals

**Goals:**
- Per window and term: what it costs, how strongly it holds each motion, and where it pulls.
- Results identical with the table on or off.
- Every function of the optimizer's terms under 80 lines.

**Non-Goals:**
- No judgement on a window (stage 4's verdict).
- No change of a weight (measured changes, later); no weights in one place (stage 5).
- No GUI display of the table.
- The `log_window_diagnostics` weak-axis and heading lines stay unchanged (decided after stage 8).

## Decisions

1. **Four motions, not translation only.** The origin fix pins the first node (weight 1e12), the gyro rows see only
   relative rotation, and the accelerometer terms only changes of velocity. A translation-only table would show the
   gyro and accelerometer rows as empty. Shift, stretch, curve and turn, each with one amplitude at the last node, make
   every term visible in the motion it resists. The velocities move as the motion's derivative, so a stretch is no
   change of acceleration. Alternative: a full per-term covariance. Rejected as unreadable; twelve numbers per row
   answer the Livox 4 questions (stall = stretch along travel, flyaway = stretch and curve, bend = turn about vertical).
2. **Weight and pull as projections of each term's own normal equations.** Weight mᵀHm, pull mᵀb / mᵀHm. They add up
   (Σw·p / Σw is the combined step along m), the sign convention is the same for every term (checked by the reviewer),
   and the numbers come from the matrices the solver uses, not from finite differences of a cost.
3. **A separate function on `const` inputs, not hooks inside the iterations.** `window_term_table` calls each term on
   an empty `WindowEquations`. The solve receives the same additions in the same order as today; the identity argument
   needs no reasoning about the iterations. Cost: two assemblies per window against up to 100. Alternative: per-term
   subtotals collected during the solve. Rejected: it couples the log to the hot loop and still lacks the per-grid and
   unprojected rows.
4. **Guess and result.** The guess shows what each term wanted, the result who won. The result call sits directly after
   the iterations and before the steps that change the window again; the guess call after `motion_model_poses` is set.
5. **Splits before the table.** The table needs one grid at a time, position and rotation edge rows apart, and the
   three velocity priors apart. The edge split uses the generated block's linearity in the six weights (checked): zero
   rotation weights give the position rows and the reverse. The NDT split forces node blocks for the table even with
   the projection off; the projection for both grids comes from the indoor normals, as in the solver.
6. **A damping row.** Late iterations run with a factor up to 5e9, which can freeze a window before its terms balance.
   The row shows the last factor as a weight, so a large remaining step is read as "frozen" and not as a missing term.
7. **Output through a port.** `WindowTermSink` in the application layer, a CSV adapter in the infrastructure, as the
   window capture. The pipeline keeps no file names.
8. **The setting outside the settings hash.** In `[lidar_odometry_debug_info]`, listed in both `TomlIO::POINTERS` and
   the section's `CATEGORIES` entry; otherwise every capture and snapshot is refused.

## Risks / Trade-offs

- [The pull ignores that the solver moves the other unknowns at the same time] → the feature file also holds the
  window's actual change from guess to result in the same twelve numbers, so the gap is visible.
- [The result state can throw where the run itself would not (Huber, non-finite distance)] → the table call catches
  it, warns and leaves the rows empty.
- [A split changes a bit in a branch no run reaches] → the characterization guard (recorded again from `addd8e4`)
  after every commit, as in stage 2.
- [Turn at fi = ±90°: the angle Jacobian has no inverse] → within 3° the turn columns stay empty and the log says so.
- [Cost / n misread as a noise check] → documented per row: about 1 only for noise-model weights; NDT, the scaled edge
  positions and the weak tie are read against their own usual value.
- [Run time with the setting on] → measured on 44_short at the end of the stage and reported.
