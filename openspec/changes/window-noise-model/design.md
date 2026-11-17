# Design

## Context

The full design is the section "Stage 5: one place for the weights" of `docs/code-split-and-region-agents-plan.md`
(2026-11-17): the inventory of every weight of the window problem (where, formula, inputs, lifetime), the noise model's
two kinds of content, why it stays bit-identical, the log line, three findings, the tests and the commit order. The
inventory was made by reading the code at `3b2d146`; a Sonnet reviewer checked the design against the code and its
findings (the layout conditions, the two dt clamps, the bias coupling, consistency sharing the sigma not the weight,
the term table's input, the construction point) are worked in. This file keeps the decisions and the risks.

The user took all seven decisions of the plan as recommended (2026-11-17).

## Goals / Non-Goals

**Goals:**
- Every weight of a window in one struct, built once per window, and readable in one log line.
- Terms that receive weights and never compute them.
- The per-term table and the weak-axis diagnostic weigh exactly as the problem does.
- Results bit-identical.

**Non-Goals:**
- No change of any weight's value (the velocity prior's covariance, the origin-fix quirk, the 0.1 scales).
- No change to loop closure's or consistency's weights (consistency only shares the rotation sigma function).
- No per-iteration model: the Huber weight, the projection and the cell information stay where they are computed.

## Decisions

1. **Scope: the window problem only.** Consistency's time-scaled rotation sigma calls the model's sigma function
   (it returns max(k·ARW·√dt, floor); consistency multiplies by its own scale as today). Sharing the sigma is
   bit-identical; sharing a weight would not be.
2. **The per-point NDT rules stay in the term, with their parameters in the model.** They depend on the current poses
   (Huber, projection) or the map (cell information). The term applies them in today's order as separate
   multiplications: generated product with the cell information, range weight, Huber weight, scale.
3. **The log:** `window noise <n>:` per window behind `log_window_noise`, the same values as `window_features.csv`
   columns with `log_window_terms`, and a `noise budget` line in every run (one line, settings only).
4. **The origin-fix quirk is kept** (rotation sigma in degrees used as radians, `window_solver.cpp:12-14`): weight 1e12
   instead of 3.3e15, both hold the first node practically fixed. Commented in the code and recorded in
   `docs/code-review-tracker.md`; a later numeric change may fix it with a measurement.
5. **`ndt_window_cost` follows the term's per-grid range weight and cap** (it applies `use_norm` and the cap of 10 to
   both grids, the term to the fine grid only). Only `log_weak_axis_diagnostics` reads it; its numbers change, no result
   does.
6. **The velocity chi² question of stage 4 is not answered here.** The model puts the velocity prior's covariance in
   one place and in the log, so a later change can measure it.
7. **The model is built once per window in `prepare_window`, with the layout** (`layout_window_unknowns`), and
   `WindowProblem` takes it as a const reference; `linearize` no longer rebuilds the layout. The model computes a
   weight only where today's term would: velocity continuity only without the accelerometer term, no bias weights in
   the first window, the velocity-continuity dt clamped at 1e-4 s and the edges' dt at 0.

## Risks / Trade-offs

- [A weight hoisted out of the iterations depends on something that changes during them] → the inventory and the review
  traced every input to settings, timestamps or carried state; `take_covariance` writes posteriors that only the next
  window reads. The characterization guard, the loop guard (step check on, early ends) and the replays would show any
  difference.
- [Precomputing changes the floating-point order] → same expressions, same operand order; weights enter the terms
  where they enter today, never folded into Jacobians or into one combined NDT factor. The builder's test compares its
  values bit for bit with the terms' before the terms switch.
- [A weight computed under a condition the term didn't have] → the model holds the layout and mirrors each term's
  guard; the "conditions" test covers the first window, rest and the path without the accelerometer term.
- [The per-term table drifts from the problem] → both read the same model; the table's guess and result share it,
  since its fixed weights depend on the timestamps only.
- [Captures made before the change stop replaying] → the model is rebuilt by `prepare_window` from the carried state,
  so the capture format doesn't change; both captures are replayed on every commit.
- [The new setting changes the settings hash] → it is in the `[lidar_odometry_debug_info]` section, outside the hash,
  like `log_window_terms` and `log_window_verdict`.
