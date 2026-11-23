# Proposal

## Why

After a window is solved, `repropagate_window_tail` (`src/domain/window_steps.cpp:201`) replaces its last `handoff_tail_nodes`
nodes by a constant-velocity line: the velocity is a finite difference of the optimized nodes over
`handoff_tail_velocity_span_nr_samples` samples, so it is the velocity at the middle of that span, not at the last kept
node. Under acceleration the line lags the truth, and the next window's fixed first pose comes from the tail's last node.
Measured in `consistency-harness` (`tests/unit_cases_inertial_test.cpp`, marker K4) on p = a t²/2 at 500 Hz, tail 15 /
span 10: the last node is -0.750 mm off at 1 m/s² and the next first pose -1.664 mm at 2 m/s²; with the shipped L2 values
(tail 30, span 20) at 2 m/s²: -6.000 mm and -6.324 mm. This is finding F2b of the audit and step 4 of the user's decided
order (`consistency-harness` task 8.2). It is done on its own because the tail also moves the points that enter the map.

## What Changes

- The re-propagated tail follows the acceleration of the window's end (the velocity at the last kept node, and the
  acceleration over the tail), so under constant acceleration the tail's last node and the next first pose equal the
  truth (K4 oracle: 0.05 mm).
- At constant velocity and at rest the tail is unchanged.
- The red marker `unit case K4 marker` loses `doctest::should_fail`.
- Numbers and map points change (a few mm per window at 2 m/s² with the shipped tail), so baselines are measured first
  and re-recorded only after the user's decision.

## Capabilities

### New Capabilities

### Modified Capabilities
- `processing-pipeline`: the hand-off tail of a window is re-propagated with the end acceleration, not as a constant-velocity line.

## Impact

- `src/domain/window_steps.cpp::repropagate_window_tail` (inertial region; the rotation half is C32, not part of this);
  `run_odometry.cpp:1286` is the caller.
- Tests: `tests/unit_cases_inertial_test.cpp` K4 marker flips; `tests/window_steps_test.cpp` cases on the tail.
- Results: every run with `handoff_tail_nodes > 0` (the shipped settings) changes; not bit-identical to the manifests.
- Order: after `start-velocity-over-gap` (F2a), measured separately so the effects can be told apart.
- Not in scope: the fixed first pose design (step 6), the rotation half of the tail (C32), the tail length itself.
