## ADDED Requirements

### Requirement: Solver tested without a map
The unit tests SHALL test the Levenberg-Marquardt solver on a small problem that has no window, map or term:
- one iteration SHALL linearize, hand the covariance of the undamped unknowns (from the undamped matrix) to the
  problem, and apply a step; without a step it SHALL end after the covariance and leave the state unchanged;
- the damping factor SHALL grow tenfold after iterations 10, 20, …, and at once after a step the problem refuses;
- the iterations SHALL end when the update falls below the convergence threshold, at the iteration limit, at a forced
  last iteration, or when the time callback says so, and the result SHALL name which;
- with the step check, a step that raises the cost SHALL be taken back, the damping raised, and the rejection counted;
  without it no state SHALL be saved.

#### Scenario: A refused step
- **WHEN** the small problem refuses the step of iteration 2
- **THEN** iteration 3 runs with ten times the damping, and the state is the one before the refused step

#### Scenario: Why it stopped
- **WHEN** the small problem converges before the iteration limit
- **THEN** the result says converged and gives the iteration it ended after

### Requirement: Window verdict tested on made-up evidence
The unit tests SHALL test the window verdict without recorded data:
- a window SHALL have a flag only above its limit (bent above 3°, bent severely above 10°, fast above 2.5 m/s, bias
  high above 0.5 °/s), and none at the limit;
- a reading that is missing SHALL give `NOT MEASURED`;
- the run summary SHALL give the counts, shares, first and worst window, coverage and triggers of a made-up list of
  windows (exactly 1 % of the windows bent is clear, more is a trigger), and the chi² statistics over the windows with
  an information ratio below 0.45 and over all;
- the window loop on the synthetic recording SHALL give the same poses, bit for bit, with `log_window_verdict` on and
  off.

#### Scenario: At the limit
- **WHEN** a window's heading bend is exactly 3.0° and another's is 3.01°
- **THEN** only the second is bent

#### Scenario: Missing evidence
- **WHEN** a window has no end velocity
- **THEN** its speed is `NOT MEASURED`, and the summary counts it as not measured, not as clear
