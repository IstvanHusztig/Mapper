## ADDED Requirements

### Requirement: The hand-off tail follows the acceleration at the window's end
The re-propagation of a window's last nodes before they seed the next window SHALL start from the velocity at the last
kept node (not the velocity at the middle of the finite-difference span) and SHALL include the acceleration over the tail,
so that under constant acceleration the tail's last node and the next window's first pose equal the true positions.
At constant velocity and at rest the tail SHALL equal the previous constant-velocity line.

#### Scenario: Constant acceleration, shipped tail
- **WHEN** a window with positions p = a t²/2 sampled at 500 Hz is re-propagated with tail 30 and span 20 at a = 2 m/s²
- **THEN** the last node and the next window's first pose are within 0.05 mm of the true positions

#### Scenario: Constant velocity and rest
- **WHEN** the window moves at constant velocity or rests
- **THEN** the re-propagated tail equals the previous result (identical positions)

#### Scenario: Window too short for the estimate
- **WHEN** the window has too few kept nodes to estimate velocity and acceleration
- **THEN** the tail falls back to the constant-velocity line, as before
