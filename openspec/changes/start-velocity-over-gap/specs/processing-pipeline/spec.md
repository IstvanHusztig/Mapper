## ADDED Requirements

### Requirement: The start-velocity prior is advanced over the gap between windows
The start-velocity prior of a window SHALL have as its mean the previous window's end velocity advanced over the time gap
between the two windows by the acceleration at the hand-over (gravity and the carried acceleration offset removed). Its
covariance SHALL continue to be grown over the same gap. With no gap, at rest, or without a carried velocity, the mean
SHALL be unchanged from the carried value (or zero as before).

#### Scenario: Constant acceleration over a 2 ms gap
- **WHEN** a window follows one whose end velocity is the true velocity, the sensor accelerates at a constant 1 m/s² or 2 m/s², and the gap is 2 ms
- **THEN** the prepared start-velocity prior mean equals the true start velocity within 0.1 mm/s

#### Scenario: At rest nothing moves
- **WHEN** the sensor is at rest across the gap
- **THEN** the prior mean equals the carried end velocity exactly, as before

#### Scenario: Joint covariance carry
- **WHEN** `imu_carry_joint_covariance` is on
- **THEN** the mean is advanced the same way and the joint (velocity, offset) covariance is unchanged by this requirement
