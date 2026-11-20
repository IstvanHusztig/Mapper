# Spec Delta

## ADDED Requirements

### Requirement: Handed-on state and honesty readings of a window
The odometry SHALL record, per window, how far the window moved each carried state from its prior, measured against
the covariance that move has under the window's own model, and, with the per-window table, the state the window
actually hands to the next one. These are readings for investigation; they are descriptive, and they are not
statements about the map's accuracy.

- **Normalised move:** for a carried state with prior mean m and prior covariance P, the window's result x and the
  window's posterior covariance P_post, the reading SHALL be (x - m)^T P^-1 (x - m) / (n - tr(P^-1 P_post)), with n
  the state's dimension. Its expected value in a linear-Gaussian update is 1.
- **States:** the start velocity, in every window that has the accelerometer term; the gyro bias; and, with
  `imu_carry_joint_covariance`, the start velocity and the acceleration offset jointly, with their joint prior and
  posterior. The start velocity's own reading SHALL also be given in that case.
- **`NOT MEASURED`:** a reading SHALL be `NOT MEASURED`, never a number, when the window has no such term; when the
  covariance it needs is absent, not from the window's accepted final state, not finite, ill-conditioned, or would
  mean a negative reduction of uncertainty; when the gyro bias was limited by its clamp; and when
  n - tr(P^-1 P_post) is below 0.5.
- **Run summary:** the verdict summary of every run, replay and cancelled run SHALL list, per reading, the largest
  value, the median, the 95th percentile and the coverage (windows measured of all, also when none was). A window
  that has a flag SHALL show its readings on its line. The readings SHALL have no limit, no flag and no run trigger.
- **The velocity chi² is unchanged:** its formula, its limits (5 and 20) over the windows with an information ratio
  below 0.45, and its statistics over all measured windows SHALL stay as they are.
- **Handed-on state:** with `log_window_terms` the window's record SHALL hold, taken after the hand-off: the end
  velocity, the carried gravity vector, the gyro bias, each with its claimed sigmas and whether it is valid; the
  velocity-offset covariance where it is carried; the last node's position as solved and as handed on; the means and
  sigmas of the priors the window was prepared with; and the window's mean rotation rate and its acceleration at its
  start. These SHALL be columns of `window_features.csv` after the columns that file had before, which SHALL keep
  their names, order and values. A record SHALL be written once per window; a window whose hand-off is cancelled
  SHALL have its handed-on values marked absent.
- **No effect on results:** no window SHALL be processed differently because of these readings. Every output file
  other than `window_features.csv` SHALL stay the same under the existing result comparison, and nothing here SHALL
  be part of the odometry settings that window captures and odometry snapshots are checked against, or of a capture
  file.

#### Scenario: A run reports the readings
- **WHEN** 44_short is processed with the shipped settings
- **THEN** the verdict summary lists the largest value, median, 95th percentile and coverage of each normalised move, and every result file matches the manifest

#### Scenario: A reading that doesn't exist
- **WHEN** a window has no accelerometer term, or adds almost nothing to a carried state, or its covariance is not from its accepted final state
- **THEN** that state's normalised move is `NOT MEASURED` and the summary's coverage shows it

#### Scenario: The table gains columns and keeps the old ones
- **WHEN** a capture is replayed with `log_window_terms` before and after this change
- **THEN** `window_terms.csv` is the same bits, and `window_features.csv` has the same values in the columns it had before, followed by the new columns

#### Scenario: A cancelled hand-off
- **WHEN** a run is cancelled at a window's hand-off
- **THEN** that window's record is written once, with its table as before and its handed-on values marked absent

#### Scenario: A missing input is named, not replaced
- **WHEN** `tools/honesty_report.py` is given a `window_features.csv` without the columns a figure needs
- **THEN** it prints that figure as unavailable and computes the others under their stated definitions
