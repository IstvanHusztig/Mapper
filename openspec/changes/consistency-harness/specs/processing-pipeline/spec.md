# Spec Delta

## ADDED Requirements

### Requirement: Handed-on state and honesty readings of a window
The odometry SHALL record, per window, how far the window moved each carried state from its prior, measured against
the covariance that move has under the window's own model, and, with the per-window table, the state the window
hands to the next one. These are readings for investigation; they are not statements about the map's accuracy.

- **Normalised move:** for a carried state with prior mean m and prior covariance P, the window's result x and the
  window's posterior covariance P_post, the reading SHALL be (x - m)^T P^-1 (x - m) / (n - tr(P^-1 P_post)), with n
  the state's dimension. Its expected value under a correct linear-Gaussian model is 1.
- **States:** the start velocity; with `imu_carry_joint_covariance` the start velocity and the acceleration offset
  jointly; the gyro bias.
- **Windows without a reading:** a window without the term (no accelerometer term, no gyro-bias estimation), and a
  window whose denominator n - tr(P^-1 P_post) is below 0.5, SHALL have the reading `NOT MEASURED`, never a number.
- **Run summary:** the verdict summary of every run, replay and cancelled run SHALL list, per reading, the median,
  the 95th percentile and the coverage (windows measured of all). The readings SHALL have no limit, no flag and no
  run trigger.
- **The velocity chi² is unchanged:** its formula, its limits (5 and 20) and its place in the summary SHALL stay as
  they are.
- **Handed-on state:** with `log_window_terms` the window's record SHALL hold what the next window starts from: the
  end velocity, the carried gravity vector, the gyro bias, each with its claimed sigmas, the velocity-offset
  covariance where it is carried, and the last node's position as solved and as handed on. It SHALL also hold the
  window's mean rotation rate and its world-frame acceleration at its start. All of these SHALL be columns of
  `window_features.csv`; the columns that file had before SHALL keep their names, order and values.
- **No effect on results:** no window SHALL be processed differently because of these readings. Every output file
  other than `window_features.csv` SHALL be the same bits as without them, and nothing here SHALL be part of the
  odometry settings that window captures and odometry snapshots are checked against, or of a capture file.

#### Scenario: A run reports the readings
- **WHEN** 44_short is processed with the shipped settings
- **THEN** the verdict summary lists the median, 95th percentile and coverage of each normalised move, and every result file matches the manifest

#### Scenario: A reading that doesn't exist
- **WHEN** a window has no accelerometer term, or adds almost nothing to a carried state (denominator below 0.5)
- **THEN** that state's normalised move is `NOT MEASURED` and the summary's coverage shows it

#### Scenario: The table gains columns and keeps the old ones
- **WHEN** a capture is replayed with `log_window_terms` before and after this change
- **THEN** `window_terms.csv` is the same bits, and `window_features.csv` has the same values in the columns it had before, followed by the new columns

#### Scenario: The readings are the same on a synthetic and a real run
- **WHEN** `tools/honesty_report.py` is given the `window_features.csv` of a real run and of a synthetic run
- **THEN** it prints the same table for both: the start-velocity move by rotation rate and by start acceleration, its autocorrelation, and the normalised moves
