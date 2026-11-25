## ADDED Requirements

### Requirement: The odometry settings hash is pinned
The unit tests SHALL hold the odometry settings hash of the built-in defaults and of each shipped settings file as
literal values, and SHALL fail with the new value printed when one of them changes. A change of a pinned value SHALL be
made only together with the change that explains it (a shipped setting's value, or a decision that older snapshots and
captures are refused).

#### Scenario: A setting is removed without care for the hash
- **WHEN** a setting that entered the hash is removed and nothing replaces its contribution
- **THEN** the pin of the defaults fails and prints the new hash

#### Scenario: A new setting that is off by default
- **WHEN** a new odometry setting is added so that it counts in the hash only when it is set
- **THEN** all three pins hold

### Requirement: The simulator's noise check rejects a planted wrong noise
The unit tests SHALL generate the synthetic inertial data with a noise that is not the claimed one, by a factor stated
before the test is run, and SHALL require the test that compares the generated noise with its claim to reject it. The
smallest factor the check rejects reliably at its sample size SHALL be stated with the test.

#### Scenario: Noise larger than claimed
- **WHEN** the accelerometer and gyro white noise are generated at 1.5 times their claimed sigma
- **THEN** the noise check rejects the claim for every seed of the test

#### Scenario: The claimed noise
- **WHEN** the noise is generated as claimed
- **THEN** the same check accepts it, as before

### Requirement: The folder comparison is tested on known folders
The Python tool tests SHALL run the baseline comparison on folders written by the test: two folders that differ only
in the fields a run changes (date and time in file names and contents, the folders' own paths) SHALL compare identical;
a folder with one changed byte in a data file SHALL be reported as different with that file named; a manifest written
from a folder SHALL compare identical with that folder.

#### Scenario: One changed byte
- **WHEN** one byte of a point file differs between the two folders
- **THEN** the comparison names that file, reports one differing file of the total, and exits non-zero

### Requirement: The run check is tested on a log with known readings
The Python tool tests SHALL give the log judge a log written by the test whose windows have known bends, speeds, gyro
bias and velocity chi-square, and SHALL require every reading and every row's verdict to be the known one, on both
sides of each limit.

#### Scenario: One window bent beyond the limit
- **WHEN** one of two hundred windows in the log is bent by 4 degrees and the others by less than 1
- **THEN** the judge counts one bent window (0.5 %), reports it below the share that triggers, and names its time when
  the worst windows are asked for
