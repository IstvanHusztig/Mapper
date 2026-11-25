## Purpose

Make map-verification evidence reproducible and prevent missing or mismatched
measurements from being presented as successful validation.

## ADDED Requirements

### Requirement: Diagnostic coverage is explicit
Reports SHALL distinguish diagnostic triggers from accuracy verdicts and SHALL
return nonzero for incomplete required evidence, including missing per-window
heading, speed or bias. Conditional uncertainty coverage SHALL be reported.

#### Scenario: Missing diagnostic lines
- **WHEN** a log contains window starts but lacks required measurements
- **THEN** the report labels them NOT MEASURED and does not return success

### Requirement: Trajectories match exactly within a named stage
Comparisons SHALL require finite positions and unique, matching timestamps and
counts. Odometry and final trajectories SHALL be selected explicitly and never
silently substituted. Unsupported or corrupt snapshots SHALL be rejected.

#### Scenario: Shifted or truncated trajectory
- **WHEN** a result has different timestamps or fewer poses than its reference
- **THEN** comparison fails without a displacement verdict

#### Scenario: Missing odometry snapshot
- **WHEN** odometry is requested but its snapshot is absent
- **THEN** comparison reports missing evidence rather than using final poses

### Requirement: Sweep reuse requires provenance and completion
Sweeps SHALL fingerprint input files, effective settings, executable and retained
outputs. Resume SHALL refuse absent or mismatching identity or unsuccessful runs.
Failed or incomplete members SHALL cause a nonzero aggregate exit. Sweeps SHALL
retain clouds and SHALL NOT overwrite unidentified output directories.

#### Scenario: Changed executable or output
- **WHEN** a resumed member differs from its recorded identity
- **THEN** reuse is refused and existing artifacts remain intact

#### Scenario: Subprocess failure
- **WHEN** a member process fails
- **THEN** the failure remains in the report and the sweep exits nonzero
