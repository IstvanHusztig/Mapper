# Spec Delta

## ADDED Requirements

### Requirement: Tolerance check for the full dataset
The comparison tool SHALL offer a tolerance mode (`--tolerance <abs>`) for comparing a new output folder with a baseline output folder.

Each file that differs after normalization is handled as follows:
- **Text files** (trajectories, session, poses, JSON, TOML): the file SHALL pass only if:
  - everything outside the numbers is identical;
  - every integer is identical;
  - every other number differs by at most the given absolute tolerance.
- **LAZ files:** a differing LAZ file SHALL be reported as derived and SHALL pass only when every text file passes.

Missing or extra files SHALL still fail.

A pure refactor SHALL reproduce `44_short` bit for bit. The full dataset 44 SHALL be identical with its manifest, or within tolerance 1e-6 of the baseline folder `raw data/44_baseline`.

#### Scenario: Last-bit differences on the full run
- **WHEN** a refactor changes only unrelated code, and the full 44 run's trajectories differ from the baseline by at most 1e-9
- **THEN** `compare_baseline.py --tolerance 1e-6` lists those files as within tolerance, lists the differing LAZ files as derived, and exits 0

#### Scenario: A real change
- **WHEN** a trajectory value differs from the baseline by more than the tolerance, or an integer such as a timestamp differs
- **THEN** the tool reports that file as different with the first mismatching value, and exits 1

#### Scenario: Manifest only
- **WHEN** `--tolerance` is given with a manifest instead of a baseline folder, and files differ
- **THEN** the tool reports a usage error (exit 2), because a manifest holds only hashes
