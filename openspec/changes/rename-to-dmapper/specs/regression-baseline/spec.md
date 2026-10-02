# Spec Delta

## MODIFIED Requirements

### Requirement: Deterministic output for identical runs
Running the CLI (`dMapper <input> <params.toml> <output>`) twice with the same Release
build, the same input folder and the same parameter file SHALL produce output folders that are
identical after normalization of the run-dependent fields.

#### Scenario: Two runs of the reference dataset
- **WHEN** the Release CLI is run twice on `raw data/44` with `settings/lidar-vertical.toml` into two different output folders
- **THEN** the baseline comparison of the two folders reports no differences
