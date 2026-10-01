# Spec Delta

## Purpose

Guarantees that the odometry pipeline produces reproducible output for a fixed input and config,
and provides a check that a new run matches the stored golden baseline, so refactors can prove
they changed nothing.

## ADDED Requirements

### Requirement: Deterministic output for identical runs
Running the CLI (`lidar_odometry_step_1 <input> <params.toml> <output>`) twice with the same Release
build, the same input folder and the same parameter file SHALL produce output folders that are
identical after normalization of the run-dependent fields.

#### Scenario: Two runs of the reference dataset
- **WHEN** the Release CLI is run twice on `raw data/44` with `settings/lidar-vertical.toml` into two different output folders
- **THEN** the baseline comparison of the two folders reports no differences

### Requirement: Run-dependent fields are normalized
The comparison SHALL ignore only these run-dependent values and nothing else:
the date-time part of the `HDMapping_params_*` and `HDMapping_results_*` file names;
elapsed-time values (`elapsed time seconds`, `elapsed_time_seconds`); `processing_date`;
`build_date`; and absolute paths, which are compared after replacing the output-folder and
input-folder prefixes with placeholders.

#### Scenario: Only timing differs
- **WHEN** two output folders differ only in elapsed time, processing date, build date, result file name timestamps and the output folder location
- **THEN** the comparison reports them as identical

#### Scenario: A trajectory value differs
- **WHEN** one value in any `trajectory_lio_*.csv` differs in its last digit
- **THEN** the comparison reports that file as different

### Requirement: Byte-exact comparison of data files
Every output file that has no run-dependent fields (LAZ files, trajectory CSVs, `.mrp`/`.mri`
pose files, `point_sizes_per_chunk.json`) SHALL be compared byte for byte. Files present on only
one side SHALL be reported as differences.

#### Scenario: Missing chunk
- **WHEN** the new output folder lacks a `scan_lio_N.laz` that the baseline has
- **THEN** the comparison reports the missing file and fails

### Requirement: Baseline manifest in the repository
The repository SHALL contain a manifest for the reference dataset. It SHALL record the dataset
folder name, the parameter file, the commit that produced the baseline, and a SHA-256 hash of
every normalized output file. A new output folder SHALL be checkable against the manifest alone,
without the baseline output folder being present.

#### Scenario: Check against the manifest
- **WHEN** the comparison tool is given a new output folder and the manifest of dataset 44
- **THEN** it reports each file whose normalized hash differs from the manifest, or that is missing or extra

#### Scenario: Regenerate the manifest
- **WHEN** a change deliberately alters numerical behaviour and the user accepts the new result
- **THEN** the tool can write a new manifest from that output folder, recording the current commit

### Requirement: Short reference dataset
Besides the full dataset 44, the repository SHALL contain a manifest for the short cut `44_short`
(the first 30 s of 44, see `recording-cut`). The short cut SHALL also satisfy the determinism
requirement.

#### Scenario: Quick check of an intermediate commit
- **WHEN** the Release CLI is run on `44_short` and its output is compared with `tools/baseline/44_short.manifest.json`
- **THEN** the tool reports identical for an unchanged pipeline, in a fraction of the full run's time

### Requirement: Comparison result is machine-checkable
The comparison tool SHALL exit with status 0 when nothing differs and with a non-zero status when
anything differs or an input is missing, and SHALL print one line per differing file.

#### Scenario: Differences found
- **WHEN** at least one file differs
- **THEN** the tool prints the differing file names and exits non-zero
