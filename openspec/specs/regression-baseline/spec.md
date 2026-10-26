# regression-baseline Specification

## Purpose

Guarantees that the odometry pipeline produces reproducible output for a fixed input and config,
and provides a check that a new run matches the stored golden baseline, so refactors can prove
they changed nothing.

## Requirements

### Requirement: Deterministic output for identical runs
Running the CLI (`dMapper <input> <params.toml> <output>`) twice with the same Release
build, the same input folder and the same parameter file SHALL produce output folders that are
identical after normalization of the run-dependent fields.

#### Scenario: Two runs of the reference dataset
- **WHEN** the Release CLI is run twice on `raw data/44` with `settings/l2-vertical.toml` into two different output folders
- **THEN** the baseline comparison of the two folders reports no differences

### Requirement: Run-dependent fields are normalized
The comparison SHALL ignore only these run-dependent values and nothing else:
the date-time part of the `dMapper_params_*` and `dMapper_results_*` file names;
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
