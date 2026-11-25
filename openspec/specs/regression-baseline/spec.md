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
`build_date`; the build's `git_commit` and `git_uncommitted_changes`; and absolute paths, which are compared after
replacing the output-folder and input-folder prefixes with placeholders. Files that dMapper does not write and the
analysis tools add to a result folder (`*.asc` cloud exports) SHALL be neither hashed nor compared.

#### Scenario: Only timing differs
- **WHEN** two output folders differ only in elapsed time, processing date, build date, build commit, result file name timestamps and the output folder location
- **THEN** the comparison reports them as identical

#### Scenario: A trajectory value differs
- **WHEN** one value in any `trajectory_lio_*.csv` differs in its last digit
- **THEN** the comparison reports that file as different

#### Scenario: Cloud exports in a baseline folder
- **WHEN** a baseline folder also holds `scan_lio_<n>.asc` files exported by `tools/map_quality.py`
- **THEN** a manifest written from it lists no `.asc` file, and a comparison with it ignores them

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

### Requirement: Step checks as one command
The repository SHALL provide the checks of a refactor commit as one command (`tools/step_checks.py`) that runs, in this
order and one at a time, the Release build, the unit tests, the characterization guard, the loop guard, the replays of
the 44_short and Livox 3 captures, 44_short against its manifest, and the per-term table of the 44_short replay, and
prints one line per check. The reference files of the guards and of the per-term table SHALL be written by the same
command (`--record`) from a tree without uncommitted source changes, with the commit they were recorded from. The
workstation's paths SHALL come from one file (`tools/local_paths.toml`). The command SHALL run in any worktree of the
repository on that worktree's build.

#### Scenario: Unchanged code
- **WHEN** the references are recorded and the command is run again on the same commit
- **THEN** every check reports identical (or passed) and the command exits 0

#### Scenario: A change that alters a number
- **WHEN** the command runs on code whose window optimization gives different bits
- **THEN** the checks that differ say so, and the command exits 1

#### Scenario: References from a changed tree
- **WHEN** `--record` is asked for while `src/` or `tests/` has uncommitted changes
- **THEN** the command refuses and writes nothing

### Requirement: Replays through a gate
The repository SHALL provide one command (`tools/run_replay.py`) through which replays of a window capture are started
when several jobs share the workstation. Before it starts the replay it SHALL wait until the free physical memory covers
the replay's estimated need (a multiple of the capture file's size, plus a reserve) and until fewer than the allowed
number of replays run (two) and no full run holds the gate. It SHALL never suspend a running replay. After the replay it
SHALL print the wall time, the CPU time, the peak memory and the number of windows that stopped at the time limit, and
SHALL return the replay's exit code.

#### Scenario: A free workstation
- **WHEN** a replay is started through the gate while nothing else runs
- **THEN** it starts at once, and its line shows wall time, CPU time, peak memory and the time-limit count

#### Scenario: Both slots taken
- **WHEN** a third replay is started through the gate while two run
- **THEN** it waits, says what it waits for, and starts when a slot is free

#### Scenario: A full run goes
- **WHEN** a replay is started through the gate while the end-of-change runs hold it
- **THEN** it waits until they have finished

#### Scenario: A stale holder
- **WHEN** a slot's holder no longer runs (its process ended without releasing)
- **THEN** the gate treats the slot as free

### Requirement: End-of-change runs as one command
The repository SHALL provide the runs that end a change as one command (`tools/end_runs.py`): the full 44, Livox 3 and
44_short with their shipped settings, one after the other, each compared with its manifest, with the odometry time of
each; with `--debug` the Debug build and `ctest` after them. It SHALL hold the replay gate for itself while it runs, SHALL refuse to start while another dMapper process
runs, and SHALL write one line per run to a log file so that it can run in the background.

#### Scenario: A change without an effect on results
- **WHEN** the command runs on code that changes no number
- **THEN** each recording's line reports IDENTICAL with its manifest

#### Scenario: Another run is active
- **WHEN** the command is started while a dMapper process is running
- **THEN** it starts no run and says so

### Requirement: Wall pairs measured from poses
A pure-Python tool SHALL measure how far apart two visits of the same wall lie, for any trajectory of a recording, on a set
of returns frozen beforehand:
- **Freeze:** from one result (its cloud and trajectory) the tool SHALL select wall pieces and their visits and store, for
  every selected return, its time, its trajectory node and its position relative to that node, together with the piece's
  position and normal, the visit it belongs to and the side it was seen from. The stored set SHALL name the result it was
  chosen on and how many pieces, visits and pairs it holds.
- **Measure:** for a trajectory with the same nodes (a result folder, an odometry snapshot, a loop closure replay) the tool
  SHALL place the stored returns with that trajectory, bring the trajectory into the frame of the earlier visit of each
  pair, and report the gap along the piece's normal for every same-side pair: median, 90th percentile and the number at or
  above a threshold, for all pairs and for stated time classes, and the pairs that cross the threshold between two
  trajectories.
- The tool SHALL refuse a trajectory whose node times differ from the stored set's, and SHALL state that its numbers are
  agreement between visits on a selected set, not accuracy.

#### Scenario: The frozen result reproduces its own cloud
- **WHEN** the set is measured with the trajectory of the result it was frozen on
- **THEN** every return lands where it lies in that result's cloud (within the cloud's stored resolution)

#### Scenario: Three stages of one run
- **WHEN** the set is measured with a run's odometry snapshot, its loop closure replay and its final trajectory
- **THEN** one table gives the gaps at the three stages for the same pairs, and no cloud is read

#### Scenario: Another recording's trajectory
- **WHEN** a trajectory with other node times is given
- **THEN** the tool refuses it with the reason

### Requirement: A run's check reports its timing and the time-limit count
Every run SHALL log the duration of each stage and the number of windows whose iterations stopped at the time limit.
`tools/run_health.py` SHALL report both; a count of zero SHALL be reported as "result independent of the machine's load",
any other count as dependent, and a log without the count as NOT MEASURED.

#### Scenario: A run that never reached the limit
- **WHEN** a run's log reports zero windows stopped at the time limit
- **THEN** `run_health.py` reports the result as independent of load and lists the stage durations

#### Scenario: An older log
- **WHEN** a log without the count is checked
- **THEN** the reading is NOT MEASURED
