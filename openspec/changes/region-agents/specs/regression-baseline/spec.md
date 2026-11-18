## ADDED Requirements

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

### Requirement: End-of-change runs as one command
The repository SHALL provide the runs that end a change as one command (`tools/end_runs.py`): the full 44, Livox 3 and
44_short with their shipped settings, one after the other, each compared with its manifest, with the odometry time of
each; with `--debug` the Debug build and `ctest` after them. It SHALL refuse to start while another dMapper process
runs, and SHALL write one line per run to a log file so that it can run in the background.

#### Scenario: A change without an effect on results
- **WHEN** the command runs on code that changes no number
- **THEN** each recording's line reports IDENTICAL with its manifest

#### Scenario: Another run is active
- **WHEN** the command is started while a dMapper process is running
- **THEN** it starts no run and says so
