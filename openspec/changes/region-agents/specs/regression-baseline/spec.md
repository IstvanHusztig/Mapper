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
