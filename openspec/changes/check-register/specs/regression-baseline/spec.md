## MODIFIED Requirements

### Requirement: Step checks as one command
The repository SHALL provide the checks of a refactor commit as one command (`tools/step_checks.py`) that runs, in this
order and one at a time, the Release build, the unit tests, the Python tool tests, the comparison of the test sources
with the check register and of the instruments' file with the register, the characterization guard, the loop guard, the
replays of the 44_short and Livox 3 captures, 44_short against its manifest, and the per-term table of the 44_short
replay, and prints one line per check. The reference files of the guards and of the per-term table SHALL be written by
the same command (`--record`) from a tree without uncommitted source changes, with the commit they were recorded from;
the check register SHALL NOT be written by it. The workstation's paths SHALL come from one file
(`tools/local_paths.toml`). The command SHALL run in any worktree of the repository on that worktree's build.

A register difference that only adds SHALL hold in a region's worktree, with the additions counted in its line, and
SHALL fail in the main tree until the register is recorded; every other register difference SHALL fail everywhere. The
line of a full run SHALL give the number of windows that stopped at the time limit, and a difference from the manifest
with a count above zero SHALL be reported as dependent on the machine's load.

#### Scenario: Unchanged code
- **WHEN** the references are recorded and the command is run again on the same commit
- **THEN** every check reports identical (or passed) and the command exits 0

#### Scenario: A change that alters a number
- **WHEN** the command runs on code whose window optimization gives different bits
- **THEN** the checks that differ say so, and the command exits 1

#### Scenario: References from a changed tree
- **WHEN** `--record` is asked for while `src/` or `tests/` has uncommitted changes
- **THEN** the command refuses and writes nothing

#### Scenario: A branch that adds tests
- **WHEN** the command runs in a region's worktree on a branch that adds test cases and changes no other check
- **THEN** the register's line holds and counts the added cases, and the command's exit code is not changed by it

#### Scenario: A branch that weakens a test
- **WHEN** the command runs on a branch that changes a tolerance of an existing assertion
- **THEN** the register's line fails and names the case, and the command exits 1

#### Scenario: A full run under load
- **WHEN** the 44_short run differs from its manifest and its log counts windows stopped at the time limit
- **THEN** the line gives that count and says that the result depended on the machine's load
