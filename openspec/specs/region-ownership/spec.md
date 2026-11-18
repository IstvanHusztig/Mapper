# region-ownership Specification

## Purpose

Divides the code into regions with one owner each, so that a region's agent can change its part in a worktree
of its own and the orchestrator can check what a branch touches and merge it under fixed rules.

## Requirements

### Requirement: Ownership list
The repository SHALL hold one list (`docs/regions/owners.toml`) that gives every file under `src/` and `tests/` exactly
one owner: one of the regions (front-end, map, lidar-matching, inertial, solver, back-end, noise-budget, verification)
or the orchestrator. The list SHALL name files, functions inside a file (which override that file's owner) and shared
test files that any region may add its own cases to; what it does not name SHALL belong to the orchestrator.

#### Scenario: The list is consistent
- **WHEN** `tools/region_owners.py` is run without arguments
- **THEN** it reports every named file or function that does not exist and everything named twice, and exits 0 only when there is none

#### Scenario: A branch stays in its region
- **WHEN** `tools/region_owners.py --diff <base>..<branch> --region map` is run on a branch that changes only `src/domain/ndt_map.cpp` and `tests/ndt_map_test.cpp`
- **THEN** it lists nothing outside the region and exits 0

#### Scenario: A branch leaves its region
- **WHEN** the same command is run on a branch that also changes `src/domain/window.h` or a function of `run_odometry.cpp` the region does not own
- **THEN** it lists those files and functions with their owners and exits 1

#### Scenario: A shared test file
- **WHEN** a branch of a region changes a shared test file
- **THEN** the file is listed as shared, for the orchestrator to read, and does not make the check fail

### Requirement: Region changes arrive as a checked branch
A region agent SHALL make its change in a git worktree of its own, on a branch of its own, and SHALL NOT change the
main working tree, merge, push or start a full run. The orchestrator SHALL merge a region's branch only after the
ownership check and the step checks on that branch; a branch whose step checks are not identical everywhere changes a
number and SHALL be merged only after the user's decision on its numbers. After each merge the step checks SHALL be run
on the main tree.

#### Scenario: A change that changes no number
- **WHEN** a region's branch passes the ownership check and its step checks are identical everywhere
- **THEN** the orchestrator may merge it and reports the merge

#### Scenario: A change that changes a number
- **WHEN** a region's branch has a step check that is not identical
- **THEN** the branch is not merged until the user has decided on its numbers

#### Scenario: Two branches
- **WHEN** two regions' branches are merged one after the other
- **THEN** the step checks run on the main tree after each merge, and a difference the branches did not show alone takes the last merge back
