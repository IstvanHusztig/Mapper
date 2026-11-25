## MODIFIED Requirements

### Requirement: Region changes arrive as a checked branch
A region agent SHALL make its change in a git worktree of its own, on a branch of its own, and SHALL NOT change the
main working tree, merge, push or start a full run. The orchestrator SHALL merge a region's branch only after the
ownership check and the step checks on that branch; a branch whose step checks are not identical everywhere changes a
number and SHALL be merged only after the user's decision on its numbers. After each merge the step checks SHALL be run
on the main tree.

A region agent SHALL NOT record the check register. A branch's difference from the register SHALL be part of its
hand-back. A difference that only adds cases, assertions or values, or that rewords an assertion with the same numbers
and comparison operators, SHALL be the orchestrator's to accept; the orchestrator SHALL record the register after the
merge, in a commit of its own. A difference that removes a case or an assertion, changes an assertion or a value, or
adds or removes a marker SHALL be merged only after the user's decision, with the reason given, unless the user's
decision on the change itself already covers it (a feature removed with its tests).

#### Scenario: A change that changes no number
- **WHEN** a region's branch passes the ownership check and its step checks are identical everywhere
- **THEN** the orchestrator may merge it and reports the merge

#### Scenario: A change that changes a number
- **WHEN** a region's branch has a step check that is not identical
- **THEN** the branch is not merged until the user has decided on its numbers

#### Scenario: Two branches
- **WHEN** two regions' branches are merged one after the other
- **THEN** the step checks run on the main tree after each merge, and a difference the branches did not show alone takes the last merge back

#### Scenario: A branch that adds tests
- **WHEN** a branch's register difference only adds cases and assertions
- **THEN** the orchestrator may merge it and records the register after the merge

#### Scenario: A branch that loosens a tolerance
- **WHEN** a branch's register difference changes the tolerance of an existing assertion
- **THEN** the branch is not merged until the user has decided on that tolerance

#### Scenario: A feature removed on the user's decision
- **WHEN** the user has decided to remove a feature and the branch removes the feature's own test cases with it
- **THEN** the removed cases are listed in the hand-back and the merge needs no second decision
