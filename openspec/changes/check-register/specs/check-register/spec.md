# Spec Delta

## Purpose

Holds what the tests assert and what proves each measuring instrument as committed data, so that a weakened check or
an unproven instrument is a visible difference in one file and not something a reviewer must find in code.

## ADDED Requirements

### Requirement: The tests' assertions are held in a committed register
The repository SHALL hold one text file (`checks/tests.register`), written by a tool from the test sources and never
by hand, that lists for every test case of the unit-test executable and of the Python tool tests: its file and name,
its markers (skipped in the regular run, expected to fail), every assertion as one line, the calls it makes to helpers
that assert, and the constants and tables declared in it. For every test source file it SHALL also list the values
declared outside the cases (named limits, tolerance structures, case tables) and the assertions of helpers. The file
SHALL carry totals (cases, markers, assertions) and SHALL be the same for the same sources on every run.

#### Scenario: A tolerance written in the assertion
- **WHEN** a case asserts that an error is below 0.001
- **THEN** the register has that assertion as a line of the case, with the 0.001 in it

#### Scenario: A tolerance held in a named value
- **WHEN** a case compares against a member of a tolerance structure declared in a test header
- **THEN** the register lists that member with its value under the header's file

#### Scenario: Unchanged sources
- **WHEN** the tool writes the register twice from the same sources
- **THEN** the two files are byte for byte the same

### Requirement: The sources are compared with the register by kind of difference
A command SHALL compare the test sources with the committed register and SHALL print every difference with its kind:
case added, case removed, case moved to another file, marker added, marker removed, assertion added, assertion removed,
assertion changed, value changed. Any difference in an assertion's text other than white space and comments SHALL be a
change. A marker set on a group of cases (a suite, a test class) SHALL count for every case of the group. It
SHALL exit 0 only when there is no difference, and SHALL fail when no register has been recorded. Differences that only add (cases, assertions, values) SHALL be
distinguished in its output and exit code from differences that remove, change or mark.

#### Scenario: A loosened tolerance
- **WHEN** a tolerance literal of an assertion is changed from 0.001 to 0.01 and the register is not recorded again
- **THEN** the command names the case, prints the old and the new line as an assertion changed, and exits non-zero

#### Scenario: A case skipped
- **WHEN** a case gains the marker that skips it in the regular run
- **THEN** the command prints a marker added for that case

#### Scenario: Only new tests
- **WHEN** a branch adds two cases and changes nothing else in the tests
- **THEN** the command lists two cases added and reports the difference as additions only

#### Scenario: The same numbers in a weaker assertion
- **WHEN** an assertion on the absolute value of an error is changed to the error itself, with the same limit
- **THEN** the command reports an assertion changed and exits non-zero

#### Scenario: A case skipped through its group
- **WHEN** existing cases are put into a suite that is marked as skipped
- **THEN** the command reports a marker added for each of them

### Requirement: The register is recorded only on purpose
The register SHALL be written to the repository only by an explicit record command, and that command SHALL refuse
while the test sources have uncommitted changes. The register and the instruments' file SHALL belong to the
orchestrator in the ownership list.

#### Scenario: Recording from a changed tree
- **WHEN** the record command is run while a test source has uncommitted changes
- **THEN** it refuses and writes nothing

### Requirement: The register's own reach is stated and tested
The tool SHALL be tested with planted edits on test sources written for the purpose: a tolerance loosened in an
assertion, in a named value, in a case table row and in the argument of an asserting helper; an assertion deleted; an
assertion commented out; an assertion inside a local function object, and the call of it, removed; a failure macro's
pinned literal changed; the list a loop runs over changed; a skip marker added on the case, on its suite or class, or
as a call inside the test; an expected-to-fail marker removed; a macro defined round an assertion; a case renamed.
Each SHALL be reported with its kind. The tool SHALL check its own count of cases and assertions against an independent count of the
same sources and SHALL fail when they disagree or when a source cannot be read to its end. What the register cannot
see (a limit computed while the test runs, an assertion that the flow of the test never reaches, a limit that lives in
the program and not in the tests) SHALL be stated in the tool's documentation and in the register's header.

#### Scenario: An assertion commented out
- **WHEN** an assertion of a case is turned into a comment
- **THEN** the comparison reports an assertion removed for that case

#### Scenario: A source the tool cannot read
- **WHEN** a test source has an unclosed bracket the tool cannot match
- **THEN** the tool fails with the file and line, and writes no register

### Requirement: Every instrument states what proves it
The repository SHALL hold one file (`checks/instruments.toml`) with a section per measuring instrument: the synthetic
simulator, the synthetic routes, the loop world, the truth statistics, the fixtures, and each tool whose output a
finding or a merge decision rests on. Each section SHALL state five things: the known-answer check with its tolerance;
the smallest effect the instrument detects and where that was measured; the fault that was planted and must be
reported, with what the instrument is known not to see; the range of inputs it is valid for, and whether an input
outside that range is detected; the precondition guards it applies to its own input. Each of the five SHALL be given
either with the test cases that prove it, or as not established, or as not applicable with the reason.

#### Scenario: A gap
- **WHEN** an instrument has no test in which a planted fault is reported
- **THEN** its section says not established for that item, and the printed table shows the gap

### Requirement: The instruments' sections are checked against the tests
A command SHALL check the instruments' file against the register: every test case a section names SHALL exist; a case
marked as expected to fail SHALL NOT count as proof; a case that is skipped in the regular run SHALL count only with the
date and the place of its last measured result. The command SHALL print, per instrument, which of the five items are
established, not established or not applicable, and SHALL exit non-zero when a named case is missing or does not count.

#### Scenario: A proof that was renamed away
- **WHEN** a test case named as an instrument's known-answer check is renamed or removed
- **THEN** the command fails and names the instrument, the item and the missing case

#### Scenario: A skipped measured set
- **WHEN** an instrument's smallest detectable effect is proven by a case that is skipped in the regular run, and the
  section gives the date and the file of its last result
- **THEN** the item counts as established and the table shows the date
