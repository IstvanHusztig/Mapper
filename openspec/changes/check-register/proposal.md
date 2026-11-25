# Proposal

## Why

What a test asserts, and how tightly, is today visible only in the test's source. With up to two editing agents and the
Codex agents at work, a loosened tolerance, a dropped check, a new `doctest::skip` or a removed `doctest::should_fail`
arrives inside a code diff that a reviewer has to spot (about 290 cases and 1,900 assertions in the C++ tests, 71 Python tool
tests, some dozens of `skip` and `should_fail` markers). The practice exists as a rule ("a tolerance is not changed after the values are seen; a marker goes only by the
user's decision"), not as a check.

The instruments have the same weakness one level up. A measurement is only as good as the tool that makes it, and on
2026-11-21 a suspected estimator fault (the loop world's lag) turned out to be mostly a fixture fault: the world used
other noise values than the run. Each instrument states its own preconditions, but nowhere is it written, per
instrument, what proves it right, how small an effect it can see, what it is blind to, and for which inputs it is valid.

The limits a run is judged by exist twice, in `src/domain/window_verdict.h` (`VerdictLimits`) and as defaults of
`tools/run_health.py`, with nothing tying the two together. And today's three removals showed that the compatibility of
old snapshots and captures (the odometry settings hash) is proven only by the step checks' replays, not by a test.

The user asked for this on 2026-11-21 (tests), 2026-11-22 (instruments) and again on 2026-11-25: limits for tests, runs
and instruments, in one place, checked by a tool. Source: R3 and R4 of `experiments/research/tests-as-data-alignment.md`.

## What Changes

- **The check register, part 1: the tests as data.** A new tool (`checks/check_register.py`) reads the test sources and
  writes one text file, `checks/tests.register`: per test case its file, name and markers, every assertion as one
  line, and the constants and case tables its limits come from. The file is committed and is the orchestrator's.
  `tools/step_checks.py` compares the sources against it and prints what differs by kind: case added or removed, marker
  added or removed, assertion added, removed or changed, limit changed. The Python tool tests are in it as well, and the
  step checks run them (they do not today).
- **A rule for register differences.** An added case or assertion is recorded by the orchestrator on the merge. A removed
  case or assertion, a changed limit and any marker change go to the user before the merge, with the reason. Only the
  orchestrator records the register, in its own commit.
- **The check register, part 2: the instruments.** One hand-kept file, `checks/instruments.toml`, with a section per
  instrument: the synthetic inertial simulator, the synthetic routes, the loop world, the truth statistics, the fixtures
  built on them, `honesty_report.py`, `trajectory_seams.py`, `wall_pairs.py`, `run_health.py`, the robustness sweep, the
  loop closure prototypes (`loop_graph.py`, `loop_fine_time.py`), the bit-identity kit (`compare_baseline.py`,
  `step_checks.py`) and `map_quality.py`. Each section states five things, each with the test case that proves it, or
  as `not established`, or as not applicable with the reason:
  1. the known-answer check and its tolerance;
  2. the smallest effect the instrument detects, with where that was measured;
  3. the blindness check: a planted fault it must report, and what it is known not to see;
  4. the range of inputs it is valid for, and whether an input outside it is detected;
  5. the precondition guards it applies to its own input.

  The tool checks that every case a section names exists in the tests part, is not marked as expected to fail, and, if
  it is skipped in the regular run, carries the date and place of its last measured result; it prints a table of what is
  established per instrument. A gap is a visible line, not an omission. A survey of 2026-11-25 found most of the five
  already present for most instruments (the truth statistics have all of them, with a measured power of 92 to 94 %);
  the file records them with their cases.
- **The gaps this change closes** (the others stay listed as `not established`):
  - the simulator has no planted fault: nothing shows that its noise check would reject a wrong noise (the fault of
    2026-11-21 was of this kind), and its smallest detectable noise ratio is not stated;
  - `run_health.py` is tested on its parsing and its missing-evidence rows, but not on a log whose bends, speeds, bias
    and chi-square are known;
  - `compare_baseline.py` has no test at all: neither that equal folders compare equal with the run-dependent fields
    normalised, nor that one changed byte is reported;
  - the blindness precondition of the loop chain and window covariance fixtures reads the run's own solved poses along
    their travel direction, while the loop world (the user's decision of 2026-11-21) measures truth-placed points along
    the true corridor axis (`tests/loop_chains_test.cpp:536-556`, `tests/window_covariance_test.cpp:429-440`);
  - the step checks do not say when their full run stopped windows at the time limit, so a difference caused by the
    machine's load reads like a changed number (it happened on 2026-11-21).
- **One source for the run's verdict limits.** Every run and replay logs the limits it judged with (one line). 
  `tools/run_health.py` takes its limits from the log it reads and says so; for a log without the line it uses its
  defaults and says that. A unit test reads the defaults of `VerdictLimits` from the header and requires
  `run_health.py`'s defaults to equal them.
- **The settings hash is pinned by a test.** The unit tests hold the odometry settings hash of the built-in defaults and
  of the two shipped settings files as literals. A removal or a new setting that would refuse every existing snapshot and
  capture fails this test at once, with the new value printed.
- **`turn_rate_threshold_deg_s` is retired at 30.0**, the only value the settings hash can vouch for, so an old file with
  another value is refused with the retired-setting message at load, not later with a hash mismatch (master reviewer,
  2026-11-25).

Not in this change: the hash manifest and readable first difference of the guard references (R1), structured result
lines from the truth tests (R2), planted faults in the C++ code built in a scratch tree (R5), any change to a
tolerance or marker of an existing test other than the loop chain's blindness definition, and any change to a result.

## Capabilities

### New Capabilities
- `check-register`: the committed register of test cases, markers, assertions and limits and its check; the instruments'
  sections and their check; the rule for register differences.

### Modified Capabilities
- `regression-baseline`: the step checks also run the Python tool tests and compare the test sources with the register.
- `verification-evidence`: a run logs the limits it was judged with; the log's judge takes them from the log.
- `unit-tests`: the settings hash of the defaults and of the shipped settings is pinned.
- `region-ownership`: a branch whose register difference removes or weakens a check is merged only after the user's
  decision.
- `processing-pipeline`: a retired setting whose old value entered the settings hash is accepted only at that value.

## Impact

- New: `checks/check_register.py`, `checks/tests.register`, `checks/instruments.toml`, `tests/test_check_register.py`.
- `tools/step_checks.py` (two more lines: the tool tests, the register), `tools/run_health.py` (limits from the log),
  `tools/README.md`.
- `src/domain/window_verdict.cpp` (one log line with the limits; no result changes), `src/infrastructure/toml_io.cpp`
  (the retired value), `tests/odometry_snapshot_test.cpp` (the pins), `tests/loop_chains_test.cpp` and
  `tests/window_covariance_test.cpp` (the blindness precondition), `tests/synthetic_inertial_test.cpp` (the planted
  noise), tool tests in `tests/test_verification_tools.py`.
- Docs: `docs/regions/README.md` (the rule), `docs/regions/verification.md`, the region agents' definitions
  (`.claude/agents/region-*.md`: what a register difference in the hand-back means), `CLAUDE.md` (Testing, Regions),
  the guide's instruments chapter, the verify-run skill.
- Runs: the step checks and one end run at the close; the loop chain's and the window covariance's measured sets once
  (`--no-skip`). No baseline changes: no
  output file of a run changes (the log is not part of a result).
- Time: the step checks grow by the tool tests (48 s measured beside another job on 2026-11-25) and the register
  comparison (seconds; measured in the tasks).
