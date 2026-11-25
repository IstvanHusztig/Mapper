# Design

## Context

See proposal.md for the motivation. What shapes the approach:

- **The tests are homogeneous.** 47 C++ files (19,400 lines) use only `TEST_CASE` (about 290, with 63 `SUBCASE`s inside
  them) and the assertion macros `CHECK` (1,495), `REQUIRE` (327), `CHECK_FALSE` (104) and a handful of others. Markers
  are doctest decorators after the name (`* doctest::skip()`, `* doctest::should_fail()`), sometimes on the next line.
  There is one `#ifdef NDEBUG` in two files and no test templates. Three Python files hold 71 `unittest` methods.
- **Where the limits live** (survey of 2026-11-25): about 720 assertions carry a numeric literal; 82 use
  `Approx(...).epsilon(x)`; named limits are rare but important (`LoopTolerances` in `tests/synthetic_loop.h:123-169`
  holds the loop world's preconditions as member defaults; `alpha` constants in the truth tests); two case tables
  (`truth_statistics_test.cpp:210`, `scan_to_map_test.cpp:281`); six helpers that assert with a tolerance argument
  (`inside`, `inside_interval`, `check_gradient`, `check_acc_gradient`, ...) and one that records preconditions
  (`add(run, name, window, value, tolerance, at_least)` in `synthetic_loop.cpp:218`).
- **Ownership.** `tools/**` and the truth tests are the verification region's; a file no entry of
  `docs/regions/owners.toml` names is the orchestrator's. The guard references are recorded "only by the orchestrator"
  by rule; they lie outside git.
- **The step checks** take about 7 minutes, print one line per check, and are run by agents in their worktree and by the
  orchestrator on the main tree after each merge. Today's five removals each ended in "FAILED: 44_short" for an
  expected difference of the parameters file: a failure line that is expected every time stops being read.
- **Pre-registration** (consistency-harness D6): a tolerance is stated before the values are seen; where one was set
  after, the source says so (seven places, e.g. `loop_chains_test.cpp:556` "set after measuring 0.0072").

## Goals / Non-Goals

**Goals:**
- A weakened or removed check is a classified line in a command's output and in a git diff of one file.
- Each instrument's proof, resolution, blindness, range and guards are written down, checked for existence, and its gaps
  are visible.
- One place decides the limits a run is judged by; the settings hash cannot change unnoticed.
- No result of a run changes; the step checks stay near their present duration.

**Non-Goals:**
- Judging whether a changed tolerance is looser or tighter, or whether a test is a good test. The tool shows the old and
  the new line; the reading is a person's.
- Parsing C++ in general. The tool reads the forms the tests use and fails on what it cannot read.
- Converting tests into data files, a scenario language, CI or commit hooks (rejected in the source note, sections 2.2,
  2.3, 2.13).
- R1, R2 and R5 of the source note; proofs for `map_quality.py` (tied to recording 44's constants).

## Decisions

### D1 The register is generated from the sources, not from the test executable

`checks/check_register.py` reads `tests/*.cpp`, `tests/*.h` and `tests/test_*.py`. It needs no build, runs in seconds, and
sees what the executable cannot tell: the assertions' text and the limits.

- *Alternative: `lidar_odometry_tests --list-test-cases`.* Gives names only, needs the build, and says nothing of
  markers on skipped cases or of limits.
- *Alternative: a hash per case body.* Flags every edit with no way to tell a renamed variable from a loosened limit;
  the register would say "changed" and nothing more.
- *Alternative: only the lines with a numeric literal.* Misses a deleted assertion without a literal, and a comparison
  turned round.

### D2 What is read, and how

A small reader of the C++ token level (not line patterns): it strips comments, keeps string and character literals
whole (raw strings too), and matches brackets. From that:

- **A case:** `TEST_CASE(` name `[* decorators] )` and the braces that follow. The name is the string literals joined;
  the markers are the decorators' names (`skip`, `should_fail`, and any other, kept as written).
- **An assertion:** a call of a macro from a fixed list (`CHECK`, `CHECK_FALSE`, `CHECK_MESSAGE`, `CHECK_NOTHROW`,
  `REQUIRE`, `REQUIRE_FALSE`, `REQUIRE_MESSAGE`, `WARN_MESSAGE`, and any other doctest macro beginning with `CHECK_`,
  `REQUIRE_` or `WARN_`) with its balanced argument, white space collapsed to one blank.
- **An asserting helper:** a function defined in `tests/` whose body holds an assertion, or calls such a function
  (closed over calls; found in a first pass over all files). Its own assertions are listed under its file; every call
  of it inside a case is a line of that case, with its arguments.
- **A value:** outside function bodies, every declaration with an initialiser that holds a numeric literal (constants,
  member defaults such as `LoopTolerances`, case tables: the whole initialiser); inside a case or helper, every `const`
  or `constexpr` declaration with an initialiser.
- **Python:** with `ast`: each `test_*` method of a `TestCase` class; markers from `unittest.skip*` and
  `expectedFailure` decorators; assertions are the `self.assert*` and `self.fail` calls (`ast.unparse`); module-level
  and class-level assignments of literals are values.
- **`#if 0` ... `#endif`** is removed like a comment. Any other preprocessor conditional inside a test file is listed
  as a line of its own (`conditional: #ifdef NDEBUG`), so that a guard put round an assertion is seen.
- **`SUBCASE`s** are not entries; their assertions are lines of their case, prefixed with the subcase's name.

The reader fails (exit 2, file and line) on an unmatched bracket or an unterminated literal. After reading, it counts
the macro names again with a plain search of the comment-free text and fails if the two counts differ in any file.

### D3 The register's form

One text file, `checks/tests.register`, sorted by file and by position in the file, one fact per line, so that a git
diff of it reads without the tool:

```
# check register of the tests. Written by checks/check_register.py --record; do not edit.
# Not seen: limits computed while a test runs, assertions the flow never reaches, limits in src/.
# totals: 292 cases (24 skip, 12 should_fail), 1939 assertions, 71 python tests

[file] tests/loop_chains_test.cpp
  value: const double jitter_m = 0.01
[case] tests/loop_chains_test.cpp :: loop chain smoke: the corridor's fixture is valid (blindness, injected error retained, no rebuild)
  markers: -
  assert: CHECK(worst_along_ratio < 1e-3)
  assert: CHECK(worst_raw_share < 0.02)
  call: inside(stat, df, 0.01 / 3.0)
[case] tests/unit_cases_inertial_test.cpp :: unit case K3 marker: the start velocity prior's mean equals the true start velocity
  markers: should_fail
  assert: CHECK(...)
```

Lines of one case keep their order in the source. The size is about 3,000 lines.

- *Alternative: JSON or TOML.* Machine-friendly, but the diff a person reads is the point; the tool parses its own
  simple format back.
- *Alternative: inside `tools/baseline/`.* `tools/**` is the verification region's. A new top-level folder `checks/`
  is unnamed in `owners.toml` and therefore the orchestrator's; `region_owners.py --diff` then lists any branch that
  touches it.

### D4 Comparison and the kinds of difference

`check_register.py` without options regenerates in memory and compares with the committed file, case by case (key: the
name; a name that exists in exactly one other file with the same lines is `moved`), line by line within a case (a
longest-common-subsequence match of the lines):

| Kind | Meaning | Class |
|---|---|---|
| case added, assertion added, call added, value added | new lines | addition |
| case moved | same lines, other file (it can pick up that file's constants and helpers of the same name) | change |
| case removed, assertion removed, call removed, value removed | lines gone | change |
| assertion changed, call changed, value changed | a line replaced, numbers or operators differ | change |
| marker added, marker removed | | change |
| conditional added or removed | | change |

Exit codes: 0 no difference, 3 additions only, 1 a change, 2 the sources cannot be read. The output lists every
difference with the case and the old and new lines, changes first.

No automatic "looser / tighter": `x < 1e-3` to `x < 1e-2` is looser, `ratio > 0.9` to `ratio > 0.8` is looser as well,
and `CHECK(a == Approx(b).epsilon(e))` depends on which side moved. The tool does not guess.

### D5 Who records, and what the step checks do

- `check_register.py --record` writes the file; it refuses with uncommitted changes under `tests/` and, when not run
  in the main working tree, without `--allow-worktree` (the main tree is where `git rev-parse --git-dir` equals
  `--git-common-dir`). Agents do not record; this also means two branches never conflict in the register.
- `step_checks.py` gains two lines after the unit tests:
  - `tool tests`: `python -m unittest discover -s tests -p "test_*.py"`, passed or failed with the count (48 s measured
    beside another job; they are not run by the step checks today, which is how four of them stayed broken unnoticed
    until 2026-11-25);
  - `check register`: `IDENTICAL`; or `HOLDS, to record: +3 cases, +14 assertions` (exit 3, in a region's worktree);
    or `FAILED: to record ...` (exit 3 in the main tree); or `FAILED: 1 assertion changed, 1 marker removed (...)`.
    The instruments' check (D7) is part of the same line.
- The asymmetry is on purpose: additions need no decision, so they must not turn an agent's step checks red (see
  Context: an always-expected failure stops being read); on the main tree an unrecorded addition must not linger,
  because a case that was never recorded can be removed later without a trace.
- The 44_short line gains the count of windows stopped at the time limit (from the run's log, as `run_replay.py`
  prints it); a difference with a count above zero reads `DIFFERENT (N windows at the time limit: the result depended
  on the machine's load; run again)`.

The rule for a difference (spec `region-ownership`) goes into `docs/regions/README.md` and into every region agent's
definition: the register's line of the hand-back is quoted as printed; a change needs its reason; an agent does not
record. The existing convention for an expected parameters-file difference stays as it is.

### D6 The register's own proof

`tests/test_check_register.py` (the orchestrator's, like the tool) holds small C++ and Python sources as strings and plants the edits of
the spec (tolerance in an assertion, in a named value, in a case table row, in a helper's argument; assertion deleted;
assertion commented out; `#if 0` round an assertion; `#ifdef` round an assertion; skip added; should_fail removed; case
renamed; case moved), and requires the kind and the exit code for each. It also runs the reader
over the real `tests/` folder and requires the cross-count of D2 to hold and the result to be stable over two runs.
The register's section of `checks/instruments.toml` names these cases: the register is an instrument like the others.

What it is blind to, stated in the header and in `tools/README.md`: a limit computed at run time from data; an
assertion inside a branch that is never taken; a loop that runs zero times; limits in `src/` (the verdict limits are
covered by D8, the settings hash by D9); the meaning of a test.

### D7 The instruments' file

`checks/instruments.toml`, written by hand by the orchestrator from the survey, one table per instrument:

```toml
[truth_statistics]
what = "normalised error, chi-square intervals, power: tests/truth_statistics.{h,cpp}"
known_answer = { cases = ["truth statistics: the quantiles against published tables"], tolerance = "5e-7 to 5e-4 per row" }
smallest_effect = { cases = ["truth statistics: the power of the interval test"], says = "variance ratio 1.5 at 200 dof, 1 %: power 0.92 / 0.94" }
planted_fault = { cases = ["exact control 4: ..."], says = "covariance x1.5 and /1.5 rejected, 160 of 200", not_seen = "dependent or non-Gaussian errors" }
valid_for = { says = "independent, centred, Gaussian errors", outside = "not detected" }
guards = { says = "throws on a covariance that is not positive definite", cases = [] }

[run_health]
smallest_effect = { not_applicable = "it applies fixed limits; it estimates nothing" }
known_answer = { not_established = "parsing only; no log with known bends, speeds, bias, chi-square" }
```

Each of the five keys holds `cases` (names as in the register) and free text, or `not_established`, or
`not_applicable`. For a case that is `skip` in the register the item needs `measured = "<date>, <file>"`.
`check_register.py --instruments` (and the plain comparison) checks the names, prints the table (instrument by item:
established with the number of cases, established by a skipped set with its date, NOT ESTABLISHED, n/a) and exits 1 on
a missing name, on a `should_fail` case given as proof, or on a skipped case without `measured`.

- *Alternative: a page of prose in the docs.* That is what exists in parts (`docs/regions/verification.md`,
  `tools/README.md` reference readings) and it cannot be checked: a renamed test leaves the sentence standing.
- *Alternative: generate the sections from tags in the tests.* The claim "this case proves that instrument" is a
  judgment; it belongs in a file a person signs, and the tool checks only that the case exists and runs.

The first content, from the survey (established unless said): simulator (planted fault and stated resolution: closed
by D10); routes (resolution not established; range stated, R2's range check closed by D10); loop world (resolution of
the class tables not established); truth statistics (all five; the range is not detected); fixtures (all five, the
blindness precondition's definition closed by D10); `honesty_report.py` (resolution n/a); `trajectory_seams.py` (all);
`wall_pairs.py` (all; blind to a gap its source map cannot show, documented); `run_health.py` (known answer closed by
D10; resolution n/a); `perturbation_sweep.py` (all); `loop_graph.py`, `loop_fine_time.py` (all); `compare_baseline.py`
(closed by D10), `step_checks.py` and `kit_lib.py` (known answer not established beyond `build_type_mismatch`; the
guards exist); `map_quality.py` (known answer not established; blind to a misplaced floor, documented; tied to one
recording, not detected).

### D8 One source for the verdict limits

The program is the source. `log_run_verdict` prints one more line, first of its block:

```
window verdict limits: bend 3 deg, severe bend 10 deg, speed 2.5 m/s, gyro bias 0.5 deg/s, bent share 0.01, chi2 median 5, chi2 p95 20, weak ratio 0.45
```

`run_health.py` reads it. Order of precedence: a `--max-*` option given on the command line; the log's line; the
defaults. Its output gains a row `limits: from the run's log | defaults (the log has no limits line) | command line`.
The 3 and 10 degrees and the 0.45 that are literals inside `analyse` today become the limits' values. A test in
`tests/test_verification_tools.py` reads `struct VerdictLimits { ... }` from `src/domain/window_verdict.h` (the
members' default initialisers), fails if the struct or a member is not found, and requires `run_health.py`'s defaults
to equal them; a second test feeds a log with other limits in the line and requires the rows to follow them.

- *Alternative: a data file read by both.* The domain layer reads no files, and a header generated at configure time
  adds a build step for eight numbers.
- *Alternative: the test only.* It ties the defaults, but a tool of today judging the log of a build with other limits
  would still judge silently with its own.

The log is not part of a result, so no baseline moves. `honesty_report.py` and the tools that read
`window verdict summary` lines are unaffected (another prefix); the tool tests confirm it.

### D9 The settings hash pins

In `tests/odometry_snapshot_test.cpp` (the back-end's file), one case: `odometry_settings_hash(OdometryConfig{})` and
the hash of `settings/l2-vertical.toml` and `settings/livox-vertical.toml` (loaded through `TomlIO` from `LO_REPO_DIR`,
as the golden TOML test does) against three literals, with a message that prints the new value and says what a change
means (every snapshot, window capture and loop closure capture made with that file is refused from now on). The
literals are computed once from master before any other change of this proposal and checked against a capture's own
stored hash (the Livox 3 step-check capture was made with a settings file of its own, so the check there is: its
stored hash equals `odometry_settings_hash` of its `settings.toml`, which the replay already requires).

`turn_rate_threshold_deg_s` moves from "any value" to 30.0 in `RETIRED_KEYS`, with a line in
`tests/toml_golden_test.cpp` (30 loads, 45 is refused).

### D10 The gaps that are closed, each with its numbers stated here, before any run

1. **Simulator, planted noise** (`tests/synthetic_inertial_test.cpp`, verification). The existing case tests about
   160 s of rest at alpha = 1e-4. New case, the same statistic, with the generator's white noise scaled by a factor the
   check does not know.
   - Factors 1.5 and 1/1.5 (the factor of the harness's exact control), 5 seeds each. **Required:** every run is
     rejected on every axis, gyro and accelerometer.
   - Factors 1.05 and 1/1.05, 5 seeds each. **Measured, not required:** the number of rejections, written into the
     instruments' file as the stated resolution, beside the sigma ratio the sample size detects with 90 % power
     computed with `truth_stats` (expected near 1.02 for the white noise).
   - The random walks (about 440 steps between windows): factor 1.5, 5 seeds. **Required:** rejected in at least 4 of
     the 5 seeds. If the measured count is lower, the number is reported and the requirement is not changed without the
     user.
2. **`run_health.py` on a known log** (`tests/test_verification_tools.py`, verification): a log of 200 windows written
   by the test with one window bent 4 degrees, one at 2.6 m/s, one bias at 0.6 deg/s, and chi-square values with a
   known median and 95th percentile; every reading and verdict required exactly, and the same log with each planted
   value just below its limit required CLEAR.
3. **`compare_baseline.py` on known folders** (same file): the three cases of the spec.
4. **The blindness precondition of the fixtures** (`tests/loop_chains_test.cpp:368-376, 487, 546-556`,
   `tests/window_covariance_test.cpp:429-440`, verification). Both use today the run's result table along
   `window_travel_direction` of the solved poses, and a raw share normalised by the trace. They take the loop world's
   definition instead (`tests/synthetic_loop.cpp:367-387`: points placed at the true poses against the truth-built map,
   the true axis, the share over the largest eigenvalue) through one shared helper, with the loop world's tolerances
   (`LoopTolerances::blind_max_projected_along = 1e-3`, `blind_max_along_share = 0.02`), and print the run's own reading
   beside it as a result. This is a change of definition after values were seen, like the loop world's on 2026-11-21,
   and is noted as such in the source. Expected: no validity verdict changes, because these fixtures walk straight along
   x without the gait, so the two directions nearly coincide (the survey's inference, to be measured). The measured sets
   run once before and once after (`-tc="loop chain*" --no-skip`, `-tc="window covariance*" --no-skip`); if any run's
   validity differs between the two definitions, or a precondition would need another tolerance, the agent stops and the
   numbers go to the user. The register shows this change as changed assertions and removed values: it is the one
   register change this proposal asks the user to accept in advance.
   **Outcome (2026-11-25):** the stop condition was met: the truth definition is stricter (window covariance case (b):
   103 / 106 of 271 runs invalid against 26 / 23); not adopted, the user kept the old definition; see tasks.md 3.4.
5. **R2's range** (`tests/synthetic_routes.cpp`, verification): the route is documented to need a sensor range of 6 m
   and nothing checks it; the generator refuses another range for R2 with a message.
6. **The step checks' time-limit count** (D5).

After these, the instruments' file is updated and the register recorded.

### D11 After the master reviewer (2026-11-25, push 82b559a..77c9a60)

The reviewer planted edits that weaken a test and still read as identical or as an addition. The reader was extended:
`FAIL` and `FAIL_CHECK` are assertions (the settings-hash pins of D9 use them and were invisible); a function object
that asserts is a helper, with its calls recorded; markers on a suite or a test class and `skipTest` calls count for
their cases, with their arguments as written; the bodies of asserting Python helpers and of `setUp` are listed; every
`#define` in a test file is a line; local values without `const`, the lists of range loops and the headers of counted
loops with a number are lines; a moved case is a change; a missing register fails everywhere; the class "reworded"
is gone. The chi-square's weak ratio in the window diagnostics comes from `VerdictLimits`, like `run_health.py`'s.

## Risks / Trade-offs

- [The reader misreads a construct and silently drops assertions] → the cross-count per file (D2), the planted edits
  (D6), failure on anything unmatched; the first recorded register is read by the orchestrator against the survey's
  counts before it is committed.
- [A noisy register: every test edit is a difference] → additions need no decision and do not fail in a worktree. Any
  change of an assertion's text is a change: a class "reworded" (the same numbers and operators) was built first and
  removed after the master reviewer showed weakenings it let through (an `abs` dropped, another variable, `CHECK(true)`).
  A renamed variable therefore reads as a change; the orchestrator may accept it with a line in the merge's message.
- [The tool that guards the tests could itself be weakened] → the tool (`checks/check_register.py`) and its test
  (`tests/test_check_register.py`) are named by no region in `owners.toml` and are therefore the orchestrator's; a
  branch that touches either is listed by `region_owners.py --diff`, and the test is in the register.
- [The step checks grow by under a minute] → measured in task 1.6; the tool tests could run only the fast files if the
  slow one (`test_loop_fine_time.py`) dominates, but a skipped test file is exactly what this change is against.
- [An agent's worktree run "HOLDS, to record" and the orchestrator forgets to record] → the main tree's step checks
  fail after the merge until it is recorded.
- [The instruments' file ages] → its names are checked on every step check; its prose is not. A section's text is
  reviewed when its cases change (the tool prints the sections whose named cases differ from the register).
- [The limits line changes what tools parse] → another prefix than the summary's; tool tests on a log with and without
  it.
- [Pins that fail on every deliberate settings change become a reflex to update] → the failure message says what the
  change costs; the pin of the defaults moves only when old snapshots are meant to be refused.

## Migration Plan

1. The tool and its tests, with nothing wired in; the register recorded from master; the step checks' two lines.
2. The instruments' file with what exists.
3. The gaps (D10), each a branch of the verification region, the register recorded after each merge.
4. The limits line and the pins (solver and back-end files; small branches or the orchestrator's own, with the regions'
   knowledge files updated).

Rollback: the two lines of the step checks can be removed without touching anything else; the register and the
instruments' file are data.

## Open Questions

- Whether the class tables of the loop world can be given a stated power (not planned here; the item stays
  `not established`).
