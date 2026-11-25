# Tasks

Who does what: the verification region's agent writes the tool, its tests and the test cases of group 3 on branches of
its own (Opus, medium effort); the orchestrator records the register, writes `checks/instruments.toml`, and merges.
Release builds only. No result of a run may change: every merge is followed by `tools/step_checks.py` on the main tree.
A register difference of kind "change" on any branch of this change, other than the one named in 3.4, stops the branch
and goes to the user.

## 1. The register of the tests

- [x] 1.1 Decide where the tool lives: a file entry under `[orchestrator]` beside verification's `tools/**`, or
      `checks/`. (2026-11-25: `region_owners.py` reports a file named by two regions as a problem
      (`tools/region_owners.py:52-53`), so a file entry cannot override the glob. The tool is
      `checks/check_register.py` and its test `tests/test_check_register.py`; neither is named in `owners.toml`, so both
      are the orchestrator's. The paths in the proposal, design and tasks are corrected.)
- [x] 1.2 The reader (design D2) and the writer (D3) for the C++ tests: cases, markers, assertions, asserting helpers
      and their calls, values, `#if 0`, other conditionals, subcase prefixes; failure with file and line on what it
      cannot match; the cross-count per file. Done when `tests/test_check_register.py` passes its reader cases (each
      construct once, on sources held in the test) and the reader runs over the real `tests/` with the cross-count
      holding in every file and two runs giving the same bytes.
      (2026-11-25: merged 92a1c38; tests/test_check_register.py 87 cases pass; the Python discover run 158 tests OK.)
- [x] 1.3 The Python tests in the register (`ast`: methods, skip and expected-failure decorators, `self.assert*` calls,
      literal assignments). Done when the three files' 71 methods are listed and a planted `@unittest.skip` and a
      changed `places=` argument are each reported by the test.
      (2026-11-25: merged 92a1c38; tests/test_check_register.py 87 cases pass; the Python discover run 158 tests OK.)
- [x] 1.4 The comparison and its kinds (D4), exit codes 0 / 3 / 1 / 2, output with the old and the new line, changes
      first. Done when the test plants every edit of the spec's list (tolerance in an assertion, in a named value, in a
      case table row, in a helper's argument; assertion deleted; assertion commented out; `#if 0` and `#ifdef` round an
      assertion; skip added; should_fail removed; case renamed; case moved; assertion reworded) and requires the kind
      and the exit code of each.
      (2026-11-25: merged 92a1c38; tests/test_check_register.py 87 cases pass; the Python discover run 158 tests OK.)
- [x] 1.5 `--record`: refuses with uncommitted changes under `tests/`, and outside the main working tree without
      `--allow-worktree`. Done when the test shows both refusals in a temporary git repository and a record in a clean
      one.
      (2026-11-25: merged 92a1c38; tests/test_check_register.py 87 cases pass; the Python discover run 158 tests OK.)
- [x] 1.6 The orchestrator records the first register from master, reads it against the survey's counts (about 290
      cases, 12 `should_fail`, 24 to 26 `skip`, about 1,940 assertions; `LoopTolerances`' members and the two case
      tables present as values; the six helpers' calls present), and commits `checks/tests.register` in its own commit.
      Done when `python checks/check_register.py` exits 0 on that commit and the totals line matches an independent
      count (`grep`) written into this task.
      (2026-11-25: recorded from 92a1c38. Totals: 292 cases (7 should_fail, 25 skip), 1,933 assertions, 158 Python tests, 996 C++ values, 101 helper calls; grep: 292 TEST_CASE, 7 should_fail and 25 skip on code lines, the macro counts equal per macro. The design's 12 should_fail / 1,939 were raw greps that counted comments. LoopTolerances' 17 members and both case tables are values.)
- [x] 1.7 `tools/step_checks.py`: the lines `tool tests` and `check register` (D5: additions hold in a region's
      worktree and fail in the main tree; a change fails everywhere), and the time-limit count on the full run's line.
      Done when, in a scratch branch in a region worktree, an added case gives `HOLDS, to record: +1 case`, a loosened
      literal gives `FAILED: 1 assertion changed`, the same added case fails in the main tree, and the step checks on
      master print both new lines as passed and identical; the added wall time is written here (expected under one
      minute).
      (2026-11-25: merged; scratch proofs: an added case gives `HOLDS, to record: +1 case, +1 assertion`, a 0.02 loosened
      to 0.2 gives `FAILED: 1 assertion changed`; the main-tree verdict is unit-tested. Full run on the branch: ALL HOLD.
      Added wall time: tool tests 33 s, register 1.4 s. Register recorded after the merge: +10 cases, +21 assertions.)
- [x] 1.8 Docs for the register: `tools/README.md` (the tool, its kinds, what it does not see), `docs/regions/README.md`
      (the rule for a register difference and who records), the eight region agents' definitions
      (`.claude/agents/region-*.md`: quote the register's line in the hand-back; never record), `CLAUDE.md` (Testing,
      Regions), `docs/regions/verification.md` (the kit's "no unit test" line corrected; the register under Owns and
      How to check). Done when `python tools/build_docs_index.py --check` holds and each named file states the rule in
      the words of the spec.
      (2026-11-25: rule 10 in the seven region agents and rule 7 in the verification agent, the hand-back quotes the register
      line; docs/regions/README.md, CLAUDE.md (kit, Check register), tools/README.md (the tool, kinds, exit codes, blind
      spots, reference reading), docs/regions/verification.md (kit, How to check). build_docs_index --check holds;
      check_agent_setup PASS.)
## 2. The instruments' file

- [x] 2.1 The schema and its check (D7): `check_register.py --instruments` and as part of the plain comparison; the
      printed table; exit 1 on a missing case, a `should_fail` case as proof, a skipped case without `measured`. Done
      when the test holds a small instruments file with one of each fault and requires the message for each, and a
      correct one prints the table.
      (2026-11-25: merged 92a1c38; InstrumentsTest covers each fault and the table.)
- [x] 2.2 `checks/instruments.toml` written by the orchestrator from the survey of 2026-11-25 (design D7's list): every
      case name copied from the register, every tolerance and number from the source line; `not_established` and
      `not_applicable` where the survey found nothing, with the reason; the register tool as an instrument of its own.
      Done when `python checks/check_register.py` exits 0 with the file in place and the printed table is pasted into
      this task.
      (2026-11-25: 15 instruments incl. the register tool itself; every named case resolves. Table: known answer not established for run_health, the bit-identity kit and map_quality; smallest effect not established for the simulator, routes and loop world; planted fault not established for the simulator, the kit and map_quality; the rest established or stated.)
- [ ] 2.3 Docs: the guide's instruments chapter (`docs/index.html#instruments`) says where each instrument's proof is
      listed and what the five items mean, in general words (no recording names); `docs/regions/verification.md` points
      to the file instead of keeping its own partial lists. Done when `python tools/build_docs_index.py --check` holds.

## 3. The gaps that are closed (design D10; each requirement is stated there and is not changed after a run)

- [x] 3.1 Simulator, planted noise (`tests/synthetic_inertial_test.cpp`): factors 1.5 and 1/1.5 required rejected in
      every run and axis; 1.05 and 1/1.05 measured; the random walks at 1.5 required rejected in at least 4 of 5 seeds;
      the 90 % power ratio computed. Done when the cases pass in `ctest` (or are `skip` with their measured result
      recorded, if they take more than a few seconds), and the measured counts and the ratio are written here.
      (2026-11-25, merged: white noise at 1.5 and 1/1.5 rejected on every axis in every run (60 of 60); measured: 1.05 and
      1/1.05 rejected in 15 of 15 seed-axes, gyro and accelerometer; the 90 % power ratio at 80,000 samples and alpha 1e-4
      is 1.013 (expected about 1.02); random walks at 1.5 rejected in 5 of 5 seeds, each statistic. Release run times: the
      smoke case 1.7 s in ctest, the skipped sets 8.4 + 8.4 + 4.2 s (`-tc="synthetic IMU: planted*" --no-skip`).)
- [x] 3.2 `run_health.py` on a log with known readings (`tests/test_verification_tools.py`): the planted bend, speed,
      bias and chi-square on both sides of each limit. Done when the cases pass and one of them fails when a limit in
      `run_health.py` is changed by hand (tried once, reverted, noted here).
      (2026-11-25, merged: every reading and verdict exact; just below the limits every row CLEAR. One 4 deg bend in 200
      windows is 0.5 %, under the 1 % share: CLEAR; the third bent window triggers. Changing the --max-speed default by hand
      made two tests error (the row label carries the limit), reverted.)
- [x] 3.3 `compare_baseline.py` on folders written by the test: equal up to the run-dependent fields; one changed byte
      named; a manifest written and compared. Done when the three cases pass.
      (2026-11-25, merged: run-dependent fields normalised, IDENTICAL 5 files; one flipped byte named, 1 of 5, exit 1; a
      changed setting not normalised away; the manifest round trip IDENTICAL.)
- [ ] 3.4 The fixtures' blindness precondition against the truth (`tests/loop_chains_test.cpp`,
      `tests/window_covariance_test.cpp`, the shared helper beside `tests/synthetic_loop.cpp`'s): the measured sets run
      once before the change (`-tc="loop chain*" --no-skip -s`, `-tc="window covariance*" --no-skip -s`; logs kept in
      the experiment folder), the definition changed with the loop world's tolerances unchanged, the sets run again.
      Done when a table of every run's validity and of the old and new precondition values, before and after, is in
      the experiment folder and summarised here, and no validity verdict differs. **If one differs, or a tolerance would
      have to move: stop, numbers to the user.** The register's difference of this branch (changed assertions, removed
      values) is quoted here; it is the change the user accepted with this proposal.
- [x] 3.5 R2's sensor range (`tests/synthetic_routes.cpp`): another range than the documented one is refused with a
      message; a test case shows the refusal. Done when the case passes and the existing route cases are unchanged in
      the register.
      (2026-11-25, merged: ranges 5.9, 8 and 100 refused with "route R2 needs SensorProfile::max_range_m = 6"; the other
      routes unchanged. Step checks on the branch ALL HOLD, register +12 cases, +50 assertions, +24 values, recorded.)
- [ ] 3.6 After 3.1 to 3.5 are merged: `checks/instruments.toml` updated (the closed items name their new cases, the
      simulator's resolution carries its measured numbers), the register recorded. Done when
      `python checks/check_register.py` exits 0 and the table shows no `not established` item among those D10 closes.

## 4. One source for the verdict limits

- [x] 4.1 The limits line in `log_run_verdict` (`src/domain/window_verdict.cpp`, the solver's file), first of its
      block, in every run, replay and cancelled run; a case in `tests/window_verdict_test.cpp` on the line's text. Done
      when the case passes and the step checks on the branch are identical everywhere (a log line changes no result).
      (2026-11-25: merged; the line is first of the block in run, cancelled run and replay; quoted from a replay log:
      `window verdict limits: bend 3 deg, severe bend 10 deg, speed 2.5 m/s, gyro bias 0.5 deg/s, bent share 0.01, chi2 median 5, chi2 p95 20, weak ratio 0.45`.
      Step checks ALL HOLD; tools parse only `window verdict summary:` lines.)
- [x] 4.2 `tools/run_health.py` takes its limits from the line (D8's precedence), prints where they came from, and uses
      them where 3, 10 and 0.45 are literals today. Done when tool tests show the three sources (log, defaults, command
      line) and a log with other limits in the line is judged by those.
      (2026-11-25, merged: precedence command line > the log's line > defaults; a `limits` row says which; bend 3/10 and
      the weak ratio 0.45 come from the limits; the last of several lines counts; a malformed one is an error.)
- [x] 4.3 The test that reads `VerdictLimits` from `src/domain/window_verdict.h` and requires the tool's defaults to
      equal it; it fails when the struct or a member is not found. Done when it passes, and fails when one default in
      `run_health.py` is changed by hand (tried once, reverted, noted here).
      (2026-11-25, merged: RunHealthLimitsTests, 7 cases; speed_mps changed by hand to 2.6 failed with "run_health.py's
      default for speed_mps is 2.6 but VerdictLimits ... has 2.5", reverted. Step checks ALL HOLD; register +7 cases,
      +32 assertions, recorded. Not measured: run_health on a real log of the new build with diagnostics.)
- [x] 4.4 Docs: `tools/README.md` (run_health), the verify-run skill (`.claude/skills/verify-run/SKILL.md` and its
      `.agents` copy: the limits are the run's own, printed in the report), `CLAUDE.md` (Window verdict),
      `docs/regions/solver.md` and `verification.md`; the instruments' file's `run_health` section. Done when
      `python tools/build_docs_index.py --check` holds and `python checks/check_register.py` exits 0 after the register
      is recorded.
      (2026-11-25: tools/README.md, the verify-run skill (both copies, check_agent_setup PASS), CLAUDE.md (Window
      verdict), docs/regions/solver.md (by the agent, merged) and verification.md, the instruments file's run_health
      guards; --check holds; the register identical.)
## 5. The settings hash and the retired threshold

- [x] 5.1 The three pins (`tests/odometry_snapshot_test.cpp`, the back-end's file): the literals computed on master
      before any other task of this group; the failure message prints the new value and what a change means. Done when
      the case passes on master, and fails with the message when a setting is added to `TomlIO::POINTERS` without the
      `added_later` entry (tried once in the worktree, reverted, noted here).
      (2026-11-25: merged; literals defaults 0x1ad4c1d5688ff94a, l2-vertical 0xf86ccbfcd39e2f17, livox-vertical 0x6342caaee7fafd53;
      a dummy setting without an added_later entry made all three fail with the message, reverted.)
- [x] 5.2 `turn_rate_threshold_deg_s` retired at 30.0 (`src/infrastructure/toml_io.cpp`), with a line in
      `tests/toml_golden_test.cpp` (30 loads, 45 is refused and the message names 30). Done when the case passes and the
      step checks on the branch are identical everywhere.
      (2026-11-25: merged; the refusal message prints 30 (shortest form); step checks ALL HOLD.)
- [x] 5.3 Docs: `docs/regions/back-end.md` (the pins: when a literal may move), the guide's settings text on retired
      settings. Done when `python tools/build_docs_index.py --check` holds.
      (2026-11-25: back-end.md entry from the agent (merged), the guide's settings text on retired settings; --check holds.)
## 6. Close

- [ ] 6.1 `tools/step_checks.py` on master: every line identical or passed, the register identical. `tools/end_runs.py`
      once (full 44, Livox 3, 44_short; about 50 minutes, background): all three IDENTICAL against their manifests. Done
      when both outputs are quoted here, with the configurations that were run (Release only).
- [ ] 6.2 The master reviewer on the push; `openspec validate "check-register" --strict`; archive. Done when the
      reviewer's findings are answered and the change is archived.
