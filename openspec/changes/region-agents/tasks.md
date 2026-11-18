# Tasks

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the repository. Result: plan subsection "Stage 7
  design", 2026-11-18 (0399a1e); the review's findings worked in.
- [x] 1.2 The user's decisions. Result: 2026-11-18, all nine as recommended, with the worktree flow in decision 4 and
  the step checks before and after each merge (8e1a399, f796d8e).

## 2. The verification kit

- [ ] 2.1 `tools/local_paths.toml` (recordings, captures, the references' folder, `vcvars64.bat`) and
  `tools/step_checks.py` (build, unit tests, characterization guard, loop guard, the two replays, 44_short against its
  manifest, the per-term table of the 44_short replay; `--record`; runs on the worktree it is started in; prints the
  commit). The captures and references move to `experiments/guards/`. Verify: `--record` on the current commit gives
  references byte-identical with the stage 6 ones; a second run reports identical everywhere and exits 0; `--record`
  refuses with an uncommitted source change.
- [ ] 2.2 `tools/end_runs.py` (full 44, Livox 3, 44_short against their manifests, odometry times; `--debug`; refuses
  while a dMapper runs; one line per run into a log). Verify: started in the background at the end of this change (task
  7.3), it reports IDENTICAL for the three recordings.
- [ ] 2.4 `tools/run_replay.py`: the gate (memory need from the capture's size, two slots, held whole by `end_runs.py`,
  stale holders ignored; wall time, CPU time, peak memory, time-limit count; the replay's exit code). `step_checks.py`
  starts its replays through it. Verify: alone it starts at once and prints the cost line; a third replay waits while
  two run and starts after; a replay waits while a made-up full-run lock is held; a lock of a dead process is ignored.
- [ ] 2.3 `tools/README.md` sections for the four; `tests/test_verification_tools.py` gets a case for
  `step_checks.py`'s reading of the paths file if it fits its form. Verify: the text matches the tools.

## 3. Ownership

- [ ] 3.1 `docs/regions/owners.toml` after the plan's table (files, `file::function` entries, shared test files) and
  `tools/region_owners.py` (the list's check; `--diff <base>..<branch> --region <name>`). Verify: the check exits 0 on
  the list; a made-up branch that changes `ndt_map.cpp` only passes for `map`, one that also changes `window.h` and an
  unowned function of `run_odometry.cpp` fails with both listed, a shared test file is listed without failing.

## 4. Knowledge files

- [ ] 4.1 `docs/regions/README.md`: the ownership table in words, the knowledge file's form, the agents' rules, the
  worktree flow, the protocol, the hand-back form.
- [ ] 4.2 The eight knowledge files (`front-end`, `map`, `lidar-matching`, `inertial`, `solver`, `back-end`,
  `noise-budget`, `verification`), seeded from `CLAUDE.md`, the tracker, `docs/agents/roadmap.md`, the archived OpenSpec
  changes and the session memory's findings; each with the top line and "See also". Verify: a Sonnet reviewer checks
  each file against the code (contract, files, flaws); its corrections are worked in; each file is under about 300
  lines.
- [ ] 4.3 `docs/code-review-tracker.md`: the references of the findings that are not fixed, moot or rejected point to
  today's files. Verify: each cited file and function exists.
- [ ] 4.4 Both docs maps (`docs/README.md`, the guide's "All documents") get the group "Region knowledge files".
  Verify: every new page is in both maps; the links resolve.

## 5. Worktrees and agent definitions

- [ ] 5.1 The two standing worktrees (`C:/Repo/dMapper-worktrees/region-a`, `region-b`), each configured and built once
  with the `x64-Release` preset. Verify: `step_checks.py` run in each reports identical everywhere; the full build's time
  is noted.
- [ ] 5.2 `.claude/agents/region-<name>.md` for the eight regions (one text, the region's name and contract filled in;
  Opus, medium; Sonnet for verification) and `conclusion-reviewer.md` (Sonnet, read-only); `master-reviewer.md` gets the
  repo's present path. Verify: `tools/check_agent_setup.py` passes.
- [ ] 5.3 `CLAUDE.md`: a "Regions" section (the list, the rules, the protocol in short, the kit) and the worktree
  exception to "do not add other build trees"; `AGENTS.md`: a pointer to `docs/regions/` and the ownership list.
  Verify: the text matches the files.
- [ ] 5.5 The guide (`docs/index.html`) gets a section "Internals" (menu entry, linked from the opening paragraph):
  how the code is laid out (the layers, the pipeline, the window loop's four steps, the terms, the solver, the noise
  model, the verdict, capture and replay, the checks) and how the work is organised (the regions and what each owns,
  the agents and their models, worktrees and branches, the gate, the protocol from question to merge), with links to
  `ARCHITECTURE.md`, the plan, `docs/regions/` and the pages in depth. Verify: a Sonnet reviewer checks it against the
  code and the region files; every link resolves.
- [ ] 5.4 Each region agent is started once, read-only: it states its contract and its files and names what in its
  knowledge file it could not find in the code. Verify: every agent answered from its files; the corrections are in the
  knowledge files. (If a new definition is not offered in the running session: the same text through a general agent
  with the definition's model, and the fact noted.)

## 6. Try-out

- [ ] 6.1 The map region's first job: split `update_rgd_legacy` (84 lines) into steps without changing a bit, in a
  standing worktree on its own branch, with its hand-back and knowledge entry. Verify: `region_owners.py --diff` passes
  for `map`; the verification agent's `step_checks.py` in the worktree is identical everywhere; the conclusion reviewer
  has checked the hand-back.
- [ ] 6.2 The main agent merges the branch; `step_checks.py` on the main tree. Verify: identical everywhere;
  `tools/function_sizes.py` shows no function of `ndt_map.cpp` above 80 lines; the knowledge entry is in
  `docs/regions/map.md`.
- [ ] 6.3 What the try-out showed about the chain (definitions, hand-back form, timings, anything an agent got wrong)
  is written into `docs/regions/README.md` and the definitions. Verify: the user has the list.

## 7. End

- [ ] 7.1 The plan's progress; the memory's model and effort table for stage 8 unchanged or corrected.
- [ ] 7.2 `tools/region_owners.py` and `tools/check_agent_setup.py` pass on the final tree.
- [ ] 7.3 `tools/end_runs.py --debug` in the background (the try-out changed map code). Verify: full 44, Livox 3 and
  44_short IDENTICAL with their manifests; Debug `ctest` passes.
