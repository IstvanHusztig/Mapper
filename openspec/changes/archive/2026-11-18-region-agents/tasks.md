# Tasks

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the repository. Result: plan subsection "Stage 7
  design", 2026-11-18 (0399a1e); the review's findings worked in.
- [x] 1.2 The user's decisions. Result: 2026-11-18, all nine as recommended, with the worktree flow in decision 4 and
  the step checks before and after each merge (8e1a399, f796d8e).

## 2. The verification kit

- [x] 2.1 `tools/local_paths.toml` (recordings, captures, the references' folder, `vcvars64.bat`) and
  `tools/step_checks.py` (build, unit tests, characterization guard, loop guard, the two replays, 44_short against its
  manifest, the per-term table of the 44_short replay; `--record`; runs on the worktree it is started in; prints the
  commit). The captures and references move to `experiments/guards/`. Verify: `--record` on the current commit gives
  references byte-identical with the stage 6 ones; a second run reports identical everywhere and exits 0; `--record`
  refuses with an uncommitted source change.
  Result: e17247e. The references recorded from that commit are byte-identical with the stage 6 ones (both guards, both
  table files); a second run reports identical everywhere and exits 0; `--record` on a tree with an uncommitted change
  in `src/` refuses and writes nothing. The captures are in `experiments/guards/`.
- [x] 2.2 `tools/end_runs.py` (full 44, Livox 3, 44_short against their manifests, odometry times; `--debug`; refuses
  while a dMapper runs; one line per run into a log). Verify: started in the background at the end of this change (task
  7.3), it reports IDENTICAL for the three recordings.
  Result: run as task 7.3: 44 IDENTICAL (35 files, odometry 909 s, wall 1413 s, peak 4.3 GB), Livox 3 IDENTICAL (35 files, odometry
  646 s, wall 1263 s, peak 4.4 GB), 44_short IDENTICAL (11 files, 98 s) at de51b4c; it held the gate and logged one line per run.
- [x] 2.4 `tools/run_replay.py`: the gate (memory need from the capture's size, two slots, held whole by `end_runs.py`,
  stale holders ignored; wall time, CPU time, peak memory, time-limit count; the replay's exit code). `step_checks.py`
  starts its replays through it. Verify: alone it starts at once and prints the cost line; a third replay waits while
  two run and starts after; a replay waits while a made-up full-run lock is held; a lock of a dead process is ignored.
  Result: tried on real replays of the 44_short capture: alone it starts at once (wall 28 s, CPU 65 s, peak 0.2 GB,
  time limit 0); a lock of a dead process is ignored and removed; behind a held full-run lock it waits and starts
  when the holder ends; of three at once the third waits for a slot, all three identical with the capture. Measured
  for the Livox 3 capture: 1.6 GB peak, 5.1 cores.
- [x] 2.3 `tools/README.md` sections for the four; `tests/test_verification_tools.py` gets a case for
  `step_checks.py`'s reading of the paths file if it fits its form. Verify: the text matches the tools.
  Result: `tools/README.md` has the kit (the three commands, the gate) and `region_owners.py`. No case added to
  `tests/test_verification_tools.py`: the kit's parts were tried on real runs instead (tasks 2.1 and 2.4).

## 3. Ownership

- [x] 3.1 `docs/regions/owners.toml` after the plan's table (files, `file::function` entries, shared test files) and
  `tools/region_owners.py` (the list's check; `--diff <base>..<branch> --region <name>`). Verify: the check exits 0 on
  the list; a made-up branch that changes `ndt_map.cpp` only passes for `map`, one that also changes `window.h` and an
  unowned function of `run_odometry.cpp` fails with both listed, a shared test file is listed without failing.
  Result: e17247e (list and tool), 883fd0d (`run_consistency` to the back-end, `log_accelerometer_fit` to the inertial
  region). The check: 9 owners, 135 files and 18 functions named, 0 problems. In a throw-away worktree two made-up
  branches: one changing `ndt_map.cpp`, `ndt_map_test.cpp`, `add_window_to_map` and a shared test file passes for
  `map` (exit 0, the shared file listed); one also changing `window.h` and `prepare_window` fails with both listed
  (exit 1). The main tree's HEAD and status were checked unchanged after the test.

## 4. Knowledge files

- [x] 4.1 `docs/regions/README.md`: the ownership table in words, the knowledge file's form, the agents' rules, the
  worktree flow, the protocol, the hand-back form.
  Result: `docs/regions/README.md` (883fd0d).
- [x] 4.2 The eight knowledge files (`front-end`, `map`, `lidar-matching`, `inertial`, `solver`, `back-end`,
  `noise-budget`, `verification`), seeded from `CLAUDE.md`, the tracker, `docs/agents/roadmap.md`, the archived OpenSpec
  changes and the session memory's findings; each with the top line and "See also". Verify: a Sonnet reviewer checks
  each file against the code (contract, files, flaws); its corrections are worked in; each file is under about 300
  lines.
  Result: 883fd0d. Drafted by four agents from the code and the written sources, each file then checked against the
  code by a Sonnet reviewer; their corrections are in (one reviewer claim was wrong and left out: `ndt_window_cost`
  does apply the information scale). 181 to 215 lines each. Two boundaries settled while reading the contracts:
  `run_consistency` is the back-end's and `log_accelerometer_fit` the inertial region's, by function entries.
- [x] 4.3 `docs/code-review-tracker.md`: the references of the findings that are not fixed, moot or rejected point to
  today's files. Verify: each cited file and function exists.
  Result: C2 partly fixed, C5, C6, C15 fixed (the region files found them fixed in the code); C13, C16, C17, C18, C31,
  C34 point to today's files; new rows C39 (weak-axis stiffness without the position scale), C40 (the rebuild always
  clears), C41 (test-only helpers, stale comments), C42 (`load_imu` column check, empty windows). Every file and
  function the open rows cite exists (checked by a script). The region files follow.
- [x] 4.4 Both docs maps (`docs/README.md`, the guide's "All documents") get the group "Region knowledge files".
  Verify: every new page is in both maps; the links resolve.
  Result: the group "Region knowledge files" (nine pages) in `docs/README.md` and the guide's "All documents"; every
  page has the top line and "See also"; every link of the region files resolves (checked by a script).

## 5. Worktrees and agent definitions

- [x] 5.1 The two standing worktrees (`C:/Repo/dMapper-worktrees/region-a`, `region-b`), each configured and built once
  with the `x64-Release` preset. Verify: `step_checks.py` run in each reports identical everywhere; the full build's time
  is noted.
  Result: both at 27d6873, each configured and built once (full build about 3 min, 311 steps) and checked:
  `step_checks.py` ALL HOLD in both (region-a 596 s, region-b 616 s in all, the full build included).
- [x] 5.2 `.claude/agents/region-<name>.md` for the eight regions (one text, the region's name and contract filled in;
  Opus, medium; Sonnet for verification) and `conclusion-reviewer.md` (Sonnet, read-only); `master-reviewer.md` gets the
  repo's present path. Verify: `tools/check_agent_setup.py` passes.
  Result: fcdc15c; the rules sharpened after the first starts (task 5.4). `check_agent_setup.py` passes. New definitions
  are read at a session's start: in the running session they were offered only after a while; until then the same
  text ran through a general agent with the definition's model (the design's fallback).
- [x] 5.3 `CLAUDE.md`: a "Regions" section (the list, the rules, the protocol in short, the kit) and the worktree
  exception to "do not add other build trees"; `AGENTS.md`: a pointer to `docs/regions/` and the ownership list.
  Verify: the text matches the files.
  Result: fcdc15c.
- [x] 5.5 The guide (`docs/index.html`) gets a section "Internals" (menu entry, linked from the opening paragraph):
  how the code is laid out (the layers, the pipeline, the window loop's four steps, the terms, the solver, the noise
  model, the verdict, capture and replay, the checks) and how the work is organised (the regions and what each owns,
  the agents and their models, worktrees and branches, the gate, the protocol from question to merge), with links to
  `ARCHITECTURE.md`, the plan, `docs/regions/` and the pages in depth. Verify: a Sonnet reviewer checks it against the
  code and the region files; every link resolves.
  Result: 09c8868: a section "Internals" (layers, one window in the code, how a change is checked, regions and agents,
  from a question to a merge), in the menu and the opening paragraph; a Sonnet reviewer found no false statement and
  two imprecise ones (what infrastructure and presentation may include), corrected, and three additions (building
  and testing, adding a setting, baselines in their own commit), made. Every link resolves.
- [x] 5.4 Each region agent is started once, read-only: it states its contract and its files and names what in its
  knowledge file it could not find in the code. Verify: every agent answered from its files; the corrections are in the
  knowledge files. (If a new definition is not offered in the running session: the same text through a general agent
  with the definition's model, and the fact noted.)
  Result: the eight agents, each started once read-only (Opus; verification Sonnet), stated their contract and files from
  their knowledge files and checked 14 to about 30 statements each against the code: map, lidar-matching,
  back-end, verification found none wrong; inertial, solver, noise-budget one or two (a wording, a stale comment
  worse than written, a second place the model is built, a stale open question); front-end four (acceleration
  rejection off for the whole run, two tracker remarks out of date, a duplicated note). Corrected. Their questions
  on the rules led to: a build-only command (`step_checks.py --build-only`), the capture paths and the `--diff` base
  in the README and the tasks, absolute paths (the shell keeps no working directory), the agent creates its
  experiment folder, "runs wanted" in the hand-back for code replays don't reach (front-end, back-end), sweeps and
  snapshot runs as the orchestrator's, `--owner` saying what a shared test file means.

## 6. Try-out

- [x] 6.1 The map region's first job: split `update_rgd_legacy` (84 lines) into steps without changing a bit, in a
  standing worktree on its own branch, with its hand-back and knowledge entry. Verify: `region_owners.py --diff` passes
  for `map`; the verification agent's `step_checks.py` in the worktree is identical everywhere; the conclusion reviewer
  has checked the hand-back.
  Result: the real `region-map` agent (new in the session by then) in `region-a` at 56bfa8b, branch
  `region/map/2026-11-18-split-update-rgd-legacy`, one commit d60e21f, `ndt_map.cpp` only: `update_rgd_legacy` 84 → 21
  lines (`new_legacy_cell`, `add_point_to_legacy_cell`, `add_to_legacy_cov`, `legacy_cell_normal`,
  `refresh_cell_inverses` shared with the exact path); no test added (file-local steps, no new mathematics). Its
  `--diff` check: 1 own file, 0 outside (mine the same). The verification agent ran `step_checks.py` on the committed
  branch: ALL HOLD; the map's unit tests 8/8 (they also cover the exact path, which no replay runs). The conclusion
  reviewer: every expression unchanged; its one correction (the agent's own log was made before the commit) settled
  by the verification run on the commit.
- [x] 6.2 The main agent merges the branch; `step_checks.py` on the main tree. Verify: identical everywhere;
  `tools/function_sizes.py` shows no function of `ndt_map.cpp` above 80 lines; the knowledge entry is in
  `docs/regions/map.md`.
  Result: merged f285eda (no-ff; changes no number, so on the orchestrator's judgment). `step_checks.py` on the main tree:
  ALL HOLD. `function_sizes.py`: no function of `ndt_map.cpp` at 80 lines or more. The knowledge entry is in
  `docs/regions/map.md` (Experiments, the line references moved, the split done, an open question on a test of the
  default statistics' view-point gate).
- [x] 6.3 What the try-out showed about the chain (definitions, hand-back form, timings, anything an agent got wrong)
  is written into `docs/regions/README.md` and the definitions. Verify: the user has the list.
  Result: the chain worked end to end: question, worktree, branch, hand-back, ownership check, verification, review,
  merge, step checks after the merge (about 10 min for the agent's job, 9 min for verification, 1 min for review).
  What it showed, now in `docs/regions/README.md` and the definitions: the step checks count for a commit only when
  run on it (the agent had run them before committing); long commands go to the background (the shell cannot
  `sleep`); the references' commit is older by design; `region_owners.py` takes repository-relative paths. Earlier,
  from the first starts: build-only command, capture paths, the `--diff` base, absolute paths, runs wanted for code
  replays don't reach. New definitions became available in the running session only after a while (the fallback
  covered the first starts).

## 7. End

- [x] 7.1 The plan's progress; the memory's model and effort table for stage 8 unchanged or corrected.
  Result: the plan's progress (de51b4c); the model and effort table unchanged for stage 8, with the user's line that guide and
  docs writing is Opus / medium.
- [x] 7.2 `tools/region_owners.py` and `tools/check_agent_setup.py` pass on the final tree.
  Result: `region_owners.py`: 0 problems; `check_agent_setup.py`: PASS (de51b4c).
- [x] 7.3 `tools/end_runs.py --debug` in the background (the try-out changed map code). Verify: full 44, Livox 3 and
  44_short IDENTICAL with their manifests; Debug `ctest` passes.
  Result: 44 IDENTICAL (35 files, odometry 909 s, wall 1413 s, peak 4.3 GB), Livox 3 IDENTICAL (35 files, odometry
  646 s, wall 1263 s, peak 4.4 GB), 44_short IDENTICAL (11 files, 98 s) at de51b4c. The Debug build and `ctest` were dropped (the user, 2026-11-18): the
  try-out only split a function into steps (no memory, thread or assert change), so `--debug` stays unproven
  until a later change needs it.
