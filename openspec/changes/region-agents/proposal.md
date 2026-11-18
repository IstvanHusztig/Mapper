# Proposal

## Why

Stage 8 of `docs/code-split-and-region-agents-plan.md` hands a faulty stretch of Livox 4 to the part of the code its
readings point at. For that each part needs an owner that starts cold and still knows its contract, its files, its
known flaws and what was already tried; the checks of every commit must not live in a session's scratch folder; and a
change must not reach the main tree before it is checked and decided on. Stage 7 builds this.

## What Changes

- **Ownership list** `docs/regions/owners.toml`: the files of each of the eight regions, functions in shared files
  (`file::function`), the shared test files; everything else is the orchestrator's. `tools/region_owners.py` checks the
  list and, with `--diff`, what a branch touches outside its region.
- **Verification kit in the repo:** `tools/step_checks.py` (build, unit tests, both guards, both replays, 44_short, the
  per-term table; `--record` for the references), `tools/end_runs.py` (full 44, Livox 3, 44_short against their
  manifests; `--debug`), `tools/local_paths.toml` (this workstation's paths).
- **A knowledge file per region** (`docs/regions/<name>.md`, eight, and a `README.md`): contract, what it owns, how to
  check its work, known flaws (the tracker's open findings by today's code), functions to split in its first job,
  experiments, ideas rejected, open questions. Seeded from what is written, each checked against the code by a reviewer.
  The tracker's file references are brought up to date.
- **Agent definitions** `.claude/agents/region-<name>.md` (eight; Opus, medium; Sonnet for verification) and
  `conclusion-reviewer.md` (Sonnet, read-only); `master-reviewer.md` gets the repo's present path.
- **Worktrees:** two standing worktrees (`C:/Repo/dMapper-worktrees/region-a`, `region-b`). An editing agent works in one,
  on its own branch, and reports back; the main agent checks the branch (ownership, step checks, review), decides and
  merges, and runs the step checks again on the main tree after each merge. Agents run replays, never full runs.
- **The protocol** in `docs/regions/README.md` and `CLAUDE.md` (a "Regions" section and the worktree exception to "no
  other build trees"); a pointer in `AGENTS.md`.
- **A try-out:** the map region splits `update_rgd_legacy` (84 lines) without changing a bit, through the whole chain.

## Capabilities

### New Capabilities

- `region-ownership`: who owns each file and function, checked by a tool, also for a branch.

### Modified Capabilities

- `regression-baseline`: the per-commit checks and the end-of-change runs as commands of the repository.

## Impact

- **Code:** `src/domain/ndt_map.cpp` (the try-out's split, bit-identical). No other source change.
- **Tools and docs:** `tools/step_checks.py`, `end_runs.py`, `region_owners.py`, `local_paths.toml`, `tools/README.md`;
  `docs/regions/` (10 files), both docs maps, `docs/code-review-tracker.md`, `CLAUDE.md`, `AGENTS.md`;
  `.claude/agents/` (10 files).
- **Results:** none. The try-out is checked identical on the guards, replays, 44_short, the full 44 and Livox 3.
- **Disk:** two build trees of about 1.6 GB.
- **Not in this change:** any region's first real job, Livox 4 (stage 8), the long functions of the front-end and the
  back-end, Codex agent definitions.
