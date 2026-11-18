# Design

## Context

The full design is the subsection "Stage 7 design" of `docs/code-split-and-region-agents-plan.md` (2026-11-18): the
ownership table, the verification kit, the knowledge file's form and the agent definition's rules, the worktree flow,
the protocol and the try-out. A Sonnet reviewer checked it against the repository; its findings are worked in (the
default owner, the shared test files, the flaws mapped by today's code, the hand-back checked against the ownership
list, knowledge entries through the orchestrator, the limits of what a definition can enforce). The worktree flow and
the checks after each merge came from the user (2026-11-18). This file keeps the decisions and the risks.

The user took all nine decisions as recommended, with the worktree flow in decision 4 (2026-11-18).

## Goals / Non-Goals

**Goals:**
- A cold-started agent per region that knows its contract, files, flaws and history from the repository alone.
- The per-commit and end-of-change checks as repository commands that run in any worktree.
- No unchecked or undecided change in the main tree.
- The chain tried once on an exactly checkable job.

**Non-Goals:**
- No region's first real job, and nothing on Livox 4 (stage 8).
- No Codex agent definitions (the ownership and the knowledge files bind Codex through `AGENTS.md`).
- No enforcement beyond what the tools can check: a definition cannot fence an agent into paths.

## Decisions

1. **Ownership in one checked list**, by file and by function in shared files; the unnamed rest is the orchestrator's.
2. **The noise-budget region owns the weights' formulas** (`window_noise_model`); a term's region owns its residual and
   Jacobian.
3. **Capture, replay, the loop's skeleton, shared types and settings are the orchestrator's; verification never edits
   `src/`.**
4. **Each editing agent in its own worktree on its own branch; the main agent decides and merges.** Two standing
   worktrees, at most two editing agents at once. Agents replay, never run full runs, never merge or push, never touch
   `settings/`. The main agent merges a branch that changes no number on its own judgment and one that changes a number
   only after the user's decision. Step checks run on the branch before the merge and on the main tree after it.
5. **The knowledge file's form; seeded from what is written; each checked by a reviewer; about 300 lines; agents hand in
   entries, the orchestrator writes the file.**
6. **The verification kit in the repo**, its references and captures outside it under `experiments/guards/`.
7. **`conclusion-reviewer` as a definition; `master-reviewer`'s path fixed; no batch-run agent.**
8. **The try-out: the map region splits `update_rgd_legacy`.**
9. **`docs/regions/` is a group of both docs maps, English only.**

## Risks / Trade-offs

- [A new agent definition is not offered in the running session] → the try-out and the read-only start of each agent
  show it; until a definition is listed, the same text is given to a general agent with the model the definition names.
- [An agent edits outside its region or in the main tree] → `region_owners.py --diff` on its branch before anything
  else; the main tree's `git status` before and after each job.
- [Two agents' replays disturb each other] → their own settings file names (the table files are named after them); the
  time-limit count of the verdict summary checked after a replay with other settings; at most two at once.
- [Two branches merged that were never tested together] → step checks on the main tree after each merge.
- [A standing worktree is stale] → reset to the main tree's commit before each job; `step_checks.py` prints the commit
  it ran on.
- [A knowledge file states something the code doesn't do] → a reviewer checks each seeded file against the code; each
  agent's read-only first start names what it could not find.
- [The knowledge files grow until a cold start costs too much] → the 300-line limit; an old experiment shrinks to one
  line and its evidence folder.
- [`local_paths.toml` holds one workstation's paths] → the same paths are already in `docs/agents/environment.md`; a
  second workstation changes one file.
- [The kit's references differ from the scratch ones used in stages 2 to 6] → the first recording is compared with the
  stage 6 references byte for byte.
