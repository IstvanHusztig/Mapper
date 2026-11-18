# Codex instructions for dMapper

Before working, read `CLAUDE.md` for project architecture, build/test requirements,
datasets and quality priorities. Also read `docs/agents/preferences.md` and
`docs/agents/handoff.md`. Read `docs/agents/environment.md` before builds or runs.
These shared documents apply to both agents; the preferences document reconciles
the older tool-installation wording in CLAUDE.md. Current user instructions take
precedence over remembered preferences and historical task state.

Before a step that needs a more capable model or higher reasoning effort, tell
the user which change would help and why, before starting the step. Do not claim
to change a model or effort setting that the runtime does not let you change.

## Codex adaptations

- Use PowerShell and `rg` for searches. Translate Claude's Read/Grep/Glob/Bash
  references to the available file/search/shell tools; they are not literal Codex
  tool names. Keep logs in files and show concise relevant output.
- Initialize MSVC using `vcvars64.bat` in the same `cmd.exe` process as CMake;
  environment changes in a child process do not initialize the parent PowerShell.
  Keep the two existing build presets. See the shared environment guide.
- Use `.agents/skills/` for Codex workflows. Do not copy Claude command files into
  `.codex/prompts` or assume `/opsx:*` is a Codex slash command.
- All agents share `openspec/` and the same specification/task state. Use the
  selected change's CLI status/instructions; do not guess from memory. Planning
  is not permission to implement, archive, commit or push.
  Separate worktrees have separate working copies: coordinate and merge artifact
  and task edits; they do not synchronize automatically across worktrees.
- Use separate worktrees for simultaneous editing. Never run two processing jobs
  on one recording folder (shared cache), nor rebuild an executable in use.
  Worktrees do not isolate external datasets. Preserve existing uncommitted work.
- The code is divided into regions with knowledge files (`docs/regions/`, ownership in
  `docs/regions/owners.toml`, checked by `tools/region_owners.py`). They bind Codex too: edit only what the task's
  region owns, and read its knowledge file first. The region agent definitions are Claude's
  (`.claude/agents/region-*.md`). The per-commit checks are `tools/step_checks.py`.
- After an authorized push to master, delegate review of the exact pushed range
  to `master-reviewer`. Before substantive causal conclusions/recommendations,
  delegate an independent evidence review, following the shared preferences.
  Never imply a reviewer was run when unavailable. Agent instructions alone do
  not enforce a separate read-only sandbox.
  The primary agent requests one independent review. Reviewers return findings
  directly and do not delegate review of their own reports.

## Workflow invocation

| Claude Code | Codex CLI/IDE skill |
|---|---|
| `/opsx:explore` | `$openspec-explore` |
| `/opsx:propose` | `$openspec-propose` |
| `/opsx:apply` | `$openspec-apply-change` |
| `/opsx:update` | `$openspec-update-change` |
| `/opsx:sync` | `$openspec-sync-specs` |
| `/opsx:archive` | `$openspec-archive-change` |

In the desktop app, select the skill or request it by name. `opsx update` revises
a change's planning artifacts; the `openspec update` CLI refreshes integrations.
The four verification skills are `$verify-recording`, `$verify-run`,
`$verify-map`, and `$verify-robustness`. New recording -> recording check;
result -> run check -> map check; candidate ranking -> robustness/noise floor.
Their evaluation contract was agreed on 2026-11-12; read
`openspec/changes/sensor-neutral-yardstick/criteria-proposal.md`. Legacy tool
defaults are not automatically compliant with that contract.

After changing verification skills, update both agent copies and run
`python tools/check_agent_setup.py`. OpenSpec skills are generated separately for
each agent by OpenSpec 1.13.2; keep their native invocation syntax.
