# Local environment and workflow maintenance

Audited 2026-11-12. Paths below describe this workstation, not portable defaults.

- Repository: `C:/Repo/dMapper`; C++20 with vendored dependencies.
- MSVC initialization: `C:/Program Files/Microsoft Visual Studio/18/Community/VC/Auxiliary/Build/vcvars64.bat`.
- VS CMake/CTest: `C:/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/`.
- VS Ninja: the sibling `Microsoft/CMake/Ninja/` directory.
- CloudCompare: `C:/Program Files/CloudCompare/CloudCompare.exe`.
- Recordings: `C:/Users/User/Downloads/dLidar/raw data/`.
- Reference plan: `C:/Users/User/Downloads/dLidar/reference/haus47_LG_plan.pdf`.
- Forbidden processing source: `C:/Users/User/Downloads/dLidar/unsynced_archive/`.
- Python imports verified: numpy, scipy, matplotlib, laspy, lazrs, PIL, pymupdf.
  Open3D is absent and only needed for future flythrough work.
- OpenSpec 1.13.2; Codex CLI 0.167.0-alpha.4.1; Claude Code 2.1.322.

PowerShell example (initialization and build run in one child process):

```powershell
cmd.exe /d /c 'call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --build --preset x64-Release --target lidar_odometry_tests'
```

Configure first with `cmake --preset x64-Release` if needed, then build/test using
the commands in CLAUDE.md. Do not add build trees. Numerical `/fp:fast`
experiments are the documented explicit scratch-worktree exception.

Both agents need suitable runtime permission for external recording caches and
result folders before processing. Do not broaden sandbox settings globally.
No project Claude MCP server or hook configuration was found. Codex's existing
global plugins and MCP configuration are retained. Browser/document workflows
use each agent's native integrations; unrelated personal agents are not ported.

## Maintaining parity

OpenSpec owns the six generated skills in each agent tree and the six Claude
commands. Initialize/refresh both with:

```powershell
openspec init --tools 'claude,codex' --profile core --no-animation
```

Quote the comma-separated argument in PowerShell. This initializer may update
OpenSpec's user-level config and require sandbox approval. Review the generated
diff. Never use `--force` casually. Do not hand-edit generated workflows; put
project constraints in shared instructions or OpenSpec project context.

The four verify skills are maintained copies: update both `.claude/skills/` and
`.agents/skills/`, then run `python tools/check_agent_setup.py`. Keep their draft
thresholds in sync with tools and the retune plan only after user agreement.

Fresh-session acceptance prompts for each agent:

1. Without editing, list your project instructions, shared preferences, all ten
   project skills and how to invoke each of the six OpenSpec workflows.
2. Use OpenSpec explore to list changes and identify the root without creating
   artifacts. Confirm it is this repository and do not auto-select among changes.
3. Delegate a read-only master-reviewer check of a named commit range; verify
   the report and that no files, builds or runs were produced.

Run these in newly opened sessions so discovery is tested rather than inferred
from this conversation. Static parity checks do not prove fresh-session loading.
For simultaneous edits use separate worktrees; assign recording cache, result
folder and executable ownership separately. No auto-push or automatic git hook
is installed; review-after-push is an agent workflow instruction.
OpenSpec has one logical source of truth, but each worktree has its own files.
Coordinate and merge changes to specs/tasks; worktrees do not share live edits.

Official references: [skills](https://learn.chatgpt.com/docs/build-skills),
[AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md),
[custom agents](https://learn.chatgpt.com/docs/agent-configuration/subagents).
