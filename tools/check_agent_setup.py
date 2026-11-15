"""Check local Claude/Codex workflow parity without writing files or running agents."""

from pathlib import Path
import re
import sys
import tomllib


ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = {
    "explore": "openspec-explore",
    "propose": "openspec-propose",
    "apply": "openspec-apply-change",
    "update": "openspec-update-change",
    "sync": "openspec-sync-specs",
    "archive": "openspec-archive-change",
}
VERIFY = ("verify-recording", "verify-run", "verify-map", "verify-robustness")


def main():
    errors = []
    for path in ("AGENTS.md", "CLAUDE.md", "openspec/config.yaml",
                 "docs/agents/preferences.md", "docs/agents/environment.md",
                 "docs/agents/handoff.md"):
        if not (ROOT / path).is_file():
            errors.append(f"Missing {path}")
    for name in (*WORKFLOWS.values(), *VERIFY):
        for agent in (".claude", ".agents"):
            path = ROOT / agent / "skills" / name / "SKILL.md"
            if not path.is_file():
                errors.append(f"Missing {path.relative_to(ROOT)}")
                continue
            content = path.read_text(encoding="utf-8-sig")
            if not content.startswith("---\n"):
                errors.append(f"Missing frontmatter in {path.relative_to(ROOT)}")
            if not re.search(rf"^name: {re.escape(name)}$", content, re.M):
                errors.append(f"Incorrect skill name in {path.relative_to(ROOT)}")
            if not re.search(r"^description: .+", content, re.M):
                errors.append(f"Missing description in {path.relative_to(ROOT)}")
            if agent == ".agents" and name.startswith("openspec-"):
                if re.search(r"/opsx:(?:explore|propose|apply|update|sync|archive)\b", content):
                    errors.append(f"Claude invocation in {path.relative_to(ROOT)}")
    for workflow in WORKFLOWS:
        if not (ROOT / f".claude/commands/opsx/{workflow}.md").is_file():
            errors.append(f"Missing Claude command {workflow}")
    for name in VERIFY:
        paths = [ROOT / agent / "skills" / name / "SKILL.md"
                 for agent in (".claude", ".agents")]
        if all(path.is_file() for path in paths):
            if paths[0].read_text(encoding="utf-8-sig") != paths[1].read_text(encoding="utf-8-sig"):
                errors.append(f"Verification skill drift: {name}")
    path = ROOT / ".codex/agents/master-reviewer.toml"
    try:
        agent = tomllib.loads(path.read_text(encoding="utf-8-sig"))
        for key in ("name", "description", "developer_instructions"):
            if not agent.get(key):
                errors.append(f"Reviewer missing {key}")
        if agent.get("name") != "master-reviewer":
            errors.append("Reviewer name mismatch")
    except (OSError, tomllib.TOMLDecodeError) as exc:
        errors.append(f"Reviewer config: {exc}")
    for error in errors:
        print(f"FAIL: {error}")
    if errors:
        return 1
    print("PASS: both instruction entries, 10 skills per agent, 6 Claude commands,")
    print("      Codex OpenSpec invocation syntax, 4 identical verify skills, reviewer TOML")
    print("Fresh-session discovery and reviewer permissions require interactive checks.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
