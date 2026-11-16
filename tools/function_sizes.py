"""List the functions of a source tree that are longer than a limit, largest first.

    python tools/function_sizes.py [src] [--min 80]

A function is counted from its first line at column 0 (not a comment, preprocessor line, declaration ending in ';',
namespace, struct, class, enum, using, typedef or template line) to the next line that starts with '}'. This is the
code style of dMapper's .cpp files; functions inside an anonymous namespace are indented and are not counted (their
enclosing namespace is skipped). The count is approximate (blank lines and comments included) and is meant for tracking
the "huge functions" of docs/code-split-and-region-agents-plan.md, not for exact sizes.
"""
import argparse
import re
from pathlib import Path

SKIP = re.compile(r"(namespace|struct|class|enum|using|typedef|template)\b")
NAME = re.compile(r"([A-Za-z_][\w:~]*)\s*\(")


def functions(path):
    lines = path.read_text(encoding="utf-8-sig", errors="replace").splitlines()
    start = None
    for i, line in enumerate(lines):
        if start is None:
            if line and not line[0].isspace() and line[0] not in "#/}{*" and not SKIP.match(line) and not line.rstrip().endswith(";"):
                start = i
        elif line.startswith("}"):
            head = " ".join(l.strip() for l in lines[start:start + 3])
            m = NAME.search(head)
            yield i - start + 1, start + 1, m.group(1) if m else head[:40]
            start = None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", nargs="?", default="src", help="folder to scan for .cpp files (default: src)")
    ap.add_argument("--min", type=int, default=80, help="smallest length to list, in lines (default: 80)")
    args = ap.parse_args()
    rows = []
    for path in sorted(Path(args.root).rglob("*.cpp")):
        rows += [(n, path.as_posix(), line, name) for n, line, name in functions(path)]
    rows.sort(key=lambda r: -r[0])
    listed = [r for r in rows if r[0] >= args.min]
    for n, p, line, name in listed:
        print(f"{n:5d}  {p}:{line}  {name}")
    print(f"{len(listed)} of {len(rows)} functions have {args.min} lines or more")


if __name__ == "__main__":
    main()
