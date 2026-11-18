"""List the functions of a source tree that are longer than a limit, largest first.

    python tools/function_sizes.py [src] [--min 80]

A function is counted from its header line (a name followed by an opening parenthesis, not a control statement) to the
brace that closes its body, at any indentation: functions inside a namespace or a class are counted too, and so are the
ones in headers. A rough reader: no preprocessor, a lambda counts inside its function, and a brace in a string or a
comment can mislead it. The count includes blank lines and comments and is meant for tracking the "huge functions" of
docs/code-split-and-region-agents-plan.md (stage 6: none above about 80 lines in the window loop and its terms), not
for exact sizes. `root` may also be a single file.
"""
import argparse
import re
from pathlib import Path

HEAD = re.compile(r"^\s*(?:[\w:<>,\*&~]+\s+)*?([~\w:]+)\s*\([^;{]*$|^\s*(?:[\w:<>,\*&~]+\s+)*?([~\w:]+)\s*\(.*\)\s*(const)?\s*(override)?\s*\{?\s*$")
SKIP = ("if", "for", "while", "switch", "return", "else", "catch", "do")


def functions(path):
    return functions_in(path.read_text(encoding="utf-8-sig", errors="replace").splitlines())


def functions_in(lines):
    """(length, first line, name) of every function in the lines of a source file."""
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        m = HEAD.match(lines[i])
        if m and not stripped.startswith(SKIP) and not stripped.startswith(("//", "#")):
            name = m.group(1) or m.group(2)
            j = i
            while j < len(lines) and "{" not in lines[j] and ";" not in lines[j]:
                j += 1
            if j < len(lines) and "{" in lines[j] and ";" not in lines[j].split("{")[0] and name not in SKIP:
                depth, k = 0, j
                while k < len(lines):
                    depth += lines[k].count("{") - lines[k].count("}")
                    if depth == 0:
                        break
                    k += 1
                yield k - i + 1, i + 1, name
                i = k
        i += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", nargs="?", default="src", help="folder to scan for .cpp and .h files, or one file (default: src)")
    ap.add_argument("--min", type=int, default=80, help="smallest length to list, in lines (default: 80)")
    args = ap.parse_args()
    root = Path(args.root)
    paths = sorted(list(root.rglob("*.cpp")) + list(root.rglob("*.h"))) if root.is_dir() else [root]
    rows = []
    for path in paths:
        rows += [(n, path.as_posix(), line, name) for n, line, name in functions(path)]
    rows.sort(key=lambda r: -r[0])
    listed = [r for r in rows if r[0] >= args.min]
    for n, p, line, name in listed:
        print(f"{n:5d}  {p}:{line}  {name}")
    print(f"{len(listed)} of {len(rows)} functions have {args.min} lines or more")


if __name__ == "__main__":
    main()
