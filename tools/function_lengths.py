"""Lists the functions longer than a limit in C++ sources (stage 6 of docs/code-split-and-region-agents-plan.md: no
function above about 80 lines in the window loop and its terms). A rough reader: a function header line, its opening brace
and the matching closing brace; no preprocessor, lambdas count inside their function, a brace in a string or comment can
mislead it. A measurement aid, not a build check.

    python tools/function_lengths.py 80 src
    python tools/function_lengths.py 80 src/domain/window_steps.cpp src/application

Prints lines, name and file:line per function, longest first."""
import re
import sys
from pathlib import Path

limit = int(sys.argv[1])
paths = []
for a in sys.argv[2:]:
    p = Path(a)
    paths += sorted(p.rglob("*.cpp")) + sorted(p.rglob("*.h")) if p.is_dir() else [p]
head = re.compile(r"^\s*(?:[\w:<>,\*&~]+\s+)*?([~\w:]+)\s*\([^;{]*$|^\s*(?:[\w:<>,\*&~]+\s+)*?([~\w:]+)\s*\(.*\)\s*(const)?\s*(override)?\s*\{?\s*$")
skip = ("if", "for", "while", "switch", "return", "else", "catch", "do")
found = []
for f in paths:
    lines = f.read_text(encoding="utf-8", errors="replace").splitlines()
    i = 0
    while i < len(lines):
        s = lines[i].strip()
        m = head.match(lines[i])
        if m and not s.startswith(skip) and not s.startswith("//") and not s.startswith("#"):
            name = m.group(1) or m.group(2)
            j = i
            while j < len(lines) and "{" not in lines[j] and ";" not in lines[j]:
                j += 1
            if j < len(lines) and "{" in lines[j] and ";" not in lines[j].split("{")[0] and name not in skip:
                depth, k = 0, j
                while k < len(lines):
                    depth += lines[k].count("{") - lines[k].count("}")
                    if depth == 0:
                        break
                    k += 1
                if k - i + 1 > limit:
                    found.append((k - i + 1, f"{f.as_posix()}:{i + 1}", name))
                i = k
        i += 1
for n, where, name in sorted(found, reverse=True):
    print(f"{n:5d}  {name:40s} {where}")
