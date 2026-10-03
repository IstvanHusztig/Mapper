#!/usr/bin/env python3
"""Rename C++ identifiers across source files, in code only (never in comments, string or character literals).

    python tools/rename_identifiers.py [options] PATH [PATH ...] -r OLD=NEW [-r OLD=NEW ...] [-m OLD=NEW ...]

  -r / --rename OLD=NEW   rename every whole identifier OLD (variables, types, functions, parameters);
                          NEW may be a dotted name, e.g. -r intermediate_trajectory=window.poses
  -m / --member OLD=NEW   rename OLD only as a member access, i.e. right after '.' or '->'
                          (NEW may contain a dot, e.g. -m gyro_bias=gyro_bias.mean)
  --lines A-B             only rename on lines A..B (1-based, inclusive) of a single file, e.g. one function
  --dry-run               list every hit with file:line and the line's code, change nothing
  --recursive             descend into sub-folders of folder PATHs
  --ext .cpp,.h,...       file extensions taken from folder PATHs (default .cpp,.cc,.c,.h,.hpp)

PATH is a file or a folder. Line endings (LF or CRLF) are kept. At the end it prints how often each name was
renamed per file and in total. Known limit: C++ raw string literals (R"(...)") are treated as code.
Exit code: 0 done (also when nothing matched), 2 usage error.
"""

import argparse
import os
import re
import sys

TOKEN = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|[A-Za-z_][A-Za-z0-9_]*', re.S)
IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*$")


def parse_pairs(pairs, allow_dot):
    out = {}
    for p in pairs:
        if "=" not in p:
            raise ValueError("expected OLD=NEW, got %r" % p)
        old, new = p.split("=", 1)
        new_ok = re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*" if allow_dot else r"[A-Za-z_][A-Za-z0-9_]*", new)
        if not IDENT.match(old) or not new_ok:
            raise ValueError("not an identifier pair: %r" % p)
        out[old] = new
    return out


def collect_files(paths, recursive, exts):
    files = []
    for path in paths:
        if os.path.isfile(path):
            files.append(path)
        elif os.path.isdir(path):
            for root, dirs, names in os.walk(path):
                files += [os.path.join(root, n) for n in sorted(names) if os.path.splitext(n)[1] in exts]
                if not recursive:
                    break
        else:
            raise ValueError("no such file or folder: %s" % path)
    return sorted(set(files))


def rename_text(src, plain, member, lines=None):
    """returns (new text, [(line, old, new)]) for one file's text (LF line endings); lines = (first, last) or None"""
    out, hits, pos = [], [], 0
    for m in TOKEN.finditer(src):
        tok = m.group(0)
        out.append(src[pos:m.start()])
        pos = m.end()
        new = tok
        if lines is not None:
            line = src.count("\n", 0, m.start()) + 1
            if not lines[0] <= line <= lines[1]:
                out.append(tok)
                continue
        if tok in member:
            before = src[:m.start()].rstrip(" \t")
            if before.endswith(".") or before.endswith("->"):
                new = member[tok]
        if new == tok and tok in plain:
            new = plain[tok]
        if new != tok:
            hits.append((src.count("\n", 0, m.start()) + 1, tok, new))
        out.append(new)
    out.append(src[pos:])
    return "".join(out), hits


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", metavar="PATH")
    ap.add_argument("-r", "--rename", action="append", default=[], metavar="OLD=NEW")
    ap.add_argument("-m", "--member", action="append", default=[], metavar="OLD=NEW")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--lines", default=None, metavar="A-B")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--ext", default=".cpp,.cc,.c,.h,.hpp")
    args = ap.parse_args()

    try:
        plain = parse_pairs(args.rename, allow_dot=True)
        member = parse_pairs(args.member, allow_dot=True)
        if not plain and not member:
            raise ValueError("nothing to rename: give -r and/or -m")
        files = collect_files(args.paths, args.recursive, set(args.ext.split(",")))
        lines = None
        if args.lines is not None:
            m = re.fullmatch(r"(\d+)-(\d+)", args.lines)
            if not m or int(m.group(1)) > int(m.group(2)):
                raise ValueError("--lines expects A-B with A <= B, got %r" % args.lines)
            if len(files) != 1:
                raise ValueError("--lines needs exactly one file")
            lines = (int(m.group(1)), int(m.group(2)))
    except ValueError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    totals = {}
    for path in files:
        with open(path, encoding="utf-8", newline="") as f:
            raw = f.read()
        nl = "\r\n" if "\r\n" in raw else "\n"
        src = raw.replace(nl, "\n")
        new, hits = rename_text(src, plain, member, lines)
        if not hits:
            continue
        lines = src.split("\n")
        counts = {}
        for line, old, repl in hits:
            counts[old] = counts.get(old, 0) + 1
            totals[old] = totals.get(old, 0) + 1
            if args.dry_run:
                print("%s:%d: %s -> %s | %s" % (path, line, old, repl, lines[line - 1].strip()[:140]))
        if not args.dry_run:
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(new.replace("\n", nl))
        print("%s %s: %s" % ("would change" if args.dry_run else "changed", path,
                            ", ".join("%s %d" % kv for kv in sorted(counts.items()))))
    print("total: %s" % (", ".join("%s %d" % kv for kv in sorted(totals.items())) or "no matches"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
