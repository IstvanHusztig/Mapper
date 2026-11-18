"""Checks the ownership list (docs/regions/owners.toml), and what a branch touches outside its region.

    python tools/region_owners.py                                   # is the list consistent?
    python tools/region_owners.py --owner src/domain/ndt_map.cpp    # who owns a file (and its functions)?
    python tools/region_owners.py --diff <base>..<branch> --region map

The list names files (or globs) and functions ("file::function", which override their file's owner) per region, and the
shared test files; whatever it does not name is the orchestrator's. Without arguments: every named file exists (a glob
matches something), nothing is named twice, every named function is found in its file. Exit 0 only then.

With --diff: the files the range changes, and in files that are not the region's own, the functions the changed lines lie
in (read from the range's right-hand side); everything outside the region's entries is listed with its owner. A shared
test file is listed for the orchestrator to read and does not fail the check. Exit 1 when the range leaves the region.
An agent's definition cannot fence it into paths: this check is the fence (docs/regions/README.md).

The function reader is tools/function_sizes.py's (rough: a lambda counts inside its function; lines outside any function
are "file scope" and belong to the file's owner)."""
import argparse
import fnmatch
import re
import subprocess
import sys
import tomllib
from pathlib import Path

from function_sizes import functions_in

ROOT = Path(__file__).resolve().parents[1]
ORCHESTRATOR = "orchestrator"


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace").stdout


class Owners:
    def __init__(self):
        with (ROOT / "docs/regions/owners.toml").open("rb") as f:
            data = tomllib.load(f)
        self.shared_tests = data.pop("shared_tests", [])
        self.regions = data
        self.tracked = [l for l in git("ls-files").splitlines() if l]
        self.problems = []
        self.file_owner = {}
        self.function_owner = {}
        for region, entry in self.regions.items():
            for pattern in entry.get("files", []):
                matched = [f for f in self.tracked if fnmatch.fnmatchcase(f, pattern)] if any(c in pattern for c in "*?[") else \
                          ([pattern] if pattern in self.tracked else [])
                if not matched:
                    self.problems.append(f"{region}: no tracked file matches {pattern}")
                for f in matched:
                    if f in self.file_owner and self.file_owner[f] != region:
                        self.problems.append(f"{f} is named by {self.file_owner[f]} and by {region}")
                    self.file_owner[f] = region
            for item in entry.get("functions", []):
                file, _, name = item.partition("::")
                if (file, name) in self.function_owner:
                    self.problems.append(f"{item} is named by {self.function_owner[(file, name)]} and by {region}")
                self.function_owner[(file, name)] = region
        for f in self.shared_tests:
            if f not in self.tracked:
                self.problems.append(f"shared test file {f} does not exist")
            if f in self.file_owner:
                self.problems.append(f"shared test file {f} is also named by {self.file_owner[f]}")

    def owner_of_file(self, file):
        return self.file_owner.get(file, ORCHESTRATOR)

    def owner_of_function(self, file, name):
        return self.function_owner.get((file, name), self.owner_of_file(file))

    def check_functions(self):
        for (file, name), region in self.function_owner.items():
            path = ROOT / file
            if not path.is_file():
                self.problems.append(f"{region}: {file}::{name}: the file does not exist")
                continue
            names = {n.split("::")[-1] for _, _, n in functions_in(path.read_text(encoding="utf-8-sig", errors="replace").splitlines())}
            if name not in names:
                self.problems.append(f"{region}: {file}::{name}: no such function in the file")


def touched_functions(range_spec, file):
    """The functions (or "file scope") the range's changed lines lie in, on its right-hand side."""
    right = range_spec.split("..")[-1] or "HEAD"
    text = git("show", f"{right}:{file}")
    spans = [(first, first + length - 1, name.split("::")[-1]) for length, first, name in functions_in(text.splitlines())]
    touched = set()
    for hunk in re.finditer(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@", git("diff", "-U0", range_spec, "--", file), re.M):
        start, count = int(hunk.group(1)), int(hunk.group(2) or 1)
        for line in range(start, start + max(count, 1)):
            inside = [name for first, last, name in spans if first <= line <= last]
            touched.add(inside[0] if inside else "file scope")
    return sorted(touched)


def check_diff(owners, range_spec, region):
    if region not in owners.regions and region != ORCHESTRATOR:
        sys.exit(f"unknown region {region}; the list has: {', '.join(owners.regions)}")
    outside, shared, own = [], [], 0
    for file in [l for l in git("diff", "--name-only", range_spec).splitlines() if l]:
        if file in owners.shared_tests:
            shared.append(file)
        elif owners.owner_of_file(file) == region and not any(f == file for f, _ in owners.function_owner):
            own += 1
        elif any(f == file for f, _ in owners.function_owner) or owners.owner_of_file(file) == region:
            # a file with function entries: owner by function
            for name in touched_functions(range_spec, file):
                owner = owners.owner_of_file(file) if name == "file scope" else owners.owner_of_function(file, name)
                if owner == region:
                    own += 1
                else:
                    outside.append(f"{file}::{name} ({owner})")
        else:
            outside.append(f"{file} ({owners.owner_of_file(file)})")
    print(f"{range_spec} for {region}: {own} own file(s) or function(s), {len(shared)} shared test file(s), {len(outside)} outside")
    for f in shared:
        print(f"  shared test file, for the orchestrator to read: {f}")
    for f in outside:
        print(f"  outside {region}: {f}")
    return 1 if outside else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--diff", metavar="RANGE", help="a git range, e.g. main..region/map/split")
    ap.add_argument("--region", help="the region the range belongs to (with --diff)")
    ap.add_argument("--owner", metavar="FILE", help="print who owns a file and the functions in it")
    args = ap.parse_args()
    owners = Owners()
    owners.check_functions()
    for p in owners.problems:
        print("problem:", p)
    if args.owner:
        file = Path(args.owner).as_posix()
        kind = "shared test file; default owner " if file in owners.shared_tests else ""
        print(f"{file}: {kind}{owners.owner_of_file(file)}")
        for (f, name), region in sorted(owners.function_owner.items()):
            if f == file:
                print(f"  {name}: {region}")
    if args.diff:
        if not args.region:
            sys.exit("--diff needs --region")
        sys.exit(1 if owners.problems else check_diff(owners, args.diff, args.region))
    if not args.owner:
        named = len(owners.file_owner)
        code = [f for f in owners.tracked if f.startswith(("src/", "tests/"))]
        print(f"ownership list: {len(owners.regions)} owners, {named} files and {len(owners.function_owner)} functions named, "
              f"{len(owners.shared_tests)} shared test files; {sum(f not in owners.file_owner and f not in owners.shared_tests for f in code)} "
              f"of {len(code)} files under src/ and tests/ are the orchestrator's by default; {len(owners.problems)} problem(s)")
    sys.exit(1 if owners.problems else 0)


if __name__ == "__main__":
    main()
