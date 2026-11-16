#!/usr/bin/env python3
"""Compare a dMapper output folder against a golden baseline.

The baseline is either another output folder or a manifest written with --write-manifest.
Files are compared bit for bit after normalizing only the values that change from run to run
(see NORMALIZE_* below). Exit code: 0 identical, 1 different, 2 usage or input error.

    python tools/compare_baseline.py <new_out> <baseline_dir | manifest.json>
    python tools/compare_baseline.py --write-manifest <out_dir> <manifest.json> --dataset 44 --params settings/l2-vertical.toml
    python tools/compare_baseline.py --tolerance 1e-6 <new_out> <baseline_dir>

--tolerance (needs a baseline folder, not a manifest) accepts text files that differ only in numbers
within the given absolute tolerance: trajectories, session, poses, JSON, TOML. Integers must still match
exactly. LAZ files can't be compared point by point here; a differing LAZ is reported as derived (its points
are the raw points placed with the trajectory) and passes only if every text file passes. Exit code 0 then
means "identical or within tolerance". It is meant for the full 44 run, where /fp:fast builds can change
the last bits of a result when unrelated code changes; 44_short stays bit-identical.
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import subprocess
import sys

# file names: the date-time of the run is part of the two dMapper_* names
NAME_DATETIME = re.compile(r"_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}(?=\.(toml|json)$)")

# text files whose content is normalized; every other file is hashed as raw bytes
TEXT_FILE = re.compile(r"^(session\.mjs|dMapper_params_.*\.toml|dMapper_results_.*\.json)$")

# files the analysis tools add to a result folder (map_quality.py / eval_odometry.py export the clouds as
# scan_lio_<n>.asc); dMapper itself writes no .asc file there, so they are neither hashed nor compared
FOREIGN_FILE = re.compile(r"\.asc$")

# run-dependent values, replaced by <IGNORED> (key = value / "key": value, up to the end of the value)
NORMALIZE_JSON_KEYS = ["elapsed time seconds", "elapsed_time_seconds", "processing_date", "build_date",
                       "git_commit", "git_uncommitted_changes"]
NORMALIZE_TOML_KEYS = ["build_date"]
NORMALIZE_TOML_LINES = [re.compile(r"^# Created on: .*$", re.M)]


def normalized_name(name):
    return NAME_DATETIME.sub("_<DATETIME>", name)


def path_forms(path):
    """The spellings of an absolute path that can appear in the outputs."""
    path = path.rstrip("\\/")
    forms = {path, path.replace("\\", "/"), path.replace("/", "\\")}
    forms |= {f.replace("\\", "\\\\") for f in list(forms)}  # JSON-escaped
    return sorted(forms, key=len, reverse=True)


def find_prefixes(folder):
    """Output and input folder as written by the app: out_folder_name in session.mjs, and the
    parent of working_directory_preview in the params TOML."""
    out_prefix = in_prefix = None
    mjs = os.path.join(folder, "session.mjs")
    if os.path.isfile(mjs):
        with open(mjs, encoding="utf-8") as f:
            out_prefix = json.load(f).get("Session Settings", {}).get("out_folder_name") or None
    for name in os.listdir(folder):
        if name.startswith("dMapper_params_") and name.endswith(".toml"):
            with open(os.path.join(folder, name), encoding="utf-8") as f:
                m = re.search(r"^working_directory_preview = '(.*)'$", f.read(), re.M)
            if m and m.group(1):
                in_prefix = os.path.dirname(m.group(1).rstrip("\\/").replace("/", os.sep))
    return out_prefix, in_prefix


def normalize_text(name, data, out_prefix, in_prefix):
    text = data.decode("utf-8")
    if name.endswith(".toml"):
        for key in NORMALIZE_TOML_KEYS:
            text = re.sub(r"^(%s = ).*$" % re.escape(key), r"\1<IGNORED>", text, flags=re.M)
        for pattern in NORMALIZE_TOML_LINES:
            text = pattern.sub("# Created on: <IGNORED>", text)
    else:
        for key in NORMALIZE_JSON_KEYS:
            text = re.sub(r'("%s"\s*:\s*)("[^"]*"|[^,\n}\]]+)' % re.escape(key), r"\1<IGNORED>", text)
    # output first: it may lie inside the input folder
    for prefix, placeholder in ((out_prefix, "<OUT>"), (in_prefix, "<IN>")):
        if prefix:
            for form in path_forms(prefix):
                text = text.replace(form, placeholder)
    return text.encode("utf-8")


def hash_folder(folder):
    """{normalized file name: sha256 of the normalized content} for every file in the folder that dMapper writes."""
    out_prefix, in_prefix = find_prefixes(folder)
    hashes = {}
    for name in sorted(os.listdir(folder)):
        path = os.path.join(folder, name)
        if not os.path.isfile(path) or FOREIGN_FILE.search(name):
            continue
        with open(path, "rb") as f:
            data = f.read()
        if TEXT_FILE.match(name):
            data = normalize_text(name, data, out_prefix, in_prefix)
        key = normalized_name(name)
        if key in hashes:
            raise ValueError("two files normalize to the same name %s in %s" % (key, folder))
        hashes[key] = hashlib.sha256(data).hexdigest()
    return hashes


def git_commit():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                              cwd=os.path.dirname(os.path.abspath(__file__)), check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def compare(new_hashes, ref_hashes):
    lines = []
    for name in sorted(set(new_hashes) | set(ref_hashes)):
        if name not in new_hashes:
            lines.append("missing    %s" % name)
        elif name not in ref_hashes:
            lines.append("extra      %s" % name)
        elif new_hashes[name] != ref_hashes[name]:
            lines.append("different  %s" % name)
    return lines


NUMBER = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
BINARY_FILE = re.compile(r"\.(laz|las)$", re.I)


def normalized_content(folder, name, out_prefix, in_prefix):
    with open(os.path.join(folder, name), "rb") as f:
        data = f.read()
    if TEXT_FILE.match(name):
        data = normalize_text(name, data, out_prefix, in_prefix)
    return data.decode("utf-8")


def compare_numbers(new_text, ref_text, tolerance):
    """(None, max |difference|) if the texts differ only in numbers within tolerance, else (reason, None)."""
    new_parts, ref_parts = NUMBER.split(new_text), NUMBER.split(ref_text)
    new_nums, ref_nums = NUMBER.findall(new_text), NUMBER.findall(ref_text)
    if new_parts != ref_parts or len(new_nums) != len(ref_nums):
        return "text other than numbers differs", None
    worst = 0.0
    for i, (a, b) in enumerate(zip(new_nums, ref_nums)):
        if a == b:
            continue
        if re.fullmatch(r"[-+]?\d+", a) and re.fullmatch(r"[-+]?\d+", b):
            return "integer %s != %s (number %d)" % (a, b, i + 1), None
        d = abs(float(a) - float(b))
        if d > tolerance:
            return "%s vs %s (|difference| %.3g, number %d)" % (a, b, d, i + 1), None
        worst = max(worst, d)
    return None, worst


def compare_with_tolerance(new_dir, ref_dir, lines, tolerance):
    """Re-checks the files listed as different; returns the report lines and whether all of them pass."""
    new_prefixes, ref_prefixes = find_prefixes(new_dir), find_prefixes(ref_dir)
    new_names = {normalized_name(n): n for n in os.listdir(new_dir)}
    ref_names = {normalized_name(n): n for n in os.listdir(ref_dir)}
    report, ok, derived = [], True, []
    for line in lines:
        kind, name = line.split(None, 1)
        if kind != "different":
            report.append(line)
            ok = False
        elif BINARY_FILE.search(name):
            derived.append(name)
        else:
            reason, worst = compare_numbers(normalized_content(new_dir, new_names[name], *new_prefixes),
                                            normalized_content(ref_dir, ref_names[name], *ref_prefixes), tolerance)
            if reason:
                report.append("different  %s: %s" % (name, reason))
                ok = False
            else:
                report.append("tolerance  %s (max |difference| %.3g)" % (name, worst))
    for name in derived:
        report.append("derived    %s (LAZ, not compared point by point)" % name)
    return report, ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("new", help="output folder to check (or to write the manifest from)")
    ap.add_argument("reference", help="baseline output folder or manifest .json (written with --write-manifest)")
    ap.add_argument("--write-manifest", action="store_true", help="write <reference> as a manifest of <new>")
    ap.add_argument("--dataset", default="", help="dataset name recorded in the manifest")
    ap.add_argument("--params", default="", help="parameter file recorded in the manifest")
    ap.add_argument("--cpu-optimization", default="AUTO", help="HD_CPU_OPTIMIZATION of the build, recorded in the manifest")
    ap.add_argument("--verbose", action="store_true", help="print the normalized fields and every file")
    ap.add_argument("--tolerance", type=float, default=None,
                    help="accept numbers in text files that differ by at most this much (needs a baseline folder)")
    args = ap.parse_args()

    if not os.path.isdir(args.new):
        print("error: not a folder: %s" % args.new, file=sys.stderr)
        return 2
    if args.verbose:
        print("normalized: file-name date-time; JSON %s; TOML %s and '# Created on'; output/input folder paths"
              % (NORMALIZE_JSON_KEYS, NORMALIZE_TOML_KEYS))
    new_hashes = hash_folder(args.new)

    if args.write_manifest:
        manifest = {
            "dataset": args.dataset,
            "params": args.params,
            "commit": git_commit(),
            "hd_cpu_optimization": args.cpu_optimization,
            "created": datetime.date.today().isoformat(),
            "files": new_hashes,
        }
        with open(args.reference, "w", encoding="utf-8", newline="\n") as f:
            json.dump(manifest, f, indent=2)
            f.write("\n")
        print("wrote %s (%d files)" % (args.reference, len(new_hashes)))
        return 0

    if os.path.isdir(args.reference):
        ref_hashes = hash_folder(args.reference)
    elif os.path.isfile(args.reference):
        with open(args.reference, encoding="utf-8") as f:
            ref_hashes = json.load(f)["files"]
    else:
        print("error: no such baseline: %s" % args.reference, file=sys.stderr)
        return 2

    lines = compare(new_hashes, ref_hashes)
    if args.verbose:
        for name in sorted(new_hashes):
            print("  %s  %s" % (new_hashes[name][:12], name))
    if lines and args.tolerance is not None:
        if not os.path.isdir(args.reference):
            print("error: --tolerance needs a baseline folder, not a manifest", file=sys.stderr)
            return 2
        report, ok = compare_with_tolerance(args.new, args.reference, lines, args.tolerance)
        for line in report:
            print(line)
        total = len(set(new_hashes) | set(ref_hashes))
        if ok:
            print("WITHIN TOLERANCE %g: %d of %d files identical, the rest within tolerance or derived"
                  % (args.tolerance, total - len(lines), total))
            return 0
        print("DIFFERENT: beyond tolerance %g" % args.tolerance)
        return 1
    for line in lines:
        print(line)
    if lines:
        print("DIFFERENT: %d of %d files" % (len(lines), len(set(new_hashes) | set(ref_hashes))))
        return 1
    print("IDENTICAL: %d files" % len(new_hashes))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as e:
        print("error: %s" % e, file=sys.stderr)
        sys.exit(2)
