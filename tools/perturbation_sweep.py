"""The noise floor of a settings file on one recording: runs it unchanged and with small, physically meaningless
changes, and reports how far the results move. A candidate setting counts as better only beyond this spread, and a
setting whose perturbed runs differ by metres is fragile, whatever its single-run score.

Runs (each a full dMapper run, one after the other, never two on the same input folder):
  base                    the settings file (plus --set changes), with log_window_diagnostics on
  span_minus, span_plus   imu_acceleration_span_s x 0.95 / x 1.05
  edge_minus, edge_plus   lidar_odometry_motion_model_{x,y,z}_1_sigma_m x 0.9 / x 1.1
  dec_minus, dec_plus     decimation - 0.001 / + 0.001

Runs retain all clouds and record SHA256 provenance. Resume requires matching inputs,
settings, executable/dependencies, tool code and output hashes. Unknown or failed
outputs are never overwritten. Both odometry snapshots and final poses are checked.
Exit 2 means incomplete/invalid evidence, 1 means triggers or stability limit exceeded;
0 means complete diagnostic/stability evidence, never map acceptance.

usage: perturbation_sweep.py <recording folder> <settings.toml> <out folder> [--set key=value ...]
                             [--only base,dec_minus,...] [--loop] [--exe <dMapper.exe>] [limits of run_health.py]
"""
import argparse
import glob
import json
import os
import re
import hashlib
from pathlib import Path
import trajectory_seams
import snapshot_trajectory
import math
import subprocess
import sys
import time

import run_health

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_EXE = os.path.join(HERE, "..", "out", "build", "x64-Release", "bin", "dMapper.exe")
EDGE_KEYS = [f"lidar_odometry_motion_model_{a}_1_sigma_m" for a in "xyz"]
PERTURBATIONS = {
    "base": {},
    "span_minus": {"imu_acceleration_span_s": ("scale", 0.95)}, "span_plus": {"imu_acceleration_span_s": ("scale", 1.05)},
    "edge_minus": {k: ("scale", 0.9) for k in EDGE_KEYS}, "edge_plus": {k: ("scale", 1.1) for k in EDGE_KEYS},
    "dec_minus": {"decimation": ("add", -0.001)}, "dec_plus": {"decimation": ("add", 0.001)},
}


def set_key(text, key, value):
    text, n = re.subn(rf"(?m)^{re.escape(key)} = .*$", f"{key} = {value}", text)
    if n != 1:
        sys.exit(f"setting '{key}' must appear exactly once in the settings file (found {n})")
    return text


def get_key(text, key):
    m = re.search(rf"(?m)^{re.escape(key)} = (.*)$", text)
    if not m:
        sys.exit(f"setting '{key}' is not in the settings file")
    return float(m.group(1))


def evidence_key(text, key):
    """Shipped sparse TOMLs may omit a debug option whose built-in default is false."""
    if re.search(rf"(?m)^{re.escape(key)}\s*=", text):
        return set_key(text, key, "true")
    section = "[lidar_odometry_debug_info]"
    if section in text:
        return text.replace(section, section + f"\n{key} = true", 1)
    return text + f"\n{section}\n{key} = true\n"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def files_identity(paths, root):
    return {str(p.relative_to(root)): sha256(p) for p in sorted(paths)}


def identity(recording, exe, text):
    recording, exe = Path(recording).resolve(), Path(exe).resolve()
    files = [p for p in recording.iterdir() if p.is_file() and p.suffix.lower() in (".csv", ".laz", ".las", ".json")]
    if not any(p.suffix.lower() == ".csv" for p in files) or not any(p.suffix.lower() in (".laz", ".las") for p in files):
        raise ValueError("recording lacks IMU or point input")
    dependencies = [exe, *exe.parent.glob("*.dll")]
    tool_names = ("perturbation_sweep.py", "run_health.py", "trajectory_seams.py", "snapshot_trajectory.py")
    return {"schema": 1, "recording": str(recording), "inputs": files_identity(files, recording),
            "executable": str(exe), "binaries": files_identity(dependencies, exe.parent),
            "settings_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "tools": {name: sha256(Path(HERE)/name) for name in tool_names}}


def output_identity(folder, toml, log):
    folder = Path(folder)
    return {"files": files_identity([p for p in folder.rglob("*") if p.is_file()], folder),
            "toml": sha256(toml), "log": sha256(log)}


def validate_resume(record, expected, folder, toml, log):
    if record.get("identity") != expected or record.get("exit") != 0 or not record.get("evidence_complete"):
        raise ValueError("resume refused: missing/mismatching identity or incomplete previous run; use a fresh output directory")
    if record.get("outputs") != output_identity(folder, toml, log):
        raise ValueError("resume refused: output artifacts changed")


def measure(folder, log, toml, args):
    m = run_health.analyse(str(log), str(toml))
    if m["missing_required"] or not m["completed"]:
        raise ValueError("incomplete run diagnostics: " + str(m["missing_required"]))
    text = Path(log).read_text(errors="replace")
    if re.search(r"(?im)^.*(?:error:|failed|aborted|cancell?ed|CHALLENGING DATA OCCURED).*$", text):
        raise ValueError("run log contains failure evidence; inspect before reuse")
    odometry, metadata = snapshot_trajectory.load_snapshot(Path(folder)/"odometry_snapshot.bin", with_metadata=True)
    if len(metadata["windows"]) != m["windows"]:
        raise ValueError("snapshot/log window count mismatch")
    for snap, logged in zip(metadata["windows"], m["window_starts"]):
        # Positions are printed to millimetres; time is relative and printed to .01s.
        if any(abs(a-b) > .000501 for a,b in zip(snap["position"],logged["position"])):
            raise ValueError("snapshot/log start position mismatch")
        relative = snap["time_s"] - metadata["windows"][0]["time_s"]
        if abs(relative - (logged["time_s"]-m["window_starts"][0]["time_s"])) > .01001:
            raise ValueError("snapshot/log start time mismatch")
    stages = {"odometry": odometry, "final": trajectory_seams.load(str(folder), "final")}
    # Snapshot and final may differ in tail coverage, but neither can be empty.
    if not (Path(folder)/"scan_lio_merged.laz").is_file():
        raise ValueError("missing merged cloud")
    triggers = sum(row[3] == "TRIGGER" for row in run_health.judge(m, args))
    return {"coverage": m["coverage"], "windows": m["windows"], "triggers": triggers,
            "stage_counts": {stage: len(rows) for stage, rows in stages.items()},
            "accuracy": "NOT ASSESSED"}, stages


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording"); ap.add_argument("toml"); ap.add_argument("out")
    ap.add_argument("--exe", default=DEFAULT_EXE)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--only", help="subset for collection; incomplete families exit 2")
    ap.add_argument("--profile", required=True, choices=("short", "full"), help="agreed 0.10 m short or 0.3 m/100 m full stability budget")
    ap.add_argument("--reference-route-m", type=float, help="frozen reference length for all full-route candidates")
    ap.add_argument("--reference-id", help="identity of the previously frozen reference route")
    run_health.add_limits(ap)
    args = ap.parse_args()
    diagnostic_limits = {name: getattr(args, name) for name in
                         ("max_bend_share", "max_speed", "max_bias", "max_chi2", "max_chi2_p95")}
    if any(not math.isfinite(v) or v < 0 for v in diagnostic_limits.values()) or args.max_bend_share > 1:
        ap.error("diagnostic limits must be finite, nonnegative; bend share at most 1")
    if args.profile == "full" and (not args.reference_id or args.reference_route_m is None or not math.isfinite(args.reference_route_m) or args.reference_route_m <= 0):
        ap.error("full profile requires positive --reference-route-m and --reference-id fixed before candidates")
    names = args.only.split(",") if args.only else list(PERTURBATIONS)
    if len(set(names)) != len(names) or any(n not in PERTURBATIONS for n in names) or not names:
        ap.error("invalid/duplicate run names")
    # A base is mandatory for a meaningful family. Resume verifies it as well.
    if "base" not in names:
        names.insert(0, "base")
    else:
        names = ["base"] + [n for n in names if n != "base"]
    if os.name == "nt":
        processes = subprocess.run(["tasklist"], capture_output=True, text=True, check=True)
        if "dmapper.exe" in processes.stdout.lower():
            raise ValueError("dMapper is already running")
    base_text = Path(args.toml).read_text(encoding="utf-8")
    base_text = evidence_key(base_text, "log_window_diagnostics")
    base_text = evidence_key(base_text, "save_odometry_snapshot")
    for kv in args.set:
        key, value = kv.split("=", 1)
        base_text = set_key(base_text, key.strip(), value.strip())
    # Evidence settings cannot be disabled by overrides.
    base_text = set_key(base_text, "log_window_diagnostics", "true")
    base_text = set_key(base_text, "save_odometry_snapshot", "true")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    results_path = out/"results.json"
    results = json.loads(results_path.read_text()) if results_path.exists() else {}
    base_stages = None
    invalid = False
    def save():
        temp = out/"results.json.tmp"
        temp.write_text(json.dumps(results, indent=2, allow_nan=False))
        temp.replace(results_path)
    for name in names:
        folder, toml, log = out/name, out/(name+".toml"), out/(name+".log")
        text = base_text
        for key, (how, amount) in PERTURBATIONS[name].items():
            old = get_key(base_text, key)
            text = set_key(text, key, f"{old * amount if how == 'scale' else old + amount:.10g}")
        expected = identity(args.recording, args.exe, text)
        expected["evaluation"] = {"profile": args.profile, "reference_id": args.reference_id,
                                  "reference_route_m": args.reference_route_m,
                                  "diagnostic_limits": diagnostic_limits}
        if name in results:
            validate_resume(results[name], expected, folder, toml, log)
            record = results[name]
        else:
            if folder.exists() or toml.exists() or log.exists():
                raise ValueError("unidentified outputs exist; use a fresh output directory")
            folder.mkdir()
            toml.write_text(text, encoding="utf-8")
            record = {"identity": expected, "evidence_complete": False}
            results[name] = record
            save()
            started = time.monotonic()
            with log.open("w") as stream:
                record["exit"] = subprocess.call([str(Path(args.exe).resolve()), str(Path(args.recording).resolve()), str(toml), str(folder)], stdout=stream, stderr=subprocess.STDOUT)
            record["wall_s"] = time.monotonic()-started
            save()
        try:
            if record["exit"] != 0:
                raise ValueError(f"mapper exited {record['exit']}")
            current = identity(args.recording, args.exe, text)
            current["evaluation"] = expected["evaluation"]
            if expected != current:
                raise ValueError("input or executable changed during processing")
            readings, stages = measure(folder, log, toml, args)
            record.update(readings)
            if name == "base":
                base_stages = stages
            else:
                if base_stages is None:
                    raise ValueError("base evidence unavailable")
                record["stability"] = {}
                for stage, rows in stages.items():
                    metric = trajectory_seams.compare(base_stages[stage], rows, [.05, .1])
                    limit = .1 if args.profile == "short" else .003*args.reference_route_m
                    record["stability"][stage] = {**metric, "limit_m":limit, "within_limit":metric["max_diff_m"] <= limit}
            record["profile"] = args.profile
            record["frozen_reference"] = {"id":args.reference_id, "route_m":args.reference_route_m}
            record["evidence_complete"] = True
            record.pop("error", None)
            record["outputs"] = output_identity(folder, toml, log)
        except (ValueError, OSError, SystemExit) as error:
            record["error"] = str(error)
            record["evidence_complete"] = False
            invalid = True
        save()
        if invalid:
            break
    selected = [results.get(n, {}) for n in PERTURBATIONS]
    complete = not invalid and set(names) == set(PERTURBATIONS) and all(r.get("evidence_complete") for r in selected)
    flagged = any(r.get("triggers", 0) or any(not v["within_limit"] for v in r.get("stability", {}).values()) for r in selected)
    lines = ["run | evidence | triggers | odometry max m | final max m"]
    for name in PERTURBATIONS:
        r = results.get(name, {})
        values = [str(r.get("stability",{}).get(stage,{}).get("max_diff_m", "NOT MEASURED")) for stage in ("odometry","final")]
        lines.append(f"{name} | {r.get('evidence_complete',False)} | {r.get('triggers','NOT MEASURED')} | " + " | ".join(values))
    lines.append("COMPLETE" if complete else "INCOMPLETE")
    lines.append("Map accuracy/acceptance NOT ASSESSED; see results.json for coverage/errors.")
    (out/"table.txt").write_text("\n".join(lines))
    print("\n".join(lines))
    return 2 if not complete else (1 if flagged else 0)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError) as error:
        print(f"verification error: {error}", file=sys.stderr)
        sys.exit(2)
