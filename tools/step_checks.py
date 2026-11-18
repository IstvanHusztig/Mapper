"""The checks of a refactor commit, as one command: does this worktree's code still give the same bits?

    python tools/step_checks.py            # run the checks, one line each; exit 0 only if all hold
    python tools/step_checks.py --record   # write the reference files from this commit (no uncommitted source changes)
    python tools/step_checks.py --build-only   # only the Release build of this worktree (dMapper and the tests)

In this order, one at a time: the Release build; the unit tests; the characterization guard and the loop guard against
their reference files (exact hexadecimal doubles); the replays of the captures of tools/local_paths.toml, every window
identical with its capture; the per-term table of the captures marked `terms`, against its reference; and the recordings
marked `step_check` (44_short) against their manifest. The replays go through the gate (tools/run_replay.py's), the full
run has the gate alone.

Whether a change "changes no number" is what this command says, not what its author says. It works on the worktree its
file lies in, so a region agent's branch is checked in its worktree and the main tree again after a merge. The reference
files are shared by all worktrees (<guards>/refs, with the commit they were recorded from)."""
import argparse
import datetime
import filecmp
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import kit_lib as kit

GUARDS = [("characterization", "scan-to-map characterization*", "LO_CHARACTERIZATION_OUT", "characterization.txt"),
          ("loop guard", "window loop guard*", "LO_LOOP_GUARD_OUT", "loop_guard.txt")]
failed = []


def say(text):
    print(text, flush=True)


def check(name, ok, detail=""):
    say(f"{name}: {detail}")
    if not ok:
        failed.append(name)


def run_guard(pattern, variable, out):
    out.unlink(missing_ok=True)
    subprocess.run([str(kit.BIN / "lidar_odometry_tests.exe"), f"-tc={pattern}", "--no-skip"], capture_output=True, text=True, cwd=kit.BIN,
                   env=dict(os.environ, **{variable: str(out)}))
    return out.is_file()


def terms_settings(capture, folder):
    """The capture's settings with log_window_terms on, under a name of this worktree's own (the table files are named
    after it, next to the capture)."""
    text = capture["settings_path"].read_text()
    if "log_window_terms = false" in text:
        text = text.replace("log_window_terms = false", "log_window_terms = true")
    elif "log_window_terms" not in text:
        text = text.replace("[lidar_odometry_debug_info]", "[lidar_odometry_debug_info]\nlog_window_terms = true", 1)
    path = folder / f"{kit.WORKTREE}_terms.toml"
    path.write_text(text)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", action="store_true", help="write the reference files from this commit instead of comparing")
    ap.add_argument("--build-only", action="store_true", help="only build this worktree (Release: dMapper and the tests)")
    args = ap.parse_args()
    paths = kit.load_paths()
    folder, refs = kit.run_folder(paths), paths["guards"] / "refs"
    refs.mkdir(parents=True, exist_ok=True)
    commit = kit.git("rev-parse", "--short", "HEAD")
    dirty = kit.git("status", "--porcelain", "--", "src", "tests", "CMakeLists.txt")
    say(f"step checks: worktree {kit.ROOT}, commit {commit}{' with uncommitted source changes' if dirty else ''}")
    if args.record and dirty:
        sys.exit("--record needs a tree without uncommitted changes in src/, tests/ and CMakeLists.txt")
    recorded = refs / "recorded.json"
    if not args.record and not args.build_only:
        if not recorded.is_file():
            sys.exit(f"no reference files in {refs}: record them from a known-good commit with --record")
        say(f"references: {json.loads(recorded.read_text())}")
    for c in paths["capture"]:
        c["file_path"], c["settings_path"] = paths["guards"] / c["file"], paths["guards"] / c["settings"]

    ok, errors = kit.build(paths)
    check("build", ok, "ok" if ok else "FAILED " + " | ".join(errors))
    if not ok or args.build_only:
        sys.exit(0 if ok else 1)

    r = subprocess.run([str(kit.BIN / "lidar_odometry_tests.exe")], capture_output=True, text=True, cwd=kit.BIN)
    counts = " ".join(l.strip() for l in (r.stdout + r.stderr).splitlines() if "test cases:" in l)
    check("tests", r.returncode == 0, ("passed " if r.returncode == 0 else "FAILED ") + counts)

    for name, pattern, variable, ref_name in GUARDS:
        out = folder / ref_name
        if not run_guard(pattern, variable, out):
            check(name, False, "NO OUTPUT")
        elif args.record:
            second = folder / ("second_" + ref_name)
            run_guard(pattern, variable, second)
            same = second.is_file() and filecmp.cmp(out, second, shallow=False)
            if same:
                shutil.copyfile(out, refs / ref_name)
            check(name, same, "recorded (two runs identical)" if same else "NOT RECORDED: two runs differ")
        else:
            same = filecmp.cmp(out, refs / ref_name, shallow=False)
            check(name, same, "IDENTICAL" if same else "DIFFERENT")

    for c in paths["capture"]:
        cost = kit.gated_replay(paths, c["file_path"], c["settings_path"], folder / f"replay_{c['file_path'].parent.name}.log",
                                f"{kit.WORKTREE}:step checks {c['name']}", say)
        line = next((l for l in cost["summary"] if "identical with the capture" in l), "no summary")
        check(f"replay {c['name']}", cost["exit"] == 0 and f"identical with the capture: {c['windows']} of {c['windows']}" in line,
              f"exit {cost['exit']} | {line} | {kit.cost_text(cost)}")
        if not c["terms"]:
            continue
        settings = terms_settings(c, folder)
        kit.gated_replay(paths, c["file_path"], settings, folder / f"replay_terms_{c['file_path'].parent.name}.log",
                         f"{kit.WORKTREE}:step checks table {c['name']}", say)
        results = []
        for kind in ("window_terms.csv", "window_features.csv"):
            made = c["file_path"].parent / f"{settings.stem}_{kind}"
            ref = refs / f"{c['file_path'].parent.name}_{kind}"
            if not made.is_file():
                results.append((kind, False, "NOT WRITTEN"))
            elif args.record:
                shutil.copyfile(made, ref)
                results.append((kind, True, "recorded"))
            else:
                same = ref.is_file() and filecmp.cmp(made, ref, shallow=False)
                results.append((kind, same, "IDENTICAL" if same else "DIFFERENT"))
        check(f"term table {c['name']}", all(r[1] for r in results), ", ".join(f"{k} {t}" for k, _, t in results))

    gate = kit.Gate(paths, say)
    for rec in (r for r in paths["recording"] if r["step_check"]):
        lock = gate.acquire("full", f"{kit.WORKTREE}:step checks {rec['name']}")
        try:
            out = folder / f"run_{rec['name']}"
            shutil.rmtree(out, ignore_errors=True)
            out.mkdir(parents=True)
            cost = kit.run_dmapper([paths["recordings"] / rec["folder"], kit.ROOT / rec["settings"], out / "result"], out / "run.log")
        finally:
            gate.release(lock)
        r = subprocess.run([sys.executable, str(kit.ROOT / "tools/compare_baseline.py"), str(out / "result"), str(kit.ROOT / rec["manifest"])],
                           capture_output=True, text=True)
        last = (r.stdout + r.stderr).strip().splitlines()[-2:]
        check(rec["name"], cost["exit"] == 0 and r.returncode == 0, f"exit {cost['exit']} | {' | '.join(last)} | {kit.cost_text(cost)}")

    if args.record and not failed:
        recorded.write_text(json.dumps({"commit": commit, "date": datetime.date.today().isoformat(), "worktree": str(kit.ROOT)}))
        say(f"references recorded from {commit} in {refs}")
    say("step checks: " + ("ALL HOLD" if not failed else "FAILED: " + ", ".join(failed)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
