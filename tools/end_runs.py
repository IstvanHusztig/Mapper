"""The runs that end a change: the full recordings with their shipped settings against their manifests.

    python tools/end_runs.py [--debug] [--only 44,livox3] [--no-build]

One after the other, with this worktree's Release build: each recording of tools/local_paths.toml (44, livox3,
44_short), compared with its manifest by tools/compare_baseline.py, with its odometry time and its cost. `--debug` adds
the Debug build and ctest at the end. It holds the gate for itself while it runs (no replay of a region agent beside a
full run: a full run is timing-sensitive and uses its recording's cache folder), and refuses to start when a dMapper
process runs that did not come through the gate. One line per run goes to <guards>/runs/<worktree>/end_runs.log and to
the console, so it can run in the background (about 50 min for the three recordings, 10 min more with --debug).
Results are written to <guards>/runs/<worktree>/end_<name>/."""
import argparse
import shutil
import subprocess
import sys

import kit_lib as kit


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--debug", action="store_true", help="also the Debug build and ctest")
    ap.add_argument("--only", help="comma-separated recording names instead of all")
    ap.add_argument("--no-build", action="store_true", help="use the Release build as it is")
    args = ap.parse_args()
    paths = kit.load_paths()
    folder = kit.run_folder(paths)
    log = folder / "end_runs.log"
    log.write_text("")

    def say(text):
        print(text, flush=True)
        with log.open("a") as f:
            f.write(text + "\n")

    recordings = paths["recording"]
    if args.only:
        wanted = args.only.split(",")
        unknown = [w for w in wanted if w not in [r["name"] for r in recordings]]
        if unknown:
            sys.exit(f"unknown recording(s): {', '.join(unknown)}")
        recordings = [r for r in recordings if r["name"] in wanted]
    failed = []
    gate = kit.Gate(paths, say)
    lock = gate.acquire("full", f"{kit.WORKTREE}:end runs")
    try:
        if kit.dmapper_processes():
            say("end runs: a dMapper process is running that did not come through the gate; nothing started")
            sys.exit(2)
        say(f"end runs: worktree {kit.ROOT}, commit {kit.git('rev-parse', '--short', 'HEAD')}")
        if not args.no_build:
            ok, errors = kit.build(paths)
            say("release build: " + ("ok" if ok else "FAILED " + " | ".join(errors)))
            if not ok:
                sys.exit(1)
        for rec in recordings:
            out = folder / f"end_{rec['name']}"
            shutil.rmtree(out, ignore_errors=True)
            out.mkdir(parents=True)
            cost = kit.run_dmapper([paths["recordings"] / rec["folder"], kit.ROOT / rec["settings"], out / "result"], out / "run.log")
            odometry = [l.split("elapsed time: ")[-1] for l in (out / "run.log").read_text(errors="replace").splitlines()
                        if l.startswith("finished computation at")]
            r = subprocess.run([sys.executable, str(kit.ROOT / "tools/compare_baseline.py"), str(out / "result"), str(kit.ROOT / rec["manifest"])],
                               capture_output=True, text=True)
            last = (r.stdout + r.stderr).strip().splitlines()[-2:]
            if cost["exit"] != 0 or r.returncode != 0:
                failed.append(rec["name"])
            say(f"{rec['name']}: exit {cost['exit']}, odometry {odometry[-1] if odometry else 'not reported'}, {kit.cost_text(cost)} | {' | '.join(last)}")
        if args.debug:
            ok, errors = kit.build(paths, "x64-Debug", ("lidar_odometry_tests",))
            say("debug build: " + ("ok" if ok else "FAILED " + " | ".join(errors)))
            if ok:
                ok, lines = kit.debug_ctest(paths)
                say("debug ctest: " + (" | ".join(lines) if lines else "no summary") + ("" if ok else " (FAILED)"))
            if not ok:
                failed.append("debug")
    finally:
        gate.release(lock)
    say("end runs: " + ("ALL HOLD" if not failed else "FAILED: " + ", ".join(failed)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
