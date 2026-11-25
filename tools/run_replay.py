"""Runs a window-capture or loop closure replay through the gate, and reports what it cost.

    python tools/run_replay.py <capture.bin> <settings.toml> [--log <file>] [--label <text>]
    python tools/run_replay.py <loop_closure_capture.bin> <settings.toml> --out <folder> [--reject-tries i-j,...]

With --out the capture is a loop closure capture and the replay is `dMapper --replay-loop-closure` into that folder.

Several jobs share the workstation (the region agents' worktrees, the step checks, a full run), and a replay costs
memory (about three times its capture file) and most cores. Every replay is started through this command: it waits
until enough memory is free and a slot is free (tools/local_paths.toml [gate]; no replay runs beside a full run), runs
this worktree's `dMapper --replay`, and prints the replay's summary, its wall time, CPU time and peak memory, and how
many windows stopped at the time limit. It waits before the start and never pauses a running replay.

The replay's own output goes to the log file (default: <settings name>_replay.log beside the settings file). The table
files of log_window_terms are written by dMapper next to the capture, named after the settings file: give your settings
file a name of your own. A window that stopped at the time limit makes the result depend on the machine's load: run the
replay again. Exit code: the replay's (1 when the capture's own settings did not reproduce the run)."""
import argparse
import sys
from pathlib import Path

import kit_lib as kit


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture", help="window capture file (window_capture_<from>-<to>s.bin)")
    ap.add_argument("settings", help="settings TOML of the replay")
    ap.add_argument("--log", help="file for the replay's output")
    ap.add_argument("--label", help="name of the job in the gate's messages (default: worktree and settings name)")
    ap.add_argument("--out", help="loop closure replay: the output folder (the capture is a loop closure capture)")
    ap.add_argument("--reject-tries", help="loop closure replay: tries counted as rejected, i-j[,i-j...]")
    args = ap.parse_args()
    capture, settings = Path(args.capture).resolve(), Path(args.settings).resolve()
    for f in (capture, settings):
        if not f.is_file():
            sys.exit(f"not found: {f}")
    if not (kit.BIN / "dMapper.exe").is_file():
        sys.exit(f"no build in this worktree: {kit.BIN / 'dMapper.exe'}")
    log = Path(args.log) if args.log else settings.with_name(settings.stem + "_replay.log")
    label = args.label or f"{kit.WORKTREE}:{settings.stem}"
    if args.reject_tries and not args.out:
        sys.exit("--reject-tries needs --out (a loop closure replay)")
    loop_out, extra = None, []
    if args.out:
        loop_out = Path(args.out).resolve()
        loop_out.mkdir(parents=True, exist_ok=True)
        extra = ["--reject-tries", args.reject_tries] if args.reject_tries else []
    cost = kit.gated_replay(kit.load_paths(), capture, settings, log, label, loop_out=loop_out, extra=extra)
    for line in cost["summary"]:
        print("replay:", line)
    limit = cost["time_limit"]
    print(f"cost: {kit.cost_text(cost)}; windows stopped at the time limit: {'not reported' if limit is None else limit}; log {log}")
    if limit:
        print("warning: a window stopped at the time limit, so the result depends on the machine's load: run this replay again")
    sys.exit(cost["exit"])


if __name__ == "__main__":
    main()
