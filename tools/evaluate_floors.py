"""Floor-alignment evaluation of dMapper results on recording 44 (and the loop summary on 45f), in one table.

Per result folder it runs the three geometry checks of this folder one after the other (each reads the chunk ASC files,
exported once by spot_check.py's CloudCompare call) and reads the loop closure summary from the run's log:
  - stairwell_check.py: drift between neighbouring floors of the round stairwell, and the floor-0 railing seen from the
    start pass against the end pass ("floor 0 start vs end");
  - facade_check.py: mean deviation and spread of the gallery's façade slice by slice;
  - spot_check.py: the railing at one point, seen from every pass; the worst median nearest-neighbour distance among
    the passes with enough points (the largest pass is the reference and isn't printed);
  - the log: loops accepted, largest segment correction, cross-floor registrations accepted, dropped loop edges,
    "big jump" lines. The results JSON holds no loop information, so the log of a result is looked up as
    <logs>/<result folder name>.log (then <logs>/<input name>.log).

Usage: evaluate_floors.py --results 44_s0=<folder> 44_s1=<folder> 44_s2=<folder> [45f=<folder>]
           [--reference 44_s0=<folder> ...] [--logs DIR] [--log-only NAME ...]
A reference (matched by the input name) is evaluated the same way, and its values are printed in parentheses next to the
result's. Inputs whose name starts with LOG_ONLY_PREFIXES (or named by --log-only) get only the log columns.
All geometric numbers are in cm, the correction in m. A check that fails or finds nothing prints "-"; "n/a" is a
column that does not apply to the input. The checks run one after the other (each is memory-hungry, 1-3 minutes per
folder); progress goes to stderr.

The geometry constants below belong to recording 44 (building plan Haus 47, flat 56) and are meaningless for any other
recording.
"""
import argparse
import os
import re
import subprocess
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))

# Recording 44 (building plan Haus 47, flat 56), coordinates of scan_lio_merged.laz.
SHAFT_CENTRE = (-4.7, -4.9)                    # the round stairwell's centre, rough guess for stairwell_check.py
FACADE = 2.15                                  # façade position across the gallery, from the windows at the start [m]
FACADE_SPACING = 1.135                         # façade to parapet, from the plan [m]
FACADE_WINDOW = (35, 95)                       # time window of the pass along the gallery [s]
RAILING_SPOT = (-3.0121, -5.4233, -0.1179)     # a point on the railing seen from several passes
RAILING_RADIUS = 0.8                           # [m]
MIN_SPOT_POINTS = 30                           # spot_check.py passes with fewer points do not count

# Recording 45f is checked by its log only (no stairwell, gallery or railing).
LOG_ONLY_PREFIXES = ("45",)


def run_tool(script, args):
    """stdout of tools/<script> <args>, or None (with a message on stderr) if it fails"""
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    cmd = [sys.executable, os.path.join(TOOLS, script)] + [str(a) for a in args]
    p = subprocess.run(cmd, capture_output=True, env=env)
    if p.returncode != 0:
        tail = p.stderr.decode("utf-8", "replace").strip().splitlines()[-3:]
        print(f"  {script} failed ({p.returncode}): " + " | ".join(tail), file=sys.stderr, flush=True)
        return None
    return p.stdout.decode("utf-8", "replace")


def cm(text):
    return float(text) if text else None


STAIRWELL_KEYS = ("stack1", "below1", "below2", "start_end", "storey1", "storey2")


def parse_stairwell(text):
    """{'stack1', 'below1', 'below2', 'start_end'} in cm and {'storey1', 'storey2'} in m (the height between the sensor's
    levels on neighbouring floors: a storey squeezed or stretched by the loop closure); None where the check did not find it"""
    out = dict.fromkeys(STAIRWELL_KEYS)
    m = re.search(r"floors \(sensor height\) (.*)", text)
    if m:
        levels = [float(z) for z in re.findall(r"\d+: ([+-]\d+\.\d+) m", m.group(1))]
        if len(levels) >= 2:
            out["storey1"] = levels[0] - levels[1]
        if len(levels) >= 3:
            out["storey2"] = levels[1] - levels[2]
    band = None
    own0 = []  # offsets of the floor-0 railing seen from floor 0 (the first pass is the reference, 0.0)
    for line in text.splitlines():
        m = re.match(r"== railing band of floor (\d+)", line)
        if m:
            band = int(m.group(1))
            continue
        if line.startswith("== drift"):
            band = None
            continue
        if band == 0:
            m = re.match(r"\s+seen from floor 0,.*offset.*=\s*([\d.]+) cm\s*$", line)
            if m:
                own0.append(float(m.group(1)))
        m = re.match(r"\s+floor (\d+) against floor (\d+): stack\s+(?:([\d.]+) cm|-\s*),\s+"
                     r"upper railing seen from below\s+(?:([\d.]+) cm|-)\s*$", line)
        if m:
            f = int(m.group(1))
            if f == 1:
                out["stack1"], out["below1"] = cm(m.group(3)), cm(m.group(4))
            elif f == 2:
                out["below2"] = cm(m.group(4))
    if len(own0) >= 2:
        out["start_end"] = own0[-1]
    return out


def parse_facade(text):
    """(mean deviation, spread) in cm of the last summary line, or (None, None)"""
    m = re.search(r"slices with the pair: \d+, deviation from the reference: mean ([+-][\d.]+) cm, "
                  r"largest [+-][\d.]+ cm, spread ([\d.]+) cm", text)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


def parse_spot(text):
    """worst median nearest-neighbour distance [cm] among the passes with at least MIN_SPOT_POINTS points"""
    worst = None
    for line in text.splitlines():
        m = re.match(r"\s*-?\d+\.\d\.\.\s*-?\d+\.\d\s+(\d+)\s+(\d+\.\d)\s+\d+\.\d\d\s+-?\d+\.\d\s+-?\d+\.\d\s+-?\d+\.\d\s*$", line)
        if m and int(m.group(1)) >= MIN_SPOT_POINTS:
            d = float(m.group(2))
            worst = d if worst is None else max(worst, d)
    return worst


def parse_log(path):
    """loop closure summary of a run log; every value None if there is no log"""
    out = dict.fromkeys(("loops", "correction", "cross", "dropped", "jumps"))
    if not path:
        return out
    dropped = jumps = 0
    with open(path, errors="replace") as f:
        for line in f:
            m = re.search(r"loop closure: (\d+) loop\(s\) accepted, largest segment correction ([\d.]+) m", line)
            if m:
                out["loops"], out["correction"] = int(m.group(1)), float(m.group(2))
            m = re.search(r"loop closure: (\d+) of \d+ cross-floor registrations accepted", line)
            if m:
                out["cross"] = int(m.group(1))
            dropped += "dropped:" in line
            jumps += "big jump" in line
    out["dropped"], out["jumps"] = dropped, jumps
    return out


def find_log(logs, label, folder):
    if not logs:
        return None
    for name in (os.path.basename(os.path.normpath(folder)), label):
        p = os.path.join(logs, name + ".log")
        if os.path.exists(p):
            return p
    print(f"  no log for {label} in {logs}", file=sys.stderr, flush=True)
    return None


def evaluate(label, folder, logs, geometry):
    """all numbers of one result folder; geometry False leaves the geometric ones as 'n/a'"""
    r = {}
    t0 = time.time()
    if geometry:
        print(f"[{label}] stairwell_check ...", file=sys.stderr, flush=True)
        text = run_tool("stairwell_check.py", [folder, SHAFT_CENTRE[0], SHAFT_CENTRE[1]])
        r.update(parse_stairwell(text) if text else dict.fromkeys(STAIRWELL_KEYS))
        print(f"[{label}] facade_check ...", file=sys.stderr, flush=True)
        text = run_tool("facade_check.py", [folder, "--facade", FACADE, "--spacing", FACADE_SPACING,
                                            "--window", FACADE_WINDOW[0], FACADE_WINDOW[1]])
        r["facade_mean"], r["facade_spread"] = parse_facade(text) if text else (None, None)
        print(f"[{label}] spot_check ...", file=sys.stderr, flush=True)
        text = run_tool("spot_check.py", [folder, *RAILING_SPOT, RAILING_RADIUS])
        r["railing"] = parse_spot(text) if text else None
    else:
        for k in STAIRWELL_KEYS + ("facade_mean", "facade_spread", "railing"):
            r[k] = "n/a"
    r.update(parse_log(find_log(logs, label, folder)))
    print(f"[{label}] done in {time.time() - t0:.0f} s", file=sys.stderr, flush=True)
    return r


# (header, key, format)
COLUMNS = [
    ("f-1/0 stack", "stack1", "{:.1f}"),
    ("f-1/0 below", "below1", "{:.1f}"),
    ("f-2/-1 below", "below2", "{:.1f}"),
    ("f0 start/end", "start_end", "{:.1f}"),
    ("storey 0/-1 [m]", "storey1", "{:.2f}"),
    ("storey -1/-2 [m]", "storey2", "{:.2f}"),
    ("facade mean", "facade_mean", "{:+.1f}"),
    ("facade spread", "facade_spread", "{:.1f}"),
    ("railing NN", "railing", "{:.1f}"),
    ("loops", "loops", "{:d}"),
    ("max corr [m]", "correction", "{:.2f}"),
    ("x-floor", "cross", "{:d}"),
    ("dropped", "dropped", "{:d}"),
    ("jumps", "jumps", "{:d}"),
]


def cell(value, fmt):
    if value == "n/a":
        return "n/a"
    return "-" if value is None else fmt.format(value)


def print_table(rows):
    """rows: [(label, result values, reference values or None)]"""
    header = ["input"] + [c[0] for c in COLUMNS]
    body = []
    for label, r, ref in rows:
        line = [label]
        for _, key, fmt in COLUMNS:
            s = cell(r[key], fmt)
            if ref is not None and r[key] != "n/a":
                s += f" ({cell(ref[key], fmt)})"
            line.append(s)
        body.append(line)
    widths = [max(len(row[i]) for row in [header] + body) for i in range(len(header))]
    fmt_row = lambda row: "  ".join(row[0].ljust(widths[0]) if i == 0 else row[i].rjust(widths[i]) for i in range(len(row)))
    print(fmt_row(header))
    print("  ".join("-" * w for w in widths))
    for row in body:
        print(fmt_row(row))
    print("f-1/0 = stairwell floor 1 against floor 0 (stack: railing seen from the floor itself; below: upper railing seen"
          " from the lower floor); f-2/-1 likewise; f0 start/end = floor-0 railing, later pass against first;")
    print("facade = mean deviation and spread over the gallery slices; railing NN = worst median nearest-neighbour"
          " distance of a pass; x-floor = cross-floor registrations accepted. Values in parentheses: reference.")


def labelled(items, what):
    out = {}
    for item in items or []:
        label, sep, folder = item.partition("=")
        if not sep or not label or not folder:
            sys.exit(f"{what}: expected NAME=FOLDER, got '{item}'")
        out[label] = folder
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--results", nargs="+", required=True, metavar="NAME=FOLDER", help="the result folders to evaluate")
    ap.add_argument("--reference", nargs="+", metavar="NAME=FOLDER", help="reference folders, matched by NAME")
    ap.add_argument("--logs", help="folder with <result folder name>.log files of the runs")
    ap.add_argument("--log-only", nargs="+", default=[], metavar="NAME", help="inputs checked by their log only")
    a = ap.parse_args()

    results, references = labelled(a.results, "--results"), labelled(a.reference, "--reference")
    for label in references:
        if label not in results:
            sys.exit(f"--reference {label}: no such input in --results")
    for folder in list(results.values()) + list(references.values()):
        if not os.path.isfile(os.path.join(folder, "session_poses.mrp")):
            sys.exit(f"{folder}: not a result folder (no session_poses.mrp)")

    done = {}  # (folder, geometry) -> values, a folder used twice is evaluated once

    def get(label, folder, reference=False):
        geometry = not (label.startswith(LOG_ONLY_PREFIXES) or label in a.log_only)
        key = (os.path.normpath(folder), geometry)
        if key not in done:
            # a reference never falls back to the result's log, which is looked up by the input name
            done[key] = evaluate(label + " ref" if reference else label, folder, a.logs, geometry)
        return done[key]

    rows = []
    for label, folder in results.items():
        r = get(label, folder)
        ref = get(label, references[label], True) if label in references else None
        rows.append((label, r, ref))
    print_table(rows)


if __name__ == "__main__":
    main()
