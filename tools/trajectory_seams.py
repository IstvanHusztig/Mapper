"""Where two dMapper results of the same recording disagree (pure Python, no numpy).

Usage:
  python tools/trajectory_seams.py <reference_result> <result> [<result> ...] [--thresholds 0.05 0.2 0.5]

Each folder holds trajectory_lio_*.csv. The poses of a result are the chunks' rows in time order; two results
are compared pose by pose, so they must come from the same recording and settings. A result that is a ghost wall
away from the reference shows up as a stretch where the position difference is large: the seam measure
eval_odometry.py does not have (it looks at yaw only).

Reported per result:
  max diff    largest position difference [m], its time [s from the first pose] and the reference position there
  first >     for each threshold the first time the difference exceeds it
  stretch     the longest run of consecutive poses above the first threshold: start, end [s], and its largest value
  route       length of the result's path [m] (reference's in the header)
"""
import argparse
import glob
import math
import os


def load(folder):
    rows = []
    files = glob.glob(os.path.join(folder, "trajectory_lio_*.csv"))
    if not files:
        raise SystemExit(f"no trajectory_lio_*.csv in {folder}")
    for fn in files:
        with open(fn) as f:
            f.readline()
            for line in f:
                v = line.split()
                if len(v) >= 13:
                    rows.append((float(v[0]), float(v[4]), float(v[8]), float(v[12])))
    rows.sort()
    return rows


def route(rows):
    return sum(math.dist(rows[i][1:], rows[i + 1][1:]) for i in range(len(rows) - 1))


def compare(ref, res, thresholds):
    n = min(len(ref), len(res))
    diff = [math.dist(ref[i][1:], res[i][1:]) for i in range(n)]
    t0 = ref[0][0]
    sec = lambda i: (ref[i][0] - t0) / 1e9
    k = max(range(n), key=lambda i: diff[i])
    out = {"count": n, "max_diff_m": diff[k], "max_time_s": sec(k), "max_ref_position": ref[k][1:], "route_m": route(res)}
    out["first_above_s"] = {}
    for thr in thresholds:
        i = next((j for j in range(n) if diff[j] > thr), None)
        out["first_above_s"][thr] = None if i is None else sec(i)
    # the longest stretch above the first threshold
    best = (0, 0, 0.0)
    start = None
    for i in range(n + 1):
        above = i < n and diff[i] > thresholds[0]
        if above and start is None:
            start = i
        if not above and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i, max(diff[start:i]))
            start = None
    out["stretch"] = None if best[1] == best[0] else (sec(best[0]), sec(best[1] - 1), best[2])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reference")
    ap.add_argument("results", nargs="+")
    ap.add_argument("--thresholds", type=float, nargs="+", default=[0.05, 0.2, 0.5])
    a = ap.parse_args()
    ref = load(a.reference)
    print(f"reference {a.reference}: {len(ref)} poses, route {route(ref):.2f} m")
    for folder in a.results:
        res = load(folder)
        if len(res) != len(ref):
            print(f"warning: {folder} has {len(res)} poses, the reference {len(ref)}; comparing the first {min(len(res), len(ref))}")
        r = compare(ref, res, a.thresholds)
        print(f"{folder}: route {r['route_m']:.2f} m")
        p = r["max_ref_position"]
        print(f"  max diff {r['max_diff_m']:.3f} m at t = {r['max_time_s']:.1f} s (reference position {p[0]:.1f} {p[1]:.1f} {p[2]:.1f})")
        print("  first >  " + "   ".join(f"{thr * 100:g} cm: " + ("never" if t is None else f"{t:.1f} s") for thr, t in r["first_above_s"].items()))
        if r["stretch"]:
            s, e, m = r["stretch"]
            print(f"  longest stretch above {a.thresholds[0] * 100:g} cm: {s:.1f} .. {e:.1f} s, largest {m:.3f} m")
        else:
            print(f"  nowhere above {a.thresholds[0] * 100:g} cm")


if __name__ == "__main__":
    main()
