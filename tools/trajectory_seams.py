"""Where two dMapper results of the same recording disagree (pure Python, no numpy).

Usage:
  python tools/trajectory_seams.py <reference_result> <result> [<result> ...] [--thresholds 0.05 0.2 0.5]

Each folder holds trajectory_lio_<n>.csv (one per chunk, relative to the chunk: the first row is the identity) and
session_poses.mrp (the chunk poses: a count, then per chunk the laz file name and a 4x4 matrix). The positions
compared are WORLD positions: each chunk's rows are transformed by that chunk's matrix from session_poses.mrp
(comparing the chunk-relative rows would hide every difference that sits in the chunk poses, as loop closure
makes them). The poses of a result are the chunks' rows in time order; two results are compared pose by pose, so
they must come from the same recording and settings. A result that is a ghost wall away from the reference shows
up as a stretch where the position difference is large: the seam measure eval_odometry.py does not have (it looks
at yaw only).

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


def chunk_poses(folder):
    """[(chunk trajectory file, 4x4 matrix as 4 rows)] from session_poses.mrp, in file order."""
    fn = os.path.join(folder, "session_poses.mrp")
    if not os.path.isfile(fn):
        raise SystemExit(f"no session_poses.mrp in {folder}")
    with open(fn) as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    count = int(lines[0])
    if len(lines) < 1 + 5 * count:
        raise SystemExit(f"{fn}: {count} chunks announced, only {(len(lines) - 1) // 5} found")
    chunks = []
    for k in range(count):
        name = lines[1 + 5 * k]
        matrix = [[float(x) for x in lines[2 + 5 * k + r].split()] for r in range(4)]
        chunks.append((name.replace("scan_lio_", "trajectory_lio_").rsplit(".", 1)[0] + ".csv", matrix))
    return chunks


def load(folder):
    """The result's poses as (timestamp, x, y, z) in world coordinates, in time order."""
    rows = []
    for name, m in chunk_poses(folder):
        fn = os.path.join(folder, name)
        if not os.path.isfile(fn):
            raise SystemExit(f"{folder}: session_poses.mrp lists {name}, which is missing")
        with open(fn) as f:
            f.readline()
            for line in f:
                v = line.split()
                if len(v) >= 13:
                    x, y, z = float(v[4]), float(v[8]), float(v[12])
                    rows.append((float(v[0]),
                                 m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
                                 m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
                                 m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]))
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
        shifted = sum(1 for i in range(min(len(res), len(ref))) if abs(res[i][0] - ref[i][0]) > 1e6)
        if shifted:
            print(f"warning: {shifted} poses of {folder} differ by more than 1 ms in time from the reference's pose with the same index")
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
