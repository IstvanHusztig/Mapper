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

Strict: timestamps are read exactly, as Decimal nanoseconds from the CSV text, and a result is compared only when it
has the reference's number of poses with the very same timestamps, strictly increasing, and every value finite.
Anything else is refused (ValueError; the CLI prints "refused" and exits 2) instead of compared approximately.
"""
import argparse
import math
import os
import sys
from decimal import Decimal, InvalidOperation


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


def load(folder, stage="final"):
    """The result's final poses as (Decimal nanosecond timestamp, x, y, z) in world coordinates, in time order.

    The timestamp is the CSV's text read exactly; a malformed row, a duplicate or a non-finite value is refused."""
    if stage != "final":
        raise ValueError(f"unknown stage {stage!r}: a result folder holds the final poses only")
    rows = []
    for name, m in chunk_poses(folder):
        fn = os.path.join(folder, name)
        if not os.path.isfile(fn):
            raise SystemExit(f"{folder}: session_poses.mrp lists {name}, which is missing")
        with open(fn) as f:
            f.readline()
            for number, line in enumerate(f, 2):
                v = line.split()
                if not v:
                    continue
                if len(v) < 13:
                    raise ValueError(f"{fn}, line {number}: {len(v)} fields, 13 expected")
                try:
                    t = Decimal(v[0])
                    x, y, z = float(v[4]), float(v[8]), float(v[12])
                except (InvalidOperation, ValueError):
                    raise ValueError(f"{fn}, line {number}: not a number") from None
                rows.append((t,
                             m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
                             m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
                             m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]))
    rows.sort(key=lambda r: r[0])
    validate(rows)
    return rows


def validate(rows):
    """Refuses (ValueError) poses that cannot be compared exactly: none, not (timestamp, x, y, z), a timestamp that is
    not an exact finite number (Decimal or int), timestamps not strictly increasing, a non-finite position."""
    if not rows:
        raise ValueError("no poses")
    previous = None
    for i, row in enumerate(rows):
        if len(row) != 4:
            raise ValueError(f"pose {i}: {len(row)} values, (timestamp, x, y, z) expected")
        t = row[0]
        if isinstance(t, bool) or not isinstance(t, (Decimal, int)):
            raise ValueError(f"pose {i}: timestamp {t!r} is not exact (Decimal or int nanoseconds)")
        if isinstance(t, Decimal) and not t.is_finite():
            raise ValueError(f"pose {i}: timestamp {t} is not finite")
        if not all(isinstance(c, (int, float)) and math.isfinite(c) for c in row[1:]):
            raise ValueError(f"pose {i}: position {row[1:]} is not finite")
        if previous is not None and not t > previous:
            raise ValueError(f"pose {i}: timestamp {t} does not follow {previous}")
        previous = t
    return rows


def threshold_key(thr):
    """'seconds_above_005' for 0.05 m: the threshold in centimetres, three digits ('p' for a decimal point)."""
    cm = Decimal(str(thr)) * 100
    return "seconds_above_" + (f"{int(cm):03d}" if cm == cm.to_integral_value() else str(cm.normalize()).replace(".", "p"))


def route(rows):
    return sum(math.dist(rows[i][1:], rows[i + 1][1:]) for i in range(len(rows) - 1))


def compare(ref, res, thresholds):
    """Pose-by-pose world position difference of res against ref. Both are validated, and they must have the same
    number of poses with identical timestamps; otherwise ValueError. Thresholds in metres, finite, positive.
    Besides the fields printed by the CLI: seconds_above_<cm> per threshold, the summed duration of the stretches
    above it (from the first to the last pose of each run of consecutive poses above)."""
    validate(ref)
    validate(res)
    if not thresholds or not all(isinstance(t, (int, float)) and math.isfinite(t) and t > 0 for t in thresholds):
        raise ValueError(f"thresholds must be finite and positive: {thresholds}")
    if len(ref) != len(res):
        raise ValueError(f"{len(res)} poses, the reference has {len(ref)}")
    for i, (a, b) in enumerate(zip(ref, res)):
        if a[0] != b[0]:
            raise ValueError(f"pose {i}: timestamp {b[0]}, the reference's is {a[0]}")
    n = len(ref)
    diff = [math.dist(ref[i][1:], res[i][1:]) for i in range(n)]
    t0 = ref[0][0]
    sec = lambda i: float(Decimal(ref[i][0] - t0) / Decimal(1_000_000_000))
    k = max(range(n), key=lambda i: diff[i])
    out = {"count": n, "max_diff_m": diff[k], "max_time_s": sec(k), "max_ref_position": ref[k][1:], "route_m": route(res)}
    out["first_above_s"] = {}
    for thr in thresholds:
        i = next((j for j in range(n) if diff[j] > thr), None)
        out["first_above_s"][thr] = None if i is None else sec(i)
    for thr in thresholds:
        nanoseconds = sum(ref[i + 1][0] - ref[i][0] for i in range(n - 1) if diff[i] > thr and diff[i + 1] > thr)
        out[threshold_key(thr)] = float(Decimal(nanoseconds) / Decimal(1_000_000_000))
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
    try:
        ref = load(a.reference)
    except ValueError as error:
        print(f"reference {a.reference} refused: {error}")
        return 2
    print(f"reference {a.reference}: {len(ref)} poses, route {route(ref):.2f} m")
    refused = 0
    for folder in a.results:
        try:
            res = load(folder)
            r = compare(ref, res, a.thresholds)
        except ValueError as error:
            print(f"{folder}: refused: {error}")
            refused += 1
            continue
        print(f"{folder}: route {r['route_m']:.2f} m")
        p = r["max_ref_position"]
        print(f"  max diff {r['max_diff_m']:.3f} m at t = {r['max_time_s']:.1f} s (reference position {p[0]:.1f} {p[1]:.1f} {p[2]:.1f})")
        print("  first >  " + "   ".join(f"{thr * 100:g} cm: " + ("never" if t is None else f"{t:.1f} s") for thr, t in r["first_above_s"].items()))
        if r["stretch"]:
            s, e, m = r["stretch"]
            print(f"  longest stretch above {a.thresholds[0] * 100:g} cm: {s:.1f} .. {e:.1f} s, largest {m:.3f} m")
        else:
            print(f"  nowhere above {a.thresholds[0] * 100:g} cm")
    return 2 if refused else 0


if __name__ == "__main__":
    sys.exit(main())
