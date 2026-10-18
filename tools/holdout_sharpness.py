"""Map sharpness on points the processing never fitted: results made with the hold-out mode (`[evaluation]
holdout_block_s` > 0) leave the points of odd time blocks out of every map and point term, but save them. This tool
measures only those held-out points, so a stage that fits its own points better without placing the walls better
(overfitting) shows as worse, not better.

At --n query points (chosen once, on the first result of a set, from the held-out points): the neighbours within --r of
the query among the held-out points, their PCA, and for planar spots (middle/largest eigenvalue > 0.2, smallest/middle
< 0.3, at least 12 neighbours) the thickness sqrt(smallest eigenvalue), taken
  all-time  from all held-out neighbours (the whole map at that spot), and
  slice     from those within +-0.05 s of the query's time (short-term node-to-node jitter).
The planar set is decided on the reference and kept for every result of the set, and the thickness is paired spot by
spot; the paired median difference [mm] gets a bootstrap interval over the spots. All results of a set must hold the
same points (same recording, same hold-out). With several sets (repeat runs such as s0/s1/s2), the sets' differences
are also combined: their mean and an interval from the lowest to the highest set's bootstrap bound, which includes the
run-to-run spread that a spot bootstrap alone misses.

Usage:
  holdout_sharpness.py [--block 0.01] [--fitted] <reference> <result> [<result> ...]
  holdout_sharpness.py [--block 0.01] --sets <ref1>,<res1> <ref2>,<res2> ...
--fitted measures the even (fitted) blocks instead, for comparison. Needs numpy, scipy, laspy (with lazrs).
"""
import argparse
import os

import laspy
import numpy as np
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument("results", nargs="+")
ap.add_argument("--block", type=float, default=0.01, help="holdout_block_s of the runs [s]")
ap.add_argument("--fitted", action="store_true", help="measure the even (fitted) blocks instead of the held-out ones")
ap.add_argument("--sets", action="store_true", help="arguments are reference,result pairs of repeat runs")
ap.add_argument("--r", type=float, default=0.12, help="neighbourhood radius [m]")
ap.add_argument("--n", type=int, default=30000, help="query points")
ap.add_argument("--half", type=float, default=0.05, help="half width of the slice in time [s]")
ap.add_argument("--seed", type=int, default=7)
a = ap.parse_args()
parity = 0 if a.fitted else 1


def load(result):
    f = laspy.read(os.path.join(result, "scan_lio_merged.laz"))
    t = np.asarray(f.gps_time) * 1e-9
    keep = np.floor(t / a.block).astype(np.int64) % 2 == parity
    return np.column_stack([f.x, f.y, f.z])[keep], t[keep]


def thickness(P, T, q):
    tree = cKDTree(P)
    out = np.full((len(q), 2), np.nan)
    planar = np.zeros((len(q), 2), bool)
    for j, (qi, ids) in enumerate(zip(q, tree.query_ball_point(P[q], a.r))):
        ids = np.asarray(ids)
        for m, sel in enumerate((np.abs(T[ids] - T[qi]) <= a.half, np.ones(len(ids), bool))):
            s = ids[sel]
            if len(s) < 12:
                continue
            X = P[s] - P[s].mean(axis=0)
            ev = np.linalg.eigvalsh(X.T @ X / len(s))
            planar[j, m] = ev[1] / ev[2] > 0.2 and ev[0] / ev[1] < 0.3
            out[j, m] = np.sqrt(max(ev[0], 0.0))
    return out, planar


def name(result):
    return result.replace("\\", "/").rstrip("/").split("/")[-1]


def compare(reference, others, rng):
    """paired all-time and slice differences [mm] of each result against the reference, with bootstrap intervals"""
    P0, T0 = load(reference)
    q = rng.choice(len(P0), min(a.n, len(P0)), replace=False)
    th0, planar = thickness(P0, T0, q)
    for m, label in enumerate(("slice", "all-time")):
        v = th0[planar[:, m], m] * 1e3
        print(f"{name(reference):20s} {label:8s}: {np.isfinite(v).sum()} planar spots, thickness p50 {np.nanmedian(v):.2f} mm")
    out = []
    for r in others:
        P, T = load(r)
        if len(P) != len(P0) or not np.allclose(T, T0):
            raise SystemExit(f"{r} holds other points than {reference} (another recording or hold-out?)")
        th, _ = thickness(P, T, q)
        row = []
        for m, label in enumerate(("slice", "all-time")):
            dd = (th[:, m] - th0[:, m])[planar[:, m]] * 1e3
            dd = dd[np.isfinite(dd)]
            bs = [np.median(rng.choice(dd, len(dd))) for _ in range(300)]
            lo, hi = np.percentile(bs, 2.5), np.percentile(bs, 97.5)
            row.append((np.median(dd), lo, hi))
            print(f"{name(r):20s} {label:8s}: thickness p50 {np.nanmedian(th[planar[:, m], m]) * 1e3:.2f} mm, paired "
                  f"difference to {name(reference)} {np.median(dd):+.2f} mm (bootstrap 95% {lo:+.2f} .. {hi:+.2f})")
        out.append(row)
    return out


rng = np.random.default_rng(a.seed)
print(f"{'fitted (even)' if a.fitted else 'held-out (odd)'} blocks of {a.block} s")
if not a.sets:
    if len(a.results) < 2:
        raise SystemExit("give a reference and at least one result")
    compare(a.results[0], a.results[1:], rng)
else:
    per_set = []
    for pair in a.results:
        ref, res = pair.split(",")
        per_set.append(compare(ref, [res], rng)[0])
    for m, label in enumerate(("slice", "all-time")):
        meds = [s[m][0] for s in per_set]
        print(f"over {len(per_set)} sets, {label}: paired difference mean {np.mean(meds):+.2f} mm (sets "
              f"{', '.join(f'{v:+.2f}' for v in meds)}), interval {min(s[m][1] for s in per_set):+.2f} .. "
              f"{max(s[m][2] for s in per_set):+.2f} mm")
