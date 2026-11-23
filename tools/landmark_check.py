"""Landmark check: how far one identified wall lies from where it should be, measured from a trajectory in seconds.

  freeze <result> <out.npz> --time T0 T1 (--plan-fit plan_fit.json --plan-y Y | --point X Y Z --normal NX NY NZ) [--band 1.0]
      The returns of the wall, from a reference result's cloud (scan_lio_merged.laz): those whose point time is within
      [T0, T1) seconds after the result's first node and within --band metres of the expected line (the line and the band
      are measured in the reference result's frame, in plan view). Each return is stored with its time, its trajectory
      node and its position relative to that node. Choose the band wide against the offsets you expect to see (a return
      further from the line than the band in the reference map is never in the cohort, which would hide a large
      offset in another trajectory): the set records how many returns of the time window lie outside it.
      The expected line is a point and a normal in the reference result's world frame: either given directly, or from a
      plan fit (tools/plan_fit.py's map_quality_cache/plan_fit.json: rotation_deg, shift_m) and the wall's y on the plan:
      normal (sin a, cos a), the line is where that normal's coordinate plus shift_y equals --plan-y. The set records
      which plan fit it came from.

  from-cohort <cohort.npz> <trajectory> <out.npz> [--raw-laz raw.laz]
      The same set from a cohort chosen without any map (Codex's geometry_marked_cohort.npz: node, body = position
      relative to the node, rotation_deg, shift_x, shift_y, target_y, raw_ids; here the cohort of the user-identified
      gallery wall, 23,411 returns at 116 to 124 s). <trajectory> is the result whose node list `node` indexes.
      --raw-laz reads the returns' point times (s) from the recording's raw LAZ by raw_ids.

  measure <set.npz> <trajectory>... [--csv out.csv]
      A trajectory is a result or replay folder, an odometry snapshot (.bin) or label=path (as wall_pairs.py). Each return
      is placed with the trajectory; per trajectory the signed distance of the cohort to the line (positive along the
      normal): median, 90th percentile of the absolute distance, and the change of the signed median against the first
      trajectory. A trajectory whose node times differ from the set's is refused.

Anchor caveat: the absolute signed value depends on the plan alignment (the plan fit's rotation and shift, fixed for all
trajectories here; another anchor, e.g. a different start position, gives another absolute value: one study found
+0.548 m for the shipped trajectory where the fixed fit gave +0.017 m). The change between two trajectories on the
same set does not depend on it. One wall tests one feature, not the map; and a wall that helped choose a variant is a
selection check, not an independent one. Numbers are agreement with a drawing, not a verdict on the map.

Needs numpy; laspy for `freeze` and --raw-laz.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

import wall_pairs as wp

CAVEAT = ("the absolute value depends on the plan alignment (fixed fit: " "{fit}); the change between trajectories does not")


def line_from_plan_fit(rotation_deg, shift_xy, plan_y):
    """normal and point of the world line whose plan coordinate y equals plan_y: y_plan = sin(a) x + cos(a) y + shift_y."""
    a = np.deg2rad(rotation_deg)
    n = np.array([np.sin(a), np.cos(a), 0.0])
    c = float(shift_xy[1]) - float(plan_y)
    return -c * n, n


def _save(out, tr, t, node, local, p0, n, meta):
    n = np.asarray(n, float)
    n = n / np.linalg.norm(n)
    np.savez_compressed(out, meta=np.array(json.dumps(meta)), node_ns=tr.ns, t=np.asarray(t, float), node=np.asarray(node, np.int32),
                        local=np.asarray(local, float), line_point=np.asarray(p0, float), line_normal=n)


def freeze(result, out, time_range, line, band=1.0, plan_fit=None, label="", log=print):
    """line: (point, normal) in the reference result's world frame."""
    import laspy
    import map_quality_lib
    p0, n = (np.asarray(v, float) for v in line)
    n = n / np.linalg.norm(n)
    T, R, P = map_quality_lib.trajectory(str(result))
    tr = wp.Trajectory("reference", np.rint(T * 1e9).astype(np.int64), R, P)
    t0, t1 = time_range
    xs, ts, window = [], [], 0
    with laspy.open(str(Path(result) / "scan_lio_merged.laz")) as rd:
        for block in rd.chunk_iterator(1_000_000):
            tt = np.asarray(block.gps_time) * 1e-9
            m = (tt - T[0] >= t0) & (tt - T[0] < t1)
            if not m.any():
                continue
            x = np.column_stack([np.asarray(block.x)[m], np.asarray(block.y)[m], np.asarray(block.z)[m]])
            window += len(x)
            near = np.abs((x[:, :2] - p0[:2]) @ n[:2]) <= band
            xs.append(x[near])
            ts.append(tt[m][near])
    if not xs or sum(len(v) for v in xs) == 0:
        raise ValueError(f"no returns within {band} m of the line between {t0} and {t1} s")
    x, t = np.concatenate(xs), np.concatenate(ts)
    o = np.argsort(t, kind="stable")
    x, t = x[o], t[o]
    node = wp.node_of(t, tr.ns)
    if node.min() < 0:
        raise ValueError("selected returns lie before the first node")
    local = wp.to_local(x, node, tr)
    err = float(np.linalg.norm(wp.place(local, node, tr) - x, axis=1).max())
    meta = dict(kind="landmark", label=label, origin_result=str(Path(result).resolve()), time_s_after_first_node=[t0, t1], band_m=band,
                returns=int(len(t)), returns_in_time_window=int(window), plan_fit=plan_fit, max_round_trip_m=err,
                line_point=p0.tolist(), line_normal=n.tolist(), nodes=len(tr.ns),
                caveat=CAVEAT.format(fit=plan_fit or "line given directly"))
    _save(out, tr, t, node, local, p0, n, meta)
    log(f"frozen {out}: {len(t)} returns of {window} in the time window ({window - len(t)} further than {band} m from the line); "
        f"round trip {err:.2e} m")
    return meta


def from_cohort(cohort, trajectory, out, raw_laz=None, label="", log=print):
    z = np.load(cohort)
    tr = wp.load_trajectory(trajectory)
    node, body = np.asarray(z["node"], int), np.asarray(z["body"], float)
    if node.max() >= len(tr.ns):
        raise ValueError("the cohort's nodes are beyond the trajectory's: another trajectory")
    p0, n = line_from_plan_fit(float(z["rotation_deg"]), (float(z["shift_x"]), float(z["shift_y"])), float(z["target_y"]))
    t = np.full(len(node), np.nan)
    if raw_laz is not None:
        import laspy
        ids = np.asarray(z["raw_ids"], np.int64)
        order = np.argsort(ids)
        sid = ids[order]
        got = np.full(len(ids), np.nan)
        base = 0
        with laspy.open(str(raw_laz)) as rd:
            for block in rd.chunk_iterator(1_000_000):
                gt = np.asarray(block.gps_time)
                lo, hi = np.searchsorted(sid, [base, base + len(gt)])
                if hi > lo:
                    got[order[lo:hi]] = gt[sid[lo:hi] - base]
                base += len(gt)
        if np.isnan(got).any():
            raise ValueError(f"{int(np.isnan(got).sum())} raw_ids are not in {raw_laz}")
        t = got
        nn = wp.node_of(t, tr.ns)
        log(f"  point times read; nodes recomputed from them differ from the cohort's for {int((nn != node).sum())} of {len(node)} returns")
    fit = f"rotation {float(z['rotation_deg']):.4f} deg, shift ({float(z['shift_x']):.4f}, {float(z['shift_y']):.4f}) m, wall at plan y = {float(z['target_y']):g}"
    meta = dict(kind="landmark", label=label, origin_result=str(Path(trajectory).resolve()), cohort_file=str(Path(cohort).resolve()),
                returns=int(len(node)), plan_fit=fit, line_point=p0.tolist(), line_normal=n.tolist(), nodes=len(tr.ns),
                caveat=CAVEAT.format(fit=fit))
    _save(out, tr, t, node, body, p0, n, meta)
    log(f"converted {out}: {len(node)} returns; line from the plan fit ({fit})")
    return meta


class Landmark:
    def __init__(self, path):
        with np.load(path) as z:
            self.a = {k: z[k] for k in z.files}
        self.meta = json.loads(str(self.a["meta"]))
        self.path = str(path)
        self.node_ns = self.a["node_ns"]
        self.node, self.local = self.a["node"], self.a["local"]
        self.p0, self.n = self.a["line_point"], self.a["line_normal"]


def signed_distances(lm, tr):
    if len(tr.ns) != len(lm.node_ns) or not np.array_equal(tr.ns, lm.node_ns):
        raise ValueError(f"{tr.name}: node times differ from the set's ({len(tr.ns)} nodes against {len(lm.node_ns)}): another recording or another odometry")
    return (wp.place(lm.local, lm.node, tr) - lm.p0) @ lm.n


def measure(set_path, specs, csv_path=None, log=print):
    lm = Landmark(set_path)
    trs = [wp.load_trajectory(s) for s in specs]
    names = [t.name for t in trs]
    if len(set(names)) != len(names):
        raise ValueError(f"trajectory labels must differ: {names}")
    d = {tr.name: signed_distances(lm, tr) for tr in trs}
    m = lm.meta
    log(f"set {lm.path}")
    log(f"  {m.get('label') or 'landmark'}: {m['returns']} returns; reference {m['origin_result']}")
    log(f"  expected line: point {np.round(lm.p0, 4).tolist()}, normal {np.round(lm.n, 4).tolist()}; plan fit: {m.get('plan_fit')}")
    log(f"caveat: {m['caveat']}")
    log("signed distance of the cohort to the line [m] (positive along the normal): median / p90 of |distance| / change of the median against the first")
    w = max(len(n) for n in names + ["trajectory"])
    rows = []
    for k, tr in enumerate(trs):
        v = d[tr.name]
        med, p90 = float(np.median(v)), float(np.percentile(np.abs(v), 90))
        change = med - float(np.median(d[names[0]]))
        rows.append((tr.name, med, p90, change))
        log(f"  {tr.name.ljust(w)}  {med:+.3f} / {p90:.3f} / " + ("(reference)" if k == 0 else f"{change:+.3f}"))
    if csv_path:
        with open(csv_path, "w", newline="") as fp:
            import csv
            wr = csv.writer(fp)
            wr.writerow(["trajectory", "median_signed_m", "p90_abs_m", "change_vs_first_m", "returns"])
            for r in rows:
                wr.writerow([r[0], f"{r[1]:.6f}", f"{r[2]:.6f}", f"{r[3]:.6f}", m["returns"]])
        log(f"written to {csv_path}")
    return dict(distances=d, names=names, rows=rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("freeze")
    f.add_argument("result")
    f.add_argument("out")
    f.add_argument("--time", type=float, nargs=2, required=True, metavar=("T0", "T1"))
    f.add_argument("--plan-fit")
    f.add_argument("--plan-y", type=float)
    f.add_argument("--point", type=float, nargs=3)
    f.add_argument("--normal", type=float, nargs=3)
    f.add_argument("--band", type=float, default=1.0)
    f.add_argument("--label", default="")
    c = sub.add_parser("from-cohort")
    c.add_argument("cohort")
    c.add_argument("trajectory")
    c.add_argument("out")
    c.add_argument("--raw-laz")
    c.add_argument("--label", default="")
    m = sub.add_parser("measure")
    m.add_argument("set")
    m.add_argument("trajectory", nargs="+")
    m.add_argument("--csv")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "freeze":
            fit = None
            if a.plan_fit:
                if a.plan_y is None:
                    raise ValueError("--plan-fit needs --plan-y")
                pf = json.loads(Path(a.plan_fit).read_text())
                line = line_from_plan_fit(pf["rotation_deg"], pf["shift_m"], a.plan_y)
                fit = f"{Path(a.plan_fit).resolve()} (rotation {pf['rotation_deg']} deg, shift {pf['shift_m'][:2]} m), wall at plan y = {a.plan_y:g}"
            elif a.point and a.normal:
                line = (np.array(a.point), np.array(a.normal))
            else:
                raise ValueError("give the expected line: --plan-fit with --plan-y, or --point with --normal")
            freeze(a.result, a.out, a.time, line, a.band, fit, a.label)
        elif a.cmd == "from-cohort":
            from_cohort(a.cohort, a.trajectory, a.out, a.raw_laz, a.label)
        else:
            measure(a.set, a.trajectory, a.csv)
    except (ValueError, FileNotFoundError, KeyError) as e:
        print(f"landmark_check: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
