"""Wall pairs from poses: how far apart two visits of the same wall lie, for any trajectory of a recording, on a set of
returns frozen beforehand (change faster-experiments, design decision 5). No cloud is read by `measure`.

  freeze <result> <wall piece csv> <out.npz> [--plan-walls]
      From a result's cloud (scan_lio_merged.laz) and trajectory and a wall_precision.py piece table
      (<result>/map_quality_cache/wall_precision_+0.50.csv), select each piece's returns per visit (same-side visits only:
      the side is the side of the piece's line the sensor was on; a wall seen from both sides shows two faces, so each
      side is its own set of visits), and store for every return its time, its trajectory node and its position relative
      to that node (the node pose inverse applied), with the piece's position and normal, the visit and the side, the
      origin result and the counts. The selection is checked against the table (visit count and times, and the offset
      difference of each pair within --pair-tol); pieces or pairs that do not match are left out and counted.
      --plan-walls keeps only the pieces on the plan's walls: plan_dist_m below --plan-dist (the table must come from
      wall_precision.py --plan) and a direction within --align-deg of the length-weighted dominant direction (mod 90).
      Without it every piece with visits is a candidate. The set says which of the two it is.

  measure <set.npz> <trajectory>... [--threshold 0.08] [--before T1] [--after T2] [--csv out.csv]
      A trajectory is a result or replay folder (trajectory_lio_*.csv), an odometry snapshot (.bin) or label=path.
      Each return is placed with the trajectory (its node's pose times the stored relative position); each pair is brought
      into the frame of the earlier visit's node pose (the rotation that maps the trajectory's orientation there onto the
      frozen result's, so a rigid move of the whole trajectory gives zero); the gap is the difference of the two visits'
      median positions along the piece's normal. Per trajectory: median, 90th percentile and the number at or above the
      threshold of |gap|, for all pairs and, with --before / --after, for both visits before T1 (the later visit ends
      before) and for the earlier visit before T1 and the later after T2. Then the pairs that cross the threshold
      between trajectories. A trajectory whose node times differ from the set's is refused.

The numbers are agreement between visits on a selected set, not accuracy. A set chosen on one result favours that
result's walls: compare rules on a set frozen before the rules were run. The set's origin is printed with every table.

Needs numpy, scipy and laspy (freeze only). Time classes use the visits' point times in seconds as in the piece table.
"""
import argparse
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SAME_PIECE_RADIUS = 0.85   # candidate search around a piece's centre [m]; a piece is at most about 0.7 m across
VISIT_TIME_TOL = 0.051     # the piece table prints visit times with 0.1 s resolution


class Trajectory:
    """Node times (int64 ns), world rotations (n,3,3) and positions (n,3)."""

    def __init__(self, name, ns, rot, pos):
        self.name, self.ns, self.rot, self.pos = name, np.asarray(ns, np.int64), np.asarray(rot, float), np.asarray(pos, float)
        if not (len(self.ns) == len(self.rot) == len(self.pos)) or len(self.ns) == 0:
            raise ValueError(f"{name}: inconsistent trajectory arrays")
        if np.any(np.diff(self.ns) <= 0):
            raise ValueError(f"{name}: node times are not strictly increasing")


def load_trajectory(spec):
    """A result or replay folder, an odometry snapshot (.bin) or label=path."""
    label, _, path = spec.partition("=") if "=" in spec and not Path(spec).exists() else ("", "", spec)
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"{path}: no such trajectory")
    label = label or (p.stem if p.is_file() else p.name)
    if p.is_file():
        import snapshot_trajectory
        rows, poses = snapshot_trajectory.load_snapshot(p, with_poses=True)
        m = np.array(poses, float).reshape(-1, 4, 4).transpose(0, 2, 1)   # stored column-major
        return Trajectory(label, [int(round(r[0])) for r in rows], m[:, :3, :3], m[:, :3, 3])
    import map_quality_lib
    t, rot, pos = map_quality_lib.trajectory(str(p))
    return Trajectory(label, np.rint(t * 1e9).astype(np.int64), rot, pos)


def node_of(t_s, node_ns):
    """Node index of each return time: the last node strictly before it (the production window-local lower_bound - 1),
    in integer nanoseconds, so a return that coincides with a node belongs to the node before. -1: before the first."""
    ns = np.rint(np.asarray(t_s) * 1e9).astype(np.int64)
    return np.searchsorted(node_ns, ns, side="left") - 1


def to_local(x, node, tr):
    """Position relative to the node: the node pose inverse applied."""
    return np.einsum("nji,nj->ni", tr.rot[node], x - tr.pos[node])


def place(local, node, tr):
    """The stored positions placed with a trajectory (same node indices)."""
    return np.einsum("nij,nj->ni", tr.rot[node], local) + tr.pos[node]


# ---- freeze ----------------------------------------------------------------------------------------------------------
def _read_slice(result, height, half_thickness):
    import laspy
    xs, ts = [], []
    with laspy.open(str(Path(result) / "scan_lio_merged.laz")) as rd:
        for block in rd.chunk_iterator(1_000_000):
            z = np.asarray(block.z)
            m = np.abs(z - height) <= half_thickness
            if m.any():
                xs.append(np.column_stack([np.asarray(block.x)[m], np.asarray(block.y)[m], z[m]]))
                ts.append(np.asarray(block.gps_time)[m] * 1e-9)
    if not xs:
        raise ValueError(f"no returns within {half_thickness} m of z = {height} in {result}")
    return np.concatenate(xs), np.concatenate(ts)


def _select_pieces(rows, plan_walls, plan_dist, align_deg):
    rows = [r for r in rows if r["visit_times_s"]]
    if not plan_walls:
        return rows
    if rows and "plan_dist_m" not in rows[0]:
        raise ValueError("--plan-walls needs a piece table made with wall_precision.py --plan (no plan_dist_m column)")
    hist = defaultdict(float)
    for r in rows:
        hist[round(float(r["dir_deg"]) % 90)] += float(r["length_m"])
    peak = max(hist, key=hist.get)
    return [r for r in rows if float(r["plan_dist_m"]) < plan_dist and abs((float(r["dir_deg"]) - peak + 45) % 90 - 45) <= align_deg]


def freeze(result, table, out, plan_walls=False, height=None, half_thickness=0.05, across=0.2, gap=2.0, min_visit_points=15,
           pair_tol=0.0012, align_deg=5.0, plan_dist=0.06, log=print):
    from scipy.spatial import cKDTree
    import map_quality_lib
    if height is None:
        m = re.search(r"wall_precision_([+-]\d+(?:\.\d+)?)", Path(table).name)
        if not m:
            raise ValueError("the height of the section is not in the table's name: pass --height")
        height = float(m.group(1))
    with open(table, newline="") as fp:
        all_rows = list(csv.DictReader(fp))
    rows = _select_pieces(all_rows, plan_walls, plan_dist, align_deg)
    T, R, P = map_quality_lib.trajectory(str(result))
    tr = Trajectory("origin", np.rint(T * 1e9).astype(np.int64), R, P)
    x, t = _read_slice(result, height, half_thickness)
    sensor = np.column_stack([np.interp(t, T, P[:, k]) for k in range(3)])
    tree = cKDTree(x[:, :2])
    visits, pairs = [], []   # visit: dict(piece, side, idx); pair: (visit a, visit b, origin gap)
    stats = dict(table_pieces=len(all_rows), candidate_pieces=len(rows), mismatched_pieces=0, excluded_pairs_piece=0,
                 excluded_pairs_offset=0, pieces_used=0)
    pieces = []
    for r in rows:
        c = np.array([float(r["x"]), float(r["y"])])
        a = np.deg2rad(float(r["dir_deg"]))
        tangent = np.array([np.cos(a), np.sin(a)])
        n = np.array([-tangent[1], tangent[0]])
        idx = np.array(tree.query_ball_point(c, SAME_PIECE_RADIUS), int)
        idx = idx[np.abs((x[idx, :2] - c) @ n) <= across]
        want = int(r["points"])
        npairs_table = int(r["pairs"])
        if len(idx) < want:
            stats["mismatched_pieces"] += 1
            stats["excluded_pairs_piece"] += npairs_table
            continue
        idx = idx[np.argsort(np.abs((x[idx, :2] - c) @ tangent))[:want]]   # the table's point count fixes the piece's ends
        side = np.where((sensor[idx, :2] - c) @ n >= 0.0, 1, -1)
        vs = []
        for s_ in (-1, 1):
            ix = idx[side == s_]
            ix = ix[np.argsort(t[ix])]
            for v in np.split(ix, np.flatnonzero(np.diff(t[ix]) > gap) + 1):
                if len(v) >= min_visit_points:
                    vs.append((s_, v))
        vs.sort(key=lambda z: t[z[1][0]])
        expect = [tuple(map(float, k.split("-"))) for k in r["visit_times_s"].split(";")]
        if len(vs) != len(expect) or any(max(abs(t[v[0]] - lo), abs(t[v[-1]] - hi)) > VISIT_TIME_TOL for (_, v), (lo, hi) in zip(vs, expect)):
            stats["mismatched_pieces"] += 1
            stats["excluded_pairs_piece"] += npairs_table
            continue
        offsets = np.array(r["visit_offsets_m"].split(";"), float)
        pid = len(pieces)
        pieces.append(dict(table_piece=int(r["piece"]), x=c[0], y=c[1], nx=n[0], ny=n[1]))
        first = len(visits)
        used = set()
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                if vs[i][0] != vs[j][0]:
                    continue
                g = np.median(x[vs[j][1], :2] @ n) - np.median(x[vs[i][1], :2] @ n)
                if abs(abs(g) - abs(offsets[j] - offsets[i])) > pair_tol:
                    stats["excluded_pairs_offset"] += 1
                    continue
                used |= {i, j}
                pairs.append((first + i, first + j, g))
        if not used:
            pieces.pop()
            continue
        stats["pieces_used"] += 1
        for i, (s_, v) in enumerate(vs):   # every visit keeps its slot so the pair indices stay valid; unused ones are empty
            visits.append(dict(piece=pid, side=s_, idx=v if i in used else v[:0]))
    # compact: drop empty visits, renumber
    keep = [i for i, v in enumerate(visits) if len(v["idx"])]
    renum = {old: new for new, old in enumerate(keep)}
    visits = [visits[i] for i in keep]
    pairs = [(renum[a], renum[b], g) for a, b, g in pairs]
    if not pairs:
        raise ValueError("no pair survived the selection")
    ridx = np.concatenate([v["idx"] for v in visits])
    counts = np.array([len(v["idx"]) for v in visits])
    start = np.r_[0, np.cumsum(counts)[:-1]]
    node = node_of(t[ridx], tr.ns)
    if node.min() < 0:
        raise ValueError(f"{int((node < 0).sum())} selected returns lie before the first node")
    world = x[ridx]
    local = to_local(world, node, tr)
    err = np.linalg.norm(place(local, node, tr) - world, axis=1)
    anchor = np.array([node[start[a] + counts[a] // 2] for a, _, _ in pairs])
    meta = dict(origin_result=str(Path(result).resolve()), piece_table=str(Path(table).resolve()),
                selection="plan walls (plan_dist_m < %g, direction within %g deg of the dominant)" % (plan_dist, align_deg) if plan_walls else "all pieces with visits",
                height_m=height, half_thickness_m=half_thickness, across_m=across, visit_gap_s=gap, min_visit_points=min_visit_points,
                pair_tolerance_m=pair_tol, nodes=len(tr.ns), pieces=len(pieces), visits=len(visits), pairs=len(pairs), returns=int(len(ridx)),
                max_round_trip_m=float(err.max()), **stats)
    np.savez_compressed(
        out, meta=np.array(json.dumps(meta)), node_ns=tr.ns, t=t[ridx], node=node.astype(np.int32), local=local, world=world,
        visit_start=start, visit_count=counts, visit_piece=np.array([v["piece"] for v in visits]), visit_side=np.array([v["side"] for v in visits]),
        visit_t0=np.array([t[v["idx"][0]] for v in visits]), visit_t1=np.array([t[v["idx"][-1]] for v in visits]),
        piece_table_id=np.array([p["table_piece"] for p in pieces]), piece_xy=np.array([[p["x"], p["y"]] for p in pieces]),
        piece_normal=np.array([[p["nx"], p["ny"], 0.0] for p in pieces]),
        pair_a=np.array([p[0] for p in pairs]), pair_b=np.array([p[1] for p in pairs]), pair_origin_gap=np.array([p[2] for p in pairs]),
        pair_anchor=anchor, pair_anchor_rot=tr.rot[anchor])
    log(f"frozen {out}: {meta['pieces']} pieces, {meta['visits']} visits, {meta['pairs']} same-side pairs, {meta['returns']} returns")
    log(f"  origin {meta['origin_result']}; selection: {meta['selection']}; table pieces {stats['table_pieces']}, candidates {stats['candidate_pieces']}")
    log(f"  left out: {stats['mismatched_pieces']} pieces that did not match the table ({stats['excluded_pairs_piece']} table pairs), "
        f"{stats['excluded_pairs_offset']} pairs whose offset difference differs from the table's by more than {pair_tol * 1000:g} mm")
    log(f"  returns placed again with the origin trajectory: largest distance to the cloud {meta['max_round_trip_m']:.2e} m")
    return meta


# ---- measure ---------------------------------------------------------------------------------------------------------
class WallSet:
    def __init__(self, path):
        with np.load(path) as z:
            self.a = {k: z[k] for k in z.files}
        self.meta = json.loads(str(self.a["meta"]))
        self.path = str(path)

    def __getattr__(self, k):
        try:
            return self.__dict__["a"][k]
        except KeyError:
            raise AttributeError(k)


def check_nodes(ws, tr):
    if len(tr.ns) != len(ws.node_ns) or not np.array_equal(tr.ns, ws.node_ns):
        raise ValueError(f"{tr.name}: node times differ from the set's ({len(tr.ns)} nodes against {len(ws.node_ns)}"
                         + ("" if len(tr.ns) != len(ws.node_ns) else f"; first difference at node {int(np.argmax(tr.ns != ws.node_ns))}")
                         + "): another recording or another odometry")


def reproduction_error(ws, tr):
    """Distance of each stored return, placed with the trajectory, from its position in the frozen result's cloud."""
    check_nodes(ws, tr)
    return np.linalg.norm(place(ws.local, ws.node, tr) - ws.world, axis=1)


def pair_gaps(ws, tr):
    """Signed gap along the piece normal for every pair: later visit's median minus the earlier's, in the frame of the
    earlier visit's node pose."""
    check_nodes(ws, tr)
    y = place(ws.local, ws.node, tr)
    gaps = np.empty(len(ws.pair_a))
    for k, (a, b) in enumerate(zip(ws.pair_a, ws.pair_b)):
        n = ws.piece_normal[ws.visit_piece[a]]
        align = ws.pair_anchor_rot[k] @ tr.rot[ws.pair_anchor[k]].T
        m = align.T @ n
        ya = y[ws.visit_start[a]:ws.visit_start[a] + ws.visit_count[a]]
        yb = y[ws.visit_start[b]:ws.visit_start[b] + ws.visit_count[b]]
        gaps[k] = np.median(yb @ m) - np.median(ya @ m)
    return gaps


def classes(ws, before, after):
    """name -> boolean mask over the pairs"""
    t0, t1 = ws.visit_t0, ws.visit_t1
    a, b = ws.pair_a, ws.pair_b
    out = {"all pairs": np.ones(len(a), bool)}
    if before is not None:
        out[f"both visits before {before:g} s"] = t1[b] < before
        if after is not None:
            out[f"before {before:g} s / after {after:g} s"] = (t1[a] < before) & (t0[b] > after)
    return out


def summarize(g, threshold):
    v = np.abs(g)
    if len(v) == 0:
        return None
    return float(np.median(v)), float(np.percentile(v, 90)), int((v >= threshold).sum()), len(v)


def measure(set_path, specs, threshold=0.08, before=None, after=None, csv_path=None, log=print):
    ws = WallSet(set_path)
    trs = [load_trajectory(s) for s in specs]
    names = [t.name for t in trs]
    if len(set(names)) != len(names):
        raise ValueError(f"trajectory labels must differ: {names}")
    for tr in trs:
        check_nodes(ws, tr)
    gaps = {tr.name: pair_gaps(ws, tr) for tr in trs}
    m = ws.meta
    log(f"set {ws.path}")
    log(f"  frozen on {m['origin_result']}; selection: {m['selection']}")
    log(f"  {m['pieces']} pieces, {m['visits']} visits, {m['pairs']} same-side pairs, {m['returns']} returns; height {m['height_m']:+.2f} m")
    log("numbers are agreement between visits on a selected set, not accuracy")
    cls = classes(ws, before, after)
    log(f"gap along the piece normal [m], |gap|: median / p90 / count at or above {threshold * 1000:g} mm")
    w = max(len(n) for n in names + ["trajectory"])
    for cname, mask in cls.items():
        log(f"  {cname} ({int(mask.sum())} pairs)")
        for tr in trs:
            s = summarize(gaps[tr.name][mask], threshold)
            log(f"    {tr.name.ljust(w)}  " + ("NOT MEASURED (no pair)" if s is None else f"{s[0]:.3f} / {s[1]:.3f} / {s[2]}"))
    for tr in trs:
        e = reproduction_error(ws, tr)
        log(f"  {tr.name}: returns moved from the frozen cloud, median {np.median(e):.3f} m, max {e.max():.3f} m")
    if len(trs) > 1:
        above = np.array([np.abs(gaps[n]) >= threshold for n in names])
        log(f"pairs crossing {threshold * 1000:g} mm between trajectories (of {above.shape[1]}):")
        for p, q in zip(names, names[1:]):
            up = int((~above[names.index(p)] & above[names.index(q)]).sum())
            down = int((above[names.index(p)] & ~above[names.index(q)]).sum())
            log(f"  {p} -> {q}: {up} cross above, {down} fall below")
        hist = defaultdict(int)
        for col in above.T:
            hist[tuple(np.array(names)[col])] += 1
        log("  history (trajectories at or above the threshold): " + "; ".join(
            f"{'none' if not k else '+'.join(k)} {v}" for k, v in sorted(hist.items(), key=lambda kv: (len(kv[0]), kv[0]))))
    if csv_path:
        with open(csv_path, "w", newline="") as fp:
            wr = csv.writer(fp)
            wr.writerow(["pair", "table_piece", "side", "earlier_t0_s", "earlier_t1_s", "later_t0_s", "later_t1_s", *[f"gap_{n}_m" for n in names]])
            for k, (a, b) in enumerate(zip(ws.pair_a, ws.pair_b)):
                wr.writerow([k, int(ws.piece_table_id[ws.visit_piece[a]]), int(ws.visit_side[a]), ws.visit_t0[a], ws.visit_t1[a], ws.visit_t0[b],
                             ws.visit_t1[b], *[f"{gaps[n][k]:.6f}" for n in names]])
        log(f"per-pair gaps written to {csv_path}")
    return dict(gaps=gaps, names=names, set=ws)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("freeze", help="select and store the returns of the wall pieces' visits")
    f.add_argument("result")
    f.add_argument("table")
    f.add_argument("out")
    f.add_argument("--plan-walls", action="store_true")
    f.add_argument("--height", type=float)
    f.add_argument("--half-thickness", type=float, default=0.05)
    f.add_argument("--across", type=float, default=0.2)
    f.add_argument("--gap", type=float, default=2.0)
    f.add_argument("--min-visit-points", type=int, default=15)
    f.add_argument("--pair-tol", type=float, default=0.0012)
    f.add_argument("--align-deg", type=float, default=5.0)
    f.add_argument("--plan-dist", type=float, default=0.06)
    m = sub.add_parser("measure", help="the gap of every frozen pair on one or more trajectories")
    m.add_argument("set")
    m.add_argument("trajectory", nargs="+")
    m.add_argument("--threshold", type=float, default=0.08)
    m.add_argument("--before", type=float)
    m.add_argument("--after", type=float)
    m.add_argument("--csv")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "freeze":
            freeze(a.result, a.table, a.out, a.plan_walls, a.height, a.half_thickness, a.across, a.gap, a.min_visit_points, a.pair_tol,
                   a.align_deg, a.plan_dist)
        else:
            measure(a.set, a.trajectory, a.threshold, a.before, a.after, a.csv)
    except (ValueError, FileNotFoundError) as e:
        print(f"wall_pairs: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
