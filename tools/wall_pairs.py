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

  freeze-union <out.npz> [label=]<result>=<wall piece csv> ... [the freeze options]
      A set that is not tied to one map: every origin is selected as `freeze` does (each against its own cloud and piece
      table; all on one trajectory node grid), then merged. Same wall: a pair of a later origin is the pair already kept
      when its two visits share at least 50 % of the smaller visit's returns with the kept pair's two visits (either
      order). The returns are the same physical measurements in every map of a recording, so their point times identify
      them whatever the trajectory; the 0.2 m admission rule of `freeze` then no longer ties the set to one map, since
      a wall seen 1 m apart in one map is admitted by another. A pair found in several maps is counted once, the first
      origin's piece and normal win, and the set stores every origin (labels, results, tables, counts) and for each pair
      the origins that found it (--by-origin on `measure` prints them). A wall piece split differently in two maps and
      overlapping by less than half is kept twice: the union is an upper bound on distinct walls, not a count of them.

  measure <set.npz> <trajectory>... [--threshold 0.08] [--severe 0.30] [--before T1] [--after T2] [--by-origin] [--csv out.csv]
      A trajectory is a result or replay folder (trajectory_lio_*.csv), an odometry snapshot (.bin) or label=path.
      Each return is placed with the trajectory (its node's pose times the stored relative position); each pair is brought
      into the frame of the earlier visit's node pose (the rotation that maps the trajectory's orientation there onto the
      frozen result's, so a rigid move of the whole trajectory gives zero); the gap is the difference of the two visits'
      median positions along the piece's normal. Per trajectory: median, 90th percentile and the number at or above the
      threshold of |gap|, for all pairs and, with --before / --after, for both visits before T1 (the later visit ends
      before) and for the earlier visit before T1 and the later after T2; also the count at or above --severe (30 cm) and
      the largest |gap|. A set of plan-independent contacts (contacts_to_wall_pairs.py) has several pairs per contact (a
      contact's gap is their median) and adds a class per storey. Then the pairs that cross the threshold
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
SAME_WALL_OVERLAP = 0.5    # union: two visits are the same stretch of wall when this share of the smaller one's returns is common


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


def _collect(result, table, plan_walls=False, height=None, half_thickness=0.05, across=0.2, gap=2.0, min_visit_points=15,
             pair_tol=0.0012, align_deg=5.0, plan_dist=0.06):
    """The pieces, visits (each with its returns' times, nodes, relative and world positions) and pairs of one result."""
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
    out_visits = []
    err_max = 0.0
    for v in visits:
        ix = v["idx"]
        node = node_of(t[ix], tr.ns)
        if node.min() < 0:
            raise ValueError(f"{int((node < 0).sum())} selected returns lie before the first node")
        local = to_local(x[ix], node, tr)
        err_max = max(err_max, float(np.linalg.norm(place(local, node, tr) - x[ix], axis=1).max()))
        out_visits.append(dict(piece=v["piece"], side=v["side"], t=t[ix], node=node, local=local, world=x[ix]))
    out_pairs = []
    for a, b, g in pairs:
        va = out_visits[a]
        anchor = int(va["node"][len(va["node"]) // 2])
        out_pairs.append(dict(a=a, b=b, gap=float(g), anchor=anchor, anchor_rot=tr.rot[anchor], mask=1))
    return dict(tr=tr, pieces=pieces, visits=out_visits, pairs=out_pairs, stats=stats, max_round_trip_m=err_max,
                params=dict(height_m=height, half_thickness_m=half_thickness, across_m=across, visit_gap_s=gap,
                            min_visit_points=min_visit_points, pair_tolerance_m=pair_tol),
                selection=("plan walls (plan_dist_m < %g, direction within %g deg of the dominant)" % (plan_dist, align_deg)
                           if plan_walls else "all pieces with visits"),
                result=str(Path(result).resolve()), table=str(Path(table).resolve()))


def _overlap(t_a, t_b):
    """Share of the smaller visit's returns that the other visit has too. The returns are the same physical measurements
    in every map of one recording, so their point times (ns) identify them whatever the trajectory."""
    if len(t_a) == 0 or len(t_b) == 0 or t_a.max() < t_b.min() or t_b.max() < t_a.min():
        return 0.0
    na, nb = np.rint(t_a * 1e9).astype(np.int64), np.rint(t_b * 1e9).astype(np.int64)
    return len(np.intersect1d(na, nb)) / min(len(na), len(nb))


def _union(cols, min_overlap=SAME_WALL_OVERLAP):
    """Merge the origins. A pair of a later origin is the same wall pair as one already kept when its two visits share
    at least min_overlap of the smaller visit's returns with the kept pair's two visits (in either order); it is then
    counted once and only marks its origin in the kept pair's mask. The first origin's pieces, returns and normals win."""
    base = cols[0]["tr"]
    for c in cols[1:]:
        if not np.array_equal(c["tr"].ns, base.ns):
            raise ValueError("the results' node times differ: the union needs one trajectory node grid")
    pieces, visits, pairs = [], [], []
    for k, c in enumerate(cols):
        known = [(p, visits[p["a"]], visits[p["b"]]) for p in pairs]   # only pairs of earlier origins can match
        vmap, pmap = {}, {}
        for p in c["pairs"]:
            va, vb = c["visits"][p["a"]], c["visits"][p["b"]]
            hit = None
            for q, qa, qb in known:
                if (_overlap(va["t"], qa["t"]) >= min_overlap and _overlap(vb["t"], qb["t"]) >= min_overlap) or \
                        (_overlap(va["t"], qb["t"]) >= min_overlap and _overlap(vb["t"], qa["t"]) >= min_overlap):
                    hit = q
                    break
            if hit is not None:
                hit["mask"] |= 1 << k
                continue
            ids = []
            for vi in (p["a"], p["b"]):
                if vi not in vmap:
                    v = dict(c["visits"][vi])
                    pc = v["piece"]
                    if pc not in pmap:
                        pmap[pc] = len(pieces)
                        pieces.append(dict(c["pieces"][pc], origin=k))
                    v["piece"] = pmap[pc]
                    vmap[vi] = len(visits)
                    visits.append(v)
                ids.append(vmap[vi])
            pairs.append(dict(p, a=ids[0], b=ids[1], mask=1 << k))
    return pieces, visits, pairs


def _save(out, cols, labels, pieces, visits, pairs, union):
    tr = cols[0]["tr"]
    counts = np.array([len(v["t"]) for v in visits])
    start = np.r_[0, np.cumsum(counts)[:-1]]
    cat = lambda key: np.concatenate([v[key] for v in visits])
    origins = [dict(label=lb, result=c["result"], piece_table=c["table"], selection=c["selection"], **c["params"], **c["stats"],
                    pairs_found=int(sum(1 for p in pairs if p["mask"] >> i & 1)))
               for i, (lb, c) in enumerate(zip(labels, cols))]
    meta = {k: v for k, v in origins[0].items() if k not in ("label", "result", "pairs_found")}
    meta.update(origin_result=origins[0]["result"], nodes=len(tr.ns), pieces=len(pieces), visits=len(visits), pairs=len(pairs),
                returns=int(counts.sum()), max_round_trip_m=max(c["max_round_trip_m"] for c in cols))
    if union:
        meta.update(origin_result="union of " + "; ".join(f"{o['label']} = {o['result']}" for o in origins), origins=origins,
                    selection=f"union of {len(origins)} results, a pair found in several counted once (its visits share at least "
                              f"{SAME_WALL_OVERLAP:g} of the smaller visit's returns, by point time)")
    arrays = dict(
        meta=np.array(json.dumps(meta)), node_ns=tr.ns, t=cat("t"), node=cat("node").astype(np.int32), local=cat("local"), world=cat("world"),
        visit_start=start, visit_count=counts, visit_piece=np.array([v["piece"] for v in visits]), visit_side=np.array([v["side"] for v in visits]),
        visit_t0=np.array([v["t"][0] for v in visits]), visit_t1=np.array([v["t"][-1] for v in visits]),
        piece_table_id=np.array([p["table_piece"] for p in pieces]), piece_xy=np.array([[p["x"], p["y"]] for p in pieces]),
        piece_normal=np.array([[p["nx"], p["ny"], 0.0] for p in pieces]),
        pair_a=np.array([p["a"] for p in pairs]), pair_b=np.array([p["b"] for p in pairs]), pair_origin_gap=np.array([p["gap"] for p in pairs]),
        pair_anchor=np.array([p["anchor"] for p in pairs]), pair_anchor_rot=np.array([p["anchor_rot"] for p in pairs]))
    if union:
        arrays.update(pair_origin_mask=np.array([p["mask"] for p in pairs], np.int64), piece_origin=np.array([p["origin"] for p in pieces]),
                      origin_labels=np.array(labels))
    np.savez_compressed(out, **arrays)
    return meta, origins


def freeze(result, table, out, plan_walls=False, height=None, half_thickness=0.05, across=0.2, gap=2.0, min_visit_points=15,
           pair_tol=0.0012, align_deg=5.0, plan_dist=0.06, log=print):
    col = _collect(result, table, plan_walls, height, half_thickness, across, gap, min_visit_points, pair_tol, align_deg, plan_dist)
    meta, _ = _save(out, [col], ["origin"], col["pieces"], col["visits"], col["pairs"], union=False)
    stats = col["stats"]
    log(f"frozen {out}: {meta['pieces']} pieces, {meta['visits']} visits, {meta['pairs']} same-side pairs, {meta['returns']} returns")
    log(f"  origin {meta['origin_result']}; selection: {meta['selection']}; table pieces {stats['table_pieces']}, candidates {stats['candidate_pieces']}")
    log(f"  left out: {stats['mismatched_pieces']} pieces that did not match the table ({stats['excluded_pairs_piece']} table pairs), "
        f"{stats['excluded_pairs_offset']} pairs whose offset difference differs from the table's by more than {pair_tol * 1000:g} mm")
    log(f"  returns placed again with the origin trajectory: largest distance to the cloud {meta['max_round_trip_m']:.2e} m")
    return meta


def freeze_union(specs, out, plan_walls=False, log=print, **kw):
    """specs: [label=]<result>=<piece table> ..., each frozen as freeze does, then merged (see _union)."""
    parsed = []
    for s in specs:
        parts = s.split("=")
        if len(parts) == 2:
            parts = [Path(parts[0]).name] + parts
        if len(parts) != 3:
            raise ValueError(f"{s}: expected [label=]<result>=<piece table>")
        parsed.append(parts)
    labels = [p[0] for p in parsed]
    if len(set(labels)) != len(labels):
        raise ValueError(f"labels must differ: {labels}")
    cols = [_collect(r, t, plan_walls, **kw) for _, r, t in parsed]
    pieces, visits, pairs = _union(cols)
    meta, origins = _save(out, cols, labels, pieces, visits, pairs, union=True)
    log(f"frozen union {out}: {meta['pieces']} pieces, {meta['visits']} visits, {meta['pairs']} same-side pairs, {meta['returns']} returns")
    for i, (o, c) in enumerate(zip(origins, cols)):
        only = sum(1 for p in pairs if p["mask"] == 1 << i)
        log(f"  origin {o['label']} ({o['result']}): {len(c['pairs'])} pairs of its own, {o['pairs_found']} of the union's found in it, "
            f"{only} found only in it; table pieces {o['table_pieces']}, left out {o['mismatched_pieces']} pieces / "
            f"{o['excluded_pairs_piece']} + {o['excluded_pairs_offset']} pairs")
    multi = sum(1 for p in pairs if bin(p["mask"]).count("1") > 1)
    log(f"  {multi} pairs found in more than one map, counted once; same wall = both visits share at least "
        f"{SAME_WALL_OVERLAP:g} of the smaller visit's returns (point times)")
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


def contacts_of(ws):
    """Index of the contact each pair belongs to. A set made from wall pieces has one pair per contact; a set of
    plan-independent contacts (contacts_to_wall_pairs.py) has one pair per shared cell, and a contact's gap is the median
    of its cells' gaps."""
    if "pair_contact" in ws.a:
        return np.asarray(ws.pair_contact, int)
    return np.arange(len(ws.pair_a))


def contact_gaps(ws, tr):
    """The gap of every contact: the pair gap itself, or the median over a contact's pairs."""
    g = pair_gaps(ws, tr)
    c = contacts_of(ws)
    if len(c) == 0 or np.array_equal(c, np.arange(len(g))):
        return g
    order = np.argsort(c, kind="stable")
    cs = c[order]
    starts = np.r_[0, np.flatnonzero(np.diff(cs)) + 1]
    return np.array([np.median(part) for part in np.split(g[order], starts[1:])])


def contact_times(ws):
    """Time ranges (earlier t0, t1, later t0, t1) of every contact: stored for contact sets, else of the pair's visits."""
    if "contact_t" in ws.a:
        z = np.asarray(ws.contact_t)
        return z[:, 0], z[:, 1], z[:, 2], z[:, 3]
    a, b = ws.pair_a, ws.pair_b
    return ws.visit_t0[a], ws.visit_t1[a], ws.visit_t0[b], ws.visit_t1[b]


def classes(ws, before, after, by_origin=False):
    """name -> boolean mask over the contacts"""
    ta0, ta1, tb0, tb1 = contact_times(ws)
    out = {"all pairs": np.ones(len(ta0), bool)}
    if before is not None:
        out[f"both visits before {before:g} s"] = tb1 < before
        if after is not None:
            out[f"before {before:g} s / after {after:g} s"] = (ta1 < before) & (tb0 > after)
    if "contact_level" in ws.a:
        for lev in np.unique(ws.contact_level):
            out[f"storey {int(lev)}"] = np.asarray(ws.contact_level) == lev
    if by_origin and "pair_origin_mask" in ws.a:
        mask = np.asarray(ws.pair_origin_mask)
        for i, lb in enumerate(ws.origin_labels):
            out[f"found in {lb}"] = (mask >> i & 1).astype(bool)
            out[f"found only in {lb}"] = mask == (1 << i)
    return out


def summarize(g, threshold, severe=0.30):
    """median, 90th percentile, count at or above threshold, count at or above severe, largest, number of |g|"""
    v = np.abs(g)
    if len(v) == 0:
        return None
    return float(np.median(v)), float(np.percentile(v, 90)), int((v >= threshold).sum()), int((v >= severe).sum()), float(v.max()), len(v)


def measure(set_path, specs, threshold=0.08, before=None, after=None, csv_path=None, severe=0.30, by_origin=False, log=print):
    ws = WallSet(set_path)
    trs = [load_trajectory(s) for s in specs]
    names = [t.name for t in trs]
    if len(set(names)) != len(names):
        raise ValueError(f"trajectory labels must differ: {names}")
    for tr in trs:
        check_nodes(ws, tr)
    gaps = {tr.name: contact_gaps(ws, tr) for tr in trs}
    m = ws.meta
    log(f"set {ws.path}")
    log(f"  frozen on {m['origin_result']}; selection: {m['selection']}")
    ncontacts = len(next(iter(gaps.values())))
    log(f"  {m['pieces']} pieces, {m['visits']} visits, {m['pairs']} same-side pairs, {m['returns']} returns"
        + ("" if m.get("height_m") is None else f"; height {m['height_m']:+.2f} m")
        + ("" if ncontacts == m["pairs"] else f"; {ncontacts} contacts (a contact's gap is the median of its pairs')"))
    log("numbers are agreement between visits on a selected set, not accuracy")
    cls = classes(ws, before, after, by_origin)
    log(f"gap along the piece normal [m], |gap|: median / p90 / count at or above {threshold * 1000:g} mm / count at or above "
        f"{severe * 1000:g} mm / largest")
    w = max(len(n) for n in names + ["trajectory"])
    for cname, mask in cls.items():
        log(f"  {cname} ({int(mask.sum())} pairs)")
        for tr in trs:
            s = summarize(gaps[tr.name][mask], threshold, severe)
            log(f"    {tr.name.ljust(w)}  " + ("NOT MEASURED (no pair)" if s is None else f"{s[0]:.3f} / {s[1]:.3f} / {s[2]} / {s[3]} / {s[4]:.3f}"))
    for tr in trs:
        e = reproduction_error(ws, tr)
        log(f"  {tr.name}: returns moved from the frozen cloud, median {np.median(e):.3f} m, max {e.max():.3f} m"
            + (" (each return against the cloud of the origin it came from)" if "origins" in m else ""))
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
        first = {}
        for k, c in enumerate(contacts_of(ws)):
            first.setdefault(int(c), k)
        ta0, ta1, tb0, tb1 = contact_times(ws)
        with open(csv_path, "w", newline="") as fp:
            wr = csv.writer(fp)
            wr.writerow(["pair", "table_piece", "side", "earlier_t0_s", "earlier_t1_s", "later_t0_s", "later_t1_s", *[f"gap_{n}_m" for n in names]])
            for c, k in sorted(first.items()):
                a = ws.pair_a[k]
                wr.writerow([c, int(ws.piece_table_id[ws.visit_piece[a]]), int(ws.visit_side[a]), ta0[c], ta1[c], tb0[c], tb1[c],
                             *[f"{gaps[n][c]:.6f}" for n in names]])
        log(f"per-pair gaps written to {csv_path}")
    return dict(gaps=gaps, names=names, set=ws)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("freeze", help="select and store the returns of the wall pieces' visits")
    f.add_argument("result")
    f.add_argument("table")
    f.add_argument("out")
    u = sub.add_parser("freeze-union", help="one set from several results: the union of their pieces, stored with every origin")
    u.add_argument("out")
    u.add_argument("origin", nargs="+", help="[label=]<result folder>=<wall piece csv>")
    for q in (f, u):
        q.add_argument("--plan-walls", action="store_true")
        q.add_argument("--height", type=float)
        q.add_argument("--half-thickness", type=float, default=0.05)
        q.add_argument("--across", type=float, default=0.2)
        q.add_argument("--gap", type=float, default=2.0)
        q.add_argument("--min-visit-points", type=int, default=15)
        q.add_argument("--pair-tol", type=float, default=0.0012)
        q.add_argument("--align-deg", type=float, default=5.0)
        q.add_argument("--plan-dist", type=float, default=0.06)
    m = sub.add_parser("measure", help="the gap of every frozen pair on one or more trajectories")
    m.add_argument("set")
    m.add_argument("trajectory", nargs="+")
    m.add_argument("--threshold", type=float, default=0.08)
    m.add_argument("--severe", type=float, default=0.30, help="second count: gaps at or above this [m] (default 0.30)")
    m.add_argument("--before", type=float)
    m.add_argument("--after", type=float)
    m.add_argument("--by-origin", action="store_true", help="a union set: also the pairs found in, and only in, each origin's map")
    m.add_argument("--csv")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "freeze":
            freeze(a.result, a.table, a.out, a.plan_walls, a.height, a.half_thickness, a.across, a.gap, a.min_visit_points, a.pair_tol,
                   a.align_deg, a.plan_dist)
        elif a.cmd == "freeze-union":
            freeze_union(a.origin, a.out, a.plan_walls, height=a.height, half_thickness=a.half_thickness, across=a.across, gap=a.gap,
                         min_visit_points=a.min_visit_points, pair_tol=a.pair_tol, align_deg=a.align_deg, plan_dist=a.plan_dist)
        else:
            measure(a.set, a.trajectory, a.threshold, a.before, a.after, a.csv, a.severe, a.by_origin)
    except (ValueError, FileNotFoundError) as e:
        print(f"wall_pairs: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
