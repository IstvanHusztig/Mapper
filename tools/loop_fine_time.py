#!/usr/bin/env python3
"""Fine-time placement of a blind stretch after dMapper's loop closure, in Python: the prototype of the loop closure's last
step (OpenSpec change `loop-closure-fine-time-guard`), written to the frozen rule
`experiments/consistency/livox4_lever/fine_time/RULE.md` (sections quoted as "rule 2" ... "rule 8" below).

What it does, for each weak run of the readings rule whose sigma is at least 1 m (rule 2):
  * compares the scans of every window of the run and of 6 windows on either side with surfaces across the run's axis that
    windows at least 12 s away saw (rule 3-5), over a grid of offsets along the axis;
  * fits a smooth profile of offsets, zero at both ends, by dynamic programming (rule 6);
  * reads every kept loop edge that touches the stretch at the times of its own information (rule 7) and classes it
    (agrees, unconfirmed, contradicted);
  * applies the profile if it is supported, the held-out points improve and no edge is contradicted (rule 8).

Inputs: a loop closure capture (`loop_closure_capture.bin`, format 2: it carries the windows' readings), the trajectory
after dMapper's loop closure (a result or replay folder), the kept loop edges (a replay folder's `loop_closure_edges.json`,
a json or a text list, as `loop_graph.py` takes them) and the recording's `cache` folder with the windows'
`intermediate_points_<w>.cache` (node frame; written by any run or `--from-snapshot` run on the recording).

Outputs in the out folder: `trajectory/` (a trajectory folder that `wall_pairs.py measure` and
`cloud_from_trajectory.py` read), `windows.csv` (one row per window of each stretch), `edges.csv` (rule 7's table),
`summary.json` (runs, what was done with each and why).

What it does NOT do: no registration, no pose graph (the trajectory is taken as given), no pruning. The loop edges'
information per source window (`H_w`) is rebuilt here from the points, at the registered pose, with cells as
`update_rgd` makes them with exact statistics; dMapper's own will be the last refinement pass's.

  loop_fine_time.py <capture> <trajectory folder> <edges> <cache folder> <out folder>
                    [--coupling C] [--min-matched N] [--min-sigma M] [--straight-gaps] [--fit-total]

`--straight-gaps` is the form of `experiments/consistency/livox4_lever/zigzag/RULE.md` section A (OpenSpec change
`blind-stretch-zigzag`): the total of the correction and the offset runs straight between the supported windows; `--fit-total`
is its section B (the profile's coupling on that total; implies `--straight-gaps`). Without them the rule is today's, and every
old column, file and key of the outputs is unchanged. With or without them the fall of every weak run (section A7) is in
`summary.json` and `windows.csv` ends with the correction, the effective offset and the total of each window.
"""
import argparse
import csv
import json
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import loop_graph as lg  # noqa: E402
import wall_pairs as wp  # noqa: E402

# a point of `intermediate_points_<w>.cache` (binary_io's Point3Di: point, timestamp, intensity, index_pose, lidarid, index_point)
CACHE_POINT = np.dtype([("xyz", "<f8", (3,)), ("t", "<f8"), ("intensity", "<f4"), ("node", "<i4"), ("id", "u1"), ("index", "<i4")])


@dataclass(frozen=True)
class Rule:
    """The constants of RULE.md."""
    min_sigma_m: float = 1.0        # rule 2: eligible runs
    flank_windows: int = 6          # rule 2: loop_closure_segment_windows
    separation_s: float = 12.0      # rule 4
    neighbours: int = 14            # rule 3
    neighbour_reach_m: float = 0.65
    flat: float = 0.12              # l0 < flat * l1
    wide: float = 0.08              # l1 > wide * l2
    across: float = 0.5             # |normal . axis| >= across
    segment_voxel_m: float = 0.1    # segment_points
    reference_voxel_m: float = 0.10  # rule 4
    source_voxel_m: float = 0.07    # rule 5
    box_margin_m: float = 1.5       # rule 4: beyond the bound
    bound_sigmas: float = 1.5       # rule 5
    bound_max_m: float = 8.0
    grid_m: float = 0.1
    holdout_cell_m: float = 1.0
    holdout_one_in: int = 5
    max_patches: int = 256
    min_patches: int = 8
    match_neighbours: int = 4
    match_reach_m: float = 0.5
    normal_agreement: float = 0.80
    min_matched: int = 8            # rule 5: supported
    coupling: float = 0.10          # rule 6
    cost_tolerance: float = 1e-12   # rule 1
    informed_share: float = 0.5     # rule 7
    cell_m: float = 0.25            # rule 7: the refinement's finest cells
    cell_min_points: int = 3
    cell_max_information: float = 1e5
    cell_initial_variance: float = 0.03 * 0.03
    footprint_cell_m: float = 0.5   # cross-floor registrations


HASH = np.array([73856093, 19349663, 83492791], np.int64)


# ---- small pieces (each tested without data in tests/test_loop_fine_time.py) -------------------------------------------
def voxel_keys(x, v):
    k = np.floor(np.asarray(x, float) / v).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def thin(x, v):
    """Indices of the points kept: the first in input order of every voxel of side v, in input order (rule 1)."""
    if len(x) == 0:
        return np.zeros(0, np.int64)
    _, first = np.unique(voxel_keys(x, v), return_index=True)
    return np.sort(first)


def patches(x, axis, v, rule=Rule()):
    """Rule 3. -> (indices into x of the patches' points, their unit normals), in input order."""
    keep = thin(x, v)
    y = np.asarray(x, float)[keep]
    if len(y) < 16:
        return np.zeros(0, np.int64), np.zeros((0, 3))
    dist, nn = cKDTree(y).query(y, k=rule.neighbours)
    idx, normals = [], []
    for s in range(0, len(y), 40000):
        nb = y[nn[s:s + 40000]]
        nb = nb - nb.mean(axis=1, keepdims=True)
        vals, vecs = np.linalg.eigh(nb.transpose(0, 2, 1) @ nb)
        nrm = vecs[:, :, 0]
        ok = ((dist[s:s + 40000, -1] < rule.neighbour_reach_m) & (vals[:, 0] < rule.flat * vals[:, 1])
              & (vals[:, 1] > rule.wide * vals[:, 2]) & (np.abs(nrm @ axis) >= rule.across))
        idx.append(s + np.flatnonzero(ok))
        normals.append(nrm[ok])
    idx = np.concatenate(idx)
    return keep[idx], np.concatenate(normals)


def held_out(q, rule=Rule()):
    """Rule 5: True for the patches held out, by a spatial hash of their position."""
    cell = np.floor(np.asarray(q, float) / rule.holdout_cell_m).astype(np.int64)
    return (cell @ HASH) % rule.holdout_one_in == 0


def take(n, limit):
    """Rule 5: the indices taken of n patches when at most `limit` are used."""
    if n <= limit:
        return np.arange(n)
    return (np.arange(limit, dtype=np.int64) * (n - 1)) // (limit - 1)


def offset_grid(sigma, rule=Rule()):
    bound = min(rule.bound_sigmas * sigma, rule.bound_max_m)
    k = int(np.floor(bound * round(1.0 / rule.grid_m)))
    return np.arange(-k, k + 1) * rule.grid_m, bound


def score_curve(q, n, grid, axis, tree, ref_normals, rule=Rule()):
    """Rule 5 for one kind of patches of one window: (mean squared distance per offset, matched patches per offset)."""
    y = q[None, :, :] + grid[:, None, None] * axis[None, None, :]
    ds, ix = tree.query(y, k=rule.match_neighbours, distance_upper_bound=rule.match_reach_m)
    found = ix < len(ref_normals)
    ix = np.minimum(ix, len(ref_normals) - 1)
    ok = found & (np.abs(np.einsum("gpkj,pj->gpk", ref_normals[ix], n)) >= rule.normal_agreement)
    d = np.where(ok, ds, rule.match_reach_m).min(axis=2)
    return (d ** 2).mean(axis=1), ok.any(axis=2).sum(axis=1)


def preference(grid):
    """Rule 6: the grid indices in the order of preference among equal costs: smaller |d| first, the negative first."""
    return np.lexsort((grid > 0, np.abs(np.rint(grid * 1e6))))


def _best(cand, pref, tol):
    """Per column of cand (rows: candidates), the lowest row, rows tried in the order `pref`, replaced only when lower by
    more than tol. -> (cost, row)."""
    best = cand[pref[0]].copy()
    arg = np.full(cand.shape[1], pref[0])
    for r in pref[1:]:
        better = cand[r] < best - tol
        best = np.where(better, cand[r], best)
        arg = np.where(better, r, arg)
    return best, arg


def profile(E, grid, rule=Rule()):
    """Rule 6. E: (windows, offsets), zero rows for unsupported windows. -> the offset index of every window."""
    pref = preference(grid)
    trans = rule.coupling * (grid[:, None] - grid[None, :]) ** 2
    zero = int(np.argmin(np.abs(grid)))
    val = trans[zero] + E[0]
    back = []
    for e in E[1:]:
        best, arg = _best(val[:, None] + trans, pref, rule.cost_tolerance)
        back.append(arg)
        val = best + e
    _, last = _best((val + trans[:, zero])[:, None], pref, rule.cost_tolerance)
    path = [int(last[0])]
    for arg in back[::-1]:
        path.append(int(arg[path[-1]]))
    return np.array(path[::-1])


def profile_total(E, grid, c, c_pin0, c_pin1, rule=Rule()):
    """RULE (zigzag) section B: the profile with the coupling on the total t_w = c_w + d_w instead of on d_w. E: (windows,
    offsets); c: the correction at each window's middle node; c_pin0, c_pin1: the correction at the two pins. -> the offset
    index of every window. Same grid, order of preference and tolerance as `profile`."""
    pref = preference(grid)
    k = rule.coupling
    val = k * (c[0] + grid - c_pin0) ** 2 + E[0]
    back = []
    for w in range(1, len(E)):
        trans = k * ((c[w] + grid[None, :]) - (c[w - 1] + grid[:, None])) ** 2
        best, arg = _best(val[:, None] + trans, pref, rule.cost_tolerance)
        back.append(arg)
        val = best + E[w]
    _, last = _best((val + k * (c_pin1 - (c[-1] + grid)) ** 2)[:, None], pref, rule.cost_tolerance)
    path = [int(last[0])]
    for arg in back[::-1]:
        path.append(int(arg[path[-1]]))
    return np.array(path[::-1])


def weak_runs(rd, weak_ratio=0.05, axis_tolerance_deg=30.0, accelerometer_error=0.05):
    """The readings rule's runs (`loop_odometry_step_information`, steps 1-3), from a capture's readings."""
    cos_tol = np.cos(np.radians(axis_tolerance_deg))
    # readable, as the C++: finite readings and not at rest (a window at rest is never weak, bridged or a pace reference)
    ok = (rd["present"] & np.isfinite(rd["ratio"]) & np.isfinite(rd["dur"]) & (np.nan_to_num(rd["dur"]) > 0)
          & np.isfinite(rd["weak"]).all(axis=1) & np.isfinite(rd["move"]).all(axis=1) & ~rd["rest"])
    weak = ok & (np.nan_to_num(rd["ratio"], nan=1.0) < weak_ratio)
    W = rd["W"]
    runs, k = [], 0
    while k < W:
        if not weak[k]:
            k += 1
            continue
        a0, e = rd["weak"][k], k

        def same(j):
            return weak[j] and abs(rd["weak"][j] @ a0) >= cos_tol

        while True:
            if e + 1 < W and same(e + 1):
                e += 1
            elif e + 2 < W and ok[e + 1] and same(e + 2):
                e += 2
            else:
                break
        b = np.arange(k, e + 1)
        bb = b[weak[b]]
        ax = (rd["weak"][bb] * np.sign(rd["weak"][bb] @ a0)[:, None]).mean(0)
        ax /= np.linalg.norm(ax)
        if rd["move"][b].sum(0) @ ax < 0:
            ax = -ax
        dur = rd["dur"][b]
        T = float(dur.sum())
        v = (rd["move"][b] @ ax) / dur
        seeing = ok & ~weak
        before = np.flatnonzero(seeing[:k])[-3:]
        after = (e + 1 + np.flatnonzero(seeing[e + 1:]))[:3]

        def pace(w):
            return float(np.median(np.linalg.norm(rd["move"][w], axis=1) / rd["dur"][w]))

        vb = pace(before) if len(before) else np.nan
        va = pace(after) if len(after) else np.nan
        vb, va = (va if np.isnan(vb) else vb), (vb if np.isnan(va) else va)
        if np.isnan(vb):
            vb = va = float(np.mean(v))
        f = (np.cumsum(dur) - dur / 2) / T
        s2 = float(np.sum(np.abs(v - (vb + (va - vb) * f)) * dur))
        s1 = 0.5 * accelerometer_error * T * T
        angle = float(np.degrees(np.arccos(np.clip(np.abs(rd["weak"][bb] @ ax), 0.0, 1.0)).max()))
        runs.append(dict(first=int(k), last=int(e), weak_windows=int(len(bb)), T=T, axis=ax, s1=s1, s2=s2,
                         sigma=float(np.hypot(s1, s2)), largest_axis_angle_deg=angle))
        k = e + 1
    return runs


def edge_class(q, rho_before, rho_after, limit, rule=Rule()):
    """Rule 7's class of an edge that touches an eligible run with scores."""
    if q < rule.informed_share:
        return "unconfirmed"
    if abs(rho_after) <= limit or abs(rho_after) <= abs(rho_before):
        return "agrees"
    return "contradicted"


def time_shares(H_w, a0):
    """Rule 7: s_w from the per-window information (W, 3, 3) along a0; None when the edge has none along it."""
    h = np.einsum("i,wij,j->w", a0, H_w, a0)
    return h / h.sum() if h.sum() > 0 else None


# ---- the data ----------------------------------------------------------------------------------------------------------
class Data:
    """Capture, trajectory after the loop closure, and the windows' points."""

    def __init__(self, capture, trajectory, cache, rule=Rule()):
        self.cap = lg.read_capture(capture) if not isinstance(capture, lg.Capture) else capture
        c = self.cap
        if c.readings is None:
            raise ValueError("the capture has no window readings (format 1): the readings rule's runs cannot be found")
        tr = wp.load_trajectory(str(trajectory)) if not isinstance(trajectory, wp.Trajectory) else trajectory
        if not np.array_equal(tr.ns, np.rint(c.node_t * 1e9).astype(np.int64)):
            raise ValueError("the trajectory's nodes are not the capture's")
        self.rule = rule
        self.rebuild_information = False  # True: rule 7's H_w from the points even when the capture carries dMapper's
        self.cache = Path(cache) if cache is not None else None
        self.P0 = c.node_pose
        self.P1 = np.tile(np.eye(4), (len(tr.pos), 1, 1))
        self.P1[:, :3, :3] = tr.rot
        self.P1[:, :3, 3] = tr.pos
        self.first = c.win_first_node
        self.W = len(self.first) - 1
        t = c.node_t
        self.start = t[self.first[:-1]]
        self.end = np.r_[self.start[1:], t[-1]]
        self.mid = (self.start + self.end) / 2
        # the first node of the window at or after its middle time
        self.mid_node = np.array([self.first[w] + min(int(np.searchsorted(t[self.first[w]:self.first[w + 1]], self.mid[w], side="left")),
                                                         self.first[w + 1] - self.first[w] - 1) for w in range(self.W)])
        self.r_min = c.setting("min_distance_lidar", 0.0)
        self.r_max = c.setting("max_distance_lidar", 1e9)
        self._seg = {}
        self._win = {}

    def window_points(self, w):
        """(local points, global node index) of window w with segment_points' filters, in file order."""
        if w not in self._win:
            r = np.fromfile(self.cache / f"intermediate_points_{w}.cache", CACHE_POINT)
            dist = np.linalg.norm(r["xyz"], axis=1)
            n_nodes = self.first[w + 1] - self.first[w]
            ok = (dist >= self.r_min) & (dist <= self.r_max) & (r["node"] >= 0) & (r["node"] < n_nodes)
            self._win[w] = (r["xyz"][ok].astype(float), self.first[w] + r["node"][ok].astype(np.int64))
            if len(self._win) > 80:
                self._win.pop(next(iter(self._win)))
        return self._win[w]

    @staticmethod
    def place(P, node, local):
        return np.einsum("nij,nj->ni", P[node, :3, :3], local) + P[node, :3, 3]

    def segment(self, k):
        """What `segment_points` holds of segment k: (odometry-placed points, local points, node, window), thinned to 0.1 m."""
        if k not in self._seg:
            s = self.cap.segments[k]
            loc, node, win = [], [], []
            for w in range(s["first"], s["last"] + 1):
                p, n = self.window_points(w)
                loc.append(p)
                node.append(n)
                win.append(np.full(len(n), w))
            loc, node, win = np.concatenate(loc), np.concatenate(node), np.concatenate(win)
            x = self.place(self.P0, node, loc)
            keep = thin(x, self.rule.segment_voxel_m)
            self._seg[k] = (x[keep], loc[keep], node[keep], win[keep])
        return self._seg[k]


# ---- rule 2: runs and supports -------------------------------------------------------------------------------------------
def find_stretches(data, min_sigma=None):
    """Every weak run with its support if eligible: adds eligible, sup_first, sup_last, pin0, pin1, a0, a, overlap."""
    c, rule = data.cap, data.rule
    min_sigma = rule.min_sigma_m if min_sigma is None else min_sigma
    runs = weak_runs(c.readings, c.setting("loop_closure_odometry_weak_ratio", 0.05),
                     c.setting("loop_closure_odometry_axis_tolerance_deg", 30.0),
                     c.setting("loop_closure_odometry_accelerometer_error_mps2", 0.05))
    for r in runs:
        r["eligible"] = r["sigma"] >= min_sigma
        r["a0"] = r.pop("axis")
        r["overlap"] = False
        if not r["eligible"]:
            continue
        r["sup_first"] = max(0, r["first"] - rule.flank_windows)
        r["sup_last"] = min(data.W - 1, r["last"] + rule.flank_windows)
        r["pin0"], r["pin1"] = float(data.start[r["sup_first"]]), float(data.end[r["sup_last"]])
        tm = (data.start[r["first"]] + data.end[r["last"]]) / 2
        n = min(int(np.searchsorted(c.node_t, tm, side="left")), len(c.node_t) - 1)
        K = data.P1[n] @ lg.inv(data.P0[n])
        r["a"] = K[:3, :3] @ r["a0"]
    el = [r for r in runs if r["eligible"]]
    for x in el:
        for y in el:
            if x is not y and x["sup_first"] <= y["sup_last"] and y["sup_first"] <= x["sup_last"]:
                x["overlap"] = True
    return runs


# ---- rules 4 and 5: reference and score curves ---------------------------------------------------------------------------
def score_stretch(data, run, runs):
    """The score curves of an eligible run's support. -> dict with grid, windows, E (training), T (held-out), counts."""
    rule = data.rule
    a = run["a"]
    grid, bound = offset_grid(run["sigma"], rule)
    windows = np.arange(run["sup_first"], run["sup_last"] + 1)
    src = []
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for w in windows:
        p, node = data.window_points(w)
        x = data.place(data.P1, node, p)
        src.append(x)
        if len(x):
            lo, hi = np.minimum(lo, x.min(0)), np.maximum(hi, x.max(0))
    # rule 4: the reference
    qualifying = (data.end <= run["pin0"] - rule.separation_s) | (data.start >= run["pin1"] + rule.separation_s)
    for r in runs:
        if r["eligible"]:
            qualifying[r["sup_first"]:r["sup_last"] + 1] = False
    margin = bound + rule.box_margin_m
    ref = []
    for k in range(len(data.cap.segments)):
        s = data.cap.segments[k]
        if not qualifying[s["first"]:s["last"] + 1].any():
            continue
        _, loc, node, win = data.segment(k)
        m = qualifying[win]
        x = data.place(data.P1, node[m], loc[m])
        ref.append(x[np.all((x >= lo - margin) & (x <= hi + margin), axis=1)])
    ref = np.concatenate(ref) if ref else np.zeros((0, 3))
    ri, rn = patches(ref, a, rule.reference_voxel_m, rule)
    X = ref[ri]
    tree = cKDTree(X) if len(X) else None
    G = len(grid)
    out = dict(grid=grid, bound=bound, windows=windows, reference_patches=int(len(X)),
               E=np.full((len(windows), G), np.nan), T=np.full((len(windows), G), np.nan),
               matched=np.zeros((len(windows), G), int), n_train=np.zeros(len(windows), int), n_test=np.zeros(len(windows), int))
    for j, x in enumerate(src):
        pi, n = patches(x, a, rule.source_voxel_m, rule)
        q = x[pi]
        held = held_out(q, rule)
        for kind, idx in (("E", np.flatnonzero(~held)), ("T", np.flatnonzero(held))):
            idx = idx[take(len(idx), rule.max_patches)]
            out["n_train" if kind == "E" else "n_test"][j] = len(idx)
            if len(idx) < rule.min_patches or tree is None:
                continue
            e, m = score_curve(q[idx], n[idx], grid, a, tree, rn, rule)
            out[kind][j] = e
            if kind == "E":
                out["matched"][j] = m
    return out


def solve_stretch(sc, rule, total=None):
    """Rule 5's supported windows and rule 6's profile for given curves. -> (supported (windows,), grid index per window).
    total = (c per window, c at the first pin, c at the last pin): the coupling on the total (zigzag RULE section B)."""
    supported = np.isfinite(sc["E"][:, 0]) & (sc["matched"].max(axis=1) >= rule.min_matched)
    E = np.where(supported[:, None], np.nan_to_num(sc["E"]), 0.0)
    if total is not None:
        return supported, profile_total(E, sc["grid"], *total, rule)
    return supported, profile(E, sc["grid"], rule)


def own_offset(e, grid, rule=Rule()):
    """The offset one window's curve prefers on its own (a reading)."""
    _, arg = _best(e[:, None], preference(grid), rule.cost_tolerance)
    return float(grid[arg[0]])


def edge_name(e):
    return f"{e[0]} {'=>' if e[2] else '->'} {e[1]}"


def node_offsets(data, run, windows, offsets):
    """Rule 6: the profile on every node [m] (0 outside the support)."""
    t = np.r_[run["pin0"], data.mid[windows], run["pin1"]]
    return np.interp(data.cap.node_t, t, np.r_[0.0, offsets, 0.0], left=0.0, right=0.0)


# ---- zigzag RULE section A: the total runs straight between the supported windows -------------------------------------------
def corrected_axis(data, run):
    """Rule 2's axis a for any weak run (an eligible run has it as run["a"])."""
    tm = (data.start[run["first"]] + data.end[run["last"]]) / 2
    n = min(int(np.searchsorted(data.cap.node_t, tm, side="left")), len(data.cap.node_t) - 1)
    return (data.P1[n] @ lg.inv(data.P0[n]))[:3, :3] @ run["a0"]


def correction(data, a, P=None):
    """A1: c(n) = a . (position after the loop closure - odometry position) of every node (P: other poses than the capture's
    trajectory after the loop closure)."""
    P = data.P1 if P is None else P
    return (P[:, :3, 3] - data.P0[:, :3, 3]) @ a


def pin_nodes(data, run):
    """A1: the nodes whose times are the support's two pins (the first node of the window after the support, or the
    recording's last node)."""
    return int(data.first[run["sup_first"]]), int(min(data.first[run["sup_last"] + 1], len(data.cap.node_t) - 1))


def fall(c):
    """A7: max over w of (max over v <= w of s c_v - s c_w), s the sign of c[-1] - c[0] (+1 when equal)."""
    c = np.asarray(c, float)
    if len(c) == 0:
        return 0.0
    sc = (1.0 if c[-1] >= c[0] else -1.0) * c
    return float(max(0.0, (np.maximum.accumulate(sc) - sc).max()))


def straight_knots(data, run, windows, supported, d, c):
    """A2: the knots (times, totals): first pin, every supported window at its middle node, last pin."""
    t = data.cap.node_t
    n0, n1 = pin_nodes(data, run)
    mids = data.mid_node[windows][supported]
    kt = np.r_[t[n0], t[mids], t[n1]]
    kc = np.r_[c[n0], c[mids] + d[supported], c[n1]]
    if not np.all(np.diff(kt) > 0):
        raise ValueError("the knots' times do not strictly increase (RULE zigzag A2)")
    return kt, kc


def round_offsets(e, grid, rule=Rule()):
    """A4: the grid index of each effective offset: the nearest 0.1 k; a middle between two integers goes to the one nearer 0;
    held to the grid's ends."""
    x = np.asarray(e, float) / rule.grid_m
    fl = np.floor(x)
    tie = np.abs((x - fl) - 0.5) < 1e-9
    k = np.where(tie, np.where(fl >= 0, fl, fl + 1), np.floor(x + 0.5))
    kmax = (len(grid) - 1) // 2
    return np.clip(k, -kmax, kmax).astype(int) + kmax


def straight_shift(data, run, kt, kc, c):
    """A3: every node of the support moves by (T(t) - c(n)) a; the other nodes stay."""
    n0, n1 = pin_nodes(data, run)
    idx = np.arange(n0, n1 + 1)
    shift = np.zeros((len(data.cap.node_t), 3))
    shift[idx] = (np.interp(data.cap.node_t[idx], kt, kc) - c[idx])[:, None] * run["a"][None, :]
    return shift


def run_shift(data, run):
    """The move of every node (N, 3) that a run's profile makes, in the form the run was solved in."""
    if run.get("straight"):
        return straight_shift(data, run, run["knots_t"], run["knots_c"], run["c_nodes"])
    return node_offsets(data, run, run["scores"]["windows"], run["offsets"])[:, None] * run["a"][None, :]


# ---- rule 7: the edges ---------------------------------------------------------------------------------------------------
def crop_to_shared_footprint(points, other, cell):
    def key(x, y):
        return (np.floor(x / cell).astype(np.int64) << 32) ^ (np.floor(y / cell).astype(np.int64) & 0xFFFFFFFF)

    foot = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            foot.update(key(other[:, 0] + dx * cell, other[:, 1] + dy * cell).tolist())
    return np.isin(key(points[:, 0], points[:, 1]), np.fromiter(foot, np.int64, len(foot)))


def window_information(data, edge):
    """Rule 7: H_w (per source window of segment j, 3 x 3) of a kept edge, with the windows it belongs to. A format 3
    capture carries dMapper's own (its last refinement pass's); otherwise they are rebuilt here at the registered pose."""
    rule = data.rule
    i, j, cross = edge
    t = data.cap.tries[edge]
    C = t["C"]
    s = data.cap.segments[j]
    if not data.rebuild_information and t.get("Hw") is not None and len(t["Hw"]) == s["last"] - s["first"] + 1:
        return np.arange(s["first"], s["last"] + 1), np.asarray(t["Hw"], float)
    sx, _, _, sw = data.segment(j)
    tx = np.concatenate([data.segment(k)[0] for k in range(max(0, i - 1), min(j - 1, i + 1) + 1)])
    if cross:
        keep_t = crop_to_shared_footprint(tx, sx, rule.footprint_cell_m)
        keep_s = crop_to_shared_footprint(sx, tx, rule.footprint_cell_m)
        tx, sx, sw = tx[keep_t], sx[keep_s], sw[keep_s]
    s = data.cap.segments[j]
    wins = np.arange(s["first"], s["last"] + 1)
    H = np.zeros((len(wins), 3, 3))
    if len(tx) == 0 or len(sx) == 0:
        return wins, H
    keys, inv_i, n = np.unique(voxel_keys(tx, rule.cell_m), return_inverse=True, return_counts=True)
    mean = np.stack([np.bincount(inv_i, weights=tx[:, a]) / n for a in range(3)], axis=1)
    dx = tx - mean[inv_i]
    cov = np.zeros((len(keys), 3, 3))
    for a in range(3):
        for b in range(3):
            cov[:, a, b] = (np.bincount(inv_i, weights=dx[:, a] * dx[:, b]) + (rule.cell_initial_variance if a == b else 0.0)) / n
    prec = np.linalg.inv(cov)
    valid = (n >= rule.cell_min_points) & np.isfinite(prec).all(axis=(1, 2)) & (np.abs(prec).max(axis=(1, 2)) <= rule.cell_max_information)
    v = sx @ C[:3, :3].T + C[:3, 3]
    vk = voxel_keys(v, rule.cell_m)
    pos = np.searchsorted(keys, vk).clip(0, len(keys) - 1)
    used = (keys[pos] == vk) & valid[pos]
    for m, w in enumerate(wins):
        sel = used & (sw == w)
        H[m] = prec[pos[sel]].sum(axis=0)
    return wins, H


def edge_sigma_along(data, edge, a0):
    """Rule 7: sigma_a of the edge along the axis, with its other components held."""
    c = data.cap
    i, j, cross = edge
    t = c.tries[edge]
    sigma_m = c.setting("loop_closure_edge_sigma_m", 0.05)
    om, ok = lg.loop_edge_information(c.initial[i], c.initial[j], t["C"], t["H"], cross, sigma_m)
    W = om[:3, :3] if ok else np.eye(3) / sigma_m ** 2
    u = c.initial[i][:3, :3].T @ a0
    h = float(u @ W @ u)
    return (h ** -0.5 if h > 0 else np.inf), ok


def read_edge(data, edge, run, wins, H, pos_after, K_after):
    """rho of an edge along a run's axis: (first-pose residual before, rho before, rho after, s_w). pos_after: every node's
    position with the profiles of the candidate set; K_after: a function node -> correction with them."""
    c = data.cap
    i, j, _ = edge
    C = c.tries[edge]["C"]
    a = run.get("a")
    if a is None:  # a run that is not eligible has no corrected axis yet
        tm = (data.start[run["first"]] + data.end[run["last"]]) / 2
        n = min(int(np.searchsorted(c.node_t, tm, side="left")), len(c.node_t) - 1)
        a = (data.P1[n] @ lg.inv(data.P0[n]))[:3, :3] @ run["a0"]
    s = time_shares(H, run["a0"])
    ni = data.first[c.segments[i]["first"]]
    nj = data.first[c.segments[j]["first"]]

    def residual(node, x, K):
        pred = (K @ C @ np.r_[data.P0[node, :3, 3], 1.0])[:3]
        return float(a @ (x - pred))

    K0 = data.P1[ni] @ lg.inv(data.P0[ni])
    first_pose = residual(nj, data.P1[nj, :3, 3], K0)
    if s is None:
        return first_pose, np.nan, np.nan, None
    nodes = data.mid_node[wins]
    before = sum(s[m] * residual(nodes[m], data.P1[nodes[m], :3, 3], K0) for m in range(len(wins)))
    K1 = K_after(ni)
    after = sum(s[m] * residual(nodes[m], pos_after[nodes[m]], K1) for m in range(len(wins)))
    return first_pose, float(before), float(after), s


# ---- the whole rule ------------------------------------------------------------------------------------------------------
def run_rule(data, edges, rule=None, min_sigma=None, log=print, straight_gaps=False, fit_total=False):
    straight_gaps = straight_gaps or fit_total  # the total-fitted offsets are read through the straight form (RULE zigzag B)
    rule = rule or data.rule
    c = data.cap
    runs = find_stretches(data, min_sigma)
    eligible = [r for r in runs if r["eligible"]]
    log(f"{len(runs)} weak runs, {len(eligible)} eligible (sigma >= {rule.min_sigma_m if min_sigma is None else min_sigma} m)")
    # rules 4-6 and conditions 1-3 of rule 8
    for r in eligible:
        r["status"], r["cause"] = "not timed", ""
        if r["overlap"]:
            r["cause"] = "supports overlap"
            continue
        sc = r["scores"] = score_stretch(data, r, runs)
        win = sc["windows"]
        r["c_nodes"] = correction(data, r["a"])
        n0, n1 = pin_nodes(data, r)
        r["straight"] = straight_gaps
        total = (r["c_nodes"][data.mid_node[win]], r["c_nodes"][n0], r["c_nodes"][n1]) if fit_total else None
        sup, k = solve_stretch(sc, rule, total)
        off = sc["grid"][k]
        r["supported"], r["offsets"], r["k"] = sup, off, k
        zero = int(np.argmin(np.abs(sc["grid"])))
        if straight_gaps:
            r["knots_t"], r["knots_c"] = straight_knots(data, r, win, sup, off, r["c_nodes"])
            r["eff"] = np.interp(data.cap.node_t[data.mid_node[win]], r["knots_t"], r["knots_c"]) - r["c_nodes"][data.mid_node[win]]
            r["k_hold"] = round_offsets(r["eff"], sc["grid"], rule)
            moved_on_grid = np.any(r["k_hold"] != zero)
        else:
            r["eff"], r["k_hold"] = off, k
            moved_on_grid = np.any(off != 0)
        own = (win >= r["first"]) & (win <= r["last"])
        has = np.isfinite(sc["T"][:, 0])
        r["held_before"] = float(sc["T"][has, zero].mean()) if has.any() else np.nan
        r["held_after"] = float(sc["T"][has, r["k_hold"][has]].mean()) if has.any() else np.nan
        if not (sup & own).any():
            r["cause"] = "no surface across the passage seen by another visit"
        elif not moved_on_grid:
            r["status"], r["cause"] = "unchanged", "the effective offsets are zero" if straight_gaps else "the profile is zero"
        elif not has.any():
            r["cause"] = "no held-out score"
        elif not r["held_after"] < r["held_before"]:
            r["cause"] = "held-out not better"
        else:
            r["status"] = "candidate"
    # rule 7 and the iteration of rule 8
    info = {}
    touching = []  # (edge, run, side)
    for e in edges:
        s, ti = c.segments[e[1]], c.segments[e[0]]["first"]
        for r in runs:
            lo, hi = (r["sup_first"], r["sup_last"]) if r["eligible"] else (r["first"], r["last"])
            source = s["first"] <= hi and lo <= s["last"]
            target = r["eligible"] and r["sup_first"] <= ti <= r["sup_last"]
            if source or target:
                touching.append((e, r, "source" if source else "target"))
                if e not in info:
                    info[e] = window_information(data, e)
    # rule 7: a window is blind when it lies in an eligible run's support (any of them) and is unsupported there; a run
    # without scores (supports overlap) has no supported window
    blind_window = np.zeros(data.W, bool)
    for r in eligible:
        blind_window[r["sup_first"]:r["sup_last"] + 1] = True
    for r in eligible:
        if "scores" in r:
            blind_window[r["scores"]["windows"][r["supported"]]] = False
    table = []
    while True:
        cand = [r for r in eligible if r["status"] == "candidate"]
        shift = np.zeros((len(c.node_t), 3))
        for r in cand:
            shift += run_shift(data, r)
        pos_after = data.P1[:, :3, 3] + shift

        def K_after(node):
            P = data.P1[node].copy()
            P[:3, 3] = pos_after[node]
            return P @ lg.inv(data.P0[node])

        table, out = [], set()
        for e, r, side in touching:
            wins, H = info[e]
            first_pose, before, after, s = read_edge(data, e, r, wins, H, pos_after, K_after)
            sigma_a, information = edge_sigma_along(data, e, r["a0"])
            limit = sigma_a * c.setting("loop_closure_max_residual_m", 0.15) / c.setting("loop_closure_edge_sigma_m", 0.05)
            row = dict(edge=edge_name(e), run=f"{r['first']}-{r['last']}", side=side,
                       first_pose_m=first_pose, rho_before_m=before, rho_after_m=after, limit_m=limit, sigma_a_m=sigma_a,
                       informed_share=np.nan, cls="", shares=s, windows=wins, H_along=float(np.einsum("i,wij,j->", r["a0"], H, r["a0"])),
                       H_capture_along=float(r["a0"] @ c.tries[e]["H"][:3, :3] @ r["a0"]), information_edge=information)
            if r["eligible"] and "scores" in r:
                if s is None:
                    row["cls"] = "unconfirmed"
                else:
                    row["informed_share"] = float(s[~blind_window[wins]].sum())
                    row["cls"] = edge_class(row["informed_share"], before, after, limit, rule)
                if row["cls"] == "contradicted" and r["status"] == "candidate":
                    out.add(id(r))
                    r["contradicted_by"] = row["edge"]
            table.append(row)
        if not out:
            break
        for r in eligible:
            if id(r) in out:
                r["status"], r["cause"] = "not applied", f"edge {r['contradicted_by']} contradicted"
    for r in eligible:
        if r["status"] == "candidate":
            r["status"] = "applied"
    shift = np.zeros((len(c.node_t), 3))
    for r in eligible:
        if r["status"] == "applied":
            shift += run_shift(data, r)
    P = data.P1.copy()
    P[:, :3, 3] += shift
    return runs, table, P


# ---- output ------------------------------------------------------------------------------------------------------------
def write_trajectory(data, P, folder):
    """A trajectory folder (one chunk, identity chunk pose), as loop_graph.py writes it."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    c = data.cap
    pv = lg.posevec(P)
    ns = np.rint(c.node_t * 1e9).astype(np.int64)
    with open(folder / "trajectory_lio_0.csv", "w", newline="") as f:
        f.write("timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                "timestampUnix_nanoseconds om_rad fi_rad ka_rad\n")
        for k in range(len(P)):
            f.write(f"{ns[k]} " + " ".join(f"{v:.17g}" for v in P[k, :3].ravel()) + f" {int(round(c.node_unix_ns[k] * 1e9))} "
                    + " ".join(f"{v:.17g}" for v in pv[k, 3:]) + "\n")
    with open(folder / "session_poses.mrp", "w") as f:
        f.write("1\nscan_lio_0.laz\n" + "\n".join(" ".join(str(float(v)) for v in row) for row in np.eye(4)) + "\n")


def window_rows(data, runs, rule):
    rows = []
    t0 = data.cap.node_t[0]
    for r in runs:
        if "scores" not in r:
            continue
        sc = r["scores"]
        zero = int(np.argmin(np.abs(sc["grid"])))
        k = r["k_hold"]
        for j, w in enumerate(sc["windows"]):
            e, t = sc["E"][j], sc["T"][j]
            rows.append(dict(run=f"{r['first']}-{r['last']}", window=int(w), in_run=int(r["first"] <= w <= r["last"]),
                             start_s=round(float(data.start[w] - t0), 4), mid_s=round(float(data.mid[w] - t0), 4),
                             offset_m=round(float(r["offsets"][j]), 3), supported=int(r["supported"][j]),
                             train_patches=int(sc["n_train"][j]), held_patches=int(sc["n_test"][j]),
                             matched_most=int(sc["matched"][j].max()),
                             train_before_m=_rms(e[zero]), train_after_m=_rms(e[k[j]]), held_before_m=_rms(t[zero]), held_after_m=_rms(t[k[j]]),
                             alone_m=round(own_offset(e, sc["grid"], rule), 3) if np.isfinite(e[0]) else "",
                             correction_m=round(float(r["c_nodes"][data.mid_node[w]]), 6),
                             effective_offset_m=round(float(r["eff"][j]), 6),
                             total_m=round(float(r["c_nodes"][data.mid_node[w]] + r["eff"][j]), 6)))
    return rows


def _rms(v):
    return round(float(np.sqrt(v)), 4) if np.isfinite(v) else ""


def edge_rows(table):
    rows = []
    for r in table:
        f = lambda v: "" if v is None or (isinstance(v, float) and not np.isfinite(v)) else round(float(v), 4)  # noqa: E731
        rows.append(dict(edge=r["edge"], run=r["run"], side=r["side"], first_pose_m=f(r["first_pose_m"]), rho_before_m=f(r["rho_before_m"]),
                         rho_after_m=f(r["rho_after_m"]), limit_m=f(r["limit_m"]), sigma_a_m=f(r["sigma_a_m"]),
                         informed_share=f(r["informed_share"]), cls=r["cls"],
                         information_ratio=f(r["H_along"] / r["H_capture_along"]) if r["H_capture_along"] > 0 else "",
                         shares="" if r["shares"] is None else " ".join(f"{int(w)}:{s:.2f}" for w, s in zip(r["windows"], r["shares"]))))
    return rows


def summary(data, runs, rule, P=None):
    t0 = data.cap.node_t[0]
    out = []
    for r in runs:
        if not r["eligible"]:
            continue
        d = dict(windows=[r["first"], r["last"]], t_s=[round(float(data.start[r["first"]] - t0), 2), round(float(data.end[r["last"]] - t0), 2)],
                 sigma_m=round(r["sigma"], 3), axis=[round(float(v), 4) for v in r["a"]], largest_axis_angle_deg=round(r["largest_axis_angle_deg"], 1),
                 status=r["status"], cause=r["cause"])
        if "scores" in r:
            sc, sup, off = r["scores"], r["supported"], r["offsets"]
            own = (sc["windows"] >= r["first"]) & (sc["windows"] <= r["last"])
            gap, longest = 0.0, 0.0
            for j in np.flatnonzero(own):
                gap = 0.0 if sup[j] else gap + float(data.end[sc["windows"][j]] - data.start[sc["windows"][j]])
                longest = max(longest, gap)
            eff = r["eff"]
            moved = ~sup & ((np.abs(eff) > 1e-9) if r["straight"] else (off != 0))  # A8: effective offsets, not rounded
            d.update(reference_patches=sc["reference_patches"], supported_in_run=int((sup & own).sum()), run_windows=int(own.sum()),
                     longest_unsupported_s=round(longest, 2), largest_offset_m=round(float(np.abs(eff).max()), 3),
                     unsupported_moved=int(moved.sum()), unsupported_largest_m=round(float(np.abs(eff[moved]).max()), 3) if moved.any() else 0.0,
                     held_before_m=_rms(r["held_before"]), held_after_m=_rms(r["held_after"]))
        d.update(falls(data, r, P))
        out.append(d)
    return out


def falls(data, r, P=None):
    """A7 for one weak run: its fall after the loop closure, and after the step when its profile was applied (from the poses P of
    the result); for an eligible run with scores also over its whole support."""
    a = r["a"] if "a" in r else corrected_axis(data, r)
    mids = data.mid_node[r["first"]:r["last"] + 1]
    c = correction(data, a)
    d = dict(fall_after_loop_closure_m=round(fall(c[mids]), 6),
             fall_after_step_m=round(fall(correction(data, a, P)[mids]), 6) if P is not None and r.get("status") == "applied" else None)
    if "scores" in r:
        sm = data.mid_node[r["sup_first"]:r["sup_last"] + 1]
        d.update(support_fall_after_loop_closure_m=round(fall(c[sm]), 6),
                 support_fall_after_step_m=round(fall(correction(data, a, P)[sm]), 6) if P is not None and r["status"] == "applied" else None)
    return d


def weak_run_falls(data, runs, P=None):
    """A7 for every weak run, eligible or not (reading R4)."""
    return [dict(windows=[r["first"], r["last"]], eligible=bool(r["eligible"]), sigma_m=round(r["sigma"], 3), **falls(data, r, P)) for r in runs]


def _write_csv(path, rows):
    if not rows:
        Path(path).write_text("")
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("capture")
    p.add_argument("trajectory", help="result or replay folder: the poses after dMapper's loop closure")
    p.add_argument("edges", help="replay folder (loop_closure_edges.json), json or text edge list: the kept loop edges")
    p.add_argument("cache", help="the recording's cache folder (intermediate_points_<w>.cache)")
    p.add_argument("out")
    p.add_argument("--coupling", type=float, help="a reading: the profile's coupling (rule: 0.10)")
    p.add_argument("--min-matched", type=int, help="a reading: matched training patches a supported window needs (rule: 8)")
    p.add_argument("--min-sigma", type=float, help="a reading: the sigma an eligible run needs (rule: 1.0 m)")
    p.add_argument("--straight-gaps", action="store_true",
                   help="zigzag RULE section A: the total of the correction and the offset runs straight between the supported windows")
    p.add_argument("--fit-total", action="store_true",
                   help="zigzag RULE section B (reading F2): the profile's coupling on the total; implies --straight-gaps")
    p.add_argument("--rebuild-information", action="store_true",
                   help="the edges' information per window from the points even when the capture (format 3) carries dMapper's")
    a = p.parse_args(argv)
    rule = Rule()
    if a.coupling is not None:
        rule = replace(rule, coupling=a.coupling)
    if a.min_matched is not None:
        rule = replace(rule, min_matched=a.min_matched)
    cap = lg.read_capture(a.capture)
    rule = replace(rule, flank_windows=int(cap.setting("loop_closure_segment_windows", rule.flank_windows)))
    data = Data(cap, a.trajectory, a.cache, rule)
    data.rebuild_information = a.rebuild_information
    runs, table, P = run_rule(data, lg.load_edges(a.edges), rule, a.min_sigma, straight_gaps=a.straight_gaps, fit_total=a.fit_total)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    write_trajectory(data, P, out / "trajectory")
    _write_csv(out / "windows.csv", window_rows(data, runs, rule))
    _write_csv(out / "edges.csv", edge_rows(table))
    s = summary(data, runs, rule, P)
    (out / "summary.json").write_text(json.dumps(dict(rule={k: getattr(rule, k) for k in rule.__dataclass_fields__}, runs=s,
                                                      options=dict(straight_gaps=bool(a.straight_gaps or a.fit_total), fit_total=bool(a.fit_total)),
                                                      weak_runs=weak_run_falls(data, runs, P)), indent=1))
    for d in s:
        print(f"run windows {d['windows'][0]}-{d['windows'][1]} (t {d['t_s'][0]}-{d['t_s'][1]} s), sigma {d['sigma_m']} m: {d['status']}"
              + (f" ({d['cause']})" if d["cause"] else "")
              + (f"; supported {d['supported_in_run']} of {d['run_windows']} run windows, longest unsupported {d['longest_unsupported_s']} s, largest "
                 f"offset {d['largest_offset_m']} m, {d['unsupported_moved']} unsupported moved (largest {d['unsupported_largest_m']} m), held-out "
                 f"{d['held_before_m']} -> {d['held_after_m']} m" if "supported_in_run" in d else ""))
    for d in s:
        print(f"fall of run {d['windows'][0]}-{d['windows'][1]}: {d['fall_after_loop_closure_m']} m after the loop closure"
              + (f", {d['fall_after_step_m']} m after the step" if d["fall_after_step_m"] is not None else ""))
    for r in edge_rows(table):
        print(f"edge {r['edge']} ({r['side']}, run {r['run']}): first pose {r['first_pose_m']} m, time support {r['rho_before_m']} -> "
              f"{r['rho_after_m']} m, limit {r['limit_m']} m, informed {r['informed_share']}, {r['cls'] or 'no class (run without scores)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
