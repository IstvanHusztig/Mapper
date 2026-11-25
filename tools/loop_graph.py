#!/usr/bin/env python3
"""Pose-graph solver of dMapper's loop closure, in Python, to prototype loop-closure rules before any C++.

It mirrors `optimize_loop_pose_graph`, `loop_edge_information` and the correction interpolation of `close_loops`
(src/application/loop_closure.cpp) on a loop closure capture (`loop_closure_capture.bin`, format DMLCCAP1, authoritative
reader/writer src/infrastructure/loop_closure_capture_file.{h,cpp}), and takes the loop edges to keep as a list.

What it gives:
  * `read_capture`: the capture's settings, segments (pose, times, windows), the nodes (pose and time of every window
    node, odometry frame, before loop closure) and every try (transform, information, overlap, flags).
  * `LoopGraph`: the graph of the segment poses for a list of kept edges with dMapper's weights (loop edges from the
    registration's information, `loop_closure_edge_sigma_m` normalisation, cross-floor edges without the height; the
    fixed sigmas where the information is unusable; odometry edges with `loop_closure_odometry_sigma_*`). Hooks: a scale
    per loop edge's information (`loop_scale`), a sigma (translation m, rotation deg) per odometry edge or all of them
    (`odo_sigma`) or a full 6x6 information per odometry edge (`odo_omega`), a given 6x6 information per loop edge
    (`loop_omega`), the robust kernel of dMapper
    (`kernel_scale`, `loop_closure_kernel_scale`; two stages, plain first).
  * `LoopGraph.solve`: Gauss-Newton with dMapper's step rule (a step that raises the cost is damped), first segment fixed;
    returns the segment poses and the cost split into odometry and loop parts and per factor.
  * `LoopGraph.node_poses` / `write_trajectory`: the per-segment correction put onto every node as dMapper does (slerp
    and linear interpolation between the segments' corrections by node time inside the segment's windows), written as a
    trajectory folder (`session_poses.mrp` with one chunk, `trajectory_lio_0.csv`) that `tools/wall_pairs.py measure` and
    `tools/cloud_from_trajectory.py` read like a replay's.
  * `leave_one_out`: for each loop edge the innovation of that edge predicted from the rest (the graph solved without
    it, the edge's residual there: weighted norm, and in metres as dMapper's weighted residual = norm x
    `loop_closure_edge_sigma_m`) and the change of cost.

What it does NOT do: no registration (the transforms are the capture's), no slot selection, no pruning or dropping by
residual (the kept edges are taken as given; dMapper's `loop_edge_to_drop`, `loop_pose_graph_plausible` are not run),
no consistency, no loop-closure replay. The result is dMapper's solution for those kept edges, not a statement about
the map. Not mirrored exactly: dMapper's analytic Jacobians (here central differences, h = 1e-6) and its Cholesky
(here a sparse LU solve); the first pose is fixed by removing its unknowns, not by a 1e12 weight.

  loop_graph.py solve <capture> <replay folder or edge list> <out folder> [--drop i-j]... [--loop-scale i-j=s]...
                      [--odo-sigma M DEG] [--kernel-scale K]
  loop_graph.py loo   <capture> <replay folder or edge list> [--csv file] [--top N]
  loop_graph.py check <capture> <replay folder>
An edge is `i-j` (same floor) or `i-jx` (cross floor). The edge list is a replay folder's `loop_closure_edges.json`
(`edges_kept`), such a json file, or a text file with one `i j [x]` per line.
"""
import argparse
import csv
import json
import os
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix, csc_matrix
from scipy.sparse.linalg import spsolve
from scipy.spatial.transform import Rotation, Slerp

H_DIFF = 1e-6  # the central-difference step of dMapper's numeric Jacobians and of the Jacobians here


# ---- pose maths: Tait-Bryan (om, fi, ka) = R = Rx(om) Ry(fi) Rz(ka), as `pose_tait_bryan_from_affine_matrix` --------------
def posevec(m):
    """(..., 4, 4) -> (..., 6): px py pz om fi ka."""
    m = np.asarray(m, float)
    ang = np.stack([np.arctan2(-m[..., 1, 2], m[..., 2, 2]), np.arcsin(np.clip(m[..., 0, 2], -1.0, 1.0)),
                    np.arctan2(-m[..., 0, 1], m[..., 0, 0])], axis=-1)
    return np.concatenate([m[..., :3, 3], ang], axis=-1)


def matvec(p):
    """(..., 6) -> (..., 4, 4)."""
    p = np.asarray(p, float)
    m = np.tile(np.eye(4), p.shape[:-1] + (1, 1))
    m[..., :3, :3] = Rotation.from_euler("XYZ", p[..., 3:].reshape(-1, 3)).as_matrix().reshape(p.shape[:-1] + (3, 3))
    m[..., :3, 3] = p[..., :3]
    return m


def inv(m):
    m = np.asarray(m, float)
    out = np.zeros_like(m)
    out[..., :3, :3] = np.swapaxes(m[..., :3, :3], -2, -1)
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", out[..., :3, :3], m[..., :3, 3])
    out[..., 3, 3] = 1.0
    return out


def wrap(d):
    """Angle components (3:) wrapped into [-pi, pi), in place; returns d."""
    d[..., 3:] = (d[..., 3:] + np.pi) % (2 * np.pi) - np.pi
    return d


def psd(m):
    """A symmetric matrix with its negative eigenvalues set to zero (`clip_negative`)."""
    e, v = np.linalg.eigh((m + m.T) / 2)
    return (v * e.clip(0)) @ v.T


# ---- capture -----------------------------------------------------------------------------------------------------------
@dataclass
class Capture:
    settings: dict
    segments: list          # dicts: first, last (windows), pose 4x4, center, path_begin, path_end, t_begin, t_end
    win_first_node: np.ndarray  # first node index of each window (len windows + 1)
    node_pose: np.ndarray   # (N, 4, 4) odometry pose of every node, all windows in order
    node_t: np.ndarray      # (N,) seconds
    node_unix_ns: np.ndarray  # (N,) unix seconds (the second stamp column of the capture)
    tries: dict = field(default_factory=dict)  # (i, j, cross) -> dict
    # the windows' readings (format 2 on; None in a format 1 capture), windows numbered from 0 as in the capture: W, t (first
    # node from the first window's first node, s), dur, ratio, rest, weak (W, 3), move (W, 3); NaN (rest False) where a window
    # has none, present (W,) bool
    readings: dict = None

    def setting(self, name, default=None):
        return float(self.settings[name]) if name in self.settings else default

    @property
    def initial(self):
        return np.array([s["pose"] for s in self.segments])


def read_capture(path):
    """Read a DMLCCAP1 capture, format 1, 2 or 3 (little-endian doubles; matrices stored column-major). Format 3 adds each try's
    information per source window: `Hw` (windows of segment j, 3, 3), the translation block of `H` summed per window (empty for a try
    that was not registered); a try of format 1 or 2 has no `Hw`."""
    with open(path, "rb") as f:
        def val(fmt):
            return struct.unpack("<" + fmt, f.read(struct.calcsize("<" + fmt)))

        def count():
            return val("Q")[0]

        def arr(n):
            return np.frombuffer(f.read(8 * n), "<f8").copy()

        def mat(n):
            return arr(n * n).reshape(n, n).T

        magic, version = val("8sI")
        if magic != b"DMLCCAP1" or version not in (1, 2, 3):
            raise ValueError(f"{path}: not a loop closure capture (magic {magic!r}, version {version})")
        cfg = {}
        for _ in range(count()):
            k = f.read(count()).decode()
            cfg[k] = f.read(count()).decode()
        poses, stamps, offsets = [], [], [0]
        for _ in range(count()):
            for name, n in (("poses", 16), ("model", 16), ("stamp", 2), ("imu", 3), ("acc", 3)):
                k = count()
                if name == "poses":
                    poses.append(arr(n * k).reshape(k, 4, 4).transpose(0, 2, 1))
                    offsets.append(offsets[-1] + k)
                elif name == "stamp":
                    stamps.append(arr(n * k).reshape(k, 2))
                else:
                    f.seek(8 * k * n, 1)
            f.seek(27 * 8, 1)  # the window's gyro bias state (mean, covariance), velocity state, ...
        segs = []
        for _ in range(count()):
            first, last = val("ii")
            pose = mat(4)
            center = arr(3)
            pb, pe, tb, te = arr(4)
            segs.append(dict(first=first, last=last, pose=pose, center=center, path_begin=pb, path_end=pe, t_begin=tb, t_end=te))
        tries, order = {}, []
        for _ in range(count()):
            i, j, xf, registered = val("iiBB")
            before, after = arr(2)
            corr = mat(4)
            info = mat(6)
            shift, angle, ratio = arr(3)
            centre = arr(3)
            accepted, asked = val("BB")
            tries[(i, j, bool(xf))] = dict(C=corr, H=info, center=centre, overlap_before=before, overlap_after=after, shift=shift,
                                          angle=angle, information_ratio=ratio, accepted=bool(accepted), asked=bool(asked),
                                          registered=bool(registered))
            order.append((i, j, bool(xf)))
        for _ in range(count()):  # the poses the capturing run left in the windows (not needed here)
            f.seek(count() * 16 * 8, 1)
        val("i")  # loops accepted
        readings = None
        if version >= 2:
            W = count()
            if W != len(poses):
                raise ValueError(f"{path}: {W} window readings for {len(poses)} windows")
            readings = dict(W=W, present=np.zeros(W, bool), dur=np.full(W, np.nan), ratio=np.full(W, np.nan), rest=np.zeros(W, bool),
                            weak=np.full((W, 3), np.nan), move=np.full((W, 3), np.nan))
            for w in range(W):
                if not val("B")[0]:
                    continue
                readings["present"][w] = True
                readings["ratio"][w] = arr(1)[0]
                readings["weak"][w] = arr(3)
                readings["rest"][w] = bool(val("B")[0])
                readings["move"][w] = arr(3)
                readings["dur"][w] = arr(1)[0]
        if version >= 3:
            if count() != len(order):
                raise ValueError(f"{path}: the per-window information does not match the tries")
            for key in order:
                k = count()
                tries[key]["Hw"] = arr(9 * k).reshape(k, 3, 3).transpose(0, 2, 1)
        if f.read():
            raise ValueError(f"{path}: trailing bytes, the format has changed")
    st = np.concatenate(stamps)
    if readings is not None:
        readings["t"] = st[np.array(offsets[:-1]), 0] - st[0, 0]
    return Capture(cfg, segs, np.array(offsets), np.concatenate(poses), st[:, 0], st[:, 1], tries, readings)


# ---- edges -------------------------------------------------------------------------------------------------------------
def parse_edge(text):
    """'24-63' -> (24, 63, False); '24-63x' -> (24, 63, True)."""
    t = text.strip()
    cross = t.endswith("x")
    a, b = t.rstrip("x").split("-")
    return int(a), int(b), cross


def edge_label(e):
    return f"{e[0]}-{e[1]}" + ("x" if e[2] else "")


def load_edges(spec):
    """Kept edges from a replay folder, a json (`edges_kept`, or a list) or a text file (`i j [x]` per line)."""
    p = Path(spec)
    if p.is_dir():
        p = p / "loop_closure_edges.json"
    if p.suffix == ".json":
        j = json.loads(p.read_text())
        rows = j["edges_kept"] if isinstance(j, dict) else j
        out = []
        for r in rows:
            out.append((int(r["i"]), int(r["j"]), bool(r["cross_floor"])) if isinstance(r, dict) else (int(r[0]), int(r[1]), bool(r[2]) if len(r) > 2 else False))
        return out
    out = []
    for ln in p.read_text().splitlines():
        w = ln.split("#")[0].split()
        if w:
            out.append((int(w[0]), int(w[1]), len(w) > 2 and w[2] in ("x", "1", "true", "True")))
    return out


# ---- the loop edge's information, as `loop_edge_information` --------------------------------------------------------------
def loop_edge_information(pose_i, pose_j, correction, H, free_height, sigma_m):
    """(omega 6x6, ok). omega is zero and ok False where the registration's information is unusable (dMapper then keeps
    the fixed sigmas). The measurement's Jacobian G is taken against the correction's Tait-Bryan parameters."""
    vec = posevec(correction)
    eye = np.eye(6) * H_DIFF
    cols = wrap(posevec(inv(pose_i) @ matvec(vec + eye) @ pose_j) - posevec(inv(pose_i) @ matvec(vec - eye) @ pose_j))
    G = cols.T / (2 * H_DIFF)
    if not (np.all(np.isfinite(G)) and np.all(np.isfinite(H))) or 1.0 / np.linalg.cond(G, 1) < 1e-9:
        return np.zeros((6, 6)), False
    gi = np.linalg.inv(G)
    omega = psd(gi.T @ H @ gi)
    if free_height:
        u = G[:, 2]
        w = omega @ u
        uu = u @ w
        if uu > 1e-12 * np.trace(omega):
            omega = psd(omega - np.outer(w, w) / uu)
    best = np.linalg.eigvalsh((omega[:3, :3] + omega[:3, :3].T) / 2)[-1]
    if not best > 0 or not np.all(np.isfinite(omega)):
        return np.zeros((6, 6)), False
    return omega / (sigma_m ** 2) / best, True


def interpolate_corrections(ca, cb, f):
    """dMapper's `interpolate`: slerp of the rotations, linear translation; ca, cb (4,4); f (n,) -> (n,4,4)."""
    out = np.tile(np.eye(4), (len(f), 1, 1))
    rots = Rotation.from_matrix(np.stack([ca[:3, :3], cb[:3, :3]]))
    out[:, :3, :3] = Slerp([0.0, 1.0], rots)(f).as_matrix()
    out[:, :3, 3] = (1 - f)[:, None] * ca[:3, 3] + f[:, None] * cb[:3, 3]
    return out


# ---- the graph ---------------------------------------------------------------------------------------------------------
@dataclass
class Solution:
    poses: np.ndarray          # (n, 4, 4) segment poses
    cost: float                # total of the cost the solver minimised (robust kernel included)
    cost_odometry: float       # plain quadratic cost, odometry edges
    cost_loops: float          # plain quadratic cost, loop edges
    factor_costs: np.ndarray   # plain quadratic cost of every factor (odometry first, then the loops in edge order)
    iterations: int = 0


class LoopGraph:
    """Segment-pose graph with dMapper's weights for the given kept edges (see the module docstring for the hooks).

    odo_sigma: None (the capture's loop_closure_odometry_sigma_m/_deg), (m, deg), or an (n-1, 2) array.
    odo_omega: None or (n-1, 6, 6) information of each odometry edge (replaces odo_sigma).
    loop_scale: {edge: s} multiplies that edge's information (edge key (i, j, cross) or 'i-j[x]' label); default 1.
    loop_omega: {edge: 6x6} information used as given instead of the one built from the registration (before loop_scale).
    kernel_scale: None (the capture's loop_closure_kernel_scale; 0 = no kernel) or a value."""

    def __init__(self, capture, edges, loop_scale=None, odo_sigma=None, odo_omega=None, kernel_scale=None, loop_omega=None):
        self.cap = capture
        c = capture
        self.initial = c.initial
        n = len(self.initial)
        self.n = n
        self.edges = [tuple(e) for e in edges]
        self.sigma_m = c.setting("loop_closure_edge_sigma_m", 0.05)
        sigma_deg = c.setting("loop_closure_edge_sigma_deg", 0.3)
        self.kernel2 = (kernel_scale if kernel_scale is not None else c.setting("loop_closure_kernel_scale", 0.0)) ** 2
        loop_omega = {(parse_edge(k) if isinstance(k, str) else tuple(k)): v for k, v in (loop_omega or {}).items()}
        scale = {}
        for k, v in (loop_scale or {}).items():
            scale[parse_edge(k) if isinstance(k, str) else tuple(k)] = float(v)
        # odometry edges
        if odo_omega is not None:
            oo = np.asarray(odo_omega, float)
            if oo.shape != (n - 1, 6, 6):
                raise ValueError(f"odo_omega must be ({n - 1}, 6, 6), got {oo.shape}")
        else:
            s = np.broadcast_to(np.asarray(odo_sigma if odo_sigma is not None else
                                           (c.setting("loop_closure_odometry_sigma_m", 0.1), c.setting("loop_closure_odometry_sigma_deg", 0.5)),
                                           float), (n - 1, 2))
            oo = np.zeros((n - 1, 6, 6))
            for k in range(3):
                oo[:, k, k] = 1.0 / s[:, 0] ** 2
                oo[:, 3 + k, 3 + k] = 1.0 / np.deg2rad(s[:, 1]) ** 2
        # loop edges: the measurement is segment i's pose against the correction moving segment j onto i
        self.information_ok = []
        lo = np.zeros((len(self.edges), 6, 6))
        li = np.array([e[0] for e in self.edges], int)
        lj = np.array([e[1] for e in self.edges], int)
        meas_loop = np.zeros((len(self.edges), 6))
        for k, e in enumerate(self.edges):
            if e not in c.tries:
                raise KeyError(f"edge {edge_label(e)} has no try in the capture")
            t = c.tries[e]
            om, ok = loop_edge_information(self.initial[e[0]], self.initial[e[1]], t["C"], t["H"], e[2], self.sigma_m)
            if e in loop_omega:
                om, ok = np.asarray(loop_omega[e], float), True
            if not ok:
                om = np.diag([1 / self.sigma_m ** 2] * 3 + [1 / np.deg2rad(sigma_deg) ** 2] * 3)
            self.information_ok.append(ok)
            lo[k] = om * scale.get(e, 1.0)
            meas_loop[k] = posevec(inv(self.initial[e[0]]) @ t["C"] @ self.initial[e[1]])
        self.i = np.r_[np.arange(n - 1), li]
        self.j = np.r_[np.arange(1, n), lj]
        self.z = np.vstack([posevec(inv(self.initial[:-1]) @ self.initial[1:]), meas_loop.reshape(-1, 6)])
        self.omega = np.concatenate([oo, lo])
        self.n_odo = n - 1
        self.loop_scale = {e: scale.get(e, 1.0) for e in self.edges}
        eig, U = np.linalg.eigh(self.omega)
        self._root = np.swapaxes(U * np.sqrt(eig.clip(0))[:, None, :], 1, 2)  # whitening: root @ d, root.T@root = omega

    def edge_index(self, e):
        return self.n_odo + self.edges.index(e if not isinstance(e, str) else parse_edge(e))

    # residual of every factor (relative pose of the two poses minus the measurement), angles wrapped
    def residuals(self, poses, mask=None):
        i, j, z = (self.i, self.j, self.z) if mask is None else (self.i[mask], self.j[mask], self.z[mask])
        return wrap(posevec(inv(poses[i]) @ poses[j]) - z)

    def factor_costs(self, poses):
        """Plain quadratic cost d' omega d of every factor."""
        d = self.residuals(poses)
        return np.einsum("ik,ikm,im->i", d, self.omega, d)

    def _robust(self, s2, kernel2):
        return s2 if kernel2 <= 0 else kernel2 * np.log1p(s2 / kernel2)

    def solution_of(self, poses, mask=None, iterations=0):
        fc = self.factor_costs(poses)
        m = np.ones(len(fc), bool) if mask is None else mask
        loop = np.arange(len(fc)) >= self.n_odo
        robust = np.where(loop, self._robust(fc, self.kernel2), fc)
        return Solution(poses, float(robust[m].sum()), float(fc[m & ~loop].sum()), float(fc[m & loop].sum()), fc, iterations)

    def solve(self, mask=None, seed=None, max_iter=30):
        """Gauss-Newton as dMapper's `optimize_loop_pose_graph`. mask: bool per factor (default all; odometry edges must
        stay in); seed: segment poses to start from (default the initial ones)."""
        mask = np.ones(len(self.i), bool) if mask is None else np.asarray(mask, bool)
        if not mask[:self.n_odo].all():
            raise ValueError("odometry edges cannot be masked out")
        x = posevec(self.initial if seed is None else seed)
        ii, jj, z, root = self.i[mask], self.j[mask], self.z[mask], self._root[mask]
        loop = np.flatnonzero(mask) >= self.n_odo
        n = self.n
        dims = (len(ii), 6, 6)
        rows = np.broadcast_to(np.arange(len(ii) * 6).reshape(len(ii), 6, 1), dims).ravel()
        ci = np.broadcast_to(ii[:, None, None] * 6 + np.arange(6)[None, None, :], dims).ravel()
        cj = np.broadcast_to(jj[:, None, None] * 6 + np.arange(6)[None, None, :], dims).ravel()
        rm, cm = np.r_[rows, rows], np.r_[ci, cj]
        total_iter = 0

        def whiten(d, w):
            return np.einsum("eik,ek->ei", root, d) * w[:, None]

        def rel(a, b):
            return wrap(posevec(inv(a) @ b) - z)

        def weights(poses, kernel2):
            if kernel2 <= 0:
                return np.ones(len(ii))
            d = rel(poses[ii], poses[jj])
            s2 = np.einsum("ik,ikm,im->i", d, self.omega[mask], d)
            return np.where(loop, 1.0 / (1.0 + s2 / kernel2), 1.0)

        def cost(poses, kernel2):
            d = rel(poses[ii], poses[jj])
            s2 = np.einsum("ik,ikm,im->i", d, self.omega[mask], d)
            return float(np.where(loop, self._robust(s2, kernel2), s2).sum())

        def stage(kernel2):
            nonlocal x, total_iter
            M = matvec(x)
            c0 = cost(M, kernel2)
            lam, it = 0.0, 0
            for _ in range(60):
                if it >= max_iter:
                    break
                M = matvec(x)
                a, b = M[ii], M[jj]
                base = rel(a, b)
                # the reweighted quadratic at the current poses (weights sqrt(f)): its step lowers the robust cost
                w = np.sqrt(weights(M, kernel2))
                right = whiten(base, w).ravel()
                j1 = np.empty(dims)
                j2 = np.empty(dims)
                for ax in range(6):
                    xv = x.copy()
                    xv[:, ax] += H_DIFF
                    P = matvec(xv)
                    j1[:, :, ax] = whiten(wrap(rel(P[ii], b) - base), w) / H_DIFF
                    j2[:, :, ax] = whiten(wrap(rel(a, P[jj]) - base), w) / H_DIFF
                J = coo_matrix((np.r_[j1.ravel(), j2.ravel()], (rm, cm)), shape=(len(ii) * 6, n * 6)).tocsc()[:, 6:]
                A = (J.T @ J).tocsc()
                if lam > 0:
                    A = A + lam * csc_matrix((A.diagonal(), (np.arange(A.shape[0]),) * 2), shape=A.shape)
                dx = spsolve(A, -(J.T @ right))
                if not np.all(np.isfinite(dx)):
                    break
                step = np.vstack([np.zeros((1, 6)), dx.reshape(n - 1, 6)])
                trial = x + step
                c1 = cost(matvec(trial), kernel2)
                if np.isfinite(c1) and c1 <= c0 * (1 + 1e-9) + 1e-12:
                    x, c0 = trial, c1
                    lam = lam * 0.1 if lam > 0 else 0.0
                    if lam < 1e-9:
                        lam = 0.0
                    it += 1
                    if np.linalg.norm(step) < 1e-8:
                        break
                else:
                    lam = lam * 10 if lam > 0 else 1e-3
                    if lam > 1e6:
                        break
            total_iter += it

        stage(0.0)
        if self.kernel2 > 0:
            stage(self.kernel2)
        poses = matvec(x)
        return self.solution_of(poses, mask, total_iter)

    def mask_without(self, *edges):
        m = np.ones(len(self.i), bool)
        for e in edges:
            m[self.edge_index(e)] = False
        return m

    # ---- the correction on the nodes -----------------------------------------------------------------------------------
    def node_poses(self, sol):
        """World pose of every node after the correction, as `close_loops` step 5 (nodes in the capture's order)."""
        c = self.cap
        corr = sol.poses @ inv(self.initial)
        out = c.node_pose.copy()
        segs = c.segments
        for k, s in enumerate(segs):
            lo, hi = c.win_first_node[s["first"]], c.win_first_node[s["last"] + 1]
            if hi <= lo:
                continue
            has_next = k + 1 < len(segs)
            t0 = s["t_begin"]
            t1 = segs[k + 1]["t_begin"] if has_next else s["t_end"]
            if has_next and t1 > t0:
                f = np.clip((c.node_t[lo:hi] - t0) / (t1 - t0), 0.0, 1.0)
            else:
                f = np.zeros(hi - lo)
            C = interpolate_corrections(corr[k], corr[k + 1], f) if has_next else np.broadcast_to(corr[k], (hi - lo, 4, 4))
            out[lo:hi] = C @ c.node_pose[lo:hi]
        return out

    def write_trajectory(self, sol, folder):
        """A trajectory folder (one chunk, identity chunk pose) that wall_pairs / cloud_from_trajectory read."""
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        c = self.cap
        P = self.node_poses(sol)
        pv = posevec(P)
        ns = np.rint(c.node_t * 1e9).astype(np.int64)
        with open(folder / "trajectory_lio_0.csv", "w", newline="") as f:
            f.write("timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                    "timestampUnix_nanoseconds om_rad fi_rad ka_rad\n")
            for k in range(len(P)):
                r = P[k]
                f.write(f"{ns[k]} " + " ".join(f"{v:.17g}" for v in r[:3].ravel()) + f" {int(round(c.node_unix_ns[k] * 1e9))} "
                        + " ".join(f"{v:.17g}" for v in pv[k, 3:]) + "\n")
        with open(folder / "session_poses.mrp", "w") as f:
            f.write("1\nscan_lio_0.laz\n" + "\n".join("0 0 0 1".split() and " ".join(str(float(v)) for v in row) for row in np.eye(4)) + "\n")
        return P


def leave_one_out(graph, sol=None, edges=None):
    """For each loop edge the innovation predicted from the rest. The graph is solved without the edge (seeded with the
    full solution), the edge's residual there is the innovation: `weighted` = sqrt(d' omega d) (omega with the edge's scale),
    `weighted_m` = weighted x loop_closure_edge_sigma_m (dMapper's weighted residual in metres), `translation_m` and
    `rotation_deg` the raw residual, `cost_change` = cost with all edges - cost of the rest at the leave-one-out solution
    (positive: the edge costs the rest that much), `in_sample_weighted_m` the same residual in the full solution."""
    sol = sol or graph.solve()
    rows = []
    full = sol.factor_costs
    for e in (edges or graph.edges):
        idx = graph.edge_index(e)
        mask = graph.mask_without(e)
        s = graph.solve(mask=mask, seed=sol.poses)
        d = graph.residuals(s.poses)[idx]
        weighted = float(np.sqrt(max(s.factor_costs[idx], 0.0)))
        rows.append(dict(edge=edge_label(e), i=e[0], j=e[1], cross=e[2], weighted=weighted, weighted_m=weighted * graph.sigma_m,
                         translation_m=float(np.linalg.norm(d[:3])), rotation_deg=float(np.rad2deg(np.linalg.norm(d[3:]))),
                         cost_without=float(s.factor_costs[mask].sum()), cost_with=float(full.sum()),
                         cost_change=float(full.sum() - s.factor_costs[mask].sum()),
                         in_sample_weighted_m=float(np.sqrt(max(full[idx], 0.0)) * graph.sigma_m)))
    return rows


# ---- CLI ---------------------------------------------------------------------------------------------------------------
def _build(a, edges):
    drop = {parse_edge(t) for t in getattr(a, "drop", [])}
    edges = [e for e in edges if e not in drop]
    scale = {}
    for item in getattr(a, "loop_scale", []):
        k, v = item.split("=")
        e = parse_edge(k)
        scale[e] = float(v)
    odo = tuple(a.odo_sigma) if getattr(a, "odo_sigma", None) else None
    return LoopGraph(read_capture(a.capture), edges, loop_scale=scale, odo_sigma=odo, kernel_scale=getattr(a, "kernel_scale", None))


def _load_trajectory_arrays(folder):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import map_quality_lib
    t, rot, pos = map_quality_lib.trajectory(str(folder))
    return np.asarray(t), np.asarray(rot), np.asarray(pos)


def cmd_solve(a):
    edges = load_edges(a.edges)
    g = _build(a, edges)
    sol = g.solve()
    out = Path(a.out)
    P = g.write_trajectory(sol, out)
    per_edge = {edge_label(e): float(sol.factor_costs[g.n_odo + k]) for k, e in enumerate(g.edges)}
    info = dict(edges=[edge_label(e) for e in g.edges], loop_scale={edge_label(e): s for e, s in g.loop_scale.items() if s != 1.0},
                information_unusable=[edge_label(e) for e, ok in zip(g.edges, g.information_ok) if not ok], iterations=sol.iterations,
                cost=sol.cost, cost_odometry=sol.cost_odometry, cost_loops=sol.cost_loops, loop_factor_cost=per_edge,
                loop_weighted_m={k: float(np.sqrt(max(v, 0)) * g.sigma_m) for k, v in per_edge.items()},
                largest_segment_correction_m=float(np.linalg.norm((sol.poses[:, :3, 3] - g.initial[:, :3, 3]), axis=1).max()))
    (out / "loop_graph_solution.json").write_text(json.dumps(info, indent=1))
    np.savez(out / "segment_poses.npz", poses=sol.poses, initial=g.initial, factor_costs=sol.factor_costs)
    print(f"{len(g.edges)} loop edges, {sol.iterations} iterations; cost {sol.cost:.4f} = odometry {sol.cost_odometry:.4f} + loops {sol.cost_loops:.4f}")
    print(f"largest segment correction {info['largest_segment_correction_m']:.3f} m; trajectory in {out} ({len(P)} nodes)")
    return 0


def cmd_loo(a):
    g = _build(a, load_edges(a.edges))
    rows = leave_one_out(g)
    rows.sort(key=lambda r: -r["weighted_m"])
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    print(f"{'edge':>9} {'innov_w_m':>10} {'trans_m':>8} {'rot_deg':>8} {'cost_change':>12} {'in_sample_w_m':>14}")
    for r in rows[:a.top]:
        print(f"{r['edge']:>9} {r['weighted_m']:10.3f} {r['translation_m']:8.3f} {r['rotation_deg']:8.3f} {r['cost_change']:12.2f} {r['in_sample_weighted_m']:14.3f}")
    print(f"({len(rows)} edges; innov_w_m = weighted norm x edge sigma, dMapper's weighted residual of the edge predicted from the rest)")
    return 0


def cmd_check(a):
    g = _build(a, load_edges(a.edges))
    sol = g.solve()
    P = g.node_poses(sol)
    t, rot, pos = _load_trajectory_arrays(a.edges)
    if len(t) != len(P):
        print(f"node counts differ: capture {len(P)}, replay {len(t)}")
        return 2
    o = np.argsort(g.cap.node_t, kind="stable")
    dt = float(np.abs(g.cap.node_t[o] - t).max())
    dp = np.linalg.norm(P[o][:, :3, 3] - pos, axis=1)
    R = np.einsum("nji,njk->nik", P[o][:, :3, :3], rot)
    vee = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], axis=1) / 2
    ang = np.rad2deg(np.arctan2(np.linalg.norm(vee, axis=1), (np.trace(R, axis1=1, axis2=2) - 1) / 2))  # exact for small angles
    print(f"{len(P)} nodes, {len(g.edges)} kept edges, cost {sol.cost:.6f}")
    print(f"largest node position difference to the replay's trajectory: {dp.max() * 1000:.6f} mm (median {np.median(dp) * 1000:.6f} mm, at node {int(dp.argmax())})")
    print(f"largest node rotation difference: {ang.max():.6f} deg; largest node time difference {dt:.2e} s")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, out=False):
        p.add_argument("capture")
        p.add_argument("edges", help="replay folder (loop_closure_edges.json), json or text edge list")
        if out:
            p.add_argument("out")
        p.add_argument("--drop", action="append", default=[], metavar="i-j", help="leave this edge out (i-jx: cross floor)")
        p.add_argument("--loop-scale", action="append", default=[], metavar="i-j=s", help="scale this edge's information")
        p.add_argument("--odo-sigma", nargs=2, type=float, metavar=("M", "DEG"), help="odometry edge sigmas")
        p.add_argument("--kernel-scale", type=float, help="robust kernel scale (0 = none; default the capture's)")

    common(sub.add_parser("solve", help="solve and write a trajectory folder"), out=True)
    p = sub.add_parser("loo", help="leave-one-out table")
    common(p)
    p.add_argument("--csv")
    p.add_argument("--top", type=int, default=15)
    sub.add_parser("check", help="solve the replay's kept edges, compare with its trajectory").add_argument("capture")
    c = sub.choices["check"]
    c.add_argument("edges", help="replay folder")
    a = ap.parse_args(argv)
    return {"solve": cmd_solve, "loo": cmd_loo, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
