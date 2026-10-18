"""How precise each wall of a result is at height H, from the map and the trajectory alone (works without a plan).

The section line at height H (section_lib: the line through the densest part of each wall, --threshold, as
plan_section.py) is cut into pieces: the cells of one connected line inside one --piece square. For each piece the
slice points (world z within --half-thickness of H) within --across of the piece's line are taken, and:
  sides, faces the side of the line the sensor was on; a wall seen from both sides shows two faces (faces_m apart),
               which is not an error, so each side is centred on its own median and visits are compared within a side
  thickness    robust spread (1.4826 MAD) of the points' offsets across the line [m]; a precise wall is ~ sensor noise
  visits       per side, the points' times split at gaps over --gap s; each visit with --min-visit-points gets its
               median offset (visit_offsets_m, visit_sides, visit_times_s)
  pair_diff    median over the pairs of visits from the same side of the difference of their offsets [m]: how far two
               visits of the wall typically disagree; unlike visit_spread (largest minus smallest offset, per side) it
               does not grow with the number of visits. The pairs of all pieces are also printed by the time between
               the two visits: drift between visits shows as a difference that grows with that time
  half_split   the longest visit's first half against its second half [m]: the measure's own noise floor
  seen_s       sum of the visits' durations [s]; points; range_m and incidence_deg (0 = head-on, 90 = grazing) are
               medians over the points, with the sensor position interpolated from the trajectory at each point's time
  windows      the odometry windows of the visits (window start times from the results JSON)
  plan_dist_m  with --plan only: median distance of the piece's line to the plan's drawn lines (plan_fit.json)
Writes <result>/map_quality_cache/wall_precision_<H>.csv (one row per piece) and wall_precision_<H>_thickness.png /
_visits.png (pair_diff): each piece's line in 1 px, coloured blue / green / orange / red by the class limits printed;
grey = no two visits from the same side. On the plan with --plan (run plan_fit.py first), else on a map-frame canvas with a 1 m grid. Prints quantiles and
Spearman rank correlations. Needs numpy, scipy, laspy (with lazrs), and pymupdf for writing the images.

Usage: wall_precision.py <result_folder> [--height H] [--threshold T] [--plan]
"""
import argparse
import csv
import json
import os

import laspy
import numpy as np
import pymupdf
from scipy import ndimage, spatial, stats

from map_quality_lib import PLAN_PDF, PLAN_PX_PER_M, cache_dir, trajectory
from section_lib import Section

ap = argparse.ArgumentParser()
ap.add_argument("result")
ap.add_argument("--height", type=float, default=0.0, help="world z of the section [m]")
ap.add_argument("--half-thickness", type=float, default=0.05, help="points within this of the height [m]")
ap.add_argument("--threshold", type=float, default=0.03, help="section line threshold (as plan_section.py)")
ap.add_argument("--min-length", type=float, default=0.5, help="shortest piece of section line kept [m]")
ap.add_argument("--cell", type=float, default=0.02, help="density grid cell of the section line [m] (as plan_section.py)")
ap.add_argument("--sigma", type=float, default=3.0, help="smoothing of the density [cells] (as plan_section.py)")
ap.add_argument("--piece", type=float, default=0.5, help="size of the squares the line is cut into [m]")
ap.add_argument("--across", type=float, default=0.2, help="points within this of the piece's line [m]")
ap.add_argument("--gap", type=float, default=2.0, help="a pause in a piece's point times longer than this starts a new visit [s]")
ap.add_argument("--min-visit-points", type=int, default=15, help="points a visit needs for its own offset")
ap.add_argument("--plan", action="store_true", help="add the distance to the plan and draw on the plan")
a = ap.parse_args()

# ---- slice, section line, trajectory --------------------------------------------------------------------------------
f = laspy.read(os.path.join(a.result, "scan_lio_merged.laz"))
z_all = np.asarray(f.z)
m = np.abs(z_all - a.height) <= a.half_thickness
P = np.column_stack([np.asarray(f.x)[m], np.asarray(f.y)[m], z_all[m]])
t = np.asarray(f.gps_time)[m] * 1e-9
print(f"slice z = {a.height:+.2f} +- {a.half_thickness:.2f} m: {len(P)} points")
sec = Section(P[:, 0], P[:, 1], a.cell, a.sigma)
ridge = sec.ridge_cells(a.threshold, a.min_length)
T, _, pos = trajectory(a.result)
sensor = np.column_stack([np.interp(t, T, pos[:, k]) for k in range(3)])
# window start times: gyro_bias_per_window of the newest results JSON (written only with gyro-bias estimation; without
# it the windows column stays empty)
results = sorted((fn for fn in os.listdir(a.result) if fn.startswith("dMapper_results") and fn.endswith(".json")),
                 key=lambda fn: os.path.getmtime(os.path.join(a.result, fn)))
window_start = np.array([])
if results:
    with open(os.path.join(a.result, results[-1])) as fp:
        window_start = np.array([w["t_start_s"] for w in json.load(fp).get("gyro_bias_per_window", [])])
if len(window_start) == 0:
    print("no gyro_bias_per_window in the results JSON: the windows column stays empty")
tree = spatial.cKDTree(P[:, :2])

# ---- pieces ---------------------------------------------------------------------------------------------------------
lab, _ = ndimage.label(ridge, structure=np.ones((3, 3)))
ri, rj = np.nonzero(ridge)
cxy = sec.centres(ri, rj)
key = np.column_stack([lab[ri, rj], np.floor(cxy / a.piece).astype(np.int64)])
_, piece_of_cell = np.unique(key, axis=0, return_inverse=True)
piece_of_cell = piece_of_cell.ravel()


def runs(ws):
    """window indices as 'a-b;c' """
    ws = np.unique(ws)
    if len(ws) == 0:
        return ""
    out, s = [], ws[0]
    for p, q in zip(ws, np.r_[ws[1:], -10]):
        if q != p + 1:
            out.append(f"{s}" if s == p else f"{s}-{p}")
            s = q
    return ";".join(out)


rows, all_pairs = [], []  # all_pairs: (piece, offset difference, time between the visits) over all pieces
for k in range(piece_of_cell.max() + 1):
    cells = cxy[piece_of_cell == k]
    c = cells.mean(0)
    _, _, vt = np.linalg.svd(cells - c, full_matrices=False)
    d, n = vt[0], np.array([-vt[0][1], vt[0][0]])
    u = (cells - c) @ d
    length = u.max() - u.min() + sec.cell  # the line's extent: a diagonal line has fewer cells per metre
    if length < 0.2:
        continue
    half = np.abs(u).max() + sec.cell / 2
    idx = np.array(tree.query_ball_point(c, np.hypot(half, a.across)), int)
    if len(idx) == 0:
        continue
    rel = P[idx, :2] - c
    off, along = rel @ n, rel @ d
    keep = (np.abs(along) <= half) & (np.abs(off) <= a.across)
    idx, off, along = idx[keep], off[keep], along[keep]
    if len(idx) < a.min_visit_points:
        continue
    # the side of the line the sensor was on: a wall seen from both sides shows two faces, which is not an error, so
    # each side's offsets are taken against that side's own median and visits are only compared within one side
    side = np.where((sensor[idx, :2] - c) @ n >= 0.0, 1, -1)
    sides = np.unique(side)
    side_median = {s_: np.median(off[side == s_]) for s_ in sides}
    faces = side_median[1] - side_median[-1] if len(sides) == 2 else np.nan
    off = off - np.array([side_median[s_] for s_ in side])
    core = np.abs(off) <= a.across
    thickness = 1.4826 * np.median(np.abs(off[core]))
    # visits: (first time, last time, median offset, points, side, robust scatter of its offsets, median position along)
    visits, half_split, longest = [], np.nan, 0
    for s_ in sides:
        sel = np.flatnonzero(side == s_)
        o = sel[np.argsort(t[idx][sel])]
        ts_, offs_, along_ = t[idx][o], off[o], along[o]
        cuts = np.flatnonzero(np.diff(ts_) > a.gap) + 1
        for s, e in zip(np.r_[0, cuts], np.r_[cuts, len(ts_)]):
            if e - s >= a.min_visit_points:
                med = np.median(offs_[s:e])
                visits.append((ts_[s], ts_[e - 1], med, e - s, s_, 1.4826 * np.median(np.abs(offs_[s:e] - med)),
                               np.median(along_[s:e])))
            if e - s >= 2 * a.min_visit_points and e - s > longest:
                # the longest visit's first half against its second half: the measure's own noise, no drift between visits
                h = (s + e) // 2
                half_split, longest = abs(np.median(offs_[s:h]) - np.median(offs_[h:e])), e - s
    visits.sort(key=lambda v: v[0])
    # every pair of visits from the same side: the difference of their offsets, the time between their middles, the
    # difference the two medians' own noise would give (1.25 x the standard error of a median difference, i.e. its
    # expected |value| for two noisy medians, ~1.2533 sqrt(s1^2/n1 + s2^2/n2) x 1.25 / 1.2533) and how far apart along the
    # piece the two visits' points lie (a corner or a second surface in the piece shifts the offset of a visit that saw
    # another stretch of it)
    pairs = [(abs(v[2] - w[2]), abs(v[0] + v[1] - w[0] - w[1]) / 2,
              1.25 * np.sqrt(v[5] ** 2 / v[3] + w[5] ** 2 / w[3]), abs(v[6] - w[6]))
             for i, v in enumerate(visits) for w in visits[i + 1:] if v[4] == w[4]]
    all_pairs += [(len(rows), dv, dt, noise, dal) for dv, dt, noise, dal in pairs]
    spread = [max(v[2] for v in visits if v[4] == s_) - min(v[2] for v in visits if v[4] == s_) for s_ in sides
              if sum(v[4] == s_ for v in visits) > 1]
    ts = np.sort(t[idx])
    ray = P[idx] - sensor[idx]  # 3D: a wall seen from above or below is grazed even if the horizontal ray hits it head-on
    rng = np.linalg.norm(ray, axis=1)
    inc = np.degrees(np.arccos(np.clip(np.abs(ray[:, :2] @ n) / (rng + 1e-12), 0, 1)))
    win = np.searchsorted(window_start, ts, side="right") - 1
    rows.append({
        "piece": len(rows), "x": c[0], "y": c[1], "dir_deg": np.degrees(np.arctan2(d[1], d[0])) % 180,
        "length_m": length, "points": len(idx), "thickness_m": thickness,
        "sides": len(sides), "faces_m": faces, "visits": len(visits), "pairs": len(pairs),
        "pair_diff_m": np.median([p[0] for p in pairs]) if pairs else np.nan,
        "pair_excess_m": np.median([p[0] - p[2] for p in pairs]) if pairs else np.nan,
        "visit_spread_m": max(spread) if spread else np.nan, "half_split_m": half_split,
        "visit_offsets_m": ";".join(f"{v[2]:+.3f}" for v in visits), "visit_sides": ";".join(f"{v[4]:+d}" for v in visits),
        "visit_times_s": ";".join(f"{v[0]:.1f}-{v[1]:.1f}" for v in visits),
        "seen_s": sum(v[1] - v[0] for v in visits), "range_m": np.median(rng), "incidence_deg": np.median(inc),
        "windows": runs(win), "t_first_s": ts[0], "t_last_s": ts[-1], "_cells": cells,
    })
print(f"pieces: {len(rows)} ({sum(r['length_m'] for r in rows):.0f} m of line), visit pairs (same side): {len(all_pairs)}")
if not rows:
    raise SystemExit("no pieces of section line at this height: nothing to measure (try another --height or --threshold)")

# ---- plan distance (optional) ---------------------------------------------------------------------------------------
if a.plan:
    with open(os.path.join(cache_dir(a.result), "plan_fit.json")) as fp:
        fit = json.load(fp)
    pix = pymupdf.open(PLAN_PDF)[0].get_pixmap(dpi=150, colorspace=pymupdf.csGRAY)
    A = np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width)
    dist = ndimage.distance_transform_edt(A > 128) / PLAN_PX_PER_M
    th = np.radians(fit["rotation_deg"])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

    def plan_px(xy):
        q = xy @ R.T + np.asarray(fit["shift_m"])
        return q[:, 0] * PLAN_PX_PER_M, A.shape[0] - q[:, 1] * PLAN_PX_PER_M

    for r in rows:
        col, row = plan_px(r["_cells"])
        r["plan_dist_m"] = np.median(ndimage.map_coordinates(dist, [row - 0.5, col - 0.5], order=1, mode="nearest"))

# ---- csv and summary ------------------------------------------------------------------------------------------------
stem = os.path.join(cache_dir(a.result), f"wall_precision_{a.height:+.2f}")
fields = [k for k in rows[0] if not k.startswith("_")]
with open(stem + ".csv", "w", newline="") as fp:
    w = csv.DictWriter(fp, fields, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items() if k in fields})
print("wrote", stem + ".csv")


def col(name):
    return np.array([r[name] for r in rows], float)


L = col("length_m")
for name in ("thickness_m", "pair_diff_m", "pair_excess_m", "half_split_m", "visit_spread_m", "faces_m", "plan_dist_m", "seen_s", "range_m", "incidence_deg", "visits"):
    if name in rows[0]:
        v = col(name)
        ok = np.isfinite(v)
        print(f"{name:15s} p10 {np.percentile(v[ok], 10):7.3f}  p50 {np.percentile(v[ok], 50):7.3f}  "
              f"p90 {np.percentile(v[ok], 90):7.3f}  ({ok.sum()} pieces)")
# visit pairs pooled over all pieces, by the time between the two visits: a precise map gives the noise floor
# (half_split_m) at every time separation; drift between visits grows with it
if all_pairs:
    P_ = np.array(all_pairs)  # piece, |difference|, time between, expected noise, |difference of position along|
    print("visit pairs (same side) by time between the visits: |offset difference| p50 / p90, the pairs' own noise p50,"
          " difference minus noise p50 [m]")
    for lo_, hi_ in ((0, 10), (10, 30), (30, 100), (100, 1e9)):
        sel = (P_[:, 2] >= lo_) & (P_[:, 2] < hi_)
        if sel.any():
            print(f"  {lo_:4.0f}-{hi_ if hi_ < 1e9 else np.inf:<5} s: {sel.sum():5d} pairs  p50 {np.median(P_[sel, 1]):.3f}"
                  f"  p90 {np.percentile(P_[sel, 1], 90):.3f}  noise {np.median(P_[sel, 3]):.3f}"
                  f"  excess {np.median(P_[sel, 1] - P_[sel, 3]):+.3f}")
    print(f"  |offset difference| vs |difference of position along the piece|: Spearman "
          f"{stats.spearmanr(P_[:, 1], P_[:, 4])[0]:+.2f} (positive: part of the difference is the piece's shape, not pose)")
else:
    print("no two visits of a piece from the same side: no visit pairs")
if "plan_dist_m" in rows[0]:
    for ns in (1, 2):
        v = np.array([r["plan_dist_m"] for r in rows if r["sides"] == ns])
        if len(v):
            print(f"plan_dist_m, pieces seen from {ns} side{'s' if ns == 2 else ''}: p50 {np.median(v):.3f} p90 "
                  f"{np.percentile(v, 90):.3f} ({len(v)} pieces; a wall drawn as two strokes puts a two-sided ridge "
                  f"between them)")
print("Spearman rank correlation (pieces with both values):")
targets = [n for n in ("thickness_m", "pair_diff_m", "plan_dist_m") if n in rows[0]]
for tgt in targets:
    parts = []
    for cause in ("seen_s", "points", "visits", "range_m", "incidence_deg"):
        u, v = col(tgt), col(cause)
        ok = np.isfinite(u) & np.isfinite(v)
        rho = stats.spearmanr(u[ok], v[ok])[0]
        parts.append(f"{cause} {rho:+.2f}")
    print(f"  {tgt:15s} vs " + ", ".join(parts))

# ---- images ---------------------------------------------------------------------------------------------------------
CLASSES = {"thickness_m": (0.02, 0.04, 0.08), "pair_diff_m": (0.02, 0.04, 0.08)}
PALETTE = np.array([(0, 70, 220), (0, 150, 0), (240, 140, 0), (220, 0, 0)], np.uint8)
GREY = np.array((150, 150, 150), np.uint8)


def canvas():
    """background image and the map xy -> pixel (col, row) function"""
    if a.plan:
        s, crop = 3, (0, 1300, 5600, 4200)
        img = np.stack([A[::s, ::s]] * 3, -1).copy()
        img = (255 - (255 - img) * 0.55).astype(np.uint8)[crop[1] // s:crop[3] // s, crop[0] // s:crop[2] // s].copy()
        return img, lambda xy: tuple(v / s - o / s for v, o in zip(plan_px(xy), crop[:2]))
    ppm = 40.0
    allc = np.concatenate([r["_cells"] for r in rows])
    lo, hi = allc.min(0) - 1.0, allc.max(0) + 1.0
    w, h = int((hi[0] - lo[0]) * ppm) + 1, int((hi[1] - lo[1]) * ppm) + 1
    img = np.full((h, w, 3), 255, np.uint8)
    for gx in np.arange(np.ceil(lo[0]), hi[0]):
        img[:, int((gx - lo[0]) * ppm)] = 225
    for gy in np.arange(np.ceil(lo[1]), hi[1]):
        img[h - 1 - int((gy - lo[1]) * ppm), :] = 225
    return img, lambda xy: ((xy[:, 0] - lo[0]) * ppm, h - 1 - (xy[:, 1] - lo[1]) * ppm)


for name, png in (("thickness_m", "_thickness.png"), ("pair_diff_m", "_visits.png")):
    img, to_px = canvas()
    limits = CLASSES[name]
    for r in rows:
        v = r[name]
        colour = GREY if not np.isfinite(v) else PALETTE[np.searchsorted(limits, v, side="right")]
        c_, r_ = (np.asarray(p).astype(int) for p in to_px(r["_cells"]))
        ok = (c_ >= 0) & (c_ < img.shape[1]) & (r_ >= 0) & (r_ < img.shape[0])
        img[r_[ok], c_[ok]] = colour
    img = np.ascontiguousarray(img)
    pymupdf.Pixmap(pymupdf.csRGB, img.shape[1], img.shape[0], img.tobytes(), False).save(stem + png)
    print(f"wrote {stem + png}: blue < {limits[0]} <= green < {limits[1]} <= orange < {limits[2]} <= red"
          + (", grey = no two visits from the same side" if name == "pair_diff_m" else "") + " [m]")
