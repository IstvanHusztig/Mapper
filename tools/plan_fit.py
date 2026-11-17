"""Fits the top floor of a result of recording 44 onto the building plan and draws the map over it.

The walls of the map (merged cloud, world z in Z_BAND, 5 cm cells with enough points) are placed on the plan
(haus47_LG_plan.pdf at 150 dpi, scale 1:50 fixed) by the 2D rotation and shift that bring them closest to drawn lines
(squared distance to the nearest dark pixel, interpolated, capped at TRUNC). The start comes from the plan anchor of
map_quality_lib (rotation, stairwell centre); the map is turned about the stairwell over +-6 deg and shifted over +-1 m
in a coarse search, then refined (the anchor predates D1, which turned the world frame of 44 by about 3 deg). The start
room and the gallery (PARTS) are also fitted on their own, turned about their own centre starting from the whole fit, so
their rotations against it show which part of the map is turned; differences below about 0.05 deg are noise. Needs
numpy, scipy, laspy (with lazrs) and pymupdf.

A recording that doesn't start where 44 does has another world frame: --start gives its map -> plan rotation and the
stairwell centre in its map (read them off the map; the search around them is the same +-6 deg and +-1 m), --z-band
the world z of the storey's walls, and --no-parts leaves out the start room and gallery fits, whose times are 44's.

Usage: plan_fit.py <result_folder> [out_prefix] [--start ROT_DEG X Y] [--z-band LO HI] [--no-parts]
Writes <out_prefix>_whole_fit.png, _start_fit.png, _full_sheet.png, _zoom_gallery.png, _zoom_core.png (default prefix
<result>/map_quality_cache/plan_fit): red the whole floor, green the start room, blue the gallery. The whole-floor fit
goes to <result>/map_quality_cache/plan_fit.json (used by plan_section.py).
"""
import argparse
import json
import os

import laspy
import numpy as np
import pymupdf
from scipy import ndimage, optimize

from map_quality_lib import MAP_SHAFT, PLAN_PDF, PLAN_PX_PER_M, PLAN_ROTATION_DEG, PLAN_SHAFT_PX, cache_dir

CELL = 0.05            # wall cell size [m]
TRUNC = 0.30           # distances to the plan are capped here in the fit [m]
PARTS = {"start room (0-15 s)": (0.0, 15.0), "gallery out+back (49-71 s)": (48.9, 70.9)}

parser = argparse.ArgumentParser(description="Fits one storey of a result onto the building plan.")
parser.add_argument("result")
parser.add_argument("out_prefix", nargs="?")
parser.add_argument("--start", nargs=3, type=float, default=(PLAN_ROTATION_DEG, *MAP_SHAFT), metavar=("ROT_DEG", "X", "Y"),
                    help="map -> plan rotation [deg] and the stairwell centre in the map [m] (default: recording 44)")
parser.add_argument("--z-band", nargs=2, type=float, default=(-0.5, 1.5), metavar=("LO", "HI"),
                    help="world z band of the storey's walls [m] (default: the top floor of 44)")
parser.add_argument("--no-parts", action="store_true", help="no start room and gallery fits (their times are 44's)")
args = parser.parse_args()
result = args.result
prefix = args.out_prefix or os.path.join(cache_dir(result), "plan_fit")
START_ROTATION_DEG, MAP_SHAFT = args.start[0], (args.start[1], args.start[2])
Z_BAND = tuple(args.z_band)  # world z band of the storey's walls [m]
if args.no_parts:
    PARTS = {}

# ---- plan ---------------------------------------------------------------------------------------------------------
pix = pymupdf.open(PLAN_PDF)[0].get_pixmap(dpi=150, colorspace=pymupdf.csGRAY)
A = np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width)
H = A.shape[0]
dist = ndimage.distance_transform_edt(A > 128) / PLAN_PX_PER_M  # metres to the nearest dark pixel


def to_plan(xy, p):
    """map xy -> plan metres for p = (rotation [deg], shift x, shift y)"""
    th = np.radians(p[0])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return xy @ R.T + p[1:3]


def plan_px(q):
    return q[:, 0] * PLAN_PX_PER_M, H - q[:, 1] * PLAN_PX_PER_M


def residuals(q):
    """distance to the nearest drawn line, interpolated between pixel centres so that the fit's cost is continuous"""
    c, r = plan_px(q)
    return ndimage.map_coordinates(dist, [r - 0.5, c - 0.5], order=1, mode="nearest")


def cost(p, xy, w):
    d = np.minimum(residuals(to_plan(xy, p)), TRUNC)
    return np.sum(w * d * d) / np.sum(w)


# ---- map: wall cells of the top floor ----------------------------------------------------------------------------------
f = laspy.read(os.path.join(result, "scan_lio_merged.laz"))
x, y, z, t = (np.asarray(f.x), np.asarray(f.y), np.asarray(f.z), np.asarray(f.gps_time) * 1e-9)
m = (z > Z_BAND[0]) & (z < Z_BAND[1])
x, y, t = x[m], y[m], t[m]


def wall_cells(sel, min_pts=8):
    """centres and weights (sqrt of the point count) of the cells with at least min_pts selected points"""
    k = np.stack([np.floor(x[sel] / CELL), np.floor(y[sel] / CELL)], 1).astype(np.int64)
    u, n = np.unique(k, axis=0, return_counts=True)
    keep = n >= min_pts
    return (u[keep] + 0.5) * CELL, np.sqrt(n[keep]).astype(float)


def fit(xy, w, p_init, pivot, span=(6.0, 1.0)):
    """coarse search turning the map about pivot (a map point, kept where p_init places it) and shifting it, then refined"""
    q0 = to_plan(np.asarray(pivot, float)[None], p_init)[0]
    best = None
    for dth in np.arange(-span[0], span[0] + 1e-9, 0.5):
        th = p_init[0] + dth
        base = q0 - to_plan(np.asarray(pivot, float)[None], [th, 0.0, 0.0])[0]
        for dx in np.arange(-span[1], span[1] + 1e-9, 0.25):
            for dy in np.arange(-span[1], span[1] + 1e-9, 0.25):
                p = np.r_[th, base + [dx, dy]]
                c = cost(p, xy, w)
                if best is None or c < best[0]:
                    best = (c, p)
    r = optimize.minimize(cost, best[1], args=(xy, w), method="Powell", options={"xtol": 1e-4, "ftol": 1e-9})
    return r.x


def stats(xy, w, p):
    """mean distance to the plan (capped at 1 m), share within 10 cm, share within 25 cm, all weighted like the fit"""
    d = residuals(to_plan(xy, p))
    return np.average(np.minimum(d, 1.0), weights=w), np.average(d < 0.10, weights=w), np.average(d < 0.25, weights=w)


xy_all, w_all = wall_cells(np.ones_like(t, bool))
th0 = np.radians(START_ROTATION_DEG)
R0 = np.array([[np.cos(th0), -np.sin(th0)], [np.sin(th0), np.cos(th0)]])
p0 = np.r_[START_ROTATION_DEG, np.array([PLAN_SHAFT_PX[0], H - PLAN_SHAFT_PX[1]]) / PLAN_PX_PER_M - R0 @ np.array(MAP_SHAFT)]

p_all = fit(xy_all, w_all, p0, MAP_SHAFT)
s_all = stats(xy_all, w_all, p_all)
print(f"whole floor fit: rotation {p_all[0]:.2f} deg, mean |d| {s_all[0]:.3f} m, within 10 cm {s_all[1]:.0%}, "
      f"within 25 cm {s_all[2]:.0%}")
with open(os.path.join(cache_dir(result), "plan_fit.json"), "w") as fp:
    json.dump({"rotation_deg": p_all[0], "shift_m": list(p_all[1:3])}, fp)  # read by plan_section.py
part_fits = {}
for name, (a, b) in PARTS.items():
    xy, w = wall_cells((t >= a) & (t < b), 3)
    p = fit(xy, w, p_all, np.average(xy, axis=0, weights=w), (2.0, 0.5))
    part_fits[name] = p
    s_own, s_glob = stats(xy, w, p), stats(xy, w, p_all)
    print(f"{name}: own-fit rotation {p[0]:.2f} deg ({p[0] - p_all[0]:+.2f} vs whole), mean |d| own {s_own[0]:.3f} m / "
          f"in whole fit {s_glob[0]:.3f} m, within 10 cm own {s_own[1]:.0%} / whole {s_glob[1]:.0%}")
names = list(PARTS)
if names:
    print(f"gallery - start rotation: {part_fits[names[1]][0] - part_fits[names[0]][0]:+.2f} deg")


# ---- overlays ---------------------------------------------------------------------------------------------------------
def overlay(p, png, crop, s=3):
    """every 4th point over the faded plan (red whole floor, blue gallery, green start room), crop in 150 dpi pixels"""
    img = np.stack([A[::s, ::s]] * 3, -1).copy()
    img = (255 - (255 - img) * 0.55).astype(np.uint8)
    sel_all = np.arange(len(t))[::4]
    layers = [(sel_all, (220, 0, 0))]
    for name, col in zip(reversed(names), ((0, 90, 255), (0, 170, 0))):
        a, b = PARTS[name]
        layers.append((sel_all[(t[sel_all] >= a) & (t[sel_all] < b)], col))
    for sel, col in layers:
        c, r = plan_px(to_plan(np.column_stack([x[sel], y[sel]]), p))
        c, r = (c / s).astype(int), (r / s).astype(int)
        ok = (c >= 0) & (c < img.shape[1]) & (r >= 0) & (r < img.shape[0])
        img[r[ok], c[ok]] = col
    out = np.ascontiguousarray(img[crop[1] // s:crop[3] // s, crop[0] // s:crop[2] // s])
    h, w_, _ = out.shape
    pymupdf.Pixmap(pymupdf.csRGB, w_, h, out.tobytes(), False).save(png)


def perimeter(p, png, crop, s=3, close_m=0.4, min_area_m2=0.5):
    """outline of the map's top floor (wall cells, gaps up to close_m closed, holes filled) in magenta over the plan"""
    img = np.stack([A[::s, ::s]] * 3, -1).copy()
    c, r = plan_px(to_plan(xy_all, p))
    c, r = (c / s).astype(int), (r / s).astype(int)
    ok = (c >= 0) & (c < img.shape[1]) & (r >= 0) & (r < img.shape[0])
    mask = np.zeros(img.shape[:2], bool)
    mask[r[ok], c[ok]] = True
    rad = max(1, int(round(close_m * PLAN_PX_PER_M / s / 2)))
    disk = np.hypot(*np.mgrid[-rad:rad + 1, -rad:rad + 1]) <= rad
    mask = ndimage.binary_fill_holes(ndimage.binary_closing(np.pad(mask, rad + 1), disk))[rad + 1:-rad - 1, rad + 1:-rad - 1]
    lab, n = ndimage.label(mask)
    area = ndimage.sum(mask, lab, range(1, n + 1)) * (s / PLAN_PX_PER_M) ** 2
    mask = np.isin(lab, 1 + np.flatnonzero(area >= min_area_m2))
    edge = mask & ~ndimage.binary_erosion(mask)
    img[ndimage.binary_dilation(edge, iterations=1)] = (230, 0, 200)
    out = np.ascontiguousarray(img[crop[1] // s:crop[3] // s, crop[0] // s:crop[2] // s])
    h, w_, _ = out.shape
    pymupdf.Pixmap(pymupdf.csRGB, w_, h, out.tobytes(), False).save(png)


perimeter(p_all, prefix + "_perimeter.png", (0, 1300, 5600, 4200))
overlay(p_all, prefix + "_whole_fit.png", (0, 1300, 5600, 4200))
if names:
    overlay(part_fits[names[0]], prefix + "_start_fit.png", (0, 1300, 5600, 4200))
overlay(p_all, prefix + "_full_sheet.png", (0, 0, A.shape[1], A.shape[0]), s=5)
overlay(p_all, prefix + "_zoom_gallery.png", (3000, 3550, 5300, 4150), s=2)
overlay(p_all, prefix + "_zoom_core.png", (1650, 2550, 3450, 3950), s=2)
