"""Fits the top floor of a result of recording 44 onto the building plan and draws the map over it.

The walls of the map (merged cloud, world z in Z_BAND, 5 cm cells with enough points) are placed on the plan
(haus47_LG_plan.pdf at 150 dpi, scale 1:50 fixed) by the 2D rotation and shift that bring them closest to drawn lines
(squared distance to the nearest dark pixel, capped at TRUNC). The start comes from the plan anchor of map_quality_lib
(rotation, stairwell centre), searched coarsely over +-3 deg / +-1 m and then refined. The start room and the gallery
(PARTS) are also fitted on their own, starting from the whole fit, so their rotations against it show which part of the
map is turned. Needs numpy, scipy, laspy (with lazrs) and pymupdf.

Usage: plan_fit.py <result_folder> [out_prefix]
Writes <out_prefix>_whole_fit.png, _start_fit.png, _full_sheet.png, _zoom_gallery.png, _zoom_core.png (default prefix
<result>/map_quality_cache/plan_fit): red the whole floor, green the start room, blue the gallery.
"""
import os
import sys

import laspy
import numpy as np
import pymupdf
from scipy import ndimage, optimize

from map_quality_lib import MAP_SHAFT, PLAN_PDF, PLAN_PX_PER_M, PLAN_ROTATION_DEG, PLAN_SHAFT_PX, cache_dir

Z_BAND = (-0.5, 1.5)   # world z band of the top floor's walls [m]
CELL = 0.05            # wall cell size [m]
TRUNC = 0.30           # distances to the plan are capped here in the fit [m]
PARTS = {"start room (0-15 s)": (0.0, 15.0), "gallery out+back (49-71 s)": (48.9, 70.9)}

result = sys.argv[1]
prefix = sys.argv[2] if len(sys.argv) > 2 else os.path.join(cache_dir(result), "plan_fit")

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
    c, r = plan_px(q)
    c = np.clip(c.astype(int), 0, A.shape[1] - 1)
    r = np.clip(r.astype(int), 0, A.shape[0] - 1)
    return dist[r, c]


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


def fit(xy, w, p_init, span=(3.0, 1.0)):
    best = None
    for dth in np.arange(-span[0], span[0] + 1e-9, 0.5):
        for dx in np.arange(-span[1], span[1] + 1e-9, 0.25):
            for dy in np.arange(-span[1], span[1] + 1e-9, 0.25):
                p = p_init + [dth, dx, dy]
                c = cost(p, xy, w)
                if best is None or c < best[0]:
                    best = (c, p)
    r = optimize.minimize(cost, best[1], args=(xy, w), method="Powell", options={"xtol": 1e-4, "ftol": 1e-9})
    return r.x


def stats(xy, w, p):
    """mean distance to the plan (capped at 1 m), share within 10 cm, share within 25 cm"""
    d = residuals(to_plan(xy, p))
    return np.average(np.minimum(d, 1.0), weights=w), np.mean(d < 0.10), np.mean(d < 0.25)


xy_all, w_all = wall_cells(np.ones_like(t, bool))
th0 = np.radians(PLAN_ROTATION_DEG)
R0 = np.array([[np.cos(th0), -np.sin(th0)], [np.sin(th0), np.cos(th0)]])
p0 = np.r_[PLAN_ROTATION_DEG, np.array([PLAN_SHAFT_PX[0], H - PLAN_SHAFT_PX[1]]) / PLAN_PX_PER_M - R0 @ np.array(MAP_SHAFT)]

p_all = fit(xy_all, w_all, p0)
s_all = stats(xy_all, w_all, p_all)
print(f"whole floor fit: rotation {p_all[0]:.2f} deg, mean |d| {s_all[0]:.3f} m, within 10 cm {s_all[1]:.0%}, "
      f"within 25 cm {s_all[2]:.0%}")
part_fits = {}
for name, (a, b) in PARTS.items():
    xy, w = wall_cells((t >= a) & (t < b), 3)
    p = fit(xy, w, p_all, (2.0, 0.5))
    part_fits[name] = p
    s_own, s_glob = stats(xy, w, p), stats(xy, w, p_all)
    print(f"{name}: own-fit rotation {p[0]:.2f} deg ({p[0] - p_all[0]:+.2f} vs whole), mean |d| own {s_own[0]:.3f} m / "
          f"in whole fit {s_glob[0]:.3f} m, within 10 cm own {s_own[1]:.0%} / whole {s_glob[1]:.0%}")
names = list(PARTS)
print(f"gallery - start rotation: {part_fits[names[1]][0] - part_fits[names[0]][0]:+.2f} deg")


# ---- overlays ---------------------------------------------------------------------------------------------------------
def overlay(p, png, crop, s=3):
    """every 4th point over the faded plan (red whole floor, blue gallery, green start room), crop in 150 dpi pixels"""
    img = np.stack([A[::s, ::s]] * 3, -1).copy()
    img = (255 - (255 - img) * 0.55).astype(np.uint8)
    sel_all = np.arange(len(t))[::4]
    layers = [(sel_all, (220, 0, 0))]
    for name, col in ((names[1], (0, 90, 255)), (names[0], (0, 170, 0))):
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
overlay(part_fits[names[0]], prefix + "_start_fit.png", (0, 1300, 5600, 4200))
overlay(p_all, prefix + "_full_sheet.png", (0, 0, A.shape[1], A.shape[0]), s=5)
overlay(p_all, prefix + "_zoom_gallery.png", (3000, 3550, 5300, 4150), s=2)
overlay(p_all, prefix + "_zoom_core.png", (1650, 2550, 3450, 3950), s=2)
