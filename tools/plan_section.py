"""Horizontal cross-section of a result of recording 44 at height H, drawn as lines over the building plan.

The points of the merged cloud within --half-thickness of the world height H are reduced to the line through the densest
part of each wall (section_lib.Section: --cell grid, --sigma smoothing, --threshold, --min-length). The lines are placed on the plan with the whole-floor fit of plan_fit.py
(<result>/map_quality_cache/plan_fit.json; run plan_fit.py first). Needs numpy, scipy, laspy (with lazrs) and pymupdf.

Usage: plan_section.py <result_folder> [--height H] [--threshold T [T ...]] [--half-thickness M] [--out PNG]
Writes <result>/map_quality_cache/plan_section_<H>_t<T>.png (whole floor) and ..._sheet.png (whole plan sheet) for each
threshold. With several thresholds also plan_section_<H>_sweep.png (and _sheet): each line in the colour of the highest
threshold it survives, red (only the lowest: faint, seen briefly or from far) -> orange -> green -> blue -> purple.
"""
import argparse
import json
import os

import laspy
import numpy as np
import pymupdf
from map_quality_lib import PLAN_PDF, PLAN_PX_PER_M, cache_dir
from section_lib import Section

ap = argparse.ArgumentParser()
ap.add_argument("result")
ap.add_argument("--height", type=float, default=0.0, help="world z of the section [m] (top floor of 44: slab -1.1)")
ap.add_argument("--half-thickness", type=float, default=0.05, help="points within this of the height [m]")
ap.add_argument("--cell", type=float, default=0.02, help="density grid cell [m]")
ap.add_argument("--sigma", type=float, default=3.0, help="smoothing of the density [cells]")
ap.add_argument("--threshold", type=float, nargs="+", default=[0.03],
                help="minimum density as a share of the 99th percentile of the density (lower keeps fainter walls); "
                     "up to 5 values give one image each plus a combined one")
ap.add_argument("--min-length", type=float, default=0.5, help="shortest piece of line kept [m]")
ap.add_argument("--out", help="output png (default in the result's map_quality_cache)")
a = ap.parse_args()
if len(a.threshold) > 5:
    ap.error("at most 5 thresholds")

with open(os.path.join(cache_dir(a.result), "plan_fit.json")) as fp:
    fit = json.load(fp)

# ---- density ridges of the slice ----------------------------------------------------------------------------------
f = laspy.read(os.path.join(a.result, "scan_lio_merged.laz"))
z = np.asarray(f.z)
m = np.abs(z - a.height) <= a.half_thickness
x, y = np.asarray(f.x)[m], np.asarray(f.y)[m]
print(f"slice z = {a.height:+.2f} +- {a.half_thickness:.2f} m: {m.sum()} points")
sec = Section(x, y, a.cell, a.sigma)


# ---- over the plan --------------------------------------------------------------------------------------------------
pix = pymupdf.open(PLAN_PDF)[0].get_pixmap(dpi=150, colorspace=pymupdf.csGRAY)
A = np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width)
th = np.radians(fit["rotation_deg"])
R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])


def draw(png, crop, s, ri, rj, colours):
    """one pixel per ridge cell (no thickening) in its colour over the faded plan, crop in 150 dpi pixels"""
    xy = sec.centres(ri, rj)
    q = xy @ R.T + np.asarray(fit["shift_m"])
    col, row = q[:, 0] * PLAN_PX_PER_M, A.shape[0] - q[:, 1] * PLAN_PX_PER_M
    img = np.stack([A[::s, ::s]] * 3, -1).copy()
    img = (255 - (255 - img) * 0.55).astype(np.uint8)
    c, r = (col / s).astype(int), (row / s).astype(int)
    ok = (c >= 0) & (c < img.shape[1]) & (r >= 0) & (r < img.shape[0])
    img[r[ok], c[ok]] = colours[ok]
    out = np.ascontiguousarray(img[crop[1] // s:crop[3] // s, crop[0] // s:crop[2] // s])
    h, w, _ = out.shape
    pymupdf.Pixmap(pymupdf.csRGB, w, h, out.tobytes(), False).save(png)
    print("wrote", png)


def draw_both(stem, ri, rj, colours):
    draw(stem + ".png", (0, 1300, 5600, 4200), 3, ri, rj, colours)
    draw(stem + "_sheet.png", (0, 0, A.shape[1], A.shape[0]), 5, ri, rj, colours)


thresholds = sorted(a.threshold)
level = np.zeros(sec.D.shape, int)  # highest threshold (1-based) a cell's line survives, 0 = not drawn
for k, t in enumerate(thresholds):
    ridge = sec.ridge_cells(t, a.min_length)
    level[ridge] = k + 1
    ri, rj = np.nonzero(ridge)
    print(f"threshold {t:g}: ridge cells kept {len(ri)} ({len(ri) * a.cell:.0f} m of line)")
    stem = a.out[:-4] if a.out and len(thresholds) == 1 else os.path.join(
        os.path.dirname(a.out) if a.out else cache_dir(a.result), f"plan_section_{a.height:+.2f}_t{t:g}")
    draw_both(stem, ri, rj, np.tile(np.array([220, 0, 0], np.uint8), (len(ri), 1)))

if len(thresholds) > 1:
    # faint (survives only the lowest threshold) red -> orange -> green -> blue (survives the highest)
    palette = np.array([(220, 0, 0), (240, 140, 0), (0, 150, 0), (0, 70, 220), (140, 0, 200)], np.uint8)
    ri, rj = np.nonzero(level)
    for k, t in enumerate(thresholds):
        print(f"combined: survives up to {t:g} -> RGB {tuple(int(v) for v in palette[k])}")
    draw_both(os.path.join(os.path.dirname(a.out) if a.out else cache_dir(a.result), f"plan_section_{a.height:+.2f}_sweep"),
              ri, rj, palette[level[ri, rj] - 1])
