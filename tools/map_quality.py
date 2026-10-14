"""Map quality of dMapper results on recording 44: the three measures of equal weight, per result and their spread.

Usage: python tools/map_quality.py [--floors] [--overlay] [label=]result_folder ...
  label         44_s0 / 44_s1 / 44_s2 (needed for --floors; default: the folder name)
  --floors      also run tools/evaluate_floors.py (railing NN and stairwell offsets; a few minutes per result)
  --overlay     write <result>/map_quality_cache/plan_overlay.png (first 100 s over the building plan)

Measures:
  global geometry  facade angle along the gallery (way out / way back) minus at the start [deg]; the plan makes them
                   parallel, so 0 is right
  consistency      revisits: heading [deg] and offset [m] between two passes of the same place (median and worst |heading|,
                   median offset; pairs with registration quality >= 1.08). `pairs` is registered / all pairs: a pair
                   is not registered when too few points lie near the place or the peak is weak, which a badly doubled
                   wall gives but so does poor geometry (the noise floor registers 4-6 of 11 on 44 s0-s2), so compare
                   the count with the noise floor's; a heading at the +-8 deg search limit is shown as >=8 (saturated)
  sharpness        pose jitter (rotation over 10 nodes, p50 / p99 [deg]); with --floors the railing NN [cm] and the
                   stairwell offsets [cm]
A measure that recording 44 always allows but that cannot be computed (the facade not found, the stairwell not
found) is shown as FAIL: the map is broken there. The last line is the spread (max - min) over the results. Needs
numpy (and pymupdf for --overlay).
"""
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_quality_lib as mq

QUALITY_MIN = 1.08


def measure(result, overlay):
    pts = mq.points(result)
    T, Rs, P = mq.trajectory(result)
    fa = mq.facade_angles(pts)
    row = {}
    start = fa.get("start")
    for key in ("gallery_out", "gallery_back"):
        row["facade_" + key] = fa[key][0] - start[0] if fa.get(key) and start else np.nan
    yaws, offs, saturated = [], [], False
    for a, b in mq.REVISIT_PAIRS:
        r = mq.revisit(pts, T, P, a, b)
        if r and r[2] >= QUALITY_MIN:
            yaws.append(abs(r[0]))
            offs.append(r[1])
            saturated = saturated or r[3]
    row["revisit_n"] = f"{len(yaws)}/{len(mq.REVISIT_PAIRS)}"
    row["revisit_yaw_med"] = np.median(yaws) if yaws else np.nan
    row["revisit_yaw_max"] = np.max(yaws) if yaws else np.nan
    row["revisit_saturated"] = saturated
    row["revisit_off_med"] = np.median(offs) if offs else np.nan
    row["jitter_p50"], row["jitter_p99"] = mq.jitter(Rs)
    if overlay:
        mq.plan_overlay(pts, os.path.join(mq.cache_dir(result), "plan_overlay.png"))
    return row


def floors(labelled):
    """railing NN and stairwell offsets per label from tools/evaluate_floors.py"""
    args = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "evaluate_floors.py"), "--results"]
    args += [f"{label}={folder}" for label, folder in labelled]
    text = subprocess.run(args, capture_output=True, text=True, env=dict(os.environ, PYTHONIOENCODING="utf-8")).stdout
    out = {}
    for line in text.splitlines():
        tok = line.split()
        if tok and any(tok[0] == label for label, _ in labelled) and len(tok) >= 10:
            def f(x):
                try:
                    return float(x)
                except ValueError:
                    return np.nan
            out[tok[0]] = {"stair": [f(x) for x in tok[1:5]], "railing": f(tok[9])}
    return out


def main():
    argv = sys.argv[1:]
    use_floors = "--floors" in argv
    overlay = "--overlay" in argv
    items = [a for a in argv if not a.startswith("--")]
    if not items:
        sys.exit(__doc__)
    labelled = []
    for it in items:
        label, folder = it.split("=", 1) if "=" in it else (os.path.basename(os.path.normpath(it)), it)
        labelled.append((label, folder))
    rows = {}
    for label, folder in labelled:
        print(f"[{label}] measuring ...", file=sys.stderr, flush=True)
        rows[label] = measure(folder, overlay)
    if use_floors:
        print("[floors] evaluate_floors.py ...", file=sys.stderr, flush=True)
        for label, v in floors(labelled).items():
            rows[label]["railing"] = v["railing"]
            rows[label]["stair_max"] = np.nanmax(v["stair"]) if np.isfinite(v["stair"]).any() else np.nan
    cols = [("facade out", "facade_gallery_out", "{:+.2f}"), ("facade back", "facade_gallery_back", "{:+.2f}"),
            ("revisit |yaw| med", "revisit_yaw_med", "{:.2f}"), ("worst", "revisit_yaw_max", "{:.2f}"),
            ("offset med [m]", "revisit_off_med", "{:.2f}"), ("pairs", "revisit_n", "{}"),
            ("jitter p50", "jitter_p50", "{:.2f}"), ("p99", "jitter_p99", "{:.2f}")]
    if use_floors:
        cols += [("railing NN [cm]", "railing", "{:.1f}"), ("stair max [cm]", "stair_max", "{:.1f}")]
    width = max(len(l) for l, _ in labelled) + 2
    print("result".ljust(width) + "".join(h.rjust(max(len(h), 7) + 2) for h, _, _ in cols))
    for label, _ in labelled:
        r = rows[label]
        cells = []
        for h, k, fmt in cols:
            v = r.get(k, np.nan)
            if v is None or (isinstance(v, float) and np.isnan(v)):
                s = "FAIL" if k in ("facade_gallery_out", "facade_gallery_back", "stair_max", "revisit_yaw_max") else "-"
            else:
                s = fmt.format(v)
                if k == "revisit_yaw_max" and r.get("revisit_saturated"):
                    s = ">=" + s
            cells.append(s.rjust(max(len(h), 7) + 2))
        print(label.ljust(width) + "".join(cells))
    if len(labelled) > 1:
        cells = []
        for h, k, fmt in cols:
            vals = [rows[l].get(k, np.nan) for l, _ in labelled]
            vals = [float(v) for v in vals if isinstance(v, (int, float)) and not np.isnan(v)]
            cells.append(("{:.2f}".format(max(vals) - min(vals)) if len(vals) > 1 else "-").rjust(max(len(h), 7) + 2))
        print("spread".ljust(width) + "".join(cells))
    print("facade: gallery minus start (0 = parallel, as on the plan); revisit: two passes of the same place (0 = no doubled "
          "wall); jitter: rotation over 10 nodes [deg]")


if __name__ == "__main__":
    main()
