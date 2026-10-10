"""Drift between floors, measured on a round stairwell: its railing circle has the same centre on every floor (the shaft
is plumb), so any offset between the circles is drift of the trajectory between the floors.

Floors are found from the trajectory's height (the levels where the sensor dwells, at least --min-storey apart). Every
point is assigned to the pass it was seen from: a stretch of time with the sensor on one floor. On each floor, the
railing band is a slice at --band below the sensor's level on that floor (about 0.5-1.0 m above the slab for a handheld
sensor). In each band a circle is fitted to the points of each pass separately (algebraic fit, trimmed to the ring).

Printed per floor band: every pass's circle (centre, radius, median residual, arc covered, points) and its offset from
the floor's own first pass. The summary gives, for each pair of neighbouring floors:
  - stack: the centre of a floor's railing seen from that floor, against the floor above's (needs a railing on both);
  - seen from below: the railing of the upper floor as seen from the lower floor, against as seen from its own floor.
Both are the drift of the lower floor against the upper one; a few cm is the noise.

Usage: stairwell_check.py <result_folder> <centre_x> <centre_y> [--radius 3.5] [--band -0.55 -0.05] [--min-storey 1.8]
centre: a rough guess of the shaft's centre in the coordinates of scan_lio_merged.laz (within half a metre is enough).
The chunk LAZ files are exported to ASCII with CloudCompare once (see spot_check.py).
"""
import argparse
import math
import os

import spot_check

PASS_GAP_S = 2.0
MIN_PASS_POINTS = 200


def trajectory(folder):
    """(t, x, y, z) of every trajectory node in world coordinates, sorted by time"""
    rows = []
    for laz, M in spot_check.chunk_poses(folder).items():
        csv = os.path.join(folder, laz.replace("scan_lio_", "trajectory_lio_").replace(".laz", ".csv"))
        with open(csv) as f:
            next(f)
            for line in f:
                v = line.split()
                x, y, z = float(v[4]), float(v[8]), float(v[12])
                rows.append((float(v[0]) * 1e-9,
                             M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
                             M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
                             M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3]))
    rows.sort()
    return rows


def floor_levels(traj, min_storey):
    """sensor heights where the trajectory dwells (time-weighted 0.1 m histogram peaks), top first"""
    hist = {}
    for (t0, _, _, z0), (t1, _, _, _) in zip(traj, traj[1:]):
        k = round(z0 / 0.1)
        hist[k] = hist.get(k, 0.0) + (t1 - t0)
    levels = []
    for k, w in sorted(hist.items(), key=lambda kv: -kv[1]):
        if w < 5.0:
            break
        if all(abs(k * 0.1 - lv) >= min_storey for lv in levels):
            levels.append(k * 0.1)
    # refine each level to the time-weighted mean height within +-0.4 m
    out = []
    for lv in levels:
        num = den = 0.0
        for (t0, _, _, z0), (t1, _, _, _) in zip(traj, traj[1:]):
            if abs(z0 - lv) < 0.4:
                num += z0 * (t1 - t0)
                den += t1 - t0
        out.append(num / den)
    return sorted(out, reverse=True)


def passes(traj, levels):
    """[(floor index, t0, t1)]: stretches with the sensor within 0.4 m of a floor's level"""
    out = []
    for t, _, _, z in traj:
        k = next((i for i, lv in enumerate(levels) if abs(z - lv) < 0.4), None)
        if k is None:
            continue
        if out and out[-1][0] == k and t - out[-1][2] <= PASS_GAP_S:
            out[-1][2] = t
        else:
            out.append([k, t, t])
    return [tuple(p) for p in out if p[2] - p[1] > 1.0]


def load_shaft(folder, cx, cy, radius, zmin, zmax):
    pts = []
    r2 = radius * radius
    for laz, M in spot_check.chunk_poses(folder).items():
        asc = os.path.join(folder, laz.replace(".laz", ".asc"))
        if not os.path.exists(asc):
            spot_check.load_near(folder, (cx, cy, 0.0), 0.01)  # exports every chunk once
        with open(asc) as f:
            for line in f:
                v = line.split()
                if len(v) < 5:
                    continue
                x, y, z = float(v[0]), float(v[1]), float(v[2])
                g = [M[r][0] * x + M[r][1] * y + M[r][2] * z + M[r][3] for r in range(3)]
                if (g[0] - cx) ** 2 + (g[1] - cy) ** 2 < r2 and zmin < g[2] < zmax:
                    pts.append((float(v[4]) * 1e-9, g[0], g[1], g[2]))
    return pts


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def fit_circle(P, cx, cy, r0):
    """algebraic (Kasa) circle fit, trimmed to the ring in three rounds; None if too few points"""
    cx, cy, r = cx, cy, r0
    Q = []
    for keep in (0.4, 0.15, 0.06):
        Q = [(x, y) for x, y in P if abs(math.hypot(x - cx, y - cy) - r) < keep]
        if len(Q) < MIN_PASS_POINTS:
            return None
        n = len(Q)
        sx = sum(q[0] for q in Q)
        sy = sum(q[1] for q in Q)
        sxx = sum(q[0] * q[0] for q in Q)
        syy = sum(q[1] * q[1] for q in Q)
        sxy = sum(q[0] * q[1] for q in Q)
        zz = [q[0] * q[0] + q[1] * q[1] for q in Q]
        A = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
        b = [-sum(z * q[0] for z, q in zip(zz, Q)), -sum(z * q[1] for z, q in zip(zz, Q)), -sum(zz)]
        d = det3(A)
        sol = []
        for k in range(3):
            M = [row[:] for row in A]
            for i in range(3):
                M[i][k] = b[i]
            sol.append(det3(M) / d)
        cx, cy = -sol[0] / 2, -sol[1] / 2
        r = math.sqrt(max(cx * cx + cy * cy - sol[2], 0.0))
    res = sorted(abs(math.hypot(x - cx, y - cy) - r) for x, y in Q)
    sectors = {int(math.degrees(math.atan2(y - cy, x - cx)) // 30) for x, y in Q}
    return {"cx": cx, "cy": cy, "r": r, "res": res[len(res) // 2], "n": len(Q), "arc": 30 * len(sectors)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("result")
    ap.add_argument("centre_x", type=float)
    ap.add_argument("centre_y", type=float)
    ap.add_argument("--radius", type=float, default=3.5, help="points within this of the centre guess [m]")
    ap.add_argument("--ring", type=float, default=1.5, help="starting guess of the railing radius [m]")
    ap.add_argument("--band", type=float, nargs=2, default=(-0.55, -0.05), help="railing band relative to the sensor level [m]")
    ap.add_argument("--min-storey", type=float, default=1.8, help="smallest height between two floors [m]")
    a = ap.parse_args()

    traj = trajectory(a.result)
    levels = floor_levels(traj, a.min_storey)
    ps = passes(traj, levels)
    name = os.path.basename(a.result.rstrip("/" + chr(92)))
    print(f"{name}: floors (sensor height) " + ", ".join(f"{k}: {lv:+.2f} m" for k, lv in enumerate(levels)))
    print("passes: " + ", ".join(f"floor {k} {t0:.0f}-{t1:.0f} s" for k, t0, t1 in ps))

    zmin = min(levels) + a.band[0] - 0.1
    zmax = max(levels) + a.band[1] + 0.1
    pts = load_shaft(a.result, a.centre_x, a.centre_y, a.radius, zmin, zmax)
    pass_of = lambda t: next((i for i, (_, t0, t1) in enumerate(ps) if t0 - 0.5 <= t <= t1 + 0.5), None)

    fits = {}  # (band floor, pass index) -> fit
    for f, lv in enumerate(levels):
        z0, z1 = lv + a.band[0], lv + a.band[1]
        by_pass = {}
        for t, x, y, z in pts:
            if z0 < z < z1:
                i = pass_of(t)
                if i is not None:
                    by_pass.setdefault(i, []).append((x, y))
        print(f"== railing band of floor {f} (z {z0:+.2f} .. {z1:+.2f})")
        ref = None
        for i in sorted(by_pass):
            k, t0, t1 = ps[i]
            fit = fit_circle(by_pass[i], a.centre_x, a.centre_y, a.ring)
            label = f"seen from floor {k}, {t0:5.0f}-{t1:4.0f} s"
            if fit is None:
                why = "too few points" if len(by_pass[i]) < MIN_PASS_POINTS else "no ring found"
                print(f"  {label}: {why} ({len(by_pass[i])} points in the band)")
                continue
            fits[(f, i)] = fit
            if ref is None and k == f:
                ref = fit
            off = ""
            if ref is not None:
                dx, dy = fit["cx"] - ref["cx"], fit["cy"] - ref["cy"]
                off = f"  offset {100 * dx:+6.1f} {100 * dy:+6.1f} cm = {100 * math.hypot(dx, dy):5.1f} cm"
            print(f"  {label}: centre {fit['cx']:+.3f} {fit['cy']:+.3f}  r {fit['r']:.3f}  res {100 * fit['res']:.1f} cm  "
                  f"arc {fit['arc']:3d} deg  n {fit['n']:6d}{off}")

    def own(f):
        """the floor's railing seen from its own floor (first such pass)"""
        return next((fits[(f, i)] for i, (k, _, _) in enumerate(ps) if k == f and (f, i) in fits), None)

    print("== drift between neighbouring floors")
    for f in range(1, len(levels)):
        up, low = own(f - 1), own(f)
        stack = f"{100 * math.hypot(low['cx'] - up['cx'], low['cy'] - up['cy']):5.1f} cm" if up and low else "    -   "
        seen = [fits[(f - 1, i)] for i, (k, _, _) in enumerate(ps) if k == f and (f - 1, i) in fits]
        below = f"{100 * math.hypot(seen[0]['cx'] - up['cx'], seen[0]['cy'] - up['cy']):5.1f} cm" if up and seen else "    -   "
        print(f"  floor {f} against floor {f - 1}: stack {stack}, upper railing seen from below {below}")


if __name__ == "__main__":
    main()
