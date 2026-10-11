"""Shape of the floors of a multi-storey result: tilt of the slabs, the lift shaft's walls on each floor, and a 2-D ICP
between the floors. A plumb building has level slabs and the same walls and cells on every floor, so a tilt, a wall
that turns or moves between floors, or an ICP rotation is drift of the trajectory between the floors.

Subcommands (all read the chunk .asc files, placed with spot_check.chunk_poses; column 5 of the .asc is the point time in ns):

  slabs <result> --levels -1.1 -4.0 -6.8
      A plane z = a + b x + c y is fitted to the points within 0.3 m of each slab height (every 4th point, trimmed least
      squares in three rounds: 0.3, 0.1, 0.05 m). Prints, per level, the points kept, their x range and the tilt about y
      (dz/dx) and about x (dz/dy) in degrees. A level slab gives 0; a floor that tilts along the path gives a few degrees.

  lift <result> --floor=-1.1:1-92,220-280 --floor=-4.0:99-158,201-212 --floor=-6.8:165-194
      The points of each floor are those seen in its own pass times within --band above its slab (default 0.3 .. 2.3 m).
      In each wall box (--wall name:x0,x1,y0,y1, repeated; default the lift shaft of 44: bottom, right, long) a line is
      fitted to the densest face (trimmed to 10, 10, 4 and 2 cm around the line). Prints per floor the line's direction (in
      brackets the change against the first floor that has the wall), the offset of its centre from the first floor's
      line, the points kept and the residual. "long@shaft" is the "long" wall restricted to the seconds in which the
      "bottom" wall was seen, so it compares the same stretch of the pass on each floor.

  floors-icp <result> --floor=...
      Per pair of floors (neighbours, then first and last), the wall points of --band (default 0.8 .. 1.8 m above the
      slab) of each floor, from its own pass times, over the 0.5 m cells (at least 20 points) both floors cover. The
      lower floor is moved onto the upper by 2-D point-to-point ICP (80 % of the pairs, 40 rounds), started at -6, 0 and
      +6 degrees. Prints per start the rotation, the shift, the rms of the matched pairs and the matched share.
      Starts that agree are one result; a rotation of a degree or more that every start finds is the floors being
      turned against each other. --box x0 x1 y0 y1 restricts the points to a region (the lift shaft of 44 is
      -7.35 -2.15 -1.2 4.0 with --band 0.3 2.3); --radius R with --centre x y keeps a ring around a point instead (from
      --radius-min, default 2.2 m, to R).

Floors are given as --floor=SLAB_Z:T0-T1[,T0-T1] (slab height in the world z, pass times in seconds; write the = so a
negative height is not read as an option). The defaults are the floors of recording 44.
The chunk LAZ files are exported to ASCII with CloudCompare once (see spot_check.py).
"""
import argparse
import math
import os

import spot_check

DEFAULT_FLOORS = ["-1.1:1-92,220-280", "-4.0:99-158,201-212", "-6.8:165-194"]
DEFAULT_WALLS = ["bottom:-5.7,-4.2,-0.50,0.15", "right:-4.05,-3.45,0.70,3.00", "long:-3.25,-2.05,-1.0,4.0"]


def parse_floor(s):
    """'-1.1:1-92,220-280' -> (-1.1, ((1, 92), (220, 280)))"""
    z, spans = s.split(":")
    return float(z), tuple(tuple(float(t) for t in sp.split("-")) for sp in spans.split(","))


def parse_wall(s):
    name, box = s.split(":")
    return name, tuple(float(v) for v in box.split(","))


def points(folder, stride=1):
    """(x, y, z, t) of every stride-th point in world coordinates, t in seconds"""
    k = 0
    for laz, M in spot_check.chunk_poses(folder).items():
        with open(os.path.join(folder, laz.replace(".laz", ".asc"))) as f:
            for line in f:
                k += 1
                if k % stride:
                    continue
                v = line.split()
                if len(v) < 5:
                    continue
                x, y, z = float(v[0]), float(v[1]), float(v[2])
                yield (M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
                       M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
                       M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3], float(v[4]) * 1e-9)


def in_spans(t, spans):
    return any(t0 <= t < t1 for t0, t1 in spans)


def solve3(A, b):
    M = [A[i][:] + [b[i]] for i in range(3)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(3):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][i] - f * M[c][i] for i in range(4)]
    return [M[i][3] / M[i][i] for i in range(3)]


def cmd_slabs(a):
    pts = {lv: [] for lv in a.levels}
    for g in points(a.result, a.stride):
        for lv in a.levels:
            if abs(g[2] - lv) < 0.3:
                pts[lv].append(g)
    name = os.path.basename(a.result.rstrip("/" + chr(92)))
    for lv in a.levels:
        P = pts[lv]
        pa, pb, pc = lv, 0.0, 0.0
        Q = []
        for keep in (0.3, 0.1, 0.05):
            Q = [p for p in P if abs(p[2] - (pa + pb * p[0] + pc * p[1])) < keep]
            if len(Q) < 500:
                break
            A = [[0.0] * 3 for _ in range(3)]
            r = [0.0] * 3
            for x, y, z, _ in Q:
                row = (1.0, x, y)
                for i in range(3):
                    r[i] += row[i] * z
                    for j in range(3):
                        A[i][j] += row[i] * row[j]
            pa, pb, pc = solve3(A, r)
        if not Q:
            print(f"{name} slab ~{lv:+.1f}: too few points ({len(P)})")
            continue
        xs = [p[0] for p in Q]
        print(f"{name} slab ~{lv:+.1f}: {len(Q)} pts, x {min(xs):+.0f}..{max(xs):+.0f} m, "
              f"tilt about y (dz/dx) {math.degrees(math.atan(pb)):+.2f} deg, about x (dz/dy) {math.degrees(math.atan(pc)):+.2f} deg")


def fit_line(P):
    """line through the densest face: principal direction, then points within 10, 4, 2 cm of it; (angle, centre, normal, n, rms)"""
    keep = P
    for tol in (0.10, 0.10, 0.04, 0.02):  # the first round only finds the direction of all points
        mx = sum(p[0] for p in keep) / len(keep)
        my = sum(p[1] for p in keep) / len(keep)
        sxx = sum((p[0] - mx) ** 2 for p in keep)
        syy = sum((p[1] - my) ** 2 for p in keep)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in keep)
        th = 0.5 * math.atan2(2 * sxy, sxx - syy)
        nx, ny = -math.sin(th), math.cos(th)
        k2 = [p for p in P if abs((p[0] - mx) * nx + (p[1] - my) * ny) < tol]
        if len(k2) < 50:
            break
        keep = k2
    res = (sum(((p[0] - mx) * nx + (p[1] - my) * ny) ** 2 for p in keep) / len(keep)) ** 0.5
    return th, mx, my, nx, ny, len(keep), res


def cmd_lift(a):
    floors = [parse_floor(s) for s in a.floor]
    walls = dict(parse_wall(s) for s in a.wall)
    xlo, xhi = min(b[0] for b in walls.values()) - 0.1, max(b[1] for b in walls.values())
    pts = {(w, f): [] for w in walls for f in range(len(floors))}
    for gx, gy, gz, t in points(a.result):
        if not xlo < gx < xhi:
            continue
        for f, (lv, ts) in enumerate(floors):
            if a.band[0] < gz - lv < a.band[1] and in_spans(t, ts):
                for w, (x0, x1, y0, y1) in walls.items():
                    if x0 < gx < x1 and y0 < gy < y1:
                        pts[(w, f)].append((gx, gy, int(t)))
    names = list(walls)
    if "bottom" in walls and "long" in walls:
        seen = {f: {p[2] for p in pts[("bottom", f)]} for f in range(len(floors))}  # seconds in which the bottom wall is seen
        for f in range(len(floors)):
            pts[("long@shaft", f)] = [p for p in pts[("long", f)] if p[2] in seen[f]]
        names.append("long@shaft")
    name = os.path.basename(a.result.rstrip("/" + chr(92)))
    for w in names:
        ref = None
        out = []
        for f in range(len(floors)):
            P = pts[(w, f)]
            if len(P) < 100:
                out.append(f"floor {-f}: {len(P)} pts")
                continue
            th, mx, my, nx, ny, n, res = fit_line(P)
            if ref is None:
                ref = (th, mx, my, nx, ny)
            d = (mx - ref[1]) * ref[3] + (my - ref[2]) * ref[4]
            dth = math.degrees((th - ref[0] + math.pi / 2) % math.pi - math.pi / 2)
            out.append(f"floor {-f}: {math.degrees(th) % 180:6.2f} deg ({dth:+.2f}), offset {d * 100:+5.1f} cm, {n} pts, res {res * 100:.1f} cm")
        print(f"{name} {w:6s}: " + " | ".join(out))


def cell(p):
    return int(p[0] // 0.5), int(p[1] // 0.5)


def dense_cells(P):
    c = {}
    for p in P:
        c[cell(p)] = c.get(cell(p), 0) + 1
    return {k for k, n in c.items() if n >= 20}


def voxel(P, r):
    d = {}
    for p in P:
        d.setdefault((int(p[0] // r), int(p[1] // r)), p)
    return list(d.values())


def icp(src, dst, th0):
    """src rotated by th about its centre, then shifted, onto dst; (deg, tx, ty, rms, matched share)"""
    G = {}
    for q in dst:
        G.setdefault((int(q[0] // 0.3), int(q[1] // 0.3)), []).append(q)
    cx = sum(p[0] for p in src) / len(src)
    cy = sum(p[1] for p in src) / len(src)
    th, tx, ty = th0, 0.0, 0.0
    for _ in range(40):
        c, s_ = math.cos(th), math.sin(th)
        pairs = []
        for p in src:
            x = cx + c * (p[0] - cx) - s_ * (p[1] - cy) + tx
            y = cy + s_ * (p[0] - cx) + c * (p[1] - cy) + ty
            gx, gy = int(x // 0.3), int(y // 0.3)
            best = None
            bd = 0.09
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    for q in G.get((gx + i, gy + j), ()):
                        d = (q[0] - x) ** 2 + (q[1] - y) ** 2
                        if d < bd:
                            bd, best = d, q
            if best:
                pairs.append((bd, p, best))
        pairs.sort(key=lambda e: e[0])
        pairs = pairs[:int(len(pairs) * 0.8)]
        # Procrustes: rotation about the source centre, then translation
        ma = [sum(e[1][k] for e in pairs) / len(pairs) for k in (0, 1)]
        mb = [sum(e[2][k] for e in pairs) / len(pairs) for k in (0, 1)]
        sxx = sum((e[1][0] - ma[0]) * (e[2][0] - mb[0]) + (e[1][1] - ma[1]) * (e[2][1] - mb[1]) for e in pairs)
        sxy = sum((e[1][0] - ma[0]) * (e[2][1] - mb[1]) - (e[1][1] - ma[1]) * (e[2][0] - mb[0]) for e in pairs)
        th = math.atan2(sxy, sxx)
        c, s_ = math.cos(th), math.sin(th)
        tx = mb[0] - (cx + c * (ma[0] - cx) - s_ * (ma[1] - cy))
        ty = mb[1] - (cy + s_ * (ma[0] - cx) + c * (ma[1] - cy))
    rms = (sum(e[0] for e in pairs) / len(pairs)) ** 0.5
    return math.degrees(th), tx, ty, rms, len(pairs) / len(src)


def cmd_floors_icp(a):
    floors = [parse_floor(s) for s in a.floor]
    names = [f"f{-i}" for i in range(len(floors))]
    pts = {n: [] for n in names}
    for gx, gy, gz, t in points(a.result, a.stride):
        if a.box and not (a.box[0] < gx < a.box[1] and a.box[2] < gy < a.box[3]):
            continue
        if a.radius and not a.radius_min < math.hypot(gx - a.centre[0], gy - a.centre[1]) < a.radius:
            continue
        for n, (lv, ts) in zip(names, floors):
            if a.band[0] < gz - lv < a.band[1] and in_spans(t, ts):
                pts[n].append((gx, gy))
    pairs = [(names[i], names[i + 1]) for i in range(len(names) - 1)]
    if len(names) > 2:
        pairs.append((names[0], names[-1]))
    name = os.path.basename(a.result.rstrip("/" + chr(92)))
    for up, low in pairs:
        shared = dense_cells(pts[up]) & dense_cells(pts[low])
        dst = voxel([p for p in pts[up] if cell(p) in shared], 0.03)
        src = voxel([p for p in pts[low] if cell(p) in shared], 0.05)
        if not src or not dst:
            print(f"{name} {low} onto {up} ({len(shared)} cells): no shared points")
            continue
        src = src[::max(1, len(src) // 6000)]
        out = []
        for th0 in (-6, 0, 6):
            th, tx, ty, rms, frac = icp(src, dst, math.radians(th0))
            out.append(f"from {th0:+d}: {th:+.2f} deg, shift {tx:+.2f} {ty:+.2f} m, rms {rms * 100:.1f} cm, {frac * 100:.0f}% matched")
        print(f"{name} {low} onto {up} ({len(shared)} cells): " + " | ".join(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("slabs", help="tilt of each floor slab")
    s.add_argument("result")
    s.add_argument("--levels", type=float, nargs="+", default=[-1.1, -4.0, -6.8], help="slab heights (world z) [m]")
    s.add_argument("--stride", type=int, default=4, help="use every n-th point")
    s.set_defaults(fn=cmd_slabs)
    for cmd, fn, band, h in (("lift", cmd_lift, (0.3, 2.3), "line fit of the lift shaft's walls per floor"),
                             ("floors-icp", cmd_floors_icp, (0.8, 1.8), "2-D ICP between floors")):
        s = sub.add_parser(cmd, help=h)
        s.add_argument("result")
        s.add_argument("--floor", action="append", help="SLAB_Z:T0-T1[,T0-T1], repeated (write --floor=-1.1:...); default the floors of 44")
        s.add_argument("--band", type=float, nargs=2, default=band, help="wall band above the slab [m]")
        s.set_defaults(fn=fn)
        if cmd == "lift":
            s.add_argument("--wall", action="append", help="name:x0,x1,y0,y1 wall box, repeated; default the lift shaft of 44")
        else:
            s.add_argument("--stride", type=int, default=2, help="use every n-th point")
            s.add_argument("--box", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"), help="only points inside this box")
            s.add_argument("--radius", type=float, help="only points between --radius-min and this distance from --centre")
            s.add_argument("--radius-min", type=float, default=2.2)
            s.add_argument("--centre", type=float, nargs=2, default=(-4.7, -4.9), metavar=("X", "Y"))
    a = ap.parse_args()
    if hasattr(a, "floor"):
        a.floor = a.floor or DEFAULT_FLOORS
    if hasattr(a, "wall"):
        a.wall = a.wall or DEFAULT_WALLS
    a.fn(a)


if __name__ == "__main__":
    main()
