"""Straightness of a long façade along a gallery: where the façade and the gallery's parapet lie, slice by slice.

A straight façade with a parapet at a known distance (from the building plan) must appear at one position across its
length. The points seen in a time window (the pass along the gallery) are turned into the room frame (--angle, the
direction of the building's walls in the map), and in each 1 m slice along the gallery the two surfaces are found as the
pair of histogram peaks across it whose spacing matches the plan (--spacing). The slice's façade position is compared
with the reference (--facade, e.g. the façade's outer face seen through the windows at the start).

Usage: facade_check.py <result_folder> --facade 2.15 --spacing 1.135 --window 35 95 [--angle 10.55] [--across 1.5 5.5]
Coordinates: u across the gallery, v along it (room frame). The chunk LAZ files are exported to ASCII once (see
spot_check.py).
"""
import argparse
import math
import os

import spot_check


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("result")
    ap.add_argument("--facade", type=float, required=True, help="reference position of the façade across the gallery [m]")
    ap.add_argument("--spacing", type=float, required=True, help="façade to parapet, from the plan [m]")
    ap.add_argument("--window", type=float, nargs=2, required=True, help="time window of the pass along the gallery [s]")
    ap.add_argument("--angle", type=float, default=10.55, help="direction of the building's walls in the map [deg]")
    ap.add_argument("--across", type=float, nargs=2, default=(1.5, 5.5), help="band across the gallery searched [m]")
    ap.add_argument("--height", type=float, nargs=2, default=(-0.6, 1.0), help="height band (world z) [m]")
    ap.add_argument("--tolerance", type=float, default=0.10, help="allowed deviation of the pair spacing from the plan [m]")
    a = ap.parse_args()

    th = math.radians(a.angle)
    c, s = math.cos(th), math.sin(th)
    slices = {}
    for laz, M in spot_check.chunk_poses(a.result).items():
        asc = os.path.join(a.result, laz.replace(".laz", ".asc"))
        if not os.path.exists(asc):
            spot_check.load_near(a.result, (0.0, 0.0, 0.0), 0.01)  # exports every chunk once
        with open(asc) as f:
            for line in f:
                v = line.split()
                if len(v) < 5:
                    continue
                t = float(v[4]) * 1e-9
                if not a.window[0] <= t < a.window[1]:
                    continue
                x, y, z = float(v[0]), float(v[1]), float(v[2])
                g = [M[r][0] * x + M[r][1] * y + M[r][2] * z + M[r][3] for r in range(3)]
                if not a.height[0] < g[2] < a.height[1]:
                    continue
                u, w = c * g[0] + s * g[1], -s * g[0] + c * g[1]
                if a.across[0] < u < a.across[1]:
                    slices.setdefault(math.floor(w), []).append(u)

    name = os.path.basename(a.result.rstrip("/" + chr(92)))
    print(f"{name}: façade reference {a.facade:.2f} m, parapet {a.spacing:.3f} m beyond, pass {a.window[0]:.0f}-{a.window[1]:.0f} s")
    devs = []
    for k in sorted(slices):
        q = slices[k]
        if len(q) < 200:
            continue
        h = {}
        for u in q:
            h[int(u // 0.02)] = h.get(int(u // 0.02), 0) + 1
        peaks = [(b * 0.02 + 0.01, n) for b, n in h.items()
                 if n >= max(h.get(b - 1, 0), h.get(b + 1, 0)) and n > 0.03 * len(q)]
        pairs = [(n1 + n2, u1) for u1, n1 in peaks for u2, n2 in peaks if abs((u2 - u1) - a.spacing) < a.tolerance]
        if pairs:
            facade = max(pairs)[1]
            devs.append(facade - a.facade)
            print(f"  v {k:+3d}..{k + 1:+3d} m: façade {facade:+.2f} (parapet found)  deviation {100 * (facade - a.facade):+6.1f} cm")
        else:
            best = max(peaks, key=lambda p: p[1]) if peaks else None
            print(f"  v {k:+3d}..{k + 1:+3d} m: no façade/parapet pair" + (f" (strongest surface at {best[0]:+.2f})" if best else ""))
    if devs:
        print(f"  slices with the pair: {len(devs)}, deviation from the reference: mean {100 * sum(devs) / len(devs):+.1f} cm, "
              f"largest {100 * max(devs, key=abs):+.1f} cm, spread {100 * (max(devs) - min(devs)):.1f} cm")


if __name__ == "__main__":
    main()
