"""Per-pass offsets around one point of a result: is a structure drawn twice, and by which passes?

The result's points within <radius> of (x, y, z) are split into passes (a gap of more than 2 s starts a new one). Each
pass is compared with the largest pass: median distance of its points to their nearest neighbour in that pass, the
share farther than 5 cm, and the median offset vector. Passes within a few cm of each other draw the same surface; a
pass 10+ cm off draws a second copy (a ghost railing or wall), and its time says which stretch of the trajectory is off.

Usage: spot_check.py <result_folder> x y z [radius_m=0.8]
The chunk LAZ files are exported to ASCII with CloudCompare once (scan_lio_<n>.asc next to them) and placed with the
chunk poses of session_poses.mrp, so the coordinates are those of scan_lio_merged.laz (pick them there in CloudCompare).
"""
import os
import subprocess
import sys

CC = r"C:\Program Files\CloudCompare\CloudCompare.exe"
PASS_GAP_S = 2.0
NN_CELL = 0.05


def chunk_poses(folder):
    lines = open(os.path.join(folder, "session_poses.mrp")).read().split("\n")
    out, i = {}, 1
    while i + 4 < len(lines) and lines[i].strip():
        out[lines[i].strip()] = [[float(x) for x in lines[i + 1 + r].split()] for r in range(4)]
        i += 5
    return out


def load_near(folder, c, radius):
    pts = []
    r2 = radius * radius
    for laz, M in chunk_poses(folder).items():
        asc = os.path.join(folder, laz.replace(".laz", ".asc"))
        if not os.path.exists(asc):
            subprocess.run([CC, "-SILENT", "-AUTO_SAVE", "OFF", "-O", os.path.join(folder, laz), "-C_EXPORT_FMT", "ASC",
                            "-SEP", "SPACE", "-PREC", "4", "-SAVE_CLOUDS", "FILE", '"' + asc + '"'], check=True, capture_output=True)
        with open(asc) as f:
            for line in f:
                v = line.split()
                if len(v) < 5:
                    continue
                x, y, z = float(v[0]), float(v[1]), float(v[2])
                g = (M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
                     M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
                     M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3])
                if (g[0] - c[0]) ** 2 + (g[1] - c[1]) ** 2 + (g[2] - c[2]) ** 2 < r2:
                    pts.append((float(v[4]) * 1e-9,) + g)
    pts.sort()
    return pts


def split_passes(pts):
    passes, cur = [], []
    for p in pts:
        if cur and p[0] - cur[-1][0] > PASS_GAP_S:
            passes.append(cur)
            cur = []
        cur.append(p)
    if cur:
        passes.append(cur)
    return passes


def key(p):
    return int(p[1] // NN_CELL), int(p[2] // NN_CELL), int(p[3] // NN_CELL)


def nearest(grid, q):
    best, bd = None, 1e18
    k = key(q)
    for r in range(1, 6):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    for p in grid.get((k[0] + dx, k[1] + dy, k[2] + dz), ()):
                        d = (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 + (p[3] - q[3]) ** 2
                        if d < bd:
                            best, bd = p, d
        if best is not None and bd ** 0.5 < r * NN_CELL:
            break
    return best, bd ** 0.5


def median(v):
    v = sorted(v)
    return v[len(v) // 2]


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    folder = sys.argv[1]
    c = tuple(float(a) for a in sys.argv[2:5])
    radius = float(sys.argv[5]) if len(sys.argv) > 5 else 0.8
    pts = load_near(folder, c, radius)  # times as stored in the LAZ files (seconds of the recording)
    passes = split_passes(pts)
    print(f"{os.path.basename(folder.rstrip('/' + chr(92)))}: {len(pts)} points within {radius} m of {c}, {len(passes)} passes")
    if not passes:
        return
    ref = max(passes, key=len)
    grid = {}
    for p in ref:
        grid.setdefault(key(p), []).append(p)
    print(f"reference: the largest pass, t {ref[0][0]:.1f}..{ref[-1][0]:.1f} s, {len(ref)} points")
    print("      t [s]        points  median NN [cm]  share > 5 cm  median offset x y z [cm]")
    for ps in passes:
        if ps is ref:
            continue
        if len(ps) < 30:
            print(f"{ps[0][0]:7.1f}..{ps[-1][0]:7.1f}  {len(ps):6d}  (too few points)")
            continue
        d, off = [], ([], [], [])
        for q in ps:
            p, dist = nearest(grid, q)
            if p is None:
                continue
            d.append(dist)
            for k in range(3):
                off[k].append(q[k + 1] - p[k + 1])
        if not d:
            print(f"{ps[0][0]:7.1f}..{ps[-1][0]:7.1f}  {len(ps):6d}  (no neighbour within {5 * NN_CELL} m)")
            continue
        far = sum(x > 0.05 for x in d) / len(d)
        print(f"{ps[0][0]:7.1f}..{ps[-1][0]:7.1f}  {len(ps):6d}  {100 * median(d):13.1f}  {far:12.2f}  "
              f"{100 * median(off[0]):6.1f} {100 * median(off[1]):6.1f} {100 * median(off[2]):6.1f}")


if __name__ == "__main__":
    main()
