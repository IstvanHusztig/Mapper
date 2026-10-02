"""Quality metrics for dMapper results (pure Python, no numpy).

Usage:
  python tools/eval_odometry.py <dataset_folder> <result_folder> [<result_folder> ...]
                                [--zmin -0.3] [--zmax 0.7] [--tsplit 10.5] [--png out.png]

<dataset_folder> holds the imu*.csv used for the run. Each <result_folder> holds
trajectory_lio_0.csv and, optionally, scan_lio_0.asc (an ASCII export of
scan_lio_0.laz with columns "x y z intensity gps_time", see tools/README.md).

Reported per result:
  jitter      node-to-node |yaw change| median / p99 [deg]
  win diff    per-window |LiDAR yaw change - gyro yaw change| mean and count > 3 deg
              (gyro corrected by the per-window bias from the results JSON if present)
  end spikes  |yaw(last node) - median yaw(nodes 30..60 before it)| median and count > 3 deg
  final yaw   total LiDAR yaw vs total gyro yaw [deg]
  cells       occupied 3/5/10 cm top-view cells of the Z section (lower = sharper)
"""
import argparse
import bisect
import glob
import json
import math
import os
import statistics as st
import struct
import zlib

D = 180.0 / math.pi
WINDOW = 360


def load_imu(dataset):
    files = sorted(glob.glob(os.path.join(dataset, "imu*.csv")))
    if not files:
        raise SystemExit(f"no imu*.csv in {dataset}")
    rows = []
    for fn in files:
        with open(fn) as f:
            header = f.readline().split()
            ti, gi = header.index("timestamp"), header.index("gyroZ")
            rows += [(int(float(l.split()[ti])) / 1e9, float(l.split()[gi])) for l in f if l.strip()]
    rows.sort()
    return [r[0] for r in rows], [r[1] for r in rows]


def load_trajectory(result):
    T, Y = [], []
    with open(os.path.join(result, "trajectory_lio_0.csv")) as f:
        f.readline()
        for n, l in enumerate(f):
            v = l.split()
            if any("nan" in x.lower() or "inf" in x.lower() for x in v[1:13]):
                raise SystemExit(f"{result}: trajectory node {n} is not finite ({v[1]} ...), the run diverged")
            T.append(float(v[0]) / 1e9)
            Y.append(math.atan2(float(v[5]), float(v[1])))
    for i in range(1, len(Y)):
        while Y[i] - Y[i - 1] > math.pi:
            Y[i] -= 2 * math.pi
        while Y[i] - Y[i - 1] < -math.pi:
            Y[i] += 2 * math.pi
    return T, Y


def load_bias_track(result):
    """[(t_start_s, bias_z_rad_s)] from the processing-results JSON, or []."""
    for fn in glob.glob(os.path.join(result, "HDMapping_results_*.json")):
        with open(fn) as f:
            track = json.load(f).get("gyro_bias_per_window", [])
        return [(e["t_start_s"], e["bias_deg_s"][2] / D) for e in track]
    return []


def gyro_yaw(it, gz, a, b, bias_track=()):
    i, j = bisect.bisect_left(it, a), bisect.bisect_left(it, b)
    s = 0.0
    for k in range(i + 1, j):
        bz = 0.0
        if bias_track:
            w = bisect.bisect_right([e[0] for e in bias_track], it[k]) - 1
            bz = bias_track[max(w, 0)][1]
        s += (gz[k] - bz) * (it[k] - it[k - 1])
    return s


def trajectory_metrics(it, gz, T, Y, bias_track, window=WINDOW):
    steps = sorted(abs(Y[i + 1] - Y[i]) * D for i in range(len(Y) - 1))
    wdiff, ends = [], []
    for w in range(len(Y) // window):
        a, e = w * window, w * window + window - 1
        wdiff.append(abs((Y[e] - Y[a]) * D - gyro_yaw(it, gz, T[a], T[e], bias_track) * D))
        ends.append(abs((Y[e] - st.median(Y[e - 60:e - 30])) * D))
    return {
        "jitter_median": st.median(steps),
        "jitter_p99": steps[int(0.99 * len(steps))],
        "win_diff_mean": st.mean(wdiff),
        "win_diff_gt3": sum(x > 3 for x in wdiff),
        "end_spike_median": st.median(ends),
        "end_spike_gt3": sum(x > 3 for x in ends),
        "final_yaw": (Y[-1] - Y[0]) * D,
        "final_gyro_yaw": gyro_yaw(it, gz, T[0], T[-1], bias_track) * D,
    }


def load_section(result, zmin, zmax):
    fn = os.path.join(result, "scan_lio_0.asc")
    if not os.path.exists(fn):
        return None
    P = []
    with open(fn) as f:
        for l in f:
            v = l.split()
            z = float(v[2])
            if zmin <= z <= zmax:
                P.append((float(v[0]), float(v[1]), float(v[4]) / 1e9))
    return P


def cell_metrics(P):
    out = {"points": len(P)}
    for g in (0.03, 0.05, 0.10):
        out[f"cells_{int(g * 100)}cm"] = len({(math.floor(x / g), math.floor(y / g)) for x, y, _ in P})
    return out


def write_png(path, sections, tsplit, res=0.02):
    """Top views side by side: red = before tsplit, cyan = after."""
    allp = [p for P in sections for p in P]
    xs, ys = sorted(p[0] for p in allp), sorted(p[1] for p in allp)
    x0, x1 = xs[len(xs) // 500], xs[-len(xs) // 500]
    y0, y1 = ys[len(ys) // 500], ys[-len(ys) // 500]
    W, H = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    grids = []
    for P in sections:
        A = [[0] * W for _ in range(H)]
        B = [[0] * W for _ in range(H)]
        for x, y, t in P:
            c, r = int((x - x0) / res), int((y1 - y) / res)
            if 0 <= c < W and 0 <= r < H:
                (A if t < tsplit else B)[r][c] += 1
        grids.append((A, B))

    def v(k):
        return min(255, int(255 * math.sqrt(min(k, 6) / 6)))

    rows = []
    for r in range(H):
        row = []
        for n, (A, B) in enumerate(grids):
            for c in range(W):
                a, b = v(A[r][c]), v(B[r][c])
                row += [a, b, b]
            if n != len(grids) - 1:
                row += [40, 40, 40] * 20
        rows.append(row)

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", len(rows[0]) // 3, len(rows), 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("results", nargs="+")
    ap.add_argument("--zmin", type=float, default=-0.3)
    ap.add_argument("--zmax", type=float, default=0.7)
    ap.add_argument("--tsplit", type=float, default=10.5, help="PNG colour split time [s]")
    ap.add_argument("--png", help="write a side-by-side top-view PNG of all results with an ASCII export")
    ap.add_argument("--window", type=int, default=WINDOW, help="nodes per optimization window (threshold_nr_poses)")
    ap.add_argument("--json", action="store_true", help="print metrics as JSON")
    args = ap.parse_args()

    it, gz = load_imu(args.dataset)
    report, sections = {}, []
    for result in args.results:
        T, Y = load_trajectory(result)
        m = trajectory_metrics(it, gz, T, Y, load_bias_track(result), args.window)
        P = load_section(result, args.zmin, args.zmax)
        if P is not None:
            m.update(cell_metrics(P))
            sections.append(P)
        report[os.path.basename(os.path.normpath(result))] = m

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for name, m in report.items():
            line = (f"{name:32s} jitter {m['jitter_median']:.3f}/{m['jitter_p99']:.2f} | "
                    f"win diff {m['win_diff_mean']:.2f} (>3: {m['win_diff_gt3']}) | "
                    f"end spikes {m['end_spike_median']:.2f} (>3: {m['end_spike_gt3']}) | "
                    f"final yaw {m['final_yaw']:.1f} vs gyro {m['final_gyro_yaw']:.1f}")
            if "cells_3cm" in m:
                line += f" | cells 3/5/10cm {m['cells_3cm']}/{m['cells_5cm']}/{m['cells_10cm']}"
            print(line)

    if args.png and sections:
        write_png(args.png, sections, args.tsplit)
        print(f"wrote {args.png}")


if __name__ == "__main__":
    main()
