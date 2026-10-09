"""LiDAR-IMU time offset and accelerometer agreement of a dMapper result (pure Python, no numpy).

Usage:
  python tools/imu_time_offset.py <dataset_folder> <result_folder> [--shift-ms S] [--from T0] [--to T1] [--no-prediction]

<dataset_folder> holds the imu*.csv of the run, <result_folder> the result (trajectory_lio_*.csv and session_poses.mrp;
the trajectory files are chunk-relative and are placed with the chunk poses). The accelerometer is rotated into the world
with the trajectory's rotations (one node per IMU sample); gravity comes from the whole recording.

Reported:
  1. prediction   from p(t0), v(t0) of the trajectory, the error of predicting p(t0 + h) by constant velocity and by
                  integrating the accelerometer (plain, and with an offset calibrated over the 10 s before), h = 0.1 .. 5 s
  2. coherence    per frequency band and axis: coherence and gain between the trajectory's acceleration and the
                  accelerometer, and both rms values (100 Hz bins, Welch, 5.12 s segments); a time-domain cross-correlation
  3. delay        the trajectory's delay behind the accelerometer from the cross-spectrum phase slope (0.5-4 Hz,
                  coherence > 0.5). Negative = the IMU timestamps are late against the LiDAR's.

--shift-ms S moves the accelerometer in time (a(t) := a(t + S)) to test an offset; --from/--to restrict the analysis to a
time range [s from the first node] (for drift: run it per segment). Found with it (2026-10-09): the dLidar recordings of
the Unitree L2 had the IMU 50-80 ms behind the LiDAR (recording 44; tools/restamp_imu.py corrects a recording).
"""
import cmath
import glob
import math
import os
import sys

G = 9.80665


def load_chunk_poses(folder):
    """session_poses.mrp: per chunk file name a 4x4 pose (world from chunk)"""
    poses = {}
    with open(os.path.join(folder, "session_poses.mrp")) as f:
        lines = [l.strip() for l in f if l.strip()]
    i = 0
    while i < len(lines):
        if lines[i].endswith(".laz"):
            name = lines[i]
            m = [[float(x) for x in lines[i + r].split()] for r in range(1, 5)]
            poses[name] = m
            i += 5
        else:
            i += 1
    return poses


def load_traj(folder):
    chunks = load_chunk_poses(folder)
    rows = []
    for fn in glob.glob(os.path.join(folder, "trajectory_lio_*.csv")):
        idx = os.path.basename(fn)[len("trajectory_lio_"):-len(".csv")]
        M = chunks[f"scan_lio_{idx}.laz"]
        Rc = [M[r][:3] for r in range(3)]
        tc = [M[r][3] for r in range(3)]
        with open(fn) as f:
            f.readline()
            for line in f:
                v = line.split()
                if len(v) < 14:
                    continue
                Rl = ((float(v[1]), float(v[2]), float(v[3])), (float(v[5]), float(v[6]), float(v[7])), (float(v[9]), float(v[10]), float(v[11])))
                pl = (float(v[4]), float(v[8]), float(v[12]))
                R = tuple(tuple(sum(Rc[r][k] * Rl[k][c] for k in range(3)) for c in range(3)) for r in range(3))
                pos = tuple(sum(Rc[r][k] * pl[k] for k in range(3)) + tc[r] for r in range(3))
                rows.append((int(round(float(v[0]))), pos, R))
    rows.sort()
    return rows


def load_imu(folder):
    out = {}
    for fn in sorted(glob.glob(os.path.join(folder, "imu*.csv"))):
        with open(fn) as f:
            f.readline()
            for line in f:
                v = line.split()
                if len(v) < 9:
                    continue
                out[int(v[7])] = (float(v[3]), float(v[4]), float(v[5]))
    return out


def nearest(keys, x, tol):
    import bisect
    i = bisect.bisect_left(keys, x)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(keys) and abs(keys[j] - x) <= tol and (best is None or abs(keys[j] - x) < abs(best - x)):
            best = keys[j]
    return best


def matvec(R, x):
    return (R[0][0] * x[0] + R[0][1] * x[1] + R[0][2] * x[2], R[1][0] * x[0] + R[1][1] * x[1] + R[1][2] * x[2], R[2][0] * x[0] + R[2][1] * x[1] + R[2][2] * x[2])


def fft(a):
    n = len(a)
    a = list(a)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    length = 2
    while length <= n:
        w = cmath.exp(-2j * math.pi / length)
        for i in range(0, n, length):
            wk = 1.0
            for k in range(length // 2):
                u, t = a[i + k], a[i + k + length // 2] * wk
                a[i + k], a[i + k + length // 2] = u + t, u - t
                wk *= w
        length <<= 1
    return a


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("result")
    ap.add_argument("--shift-ms", type=float, default=0.0)
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--no-prediction", action="store_true")
    args = ap.parse_args()
    dataset, result = args.dataset, args.result
    traj = load_traj(result)
    imu = load_imu(dataset)
    t, p, fw = [], [], []
    missing = 0
    keys = sorted(imu)
    for unix, pos, R in traj:
        kk = nearest(keys, unix, 100000)
        f = imu[kk] if kk is not None else None
        if f is None:
            missing += 1
            continue
        t.append(unix * 1e-9)
        p.append(pos)
        fw.append(matvec(R, f))
    t0 = t[0]
    t = [x - t0 for x in t]
    t_from = args.t_from if args.t_from is not None else -1e9
    t_to = args.t_to if args.t_to is not None else 1e9
    if args.t_from is not None or args.t_to is not None:
        keep = [k for k in range(len(t)) if t_from <= t[k] <= t_to]
        t = [t[k] - t[keep[0]] for k in keep]; p = [p[k] for k in keep]; fw = [fw[k] for k in keep]
        print(f"segment {t_from:.0f} .. {t_to:.0f} s of the recording")
    n = len(t)
    print(f"nodes {len(traj)}, matched to IMU samples {n}, unmatched {missing}, duration {t[-1]:.1f} s")
    gvec = tuple(sum(x[k] for x in fw) / n for k in range(3))
    gn = math.sqrt(sum(c * c for c in gvec))
    print(f"mean world specific force {gvec[0]:.4f} {gvec[1]:.4f} {gvec[2]:.4f} g (|g| {gn:.4f})")
    s = G / gn
    a = [tuple((x[k] - gvec[k]) * s for k in range(3)) for x in fw]
    shift = args.shift_ms / 1000.0
    if shift != 0.0:
        import bisect as _b
        a = [a[min(max(_b.bisect_left(t, tt + shift), 0), n - 1)] for tt in t]
        print(f"accelerometer shifted: a(t) := a(t {shift * 1000:+.1f} ms)")

    # cumulative integrals of a (trapezoid) for fast window integrals: V[k] = int_0^t_k a, P[k] = int_0^t_k V
    V = [(0.0, 0.0, 0.0)]
    P = [(0.0, 0.0, 0.0)]
    for k in range(1, n):
        dt = t[k] - t[k - 1]
        v = tuple(V[-1][c] + 0.5 * (a[k][c] + a[k - 1][c]) * dt for c in range(3))
        P.append(tuple(P[-1][c] + 0.5 * (V[-1][c] + v[c]) * dt for c in range(3)))
        V.append(v)

    import bisect

    def idx(time):
        return min(max(bisect.bisect_left(t, time), 0), n - 1)

    def vel(time, half=0.05):
        i0, i1 = idx(time - half), idx(time + half)
        dt = max(t[i1] - t[i0], 1e-6)
        return tuple((p[i1][c] - p[i0][c]) / dt for c in range(3))

    # 1. prediction
    horizons = [0.1, 0.2, 0.4, 1.0, 2.0, 5.0]
    errs = {h: {"cv": [], "imu": [], "imu_cal": [], "cv_z": [], "imu_z": [], "imu_cal_z": []} for h in horizons}
    start = 12.0
    while start + 5.0 < t[-1] - 0.1 and not args.no_prediction:
        i = idx(start)
        v0 = vel(start)
        # offset over the 10 s before: (velocity change of the IMU - of the trajectory) / 10 s
        ta, tb = start - 10.0, start
        ia, ib = idx(ta), idx(tb)
        va, vb = vel(ta), vel(tb)
        off = tuple(((V[ib][c] - V[ia][c]) - (vb[c] - va[c])) / (t[ib] - t[ia]) for c in range(3))
        for h in horizons:
            j = idx(start + h)
            T = t[j] - t[i]
            disp_imu = tuple(P[j][c] - P[i][c] - V[i][c] * T for c in range(3))
            truth = tuple(p[j][c] - p[i][c] for c in range(3))
            cv = tuple(v0[c] * T for c in range(3))
            im = tuple(cv[c] + disp_imu[c] for c in range(3))
            ic = tuple(im[c] - 0.5 * off[c] * T * T for c in range(3))
            for name, pred in (("cv", cv), ("imu", im), ("imu_cal", ic)):
                d = [pred[c] - truth[c] for c in range(3)]
                errs[h][name].append(math.sqrt(d[0] ** 2 + d[1] ** 2))
                errs[h][name + "_z"].append(abs(d[2]))
        start += 0.5

    def q(xs, f):
        xs = sorted(xs)
        return xs[min(len(xs) - 1, int(f * len(xs)))]

    if errs[1.0]['cv']:
        _print_pred = True
    else:
        _print_pred = False
    if _print_pred:
        print(f"\n1. prediction error against the trajectory, {len(errs[1.0]['cv'])} starts every 0.5 s (median / p90) [m]")
        print("   horizon | horizontal: const. velocity | IMU | IMU, offset from prev. 10 s || vertical: const. velocity | IMU | IMU calibrated")
        for h in horizons:
            e = errs[h]
            print(f"   {h:4.1f} s  | {q(e['cv'],.5):.4f}/{q(e['cv'],.9):.4f} | {q(e['imu'],.5):.4f}/{q(e['imu'],.9):.4f} | {q(e['imu_cal'],.5):.4f}/{q(e['imu_cal'],.9):.4f}"
                  f" || {q(e['cv_z'],.5):.4f}/{q(e['cv_z'],.9):.4f} | {q(e['imu_z'],.5):.4f}/{q(e['imu_z'],.9):.4f} | {q(e['imu_cal_z'],.5):.4f}/{q(e['imu_cal_z'],.9):.4f}")

    # 2. spectra at 100 Hz: bin means of position and acceleration
    fs = 100.0
    nb = int(t[-1] * fs)
    pos_b = [[0.0] * 3 for _ in range(nb)]
    acc_b = [[0.0] * 3 for _ in range(nb)]
    cnt = [0] * nb
    for k in range(n):
        b = int(t[k] * fs)
        if b >= nb:
            continue
        cnt[b] += 1
        for c in range(3):
            pos_b[b][c] += p[k][c]
            acc_b[b][c] += a[k][c]
    for b in range(nb):
        if cnt[b]:
            for c in range(3):
                pos_b[b][c] /= cnt[b]
                acc_b[b][c] /= cnt[b]
        elif b > 0:
            pos_b[b] = list(pos_b[b - 1])
            acc_b[b] = list(acc_b[b - 1])
    empty = sum(1 for c in cnt if c == 0)
    # trajectory acceleration by second difference of bin means
    atr = [[0.0] * 3 for _ in range(nb)]
    for b in range(1, nb - 1):
        for c in range(3):
            atr[b][c] = (pos_b[b + 1][c] - 2 * pos_b[b][c] + pos_b[b - 1][c]) * fs * fs
    seg = 512
    hann = [0.5 - 0.5 * math.cos(2 * math.pi * k / (seg - 1)) for k in range(seg)]
    freqs = [k * fs / seg for k in range(seg // 2)]
    bands = [(0.0, 0.3), (0.3, 0.7), (0.7, 1.2), (1.2, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 8.0), (8.0, 15.0), (15.0, 30.0)]
    print(f"\n2. coherence / gain (trajectory acceleration vs accelerometer) per band, 100 Hz bins ({empty} empty), Welch {seg} samples")
    print("   band [Hz]    | " + " | ".join(f"{ax}: coh  gain  |traj| |imu| [m/s^2]" for ax in "xyz"))
    res_axes = []
    for c in range(3):
        Sxx = [0.0] * (seg // 2)
        Syy = [0.0] * (seg // 2)
        Sxy = [0j] * (seg // 2)
        nseg = 0
        for st in range(2, nb - seg - 2, seg // 2):
            x = [atr[st + k][c] for k in range(seg)]
            y = [acc_b[st + k][c] for k in range(seg)]
            mx, my = sum(x) / seg, sum(y) / seg
            X = fft([(x[k] - mx) * hann[k] for k in range(seg)])
            Y = fft([(y[k] - my) * hann[k] for k in range(seg)])
            for k in range(seg // 2):
                Sxx[k] += abs(X[k]) ** 2
                Syy[k] += abs(Y[k]) ** 2
                Sxy[k] += X[k] * Y[k].conjugate()
            nseg += 1
        res_axes.append((Sxx, Syy, Sxy, nseg))
    norm = sum(h * h for h in hann) * fs
    for lo, hi in bands:
        cells = []
        for c in range(3):
            Sxx, Syy, Sxy, nseg = res_axes[c]
            ks = [k for k in range(1, seg // 2) if lo <= freqs[k] < hi]
            sxx = sum(Sxx[k] for k in ks)
            syy = sum(Syy[k] for k in ks)
            sxy = sum(Sxy[k] for k in ks)
            coh = abs(sxy) ** 2 / (sxx * syy) if sxx * syy > 0 else 0.0
            gain = abs(sxy) / syy if syy > 0 else 0.0  # trajectory per IMU
            # rms in the band: sum of the one-sided PSD * df
            df = fs / seg
            rx = math.sqrt(2 * sxx / nseg / norm * df) if nseg else 0.0
            ry = math.sqrt(2 * syy / nseg / norm * df) if nseg else 0.0
            cells.append(f"{coh:5.2f} {gain:5.2f} {rx:6.3f} {ry:6.3f}")
        print(f"   {lo:4.1f}-{hi:4.1f}     | " + " | ".join(cells))
    # time-domain cross-correlation, band-passed by a difference of moving averages (about 0.7 to 8 Hz)
    def band(x, c):
        def ma(w):
            out, acc = [], 0.0
            q = []
            for b in range(len(x)):
                q.append(x[b][c]); acc += x[b][c]
                if len(q) > w:
                    acc -= q.pop(0)
                out.append(acc / len(q))
            return out
        short, long_ = ma(25), ma(200)
        return [short[b] - long_[b] for b in range(len(x))]
    print()
    print("   time-domain cross-correlation (about 0.25-2 Hz), peak lag of the accelerometer behind the trajectory:")
    for c in range(3):
        xs, ys = band(atr, c), band(acc_b, c)
        best = None
        vals = {}
        for lag in range(-30, 31):
            num = sxx = syy = 0.0
            for b in range(100, nb - 100):
                xv, yv = xs[b], ys[b + lag]
                num += xv * yv; sxx += xv * xv; syy += yv * yv
            r = num / math.sqrt(sxx * syy)
            vals[lag] = r
            if best is None or r > vals[best]:
                best = lag
        l0 = best
        if -30 < l0 < 30:
            ym, y0, yp = vals[l0 - 1], vals[l0], vals[l0 + 1]
            frac = 0.5 * (ym - yp) / (ym - 2 * y0 + yp) if (ym - 2 * y0 + yp) != 0 else 0.0
        else:
            frac = 0.0
        print(f"   {'xyz'[c]}: peak at {(l0 + frac) * 10:+.1f} ms, r = {vals[l0]:.3f} (r at 0 ms: {vals[0]:.3f})")

    # 3. delay from the cross-spectrum phase slope where coherence is high (0.5-4 Hz)
    print("\n3. delay of the trajectory behind the accelerometer (cross-spectrum phase slope, 0.5-4 Hz, coherence > 0.5):")
    for c in range(3):
        Sxx, Syy, Sxy, nseg = res_axes[c]
        num = den = 0.0
        used = 0
        for k in range(1, seg // 2):
            f = freqs[k]
            if not 0.5 <= f <= 4.0:
                continue
            coh = abs(Sxy[k]) ** 2 / (Sxx[k] * Syy[k]) if Sxx[k] * Syy[k] > 0 else 0
            if coh < 0.5:
                continue
            w = 2 * math.pi * f
            ph = cmath.phase(Sxy[k])  # phase of X Y*: x lags y by tau -> phase -w tau
            num += w * ph * coh
            den += w * w * coh
            used += 1
        tau = -num / den if den else float("nan")
        print(f"   {'xyz'[c]}: {tau * 1000:+.1f} ms ({used} frequency bins)")


main()
