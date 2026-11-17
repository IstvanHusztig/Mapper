"""Is a recording's IMU data usable? Checks the raw imu*.csv of a recording folder, independent of dMapper.

  rate, gaps     sample rate, steps over 2.5 x the median, non-increasing timestamps (dMapper needs strictly increasing)
  rest at start  how long the sensor is still at the start (the start-up gyro bias needs about 1 s)
  gravity        |a| at rest, in g (a unit or scale problem shows here)
  bias           gyro mean in the first and in the last rest, and the change between them
  false offsets  stretches where the gyro's and the accelerometer's noise are at their rest level, but the gyro mean
                 is far from the start bias: a hand never holds a rate that steady, so the sensor reports a turn that
                 does not happen (found on a faulty Unitree L2: 11-32 deg/s for 2.5-4 s after turns)
  noise          gyro angle random walk and accelerometer noise density from the longest rest, from single samples and
                 from 0.1 s means; they agree only for white noise, so a handheld rest gives upper bounds
  peaks          largest rotation rate and acceleration

"Quiet" blocks are 1 s long, half overlapping; a block is quiet when every gyro axis, |a| and every accelerometer axis
have a std below `--rest-factor` x the recording's own floor (the 3rd percentile of the block stds). A recording with no
rest at all has no meaningful floor: the tool says so and the false-offset check is then only a hint.

Columns as dLidar writes them: gyro x y z [rad/s], accelerometer x y z [g], id, timestamp [ns], unix timestamp [ns].
Exit code 1 if a check fails.

usage: imu_health.py <recording folder | imu.csv> [--rest-factor 2.0] [--offset-deg-s 3.0] [--min-offset-s 2.0]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

BLOCK_S = 1.0


def load(path):
    files = sorted(glob.glob(os.path.join(path, "imu*.csv"))) if os.path.isdir(path) else [path]
    if not files:
        sys.exit(f"no imu*.csv in {path}")
    d = np.vstack([np.loadtxt(f, skiprows=1, usecols=(0, 1, 2, 3, 4, 5, 7), ndmin=2) for f in files])
    return files, d[:, 6] / 1e9, np.degrees(d[:, 0:3]), d[:, 3:6]


def merge(indices):
    """Consecutive block indices -> (first, last) runs."""
    runs, k = [], 0
    while k < len(indices):
        j = k
        while j + 1 < len(indices) and indices[j + 1] == indices[j] + 1:
            j += 1
        runs.append((indices[k], indices[j]))
        k = j + 1
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording")
    ap.add_argument("--rest-factor", type=float, default=2.0)
    ap.add_argument("--offset-deg-s", type=float, default=3.0, help="a quiet stretch this far from the start bias is a false offset")
    ap.add_argument("--min-offset-s", type=float, default=2.0, help="a false offset shorter than this is only a warning")
    args = ap.parse_args()

    files, t_abs, w, a = load(args.recording)
    t = t_abs - t_abs[0]
    rows = []  # (check, value, verdict)

    def row(check, value, verdict="ok"):
        rows.append((check, value, verdict))

    # timing file of the recorder, if there is one
    folder = args.recording if os.path.isdir(args.recording) else os.path.dirname(args.recording)
    for tf in sorted(glob.glob(os.path.join(folder, "timing*.json"))):
        try:
            tj = json.load(open(tf, encoding="utf-8"))
            lost = tj.get("lost_samples")
            row("recorder timing", f"lost samples {lost}, stamp jumps {tj.get('stamp_jumps')}, sensor {tj.get('sensor_model', '?')}",
                "ok" if not lost else "WARN")
        except (OSError, ValueError) as e:
            row("recorder timing", f"unreadable ({e})", "WARN")

    # rate and gaps
    dt = np.diff(t)
    med = float(np.median(dt))
    non_increasing = int((dt <= 0).sum())
    gaps = int((dt > 2.5 * med).sum())
    row("sample rate", f"{1.0 / med:.0f} Hz (median step {med * 1000:.3f} ms), {len(t)} samples, {t[-1]:.1f} s")
    row("non-increasing timestamps", str(non_increasing), "FAIL" if non_increasing else "ok")
    row("gaps (> 2.5 x median step)", f"{gaps}, longest step {dt.max() * 1000:.1f} ms", "WARN" if gaps else "ok")

    # block statistics
    starts = np.arange(0.0, t[-1] - BLOCK_S, BLOCK_S / 2)
    i0 = np.searchsorted(t, starts)
    i1 = np.searchsorted(t, starts + BLOCK_S)
    keep = (i1 - i0) >= 16
    starts, i0, i1 = starts[keep], i0[keep], i1[keep]
    if len(starts) < 4:
        sys.exit("recording too short for the block statistics")
    g_mean = np.array([w[p:q].mean(0) for p, q in zip(i0, i1)])
    g_std = np.array([w[p:q].std(0) for p, q in zip(i0, i1)])
    a_norm_std = np.array([np.linalg.norm(a[p:q], axis=1).std() for p, q in zip(i0, i1)])
    a_axis_std = np.array([a[p:q].std(0).max() for p, q in zip(i0, i1)])
    g_floor = np.percentile(g_std, 3, axis=0)
    quiet = ((g_std < args.rest_factor * g_floor).all(1) & (a_norm_std < args.rest_factor * np.percentile(a_norm_std, 3))
             & (a_axis_std < args.rest_factor * np.percentile(a_axis_std, 3)))
    quiet_runs = merge(np.where(quiet)[0])

    # rest at the start
    start_rest_s = 0.0
    if quiet_runs and quiet_runs[0][0] == 0:
        start_rest_s = float(starts[quiet_runs[0][1]] + BLOCK_S)
    has_rest = start_rest_s >= 1.0
    row("rest at the start", f"{start_rest_s:.1f} s", "ok" if start_rest_s >= 2.0 else "WARN")
    if not has_rest:
        row("noise floor", "no rest at the start: the floor is the calmest motion, false offsets below are a hint only", "WARN")

    # bias and gravity in the first and last rest
    def span(run):
        return slice(i0[run[0]], i1[run[1]])

    first = span(quiet_runs[0]) if quiet_runs else slice(0, min(len(t), int(1.0 / med)))
    bias0 = w[first].mean(0)
    gravity = float(np.linalg.norm(a[first].mean(0)))
    row("gravity at rest", f"{gravity:.4f} g", "ok" if 0.98 <= gravity <= 1.02 else "WARN")
    row("gyro bias, first rest", f"{bias0.round(3)} deg/s", "ok" if np.abs(bias0).max() <= 1.0 else "WARN")
    rest_runs = [r for r in quiet_runs if np.linalg.norm(g_mean[r[0]:r[1] + 1].mean(0) - bias0) <= args.offset_deg_s]
    if len(rest_runs) > 1:
        last = span(rest_runs[-1])
        bias1 = w[last].mean(0)
        drift = float(np.abs(bias1 - bias0).max())
        row("gyro bias, last rest", f"{bias1.round(3)} deg/s at {t[last.start]:.0f} s (change {drift:.2f} deg/s)", "ok" if drift <= 0.5 else "WARN")

    # false offsets: quiet, but the mean is off
    off = np.linalg.norm(g_mean - bias0, axis=1)
    bad_runs = merge(np.where(quiet & (off > args.offset_deg_s))[0])
    long_bad = [r for r in bad_runs if starts[r[1]] + BLOCK_S - starts[r[0]] >= args.min_offset_s]
    verdict = "ok" if not bad_runs else ("FAIL" if long_bad and has_rest else "WARN")
    row("false gyro offsets", f"{len(bad_runs)} stretch(es), {len(long_bad)} of {args.min_offset_s:g} s or longer", verdict)
    for r in sorted(bad_runs, key=lambda r: r[0] - r[1])[:5]:
        m = g_mean[r[0]:r[1] + 1].mean(0)
        s = g_std[r[0]:r[1] + 1].mean(0)
        row("  offset", f"{starts[r[0]]:.1f}-{starts[r[1]] + BLOCK_S:.1f} s: gyro mean {m.round(1)} deg/s, std {s.round(2)} (floor {g_floor.round(2)})", "")

    # noise from the longest rest
    if rest_runs:
        longest = max(rest_runs, key=lambda r: r[1] - r[0])
        sl = span(longest)
        n = max(2, int(round(0.1 / med)))
        k = ((sl.stop - sl.start) // n) * n
        if k >= 4 * n:
            wm = w[sl.start:sl.start + k].reshape(-1, n, 3).mean(1)
            am = a[sl.start:sl.start + k].reshape(-1, n, 3).mean(1) * 9.80665
            arw_sample = w[sl].std(0) * np.sqrt(med)
            arw_block = np.diff(wm, axis=0).std(0) / np.sqrt(2) * np.sqrt(n * med)
            den_sample = a[sl].std(0) * 9.80665 * np.sqrt(med)
            den_block = np.diff(am, axis=0).std(0) / np.sqrt(2) * np.sqrt(n * med)
            white = bool((arw_block / arw_sample).max() < 1.5)
            row("gyro angle random walk", f"{arw_sample.round(4)} deg/sqrt(s) from samples, {arw_block.round(4)} from 0.1 s means "
                f"({t[sl.start]:.0f}-{t[sl.stop - 1]:.0f} s)", "ok" if white else "WARN")
            row("accelerometer noise density", f"{den_sample.round(4)} m/s^2/sqrt(Hz) from samples, {den_block.round(4)} from 0.1 s means", "")
            if not white:
                row("  noise note", "the two estimates differ: the rest is not still (handheld?), so these are upper bounds", "")

    row("largest rotation rate", f"{np.linalg.norm(w, axis=1).max():.0f} deg/s")
    row("largest acceleration", f"{np.linalg.norm(a, axis=1).max():.2f} g")

    print(f"IMU health: {', '.join(os.path.basename(f) for f in files)}")
    width = max(len(r[0]) for r in rows)
    for check, value, verdict in rows:
        print(f"  {check.ljust(width)}  {verdict.ljust(4)}  {value}")
    fails = sum(r[2] == "FAIL" for r in rows)
    warns = sum(r[2] == "WARN" for r in rows)
    print(f"result: {'FAIL' if fails else ('WARN' if warns else 'PASS')} ({fails} failed, {warns} warnings)")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
