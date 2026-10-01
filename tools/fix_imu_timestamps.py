"""Reconstruct a steady sample clock for a dLidar imu*.csv from bursty or jittery timestamps.

The IMU samples at a constant rate, but the recorded timestamps are arrival times: samples come in
bursts (several within ~20 us, then a gap of several ms), or the clock is corrected in sudden steps.
Arrival is never earlier than acquisition, so the true clock follows the lower edge of the arrival
times. The tool:

  1. estimates the sample period from the whole file (lower edge of arrival - k * period, fitted
     over 1 s blocks),
  2. sets t_k = min over the samples j of the next `window` seconds of (arrival_j - (j - k) * period):
     a sample is acquired no later than its own arrival and no later than any later arrival minus the
     periods in between. Dropped samples only make this bound looser, so no drop detection is needed,
  3. writes t_k for `timestamp` and shifts `timestampUnix` by the same amount, so both columns keep
     their relation.

Measurement columns are copied verbatim. The original file is never modified.
Prints the spacing statistics before and after and how much later the arrival was than the
reconstructed time (latency).
"""
import argparse
import os
import sys

TS_COL, UNIX_COL = 7, 8


def percentiles(values, ps):
    s = sorted(values)
    return [s[min(len(s) - 1, int(p / 100 * len(s)))] for p in ps]


def spacing_report(label, t):
    d = [(b - a) / 1e6 for a, b in zip(t, t[1:])]
    p = percentiles(d, (1, 10, 50, 90, 99))
    print(f"{label}: dt p1 {p[0]:.3f} p10 {p[1]:.3f} p50 {p[2]:.3f} p90 {p[3]:.3f} p99 {p[4]:.3f} ms, "
          f"min {min(d):.3f} max {max(d):.3f} ms")


def fit_period(t, index, block_s=1.0):
    """Least-squares line through the per-block minima of arrival vs sample index."""
    # start from first/last, then refine with the block minima of the residual
    period = (t[-1] - t[0]) / (index[-1] - index[0])
    for _ in range(3):
        mins = {}
        for k, ti in zip(index, t):
            r = ti - k * period
            b = int((ti - t[0]) / (block_s * 1e9))
            if b not in mins or r < mins[b][1]:
                mins[b] = (k, r)
        pts = sorted(mins.values())
        if len(pts) < 3:
            break
        n = len(pts)
        mk = sum(p[0] for p in pts) / n
        mr = sum(p[1] for p in pts) / n
        sxx = sum((p[0] - mk) ** 2 for p in pts)
        sxy = sum((p[0] - mk) * (p[1] - mr) for p in pts)
        period += sxy / sxx
    return period


def future_min(values, times, window_ns):
    """Minimum of values over [t, t + window] (monotonic deque, backward pass)."""
    from collections import deque
    n = len(values)
    out = [0.0] * n
    dq = deque()  # indices with increasing values, front = minimum
    hi = n - 1
    for k in range(n - 1, -1, -1):
        while dq and values[dq[-1]] >= values[k]:
            dq.pop()
        dq.append(k)
        while times[dq[0]] > times[k] + window_ns:
            dq.popleft()
        out[k] = values[dq[0]]
    return out


def reconstruct(t, window_s):
    """t_k = min over later samples j within the window of (arrival_j - (j - k) * period).

    Every sample is acquired no later than its own arrival and no later than the arrival of any later
    sample minus the periods in between; missing samples only make that bound looser, never wrong.
    """
    index = list(range(len(t)))
    period = fit_period(t, index)
    v = [ti - k * period for k, ti in zip(index, t)]
    lower = future_min(v, t, window_s * 1e9)
    return [k * period + o for k, o in zip(index, lower)], period


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="original imu*.csv")
    ap.add_argument("-o", "--output", required=True, help="output csv (must differ from the input)")
    ap.add_argument("--window", type=float, default=0.5, help="look-ahead window [s] (default 0.5)")
    args = ap.parse_args()
    if os.path.abspath(args.output) == os.path.abspath(args.input):
        sys.exit("error: output would overwrite the input")

    with open(args.input, newline="") as f:
        header = f.readline()
        rows = [line.split() for line in f if line.strip()]
    cols = header.split()
    if len(cols) <= UNIX_COL or cols[TS_COL] != "timestamp" or cols[UNIX_COL] != "timestampUnix":
        sys.exit(f"error: unexpected header: {header.strip()}")
    rows = [r for r in rows if len(r) == len(cols)]  # a recording cut off mid-line leaves a partial row
    t = [int(float(r[TS_COL])) for r in rows]
    unix = [int(float(r[UNIX_COL])) for r in rows]
    if any(b < a for a, b in zip(t, t[1:])):
        sys.exit("error: timestamps are not sorted")

    spacing_report("before", t)
    new_abs, period = reconstruct(t, args.window)
    # the offset is the lower edge of arrival - k * period, so new_abs is already in the arrival clock
    new_t = [int(round(v)) for v in new_abs]
    # keep at least half a period between samples (buffer dumps at the start of a recording can
    # otherwise end up with equal times)
    min_step = int(round(0.5 * period))
    for k in range(1, len(new_t)):
        if new_t[k] < new_t[k - 1] + min_step:
            new_t[k] = new_t[k - 1] + min_step
    spacing_report("after ", new_t)
    lat = [(a - b) / 1e6 for a, b in zip(t, new_t)]
    p = percentiles(lat, (1, 50, 99))
    print(f"period {period / 1e6:.4f} ms ({1e9 / period:.1f} Hz); arrival - new time: p1 {p[0]:.3f} p50 {p[1]:.3f} p99 {p[2]:.3f} max {max(lat):.1f} ms")

    with open(args.output, "w", newline="") as f:
        f.write(header)
        for r, old, new, u in zip(rows, t, new_t, unix):
            r = r[:]
            r[TS_COL] = str(new)
            r[UNIX_COL] = str(u + (new - old))
            f.write(" ".join(r) + "\n")
    print(f"wrote {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
