#!/usr/bin/env python3
"""Summarize the iteration convergence of a dMapper run (setting log_iteration_convergence = true).

    python tools/convergence_summary.py <console_log> [--top N] [--bin B]

Reads the "convergence window ..." lines a run prints to the console (redirect it to a file:
dMapper <in> <params.toml> <out> > run.log) and prints:
  - the number of windows and the median / p90 / max of both counts (nearest-rank, as the run's own summary line);
  - a histogram of "settled after" (the iteration after which the poses stay within 0.1 mm / 0.001 deg of the result);
  - a histogram of "delta < 1e-2 after" (where convergence_delta_threshold = 1e-2 would stop the window);
  - the --top windows (default 10) with the largest position error at that fast-mode stop.
Pure Python 3. Exit code: 0 done, 1 no convergence lines found, 2 usage error.
"""

import argparse
import math
import re
import sys

LINE = re.compile(r"convergence window (\d+): settled after (\d+)/(\d+) iterations .*?after (\d+|never)"
                  r"(?:, error then ([\d.]+) mm / ([\d.]+) deg)?")


def percentile(values, pct):
    v = sorted(values)
    rank = max(1, math.ceil(pct / 100.0 * len(v)))
    return v[min(rank, len(v)) - 1]


def histogram(values, bin_width, top):
    counts = {}
    for x in values:
        b = (x - 1) // bin_width
        counts[b] = counts.get(b, 0) + 1
    largest = max(counts.values())
    last_bin = (top - 1) // bin_width
    for b in range(0, last_bin + 1):
        n = counts.get(b, 0)
        print("  %3d-%3d  %5d  %s" % (b * bin_width + 1, (b + 1) * bin_width, n, "#" * round(50 * n / largest)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--top", type=int, default=10, help="windows listed with the largest error at the fast-mode stop")
    ap.add_argument("--bin", type=int, default=5, help="histogram bin width in iterations")
    args = ap.parse_args()

    try:
        text = open(args.log, encoding="utf-8", errors="replace").read()
    except OSError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    rows = []
    for m in LINE.finditer(text):
        window, settled, iterations, fast, err_mm, err_deg = m.groups()
        rows.append({
            "window": int(window), "settled": int(settled), "iterations": int(iterations),
            "fast": None if fast == "never" else int(fast),
            "err_mm": float(err_mm) if err_mm else None, "err_deg": float(err_deg) if err_deg else None,
        })
    if not rows:
        print("no 'convergence window' lines in %s (was log_iteration_convergence = true?)" % args.log, file=sys.stderr)
        return 1

    nr_iter = max(r["iterations"] for r in rows)
    settled = [r["settled"] for r in rows]
    fast = [r["fast"] for r in rows if r["fast"] is not None]
    errs = [r for r in rows if r["err_mm"] is not None]

    print("%d windows, %d iterations each at most" % (len(rows), nr_iter))
    print("settled after:       median %d, p90 %d, max %d" % (percentile(settled, 50), percentile(settled, 90), max(settled)))
    if fast:
        print("delta < 1e-2 after:  median %d, p90 %d, max %d (in %d windows, never in %d)"
              % (percentile(fast, 50), percentile(fast, 90), max(fast), len(fast), len(rows) - len(fast)))
        e = [r["err_mm"] for r in errs]
        print("error at that point: median %.4f mm, p90 %.4f mm, max %.4f mm" % (percentile(e, 50), percentile(e, 90), max(e)))
    else:
        print("delta < 1e-2 after:  never")

    print("\nsettled after (iterations):")
    histogram(settled, args.bin, nr_iter)
    if fast:
        print("\ndelta < 1e-2 after (iterations):")
        histogram(fast, args.bin, nr_iter)
        print("\nlargest error at the fast-mode stop:")
        for r in sorted(errs, key=lambda r: -r["err_mm"])[:args.top]:
            print("  window %4d: %.4f mm / %.4f deg after %d iterations (settled after %d)"
                  % (r["window"], r["err_mm"], r["err_deg"], r["fast"], r["settled"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
