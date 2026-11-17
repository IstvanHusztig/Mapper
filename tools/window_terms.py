"""Prints the per-term window tables of a dMapper run or replay (log_window_terms; docs/code-split-and-region-agents-plan.md,
stage 3).

Usage:
  window_terms.py <window_terms.csv> --window N [--state result|guess]
      one window's table: per term the cost per residual, and for every motion and direction the hold (1 / sqrt(weight),
      a sigma: how tightly the term alone keeps that motion) and the pull (where the term alone would move the window,
      at its last node; m, deg for turn)
  window_terms.py <window_terms.csv> [--from S] [--to S] [--state result|guess]
      per term over the windows in [from, to] seconds: for the motions that matter most (stretch and curve along travel,
      curve vertical, turn about vertical) how often the term holds the window most, its median hold, and its median and
      largest pull

The remaining step is the 'total' row's pull: near zero when the terms balance, large when the damping froze the window.
"""
import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict

MOTIONS = ["shift", "stretch", "curve", "turn"]
DIRECTIONS = ["along", "sideways", "vertical"]
KEY_MOTIONS = [("stretch", "along"), ("curve", "along"), ("curve", "vertical"), ("turn", "vertical")]


def number(text):
    return float(text) if text not in ("", None) else math.nan


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def hold(weight):
    return 1.0 / math.sqrt(weight) if weight > 0.0 else math.inf


def unit(motion):
    return "deg" if motion == "turn" else "m"


def print_window(rows, window, state):
    rows = [r for r in rows if int(r["window"]) == window and r["state"] == state]
    if not rows:
        sys.exit(f"window {window} ({state}) not in the file")
    print(f"window {window}, {state}, t {number(rows[0]['time_s']):.2f} s")
    print(f"{'term':<20} {'cost/n':>9}  " + "  ".join(f"{m[:5]}-{d[:4]:<4} hold/pull" for m in MOTIONS for d in DIRECTIONS))
    for r in rows:
        n = int(r["residuals"])
        cost = f"{number(r['cost']) / n:9.3g}" if n > 0 else f"{'':9}"
        cells = []
        for m in MOTIONS:
            for d in DIRECTIONS:
                w, p = number(r[f"weight_{m}_{d}"]), number(r[f"pull_{m}_{d}"])
                h = hold(w) if not math.isnan(w) else math.nan
                cells.append(f"{h:8.1e}/{p:+8.4f}" if not math.isnan(p) else f"{h:8.1e}/{'':8}")
        marker = "" if r["in_total"] == "1" else "  (not in total)"
        print(f"{r['term']:<20} {cost}  " + "  ".join(cells) + marker)


def print_summary(rows, t_from, t_to, state):
    rows = [r for r in rows if r["state"] == state and t_from <= number(r["time_s"]) <= t_to]
    windows = sorted({int(r["window"]) for r in rows})
    if not windows:
        sys.exit("no window in that range")
    print(f"windows {windows[0]} to {windows[-1]} ({len(windows)}), {state}, t {t_from} .. {t_to} s")
    by_window = defaultdict(list)
    for r in rows:
        by_window[int(r["window"])].append(r)
    for m, d in KEY_MOTIONS:
        strongest = defaultdict(int)
        holds, pulls, remaining = defaultdict(list), defaultdict(list), []
        for w in windows:
            terms = [r for r in by_window[w] if r["in_total"] == "1" and number(r[f"weight_{m}_{d}"]) > 0.0]
            for r in by_window[w]:
                if r["term"] == "total" and not math.isnan(number(r[f"pull_{m}_{d}"])):
                    remaining.append(abs(number(r[f"pull_{m}_{d}"])))
            if not terms:
                continue
            strongest[max(terms, key=lambda r: number(r[f"weight_{m}_{d}"]))["term"]] += 1
            for r in terms:
                holds[r["term"]].append(hold(number(r[f"weight_{m}_{d}"])))
                p = number(r[f"pull_{m}_{d}"])
                if not math.isnan(p):
                    pulls[r["term"]].append(p)
        rem = f", remaining step median {statistics.median(remaining):.4f} max {max(remaining):.4f} {unit(m)}" if remaining else ""
        print(f"\n{m} {d}{rem}")
        print(f"  {'term':<20} {'strongest':>9} {'hold med':>10} {'pull med':>10} {'|pull| max':>10}")
        for term in sorted(holds, key=lambda t: statistics.median(holds[t])):
            p = pulls.get(term, [])
            med = f"{statistics.median(p):+10.4f}" if p else f"{'':10}"
            mx = f"{max(abs(x) for x in p):10.4f}" if p else f"{'':10}"
            print(f"  {term:<20} {strongest.get(term, 0):9d} {statistics.median(holds[term]):10.2e} {med} {mx}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("terms_csv")
    parser.add_argument("--window", type=int)
    parser.add_argument("--from", dest="t_from", type=float, default=-math.inf)
    parser.add_argument("--to", dest="t_to", type=float, default=math.inf)
    parser.add_argument("--state", choices=["result", "guess"], default="result")
    a = parser.parse_args()
    rows = read_rows(a.terms_csv)
    if a.window is not None:
        print_window(rows, a.window, a.state)
    else:
        print_summary(rows, a.t_from, a.t_to, a.state)


if __name__ == "__main__":
    main()
