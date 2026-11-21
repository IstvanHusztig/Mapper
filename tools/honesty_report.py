#!/usr/bin/env python3
"""Honesty report: descriptive figures from one or more window_features.csv tables, side by side.

openspec change consistency-harness, design D11 (the tool) and D10 (the readings). Pure Python, standard library only.

For each table: the start-velocity move (median of its norm, per axis, by quarter of the rotation rate and of the start
acceleration, lag-1 autocorrelation, at rest) and the readings of D10 (the three normalised moves with their NOT MEASURED
reasons, the velocity chi2), each under one stated mask and definition (printed under the table and in tools/README.md).

A figure whose input column is missing from a table prints "unavailable" for that table: the tool never rebuilds a
quantity another way. A figure whose columns exist but whose mask selects no window prints "n 0". No verdict, no limits.

    python tools/honesty_report.py <window_features.csv> [<window_features.csv> ...] [--label NAME ...]
"""
import argparse
import csv
import math
import sys
from pathlib import Path

UNAVAILABLE = "unavailable"
AXES = ("x", "y", "z")
VELOCITY_START = [f"velocity_start_{a}" for a in AXES]
VELOCITY_PRIOR = [f"prepared_velocity_prior_{a}_mps" for a in AXES]
MOVE_COLUMNS = ["acc_term"] + VELOCITY_START + VELOCITY_PRIOR
RATE = "rotation_rate_mean_deg_s"
START_ACC = [f"start_acceleration_{a}_mps2" for a in AXES]
WEAK_RATIO = 0.45  # the verdict's weak windows (VerdictLimits::weak_ratio), used only to select, never as a limit
NORMALISED_MOVES = (("move_start_velocity", "start velocity", 3),
                    ("move_joint", "start velocity + offset", 6),
                    ("move_gyro_bias", "gyro bias", 3))


def number(text):
    """A CSV field as a finite float, or None (empty, not a number, NaN or infinite)."""
    if text is None:
        return None
    try:
        v = float(text)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def percentile(values, p):
    """Linear interpolation between the closest ranks (numpy's default and percentile_linear in window_verdict.cpp)."""
    v = sorted(values)
    if not v:
        return None
    pos = p / 100.0 * (len(v) - 1)
    lo = math.floor(pos)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (pos - lo)


def median(values):
    return percentile(values, 50.0)


def lag1_autocorrelation(series):
    """series: [(window, value)] in any order. r1 = sum over consecutive pairs (windows k, k+1 both present) of
    (v_k - mean)(v_k+1 - mean), divided by the sum over all values of (v - mean)^2; mean over all values.
    None with fewer than two values, no consecutive pair or zero variance."""
    if len(series) < 2:
        return None, 0
    by_window = dict(series)
    mean = sum(by_window.values()) / len(by_window)
    denominator = sum((v - mean) ** 2 for v in by_window.values())
    pairs = [(by_window[w], by_window[w + 1]) for w in sorted(by_window) if w + 1 in by_window]
    if not pairs or denominator <= 0.0:
        return None, len(pairs)
    return sum((a - mean) * (b - mean) for a, b in pairs) / denominator, len(pairs)


def fmt(v):
    return "-" if v is None else f"{v:.4g}"


class Table:
    def __init__(self, path, label=None):
        self.path = Path(path)
        with open(self.path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            self.columns = set(reader.fieldnames or [])
            if "window" not in self.columns:
                raise ValueError(f"{self.path}: no 'window' column, not a window_features.csv")
            self.rows = list(reader)
        self.label = label or "/".join(self.path.parts[-2:])

    def has(self, columns):
        return all(c in self.columns for c in columns)

    def window(self, row):
        return int(row["window"])

    def moves(self):
        """The start-velocity move of every window in its mask: (row, d) with d = velocity_start - prepared prior."""
        out = []
        for row in self.rows:
            if row["acc_term"] != "1":
                continue
            v = [number(row[c]) for c in VELOCITY_START]
            m = [number(row[c]) for c in VELOCITY_PRIOR]
            if None in v or None in m:
                continue
            out.append((row, [a - b for a, b in zip(v, m)]))
        return out


def norm(d):
    return math.sqrt(sum(c * c for c in d))


# --- figures: each returns the cell text for one table ------------------------------------------------------------

def coverage_cell(n, total):
    return f"n {n}/{total}"


def fig_windows(t):
    return str(len(t.rows))


def fig_handoff(t):
    if not t.has(["handoff"]):
        return UNAVAILABLE
    counts = {}
    for row in t.rows:
        key = row["handoff"] or "not recorded"
        counts[key] = counts.get(key, 0) + 1
    return ", ".join(f"{k} {counts[k]}" for k in sorted(counts))


def fig_move_coverage(t):
    if not t.has(MOVE_COLUMNS):
        return UNAVAILABLE
    return coverage_cell(len(t.moves()), len(t.rows))


def fig_move_norm(t):
    if not t.has(MOVE_COLUMNS):
        return UNAVAILABLE
    m = t.moves()
    if not m:
        return "n 0"
    norms = [norm(d) for _, d in m]
    return f"{fmt(median(norms))} (p95 {fmt(percentile(norms, 95))}, max {fmt(max(norms))})"


def fig_move_axis(axis):
    def figure(t):
        if not t.has(MOVE_COLUMNS):
            return UNAVAILABLE
        m = t.moves()
        if not m:
            return "n 0"
        return fmt(median([abs(d[axis]) for _, d in m]))
    return figure


def quarters_of(t, feature_columns, feature):
    """The move mask with the feature finite, cut at the feature's 25/50/75 % percentiles of those windows;
    quarter 1: f <= p25, 2: p25 < f <= p50, 3: p50 < f <= p75, 4: f > p75."""
    pairs = []
    for row, d in t.moves():
        f = feature(row)
        if f is not None:
            pairs.append((f, norm(d)))
    if not pairs:
        return None
    cuts = [percentile([f for f, _ in pairs], p) for p in (25, 50, 75)]
    groups = [[], [], [], []]
    for f, n in pairs:
        q = 0 if f <= cuts[0] else 1 if f <= cuts[1] else 2 if f <= cuts[2] else 3
        groups[q].append((f, n))
    return groups


def fig_move_quarter(feature_columns, feature, quarter):
    def figure(t):
        if not t.has(MOVE_COLUMNS + feature_columns):
            return UNAVAILABLE
        groups = quarters_of(t, feature_columns, feature)
        if groups is None:
            return "n 0"
        g = groups[quarter]
        if not g:
            return "n 0"
        fs = [f for f, _ in g]
        return f"{fmt(median([n for _, n in g]))} (n {len(g)}; {fmt(min(fs))}..{fmt(max(fs))})"
    return figure


def rotation_rate(row):
    return number(row[RATE])


def start_acceleration(row):
    a = [number(row[c]) for c in START_ACC]
    return None if None in a else norm(a)


def fig_move_lag1(axis):
    def figure(t):
        if not t.has(MOVE_COLUMNS):
            return UNAVAILABLE
        series = [(t.window(row), (norm(d) if axis is None else d[axis])) for row, d in t.moves()]
        r, pairs = lag1_autocorrelation(series)
        return f"{fmt(r)} ({pairs} pairs)"
    return figure


def fig_move_rest(rest):
    def figure(t):
        if not t.has(MOVE_COLUMNS + ["at_rest"]):
            return UNAVAILABLE
        m = [d for row, d in t.moves() if row["at_rest"] == ("1" if rest else "0")]
        if not m:
            return "n 0"
        return f"{fmt(median([norm(d) for d in m]))} (n {len(m)})"
    return figure


def fig_normalised(name):
    def figure(t):
        if not t.has([name, name + "_reason"]):
            return UNAVAILABLE
        values = [number(r[name]) for r in t.rows if r[name + "_reason"] == "measured"]
        values = [v for v in values if v is not None]
        cell = coverage_cell(len(values), len(t.rows))
        if values:
            cell += f": median {fmt(median(values))}, p95 {fmt(percentile(values, 95))}, max {fmt(max(values))}"
        return cell
    return figure


def fig_normalised_reasons(name):
    def figure(t):
        if not t.has([name + "_reason"]):
            return UNAVAILABLE
        counts = {}
        for r in t.rows:
            reason = r[name + "_reason"] or "(empty)"
            if reason != "measured":
                counts[reason] = counts.get(reason, 0) + 1
        return ", ".join(f"{k} {counts[k]}" for k in sorted(counts)) or "none"
    return figure


def fig_reduction(name):
    def figure(t):
        if not t.has([name + "_reason", name + "_reduction"]):
            return UNAVAILABLE
        values = [number(r[name + "_reduction"]) for r in t.rows if r[name + "_reason"] == "measured"]
        values = [v for v in values if v is not None]
        if not values:
            return "n 0"
        return f"{fmt(median(values))} (n {len(values)})"
    return figure


def fig_chi2(weak):
    def figure(t):
        needed = ["velocity_chi2"] + (["information_ratio"] if weak else [])
        if not t.has(needed):
            return UNAVAILABLE
        values = []
        for r in t.rows:
            c = number(r["velocity_chi2"])
            if c is None:
                continue
            if weak:
                ratio = number(r["information_ratio"])
                if ratio is None or ratio >= WEAK_RATIO:
                    continue
            values.append(c)
        cell = coverage_cell(len(values), len(t.rows))
        if values:
            cell += f": median {fmt(median(values))}, p95 {fmt(percentile(values, 95))}, max {fmt(max(values))}"
        return cell
    return figure


FIGURES = [
    ("Table", None, None),
    ("windows", fig_windows, "rows of the table"),
    ("hand-off", fig_handoff, "windows per value of the handoff column (empty: not recorded)"),
    ("Start-velocity move d = velocity_start - prepared_velocity_prior [m/s, world]", None, None),
    ("coverage", fig_move_coverage, "MOVE MASK: acc_term = 1 and velocity_start_xyz and prepared_velocity_prior_xyz_mps all finite"),
    ("median |d|", fig_move_norm, "median of the Euclidean norm of d over the move mask (p95, largest)"),
] + [(f"median |d_{a}|", fig_move_axis(i), f"median of |d_{a}| over the move mask") for i, a in enumerate(AXES)] + [
    (f"|d| rotation-rate Q{q + 1}", fig_move_quarter([RATE], rotation_rate, q),
     "median |d| in a quarter of rotation_rate_mean_deg_s (move mask with the rate finite, cut at its 25/50/75 % "
     "percentiles; Q1 <= p25 < Q2 <= p50 < Q3 <= p75 < Q4); n and the rate range [deg/s]" if q == 0 else None)
    for q in range(4)
] + [
    (f"|d| start-acc Q{q + 1}", fig_move_quarter(START_ACC, start_acceleration, q),
     "the same by quarter of the norm of start_acceleration_xyz_mps2 [m/s^2, world, gravity removed]" if q == 0 else None)
    for q in range(4)
] + [
    (f"lag-1 autocorr d_{a}", fig_move_lag1(i),
     "r1 over the move mask: sum over pairs of windows k, k+1 both in the mask of (v_k - mean)(v_k+1 - mean) / "
     "sum over the mask of (v - mean)^2, v the signed component" if i == 0 else None)
    for i, a in enumerate(AXES)
] + [
    ("lag-1 autocorr |d|", fig_move_lag1(None), "the same for the norm |d|"),
    ("median |d| at rest", fig_move_rest(True), "median |d| over the move mask with at_rest = 1"),
    ("median |d| moving", fig_move_rest(False), "median |d| over the move mask with at_rest = 0"),
    ("Normalised moves q (D10; expected value 1, descriptive)", None, None),
] + [
    row for name, title, n in NORMALISED_MOVES for row in (
        (f"{title} (n={n})", fig_normalised(name),
         f"{name} over the windows whose {name}_reason is 'measured' (coverage of all rows; median, p95, largest)"),
        (f"{title}: not measured", fig_normalised_reasons(name), f"windows per {name}_reason other than 'measured'"),
        (f"{title}: reduction n - tr", fig_reduction(name), f"median of {name}_reduction over the measured windows"),
    )
] + [
    ("Velocity chi2 (start velocity against its prior, 3 residuals)", None, None),
    ("all windows", fig_chi2(False), "velocity_chi2 over the windows where it is finite (median, p95, largest)"),
    ("weak windows", fig_chi2(True), f"the same over the windows with information_ratio < {WEAK_RATIO} (the verdict's weak windows)"),
]


def figure_cells(table):
    """{figure label: cell text} for one table (the figures, not the section titles)."""
    return {label: fn(table) for label, fn, _ in FIGURES if fn is not None}


def report(tables, definitions=True):
    label_width = max(len(label) for label, fn, _ in FIGURES if fn is not None) + 4
    cells = [[fn(t) if fn else "" for t in tables] for _, fn, _ in FIGURES]
    widths = [max(len(t.label), *(len(row[i]) for row in cells)) + 2 for i, t in enumerate(tables)]
    lines = ["Honesty report (consistency-harness D10/D11): descriptive figures, no verdict, no limits.", ""]
    lines.append(" " * label_width + "".join(t.label.ljust(w) for t, w in zip(tables, widths)).rstrip())
    for (label, fn, _), row in zip(FIGURES, cells):
        if fn is None:
            lines.append(label)
        else:
            lines.append(("  " + label).ljust(label_width) + "".join(c.ljust(w) for c, w in zip(row, widths)).rstrip())
    for t in tables:
        lines.append(f"{t.label}: {t.path}")
    if definitions:
        lines += ["", "Definitions (mask and definition of each figure; 'unavailable': an input column is missing from that "
                  "table, nothing is rebuilt another way; 'n 0': the mask selects no window):"]
        for label, fn, definition in FIGURES:
            if fn is not None and definition:
                lines.append(f"  {label.strip()}: {definition}")
    return "\n".join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("tables", nargs="+", help="window_features.csv files")
    p.add_argument("--label", action="append", default=[], help="a column label per table, in order")
    p.add_argument("--no-definitions", action="store_true", help="omit the definitions under the table")
    a = p.parse_args(argv)
    if a.label and len(a.label) != len(a.tables):
        p.error("give one --label per table or none")
    try:
        tables = [Table(path, a.label[i] if a.label else None) for i, path in enumerate(a.tables)]
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(report(tables, not a.no_definitions))
    return 0


if __name__ == "__main__":
    sys.exit(main())
