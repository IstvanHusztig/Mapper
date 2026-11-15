"""Did the odometry of a run behave? Reads the log of a run made with `log_window_diagnostics = true` and judges it
against fixed limits. It needs no map and no reference run, so it is the first check of a candidate.

  bends          windows whose heading the optimizer turned against the gyro's prediction ("yaw correction last"):
                 share over 3 deg, count over 10 deg, the largest
  slides         windows whose end speed is above --max-speed (a walker does not reach it)
  bias           the largest gyro bias estimate per axis (on top of the start-up bias; a healthy gyro stays near 0)
  consistency    chi2 of the start-velocity jump against the carried velocity covariance, median and 95th percentile
                 (about 3 when the covariance is honest), and how much stiffer the scan matching is than the velocity
                 prior plus motion model along the weak axis. These lines exist only for windows with an information
                 ratio below 0.45. Diagnostic: a looser sigma lowers a chi2 without improving the map.
  closure        distance between the last and the first window start, per 100 m of path, before loop closure;
                 reported only as a proxy, never a physical closure verdict
  loop closure   accepted loops and the largest segment correction
  log            warnings, dropped window slices, iteration range (stop reasons unavailable)

Exit 2 for incomplete evidence, 1 for diagnostic triggers, 0 for complete diagnostics without triggers. Never an accuracy PASS.

usage: run_health.py <run.log> [--toml <settings used>] [--from T0] [--to T1] [limits, see --help]
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np

NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def analyse(log_path, toml_path=None, t_from=None, t_to=None):
    """Per-run metrics from a diagnostics log, as a dict."""
    text = Path(log_path).read_text(errors="replace")
    windows = {}
    for m in re.finditer(r"^window diagnostics (\d+): t (" + NUM + r") s, dt (" + NUM + r") s, iterations (\d+),.*?start (" + NUM + ") (" + NUM + ") (" + NUM
                         + r"), guess move.*?info ratio (" + NUM + ")", text, re.M):
        windows[int(m.group(1))] = {"t": float(m.group(2)), "dt": float(m.group(3)), "it": int(m.group(4)),
                                    "start": [float(m.group(k)) for k in (5, 6, 7)], "info": float(m.group(8))}
    if not windows:
        sys.exit(f"{log_path}: no 'window diagnostics' lines (run with log_window_diagnostics = true)")
    for m in re.finditer(r"^window diagnostics (\d+): velocity start \S+ \S+ \S+, end (" + NUM + ") (" + NUM + ") (" + NUM + ")", text, re.M):
        windows.get(int(m.group(1)), {})["speed"] = float(np.linalg.norm([float(m.group(k)) for k in (2, 3, 4)]))
    for m in re.finditer(r"^window diagnostics (\d+): heading \S+ deg, yaw correction first \S+ last (" + NUM + ") deg", text, re.M):
        windows.get(int(m.group(1)), {})["bend"] = float(m.group(2))
    for m in re.finditer(r"^window diagnostics (\d+): weak axis.*?start-velocity jump (" + NUM + ") (" + NUM + ") (" + NUM + r") m/s, chi2 (" + NUM
                         + r"), prior sigma along it (" + NUM + r") m/s;.*?: ndt (" + NUM + r"), velocity prior (" + NUM + r"), motion model (" + NUM + ")",
                         text, re.M):
        w = windows.get(int(m.group(1)), {})
        w["jump"] = float(np.linalg.norm([float(m.group(k)) for k in (2, 3, 4)]))
        w["chi2"], w["prior_sigma"] = float(m.group(5)), float(m.group(6))
        w["ndt"], w["prior"] = float(m.group(7)), float(m.group(8)) + float(m.group(9))
    for m in re.finditer(r"^gyro bias \[deg/s\] window (\d+): (" + NUM + ") (" + NUM + ") (" + NUM + ")", text, re.M):
        windows.get(int(m.group(1)), {})["bias"] = [float(m.group(k)) for k in (2, 3, 4)]

    for metric, pattern in (("bend", r"^window diagnostics (\d+): heading "),
                            ("speed", r"^window diagnostics (\d+): velocity start "),
                            ("chi2", r"^window diagnostics (\d+): weak axis "),
                            ("bias", r"^gyro bias \[deg/s\] window (\d+):")):
        ids = list(map(int, re.findall(pattern, text, re.M)))
        if len(ids) != len(set(ids)) or any(k not in windows for k in ids) or len(ids) != sum(metric in w for w in windows.values()):
            raise ValueError(f"duplicate, orphan or malformed {metric} diagnostic lines")

    keys = [k for k in sorted(windows) if (t_from is None or windows[k]["t"] >= t_from) and (t_to is None or windows[k]["t"] <= t_to)]
    if not keys:
        sys.exit("no windows in the requested time range")
    col = lambda name: np.array([windows[k][name] for k in keys if name in windows[k]], dtype=float)
    starts = re.findall(r"^window diagnostics (\d+): t ", text, re.M)
    if len(starts) != len(set(starts)) or len(starts) != len(windows):
        raise ValueError("duplicate or malformed window diagnostic headers")
    if sorted(windows) != list(range(1, max(windows) + 1)):
        raise ValueError("missing complete window diagnostics")
    r = {"windows": len(keys), "t_first": windows[keys[0]]["t"], "t_last": windows[keys[-1]]["t"]}

    r["coverage"] = {name: sum(name in windows[k] for k in keys)
                     for name in ("bend", "speed", "bias", "chi2")}
    r["missing_required"] = [name for name in ("bend", "speed", "bias")
                             if r["coverage"][name] != len(keys)]
    progress = [(int(a), int(b)) for a,b in re.findall(r"^finished at iteration .*? optimizing window (\d+)/(\d+) ", text, re.M)]
    totals = {total for _,total in progress}
    expected = next(iter(totals)) if len(totals) == 1 else None
    r["expected_windows"] = expected
    if expected is None or set(windows) != set(range(1, expected+1)) or len(progress) != expected or {i for i,_ in progress} != set(windows):
        r["missing_required"].append("expected total window coverage NOT ESTABLISHED")
    # Ratio is printed to three decimals; <=0.450 is conservatively eligible.
    # Zero/negative information can suppress the source diagnostic: absence then
    # remains unknown, rather than silently declaring its uncertainty measured.
    r["uncertainty_missing"] = [k for k in keys if windows[k]["info"] <= .450 and "chi2" not in windows[k]]
    if r["uncertainty_missing"]:
        r["missing_required"].append("conditional chi2 (eligibility unresolved or missing)")
    r["window_starts"] = [{"id": k, "time_s": windows[k]["t"], "position": windows[k]["start"]} for k in keys]
    r["completed"] = bool(re.search(r"Processing time", text)) and not bool(re.search(r"(?im)^.*(?:calculations cancell?ed|CHALLENGING DATA OCCURED|aborted).*$", text))
    for name in ("dt", "start", "info", "bend", "speed", "bias", "chi2", "jump", "prior_sigma"):
        if not np.isfinite(col(name)).all():
            raise ValueError(f"non-finite {name} diagnostics")
    bend = np.abs(col("bend"))
    if len(bend):
        r["bend_gt3"], r["bend_gt10"], r["bend_max"] = int((bend > 3).sum()), int((bend > 10).sum()), float(bend.max())
        r["bend_gt3_share"] = r["bend_gt3"] / len(bend)
    speed = col("speed")
    if len(speed):
        r["speed_max"] = float(speed.max())
        r["speeds"] = speed
    bias = np.abs(col("bias"))
    if len(bias):
        r["bias_max"] = [float(v) for v in bias.max(0)]
    chi2 = col("chi2")
    if len(chi2):
        r["chi2_windows"], r["chi2_p50"], r["chi2_p95"] = len(chi2), float(np.median(chi2)), float(np.percentile(chi2, 95))
        r["jump_p50"], r["prior_sigma_p50"] = float(np.median(col("jump"))), float(np.median(col("prior_sigma")))
        ndt, prior = col("ndt"), col("prior")
        ok = prior > 0
        ratio = ndt[ok] / prior[ok]
        r["ndt_over_prior_p50"], r["ndt_over_prior_p90"] = (float(np.median(ratio)) if len(ratio) else None), (float(np.percentile(ratio, 90)) if len(ratio) else None)
        r["ndt_negative_share"] = float((ndt < 0).mean())
    start = col("start").reshape(-1, 3)
    path = float(np.linalg.norm(np.diff(start, axis=0), axis=1).sum())
    r["path_m"] = path
    gap = start[-1] - start[0]
    r["closure_m"], r["closure_z_m"] = float(np.linalg.norm(gap)), float(abs(gap[2]))
    r["z_min"], r["z_max"] = float(start[:, 2].min()), float(start[:, 2].max())
    r["per_window"] = [(k, windows[k]["t"], windows[k].get("bend"), windows[k].get("speed"), windows[k].get("chi2")) for k in keys]
    it = col("it")
    r["iteration_range"] = [int(it.min()), int(it.max())]

    m = re.search(r"loop closure: (\d+) loop\(s\) accepted, largest segment correction (" + NUM + ") m", text)
    r["loops"], r["lc_max_m"] = (int(m.group(1)), float(m.group(2))) if m else (None, None)
    r["warnings"] = len(re.findall(r"(?im)^.*warning.*$", text))
    r["dropped_slices"] = len(re.findall(r"window slice \d+ dropped", text))
    return r


def judge(r, args):
    """Rows (check, value, limit, verdict) for the metrics of analyse()."""
    rows = []

    def row(check, value, limit="", passed=None):
        rows.append((check, value, limit, "" if passed is None else ("CLEAR" if passed else "TRIGGER")))

    for metric, count in r["coverage"].items():
        rows.append((f"coverage {metric}", f"{count}/{r['windows']}",
                     "conditional subset" if metric == "chi2" else "all windows required",
                     "NOT MEASURED" if not count else ("INCOMPLETE" if metric != "chi2" and count != r['windows'] else "")))
    if not r["completed"]:
        rows.append(("completion", "NOT MEASURED", "Processing time marker required", "INCOMPLETE"))
    if r["uncertainty_missing"]:
        rows.append(("conditional uncertainty", str(r["uncertainty_missing"]), "eligibility unresolved or missing", "INCOMPLETE"))
    row("windows", f"{r['windows']} ({r['t_first']:.0f}-{r['t_last']:.0f} s)")
    if "bend_max" in r:
        row("windows bent > 3 deg against the gyro", f"{r['bend_gt3']} ({r['bend_gt3_share'] * 100:.1f} %)", f"<= {args.max_bend_share * 100:g} %",
            r["bend_gt3_share"] <= args.max_bend_share)
        row("windows bent > 10 deg", str(r["bend_gt10"]), "0", r["bend_gt10"] == 0)
        row("largest bend", f"{r['bend_max']:.1f} deg")
        if "bend_sigmas_p50" in r:
            row("bend in sigmas of the gyro term, median / p95", f"{r['bend_sigmas_p50']:.1f} / {r['bend_sigmas_p95']:.1f} "
                f"(sigma {r['gyro_sigma_window_deg']:.3f} deg per window)", "diagnostic")
    if "speed_max" in r:
        fast = int((r["speeds"] > args.max_speed).sum())
        row(f"windows faster than {args.max_speed:g} m/s", f"{fast} (max {r['speed_max']:.2f} m/s)", "0", fast == 0)
    if "bias_max" in r:
        row("largest gyro bias estimate x / y / z", " / ".join(f"{v:.2f}" for v in r["bias_max"]) + " deg/s", f"<= {args.max_bias:g}",
            max(r["bias_max"]) <= args.max_bias)
    if "chi2_p50" in r:
        row("chi2 of the velocity jump, median / p95", f"{r['chi2_p50']:.0f} / {r['chi2_p95']:.0f} ({r['chi2_windows']} windows)",
            f"<= {args.max_chi2:g} / {args.max_chi2_p95:g}", r["chi2_p50"] <= args.max_chi2 and r["chi2_p95"] <= args.max_chi2_p95)
        row("velocity jump between windows / its carried sigma, medians", f"{r['jump_p50'] * 1000:.0f} mm/s / {r['prior_sigma_p50'] * 1000:.1f} mm/s", "diagnostic")
    row("path before loop closure", f"{r['path_m']:.0f} m, height {r['z_min']:.2f} to {r['z_max']:.2f} m")
    row("window-start endpoint proxy", f"{r['closure_m']:.2f} m (height {r['closure_z_m']:.2f} m)", "not physical closure accuracy")
    row("loop closure", "NOT MEASURED" if r["loops"] is None else f"{r['loops']} loops, largest correction {r['lc_max_m']:.2f} m", "descriptive only")
    row("log", f"{r['warnings']} warnings, {r['dropped_slices']} dropped slices; iteration range {r['iteration_range']}", "stop cause NOT MEASURED")
    return rows


def add_limits(ap):
    ap.add_argument("--max-bend-share", type=float, default=0.01, help="share of windows bent > 3 deg (default 0.01)")
    ap.add_argument("--max-speed", type=float, default=2.5, help="m/s (default 2.5)")
    ap.add_argument("--max-bias", type=float, default=0.5, help="deg/s (default 0.5)")
    ap.add_argument("--max-chi2", type=float, default=5.0, help="median chi2 of the velocity jump (default 5)")
    ap.add_argument("--max-chi2-p95", type=float, default=20.0, help="95th percentile (default 20)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("--toml", help="retained context argument; does not certify calibrated uncertainty")
    ap.add_argument("--from", dest="t_from", type=float, help="only windows from this time [s]")
    ap.add_argument("--to", dest="t_to", type=float, help="only windows up to this time [s]")
    ap.add_argument("--worst", type=int, default=0, metavar="N", help="also list the N windows with the largest bend and the N fastest")
    add_limits(ap)
    args = ap.parse_args()
    metrics = analyse(args.log, args.toml, args.t_from, args.t_to)
    rows = judge(metrics, args)
    print(f"run health: {args.log}")
    w0, w1 = max(len(r[0]) for r in rows), max(len(r[1]) for r in rows)
    for check, value, limit, verdict in rows:
        print(f"  {check.ljust(w0)}  {verdict.ljust(4)}  {value.ljust(w1)}  {limit}")
    if args.worst:
        for label, column in (("largest bends", 2), ("fastest windows", 3)):
            print(f"  {label}:")
            valid = [w for w in metrics["per_window"] if w[column] is not None]
            for w in sorted(valid, key=lambda w: -abs(w[column]))[:args.worst]:
                print(f"    window {w[0]} at {w[1]:.1f}s: bend={w[2]}, speed={w[3]}, chi2={w[4]}")
    incomplete = bool(metrics["missing_required"] or not metrics["completed"])
    triggers = sum(r[3] == "TRIGGER" for r in rows)
    print(f"result: {'INCOMPLETE' if incomplete else ('TRIGGERS' if triggers else 'DIAGNOSTICS COMPLETE')}; accuracy NOT ASSESSED")
    sys.exit(2 if incomplete else (1 if triggers else 0))


if __name__ == "__main__":
    main()
