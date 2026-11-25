---
name: verify-run
description: Judge whether a dMapper run's odometry behaved, from its log alone - heading bends against the gyro, position slides, gyro bias estimate, consistency statistics, closure, loop-closure corrections - against fixed limits. Use this after every dMapper run whose result will be compared, shown or reported, whenever a map looks wrong and the cause must be located in time, and before and after any change to the odometry, its settings, loop closure or consistency, even if the user only asks "how did the run go?" or "is it better now?".
---

# Verify a run

## Agreed evaluation rules (2026-11-12)

Read `openspec/changes/archive/2026-11-25-sensor-neutral-yardstick/criteria-proposal.md` before any
verdict. The user agreed this contract; it supersedes legacy tool defaults.
Report missing evidence as NOT MEASURED. Diagnostic triggers are not map-accuracy
verdicts. Overlapping empirical envelopes are inconclusive, not proof of equality
or non-regression. Preserve every perturbation member and check odometry and
final results separately. The user's review of readings and clouds remains required.


The log of a run tells where and how the odometry went wrong long before the map does, and it gives hundreds of
windows per run instead of one score. This check needs no map and no reference, so it comes first; `verify-map`
follows for the runs that pass or that must be compared.

## Steps

1. The run must have been made with `log_window_diagnostics = true` (in `[lidar_odometry_debug_info]`). If it was not,
   say so and say what a rerun costs before starting one. Results do not change with the flag.

2. Run the check. `--toml` is the settings file the run used (the `dMapper_params_*.toml` in the result folder works).
   Physical closure needs independent endpoint evidence; the log only supplies a proxy.

   ```
   python tools/run_health.py "<run>.log" --toml "<settings used>" --worst 5
   ```

3. To compare with a reference run of the same recording, run the same command on its log and put the two tables side
   by side. To look at one stretch, add `--from T0 --to T1` (seconds).

## Diagnostic interpretation

The CLI reports CLEAR/TRIGGER diagnostics with explicit coverage. Exit2 means
incomplete evidence, exit1 triggers, exit0 complete logged diagnostics without
triggers. None is a map-accuracy verdict. Missing expected windows, duplicate or
malformed lines, missing eligible uncertainty, and cancellation cannot pass.
Endpoint proxies are not independently known physical closure errors.
Bend counts (3/10 degrees), speed crossings (2.5 m/s), residual bias (0.5 degree/s)
and chi-square median/p95 (5/20) are investigation triggers. Record actual metric
coverage per window. Missing rows cannot pass. Report velocity jump and sigma
separately. Bias is residual to startup bias, not raw sensor bias. Approximate bend sigmas and inferred stop causes are no longer reported. Loop-correction size has no universal 1 m fail limit.
Use the agreed contract for endpoint and height targets and their prerequisites.

## Report

Show the table (both tables when comparing), then the times of the worst windows from `--worst`. Say which checks
trigger and where in the recording. Stop there: the cause and the next step are decided with the user, after the numbers.
A single run proves little about a setting; `verify-robustness` gives the spread a difference must exceed.
