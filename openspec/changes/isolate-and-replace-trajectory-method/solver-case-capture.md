# Checked-solver tracing and frozen-call capture

2026-11-14. Authorized implementation following `108s-boundary-stall-analysis.md`.
Use OpenSpec apply for this local diagnostic detour; Charlie remains 3/19.
No new recording replay is authorized by this work.

## Contract

- Keep cell validity, refreshed matches, rotation/merit guards, damping policy
  and seed behavior unchanged. Diagnostics must not modify solver inputs or
  production factorization caches.
- Pass detailed traces into the actual checked proposal and the base full-merit
  assembly. Identify proposal attempt and merit context separately. Include
  sparse Hessians, full/factor RHS, actual projection, raw steps, factor push,
  full-merit linear directional derivative and trial-bound slack. The derivative
  is the assembled local derivative, not an independently verified finite difference.
- Opt-in `DMAPPER_CAPTURE_LIVOX4_SOLVER_CASE=1` selects W301 iterations 0, 5,
  10 and 51, if reached. No automatic run/retry, relaxation or success fallback.
- Freeze one checked call's numerical inputs: points/order, state/covariances,
  prediction, priors, accelerometer samples, options, LM and owned guarded
  target cells. Stream precision17 text with schema and alpha/beta identity.
  Domain owns the stream codec, not filesystem access. Log extraction and a
  standalone diagnostic executable own files.
- This is NOT a full map checkpoint, refreshed-window replay or odometry restart.
  Replaying a call cannot assess the effect of changed carry on later windows.
  Unused point metadata, cell ranges and grid bookkeeping are normalized; they
  are not numerical inputs to this guarded call.
- Caps: 4096 nodes / 200000 points per case. Refuse malformed/truncated,
  incompatible, nonfinite or stale-source cases. Original guards still execute.
- Preserve executable/source identity and input/settings hashes in the eventual
  experiment manifest. The stream schema/weights alone do not prove matching
  compiler or source version. Extractor records source-log and case SHA256.

## Preparation checklist

- [x] Connect proposal and base-merit tracing; sparse H/RHS and bound diagnostics.
- [x] Add owned single-call codec and standalone replay executable.
- [x] Add bounded W301 capture and Python extraction/reference comparison.
- [x] Pass Release and focused Debug tests, including trace-on/off exact equality,
  accepted/rejected round trips, coupled state and malformed inputs.
- [x] Independent review; resolve material findings.
- [x] Freeze tested source/build and document concrete capture/replay commands.
- [x] Obtain separate run approval and capture actual W301 cases.
- [x] Verify captured-call replay agrees with original sequential call before
  interpreting any alternative solve. All four cases match exactly, including
  output state. See `solver-case-analysis.md`.

## Commands after a separately approved capture

Build target `replay_solver_case` using the same Release build as dMapper.
Set `DMAPPER_CAPTURE_LIVOX4_SOLVER_CASE=1` only for the approved diagnostic run;
retain Huber3 and its frozen settings. This mode selects four calls, whereas
ordinary `DMAPPER_TRACE_LIVOX4=1` supplies W301 detail for iterations 0-10 and 51.
For checked Livox4 preceding windows, detailed iteration tracing is bounded to
0 and 99. Recording data remain outside the repo.

```
python tools/extract_solver_cases.py <capture-run.log> --output <new-case-directory>
replay_solver_case.exe <new-case-directory>/w301-i5.case > <replay.log>
python tools/extract_solver_cases.py <replay.log> --compare-reference <new-case-directory>/w301-i5.reference.txt
```

Replay executable exit 0 means an accepted/stationary call; 1 means expected
checked-solver rejection; 2 means a tool/input error. An exit 1 can be the correct
reproduction of the captured failure. Exact decision/cost-log comparison is
required; it is not proof of map accuracy. Comparison includes serialized output
poses, velocities, bias, offset and covariances in addition to scalar logs.
Synthetic tests also compare those states exactly across round trip.

The previous recording settings allow 3600 seconds per window. Tracing costs
time and disk; do not use a small wall-clock termination threshold and assume
diagnostic parity. Runtime/disk cost of real captures is not yet measured.

## Completed validation and review

Scratch `C:/Repo/dMapper/out/ahrs-gain-worktree`, clean at `39d57e4`, including
codec commit `24574cd`. No shipped-default changes or push.

- Release dMapper, replay tool and tests built successfully. Initial build found
  a missing test-only NullLogger include; corrected before final validation.
- Full Release CTest: 157 cases pass, 3 skipped, 15.54 s. The same three
  pre-existing expected consistency assertions remain expected, not test failures.
- Focused Release and Debug: 6 cases / 195 assertions each pass. Includes
  trace-on/off exact decisions and state, accepted/boundary-rejected cases,
  coupled IMU/bias/velocity round trips, capture-to-replay output equality,
  stale source, schema, size, truncation and invalid cell checks.
- Python extractor: 2 tests pass; existing acceptance-health: 9 tests pass.
- Standalone replay executable missing-file smoke check returns native exit 2.
- Independent reviewer `solver_capture_review` found two material gaps: loaded
  targets needed the original cell validator, and early return still omitted
  range/separated motion RHS. Both corrected and re-reviewed; no remaining
  blocker. Output-state comparison was also added in response to review.
- OpenSpec strict validation passes. Logs are scratch `out/solver-capture-*`.

Release dMapper SHA256:
`6d3edd7052bdecddfd2dd46f1b65293cfbbb8d39167158174ebb1eaed91126e7`.
Release replay tool SHA256:
`03851981f73f7ef0e4116443051b200e939de05e18222521ab9a3ea44b892bfd`.

Preparation is complete. Actual W301 input case is NOT CAPTURED; old logs cannot
recover it. Next recording scope would use the existing 125-second prefix and
unchanged Huber3/guards/settings, with bounded capture enabled, in a fresh external
bundle. The preceding pass took about 52 minutes; additional trace overhead is
unmeasured. Stop for explicit approval before this run and any recorded-case
solver replay. Medium is sufficient for approved execution; retain high for
subsequent interpretation of gradients and constrained steps.

## Approved Livox4 capture run

User approved the 125-second prefix capture. Started 2026-11-14 13:11 CET,
PID 3908, using the verified `39d57e4` Release binary, the same TOML and
the same SHA256-verified LiDAR/IMU files as the prior run. Huber3 and
`DMAPPER_CAPTURE_LIVOX4_SOLVER_CASE=1` are active. Bundle:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-solver-capture-125s`.
The launcher records native exit and finish time in `run.json`.

At 14:01 CET, PID 3908 was absent and the log ended at window 130,
iteration 70. The launcher metadata still said `running` and contained no
native exit code. The stop reason is unconfirmed. Window 301 was not
reached and no solver cases were extracted. A fresh prefix run is needed.

The user approved a restart. Neither dMapper log showed an allocation error,
and the queried Windows logs returned no resource exhaustion event or dMapper
crash report. Before restart, free physical memory was about 14 GiB. This does
not establish memory availability at the original stop.

Retry started at 14:14 CET in `livox4-solver-capture-125s-retry`, PID 16804.
The detached Windows launcher records process and system memory every 30 seconds,
saves the native exit status and extracts solver cases after exit.

Retry finished at 15:03:51 CET after about 49m43s. It completed 300/346
windows and reproduced `merit_not_decreased` at W301 iteration 51.
`cases/manifest.json` records four extracted cases (iterations 0, 5, 10, 51)
with reference logs. The launcher recorded a null native exit code. The
failure is confirmed by stderr. Last sampled free RAM was 13.1 GiB.
Run health reports incomplete processing, zero completed-window bends above
3 degrees, and velocity chi-square median/p95 of 95/1172 over 104 windows.
Switch to high for analysis. Standalone reproduction remains pending.
