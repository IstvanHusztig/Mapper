# Full Livox2 diagnostic replay: collection complete

The single explicitly authorized replay completed successfully overnight
2026-11-12/13 (Europe/Berlin). Mapper exit0; runner status `collected_complete`;
evidence and exact reproduction checks complete. No extra mapper run was started.

Artifact root:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-trace/`.
Authoritative records: `run.json`, `preflight.json`, `bundle.json`,
`preparation-manifest.json`, `run.log`, `trace.log`, `run-health.txt`, `result/`.
The prepared executable, original raw2 input, archived settings, runner and
retained control identities passed the runner's post-run checks.

## Reproduction and trace coverage

- All63 checked output files match the retained full run by exact SHA256:
  odometry snapshot, all chunk trajectories/poses/clouds and merged cloud.
- All116 selected windows and11,600 iterations passed required-record,
  numeric payload, matrix shape and window-state completeness checks.
- All23,200 before-step assembly identity checks passed; zero mismatches.
- Trace size:2,497,193,543 bytes. SHA256:
  `37e3b3decf92cf9ed2a14bf2392b0cb5e9ad4edf7121760df9806d2cd2b83e61`.
- Mapper/capture wall time:3,993.63s (66.56min), versus2,998.45s for the
  retained run. Post-run validation adds time beyond that measurement.

The cloud is reproduced, not corrected. Exact cloud identity means no new visual
improvement should be expected from opening this replay's result.

## Logged diagnostics

| Check | Retained full run | Traced replay |
|---|---:|---:|
| Windows with bend/speed/bias coverage |1517/1517|1517/1517|
| Heading corrections above3degrees |84|84|
| Heading corrections above10degrees |14|14|
| Endpoint speeds above2.5m/s |93|93|
| Maximum endpoint speed |7.647m/s|7.647m/s|
| Conditional velocity chi-square coverage |831/1517|831/1517|
| Loop closures / maximum correction |64 /2.56m|64 /2.56m|

Run-health exit1 denotes diagnostic triggers, not execution failure. Physical
closure accuracy and overall map accuracy remain NOT ASSESSED; the window-start
endpoint proxy is not an independently measured return error. Matching this
known-bad result does not constitute map acceptance or a robustness pass.

## Next step and effort boundary

Request High before substantive causal analysis. Use the existing completed
trace to distinguish matching/geometry changes, projection, carried velocity,
orientation/gravity and factor contributions around windows747–759 and the
second escalation. Align those records with existing raw scene evidence.
Use streaming Python analysis and structured outputs; retain the existing HTML
viewer without extending or deleting it. Do not load the entire2.5GB trace into
the older fixed-window Livox1 parser unchanged.

The unique initiating cause remains unresolved. No additional run is currently
required to begin analysis. Independently review causal findings before proposing
a correction. User requested quiet monitoring: report completion, failures or
decisions only. OpenSpec documentation is a standing requirement.
