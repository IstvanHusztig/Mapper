# Livox1 divergence localization — existing runs

2026-11-12. User authorized read-only analysis of existing alpha1/alpha0.1 runs,
with OpenSpec documentation and advance notice before higher effort. Medium
sufficed for this localization. No new mapper runs or application changes.

## What starts when

Times below are seconds from the first retained pose, whose recording timestamp
is0.37497177s. Add that offset when looking up original recording timestamps.
Log window starts are rounded to0.01s and agree with snapshot window starts.
Each stage compares46,980 poses with exactly equal ordered timestamps and counts
between weights, without aligning the candidate to the control. These distances
are relative disagreement, not surveyed errors or neutral-perturbation scores.

| Event | Pre-loop time | Final time | Interpretation |
|---|---:|---:|---|
| First >1cm difference |9.346s|10.238s|Small early weight-dependent separation|
| First >5cm |13.580s|13.532s|First substantial local episode|
| First >20cm |15.176s|13.624s|Not yet persistent runaway|
| At30s |2.35mm difference|7.21mm difference|The early episode largely recovers|
| Thereafter always >5cm |38.324s|38.228s|Persistent divergence begins around38s|
| Thereafter always >20cm |38.872s|38.840s|Sustained separation|
| First >0.5m |40.822s|39.948s|Pre-loop remains above0.5m thereafter; final remains above from40.854s|
| First >1m |47.444s|47.990s|Brief recovery below1m; permanently above from52.246/52.294s|
| First >2m |53.412s|See retained JSON|Runaway transition|
| First >5m |55.160s|55.162s|Rapid divergence|
| First >10m |57.424s|57.422s|Large pre-existing odometry failure|
| Maximum difference |31.147m at70.948s|31.235m at70.898s|Post-processing is not where the large divergence originates|

The final and pre-loop stages are compared independently; their timestamp
representations are not approximately paired with each other. First crossings
are sample-level locations, not uncertainty estimates or physical acceptance
limits. 'Thereafter' means all subsequent retained samples through93.958s.

## Logged events around the transitions

- Window39,13.68s: both weights have a heading-correction spike. Control bend
  -18.027deg, candidate -8.053deg. This is not a candidate-only failure trigger.
  The pre-loop difference reaches0.408m at20s, but is only2.35mm at30s.
- Windows106–111,37.80–39.60s: alpha0.1 reports lower end speeds than control.
  At38.52s,0.302 versus0.594m/s; at39.24s,0.244 versus0.401m/s. At40s the
  difference is0.440m, mostly +0.438m along the control frame's Y direction,
  with only -0.0015m vertical difference. This is initially horizontal separation.
- Window149,53.28s: candidate bend18.258deg and speed1.279m/s; control bend
  0.977deg and speed0.541m/s. Candidate window150 at53.64s reaches2.001m/s.
- Window151,54.00s: candidate first crosses the2.5m/s investigation trigger,
  reaching2.947m/s with14.471deg bend. Control is0.546m/s and0.084deg bend.
  Candidate speed then rises to5.493m/s at55.44s.
- At60s, pre-loop separation is18.799m. At71s it is31.146m, with candidate-minus-
  control XYZ approximately[19.822,15.535,-18.326]m. Vertical collapse is a later
  part of the failure, not the original38s horizontal separation.

The useful diagnostic intervals are37–41s (persistent divergence begins) and
52–56s (transition into runaway), with13–17s retained as an earlier recovered
contrast. Localizing them does not establish the responsible optimizer term.
Control itself has large conditional chi-square and heading corrections; it is
not ground truth. No sensor fault, reflection cause or universal weighting fix
is established by these two runs. The user's mirror annotations concern Livox2;
they are not evidence that Livox1's failure is caused by reflections.

## Reproducibility and artifacts

Analysis root:
C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox1-divergence/

- localization.json: stage thresholds, persistent thresholds, selected positions,
  all logged per-window bend/speed/chi-square rows and input/dependency hashes.
- timeline.png: position disagreement, pre-loop XYZ disagreement and logged speed.
- localize.py: retained analysis script, SHA256 recorded in localization.json.

Uses existing trajectory_seams.py, snapshot_trajectory.py and run_health.py.
Every source log, TOML, snapshot, final trajectory and transform used is hashed.
No point cloud filtering or fitting, no new mapper runs and no deletion. These
new analysis artifacts postdate the721-file relocation manifest; that manifest
continues to describe the original move, not newly created files.

## Review and next boundary

Independent Codex review (not Sonnet) replays the thresholds and inspects logs.
The next question is why the two settings separate at37–41s and why only0.1
runs away near53–56s. Answering it may require deeper optimizer-state analysis;
no root-cause fix or additional experiment is authorized by this localization.
Notify the user before any step judged to need higher effort.
