# Flyaway tracking: tooling assessment

Requested after the user stopped the pending replay. Analysis only: no run,
installation, new detector or corrective behaviour was started. The stop remains
in force. This plan does not authorize resuming task 11.

## Existing capabilities and gaps

| Tool | Useful evidence | Current status / limit |
|---|---|---|
| tools/run_health.py | Speed, heading correction, residual bias, velocity jump versus sigma, coverage and completion | Available for retained logs; fixed diagnostic triggers are not accuracy verdicts. Conditional uncertainty is a selected subset. |
| out/full_livox2_live_monitor.py | Reads a complete-line prefix while processing; tracks end speed, successive-start displacement, heading and guard messages | Used on the original run. Hardcoded experiment path and full-prefix reparse; not a reusable continuously running alert service. Must retarget and retain missing-data checks before reuse. |
| out/full_livox2_jump_diagnosis.py, tools/snapshot_trajectory.py | Pre-loop versus final trajectory, prediction versus optimized displacement, exact timing and local IMU context | Available offline. Final speed derivatives differ from optimizer endpoint velocity. Estimated route distance is useful context, not physical distance truth. |
| tools/cloud_context.py plus raw-frame extracts | Pose-independent raw range/return context; short original sensor-frame views and time-selected final world clouds | Available and checked on current data. Nearby returns do not identify indoors/outdoors, obstruction or matcher support. |
| Scratch focused trace, commit 5eb882e | Prediction stages, carry/covariance, IMU samples, match turnover, range-group geometry, actual projected NDT step contributions, other factors and acceptance | Built and parity-tested; the full recorded replay is stopped. No validated Livox2 trace evidence yet. Runner review fixes still pending. |
| CloudCompare | Inspect small extracts, surfaces and cloud-to-cloud differences | Installed/used by user. Geometric inspection and comparison, not an automatic physical-motion detector. Reference quality and correspondence matter. |
| tools/trajectory_seams.py, tools/wall_precision.py | Locate disagreement between retained results; wall spread and revisit disagreement | Available; same-input/time/region checks required. Agreement of two wrong runs does not prove accuracy. |
| tools/holdout_sharpness.py, tools/perturbation_sweep.py | Held-out consistency or sensitivity to negligible changes | Existing tools, but holdout requires appropriately produced data and perturbations require additional approved runs. Not applicable retroactively as independent holdout proof for this run. |

## Recommended next tool: one synchronized event timeline

Build on existing Python readers, with separate lanes rather than a single
opaque quality score. Proposed, not implemented:

1. Motion: predicted within-window displacement, optimized displacement, incoming
   and outgoing velocity, successive-start movement and accumulated path. Use
   fixed-duration movement too: individual sample derivatives amplify jitter.
2. Solver correction: changes relative to prediction, heading corrections,
   guard/step rejection and carry fallback. Large inherited prediction plus a
   small correction remains visible; the one-metre guard is not a flyaway test.
3. Support: near/far eligible and matched counts, gained/lost/changed matches,
   normal geometry/projection and actual factor contributions when traced.
   Raw inverse-covariance sums must not be mistaken for projected weighted H.
4. Confidence: velocity changes beside carried sigma/covariance, residual bias
   and sensor timing checks. Small conditional covariance can coexist with a
   wrong registration basin. Missing evidence is a separate alert state.
5. Scene: a short raw sensor-frame slice beside pre-loop/final map context and
   the trajectory. Show time, window, estimated distance and coordinate frame.

Alert families should remain separately visible: sustained speed/acceleration
growth; growing motion already present in prediction; heading/bias changes;
uncertainty inconsistent with updates; support/association change; incomplete
logging. Persistence and agreement can raise severity, but requiring every
signal would miss slow drift. Numerical persistence thresholds must be evaluated
against retained quiet periods and both known episodes before being promoted
to operational defaults. Do not train/tune solely to reproduce these two events.

Each event should link to a proposed 20-second pre-event / 10-second post-event
context packet: log rows, plots, small raw/map extracts, settings/build identity
and a reason for the alert. Those durations are a starting design choice, not
measured optimums. Capturing lightweight context does not recover heavy solver
traces that were never recorded. General trigger-driven detailed tracing would
require a separately designed bounded buffer or a later authorized replay.

The first version should report and preserve evidence only. It must not silently
alter weights, remove points, accept/reject solver steps, restart or stop mapper
processing. Any protective control requires separate design and authorization.

## Optional viewer

Rerun is a plausible visual front end: its documented timelines and visible-time
ranges support synchronized time navigation and trails/accumulated point clouds.
This is a convenience layer over our evidence, not a SLAM validator.
[Rerun timelines](https://rerun.io/docs/concepts/logging-and-ingestion/timelines),
[visible time ranges](https://rerun.io/docs/howto/visualization/visible-time-ranges).

Rerun, Open3D and evo Python modules were absent in the current environment;
NumPy, SciPy, Matplotlib, laspy and lazrs were present. No package was installed.
Existing Python plus CloudCompare can supply the initial report without another
dependency. An optional Rerun installation/integration would be a separate step.

CloudCompare's nearest-neighbour cloud distance can highlight local separation,
but is not necessarily a physical point correspondence or accuracy measurement.
[Official distance documentation](https://cloudcompare.org/doc/wiki/index.php/Distances_Computation).
Avoid independently realigning each suspicious fragment in a way that hides drift.

## False positives, blind spots and review

Stairs, legitimate handheld turns, stationary rotation, changed walking speed,
and nearby walls outdoors can trip individual indicators. Short range alone
does not establish a confined indoor room. Slow coherent drift, wrong but
apparently strong matches, moving objects, synchronization/extrinsic errors and
miscalibrated covariance may evade internally consistent diagnostics. Independent
geometry, revisits and references remain necessary for accuracy claims.

Independent Codex review (not Sonnet) supported the stack with the above limits.
Its main corrections are explicit readiness states, separate missing-evidence
alerts, no all-signals-required rule, and no implication that held-out evidence
can be manufactured after this run. High is not needed for routine tooling setup
or monitoring; return to High before substantive causal interpretation of a
validated replay. Replay remains stopped until the user's explicit instruction.
