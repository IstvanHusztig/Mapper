# Livox4 gain-only rotation jump: existing-data investigation

> Provenance correction (2026-11-13 diagnostic preparation): the saved control
> log reports alpha=0.1; the saved candidate log reports alpha=1, and its build
> defines beta=1. The earlier �gain-only� label and alpha0.1/beta0.1 candidate
> description are incorrect. Candidate/control differences cannot be attributed
> solely to the AHRS gain fix. The saved candidate jump remains a valid event
> to investigate. See `livox4-rotation-factor-trace.md` for the authorized trace.

2026-11-13. User enabled high effort and requested code inspection, a research
agent and OpenSpec documentation. No new mapping, checkpoint or synthetic runs,
builds, numerical changes or setting changes were performed.

## Finding and its boundary

The gain setter does not directly generate the large rotation. The corrected
AHRS changes the preceding trajectory/map, but its incremental rotation at W298
is almost the same as the control. The joint optimizer then creates a substantial
rotation against that prediction. Its acceptance rule allows this because it
checks solve success and translation displacement, without an angular or
support-aware objective test. The gyro factor is present and strongly violated.

This locates the calculation producing and accepting the bend. It does NOT yet
identify the first offending correspondence, iteration or factor interaction.
Both runs lack those traces. Changing map history means this is not a controlled
single-window comparison with identical incoming state.

## Measurements

Source: executed gain-only worktree `out/ahrs-gain-worktree` at fc73404, frozen
control cbca36d, existing `livox4-ahrs-gain-prefix` bundle. W298 starts at raw
sensor time **107.2746488 s**, or **106.92 s on the run log's relative clock**.
Do not interchange these time origins.

All XYZ values below are rotation-vector components, in degrees. The logged
"yaw correction" is the z component of the world-relative rotation vector;
it is not an Euler-heading subtraction or a wraparound artifact.

| W298 quantity | Control | Gain-only |
|---|---:|---:|
| Saved AHRS intrawindow rotation XYZ | [0.8214, -2.0316, -1.9298] | [0.8209, -2.0288, -1.9387] |
| Saved optimized intrawindow rotation XYZ | [0.330, -1.574, -4.399] | [-0.485, -2.954, -19.806] |
| Optimizer correction z at untouched node164 | -2.4642 | -17.9158 |
| Logged last-node correction z before tail | -2.376 | -16.959 |
| Reconstructed last-node correction z after tail | -2.4652 | -17.9231 |
| Approximate gyro z residual cost on edges0–164 | 161.52 | 10,342.25 |
| Largest edge z residual / nominal sigma | 2.23 | 23.59 |
| z bias update, degrees/s | 0.0352 | 0.2446 |

Prediction reconstruction uses saved AHRS Euler angles and the previous saved
post-tail orientation/bias. The overwritten `motion_model_poses` are deliberately
not treated as the original prediction. First-node correction agrees with the
logged value within 0.000085 degrees control and 0.000206 degrees candidate over
W290–305. This is a consistency check, not a rigorous full-attitude error bound.
Residual costs use the code's XYZ relative-angle convention and configured
time-scaled sigmas; they are not calibrated statistical confidence measures.

At W297 the candidate/control AHRS difference is only about 0.30 degrees, mostly
tilt; their optimized endpoint rotations already differ about 0.90 degrees.
Odometry positions had exceeded 0.5 m disagreement at log-relative 104.20 s.
Thus the gain change is an upstream perturbation, not an isolated change applied
at the failing window. W299/300 candidate gyro z costs remain about 10,281/11,367.

## Code path and exclusions

Locations below refer to fc73404, under `out/ahrs-gain-worktree`:

- `src/infrastructure/fusion_orientation_estimator.cpp:27–40`: public setter
  updates settings, not quaternion. Update calls `FusionAhrsUpdate` with zero
  magnetometer, avoiding vendor startup heading reset. First requested zero gain
  is at raw25.458765 s, well before this event.
- `src/application/lidar_odometry.cpp:200–209`: W298 requests gain zero for
  108/180 samples, versus36/180 in W297. Nevertheless the saved incremental AHRS
  rotation differs only slightly. W298 raw gyro norm peaks at24.49 degrees/s and
  maximum sample gap is2.37 ms; no raw 17-degree step appears in the AHRS output.
- `src/domain/window_steps.cpp:137–151`: carried bias corrects the initial
  rotation increments. The new W298 bias increment contributes only about0.080
  degrees over the first164 edges; it cannot directly explain17.9 degrees.
- `src/domain/scan_to_map.cpp:843–861`: same time-scaled angular weights in both
  runs. Beta scales translation rows only. Neither a missing gyro factor nor an
  AHRS-driven angular-weight switch explains the event. Raw and saved attitudes
  near W295–300 are far from the XYZ Euler singularity, min|cos(pitch)| about0.999.
- `src/domain/scan_to_map.cpp:475–510,627–652`: each point selects its currently
  containing voxel; absent, invalid or back-facing cells contribute nothing.
  Small pose/map changes can change support discontinuously. Huber limits an
  individual residual's influence but does not preserve support or constrain
  aggregate angular influence.
- `src/domain/scan_to_map.cpp:1208–1215`: first-node anchoring leaves downstream
  nodes free to bend against the incremental gyro chain. At W298 candidate
  node30/60/90/120/150/164 z corrections are approximately
  -1.52/-2.75/-6.42/-10.54/-15.08/-17.92 degrees: the bend develops along the window.
- `src/domain/scan_to_map.cpp:214–248,1290–1328`: the accepted-step guard measures
  only translation from the guess. W298 logged endpoint result-minus-guess is
  [-0.042,0.092,0.005] m while the angular correction is16.959 degrees. All34,600
  steps in the candidate were accepted. Damping follows an iteration schedule,
  not a full-objective decrease test; a small final update is not a correctness
  certificate.
- `src/application/run_odometry.cpp:1116–1168`: diagnostics precede tail
  replacement; the accepted state is carried forward and inserted into the map.
  Node164 already has the large bend, so tail replacement is not its initiator.
  It carries that anchor forward, increasing the candidate last-node correction
  by about0.96 degrees relative to the logged pre-tail value.

The candidate's rematched NDT yaw stiffness falls from1.247e5 at W296 to7.382e3
at W297, and is negative(-2.278e4) at W299. This supports a problematic matching
landscape; it is not a full-system Hessian/observability measurement or proof of
which factor initiated the rotation.

## Geometry at the transition

Existing intermediate-point caches show no ranges above5 m in W290–297. W298
introduces203 points at15–40 m;12,324/14,367 points remain below2 m. The range
change coincides with the angular failure and supports investigating a visibility
transition, but does not identify a specific physical surface.

An approximate occupancy check at saved candidate poses used all previously
inserted cached points (no map rebuilds were logged) plus a superset of seed
points from the first raw second. None of the203 distant points shares an
occupied0.15 m or0.4 m voxel. Below2 m,10,584 points share fine-grid cells.
This does not apply covariance/viewpoint tests and is NOT an exact match count.
Most importantly, the queried poses already contain the bend: zero distant
matches here cannot exclude distant-point influence during earlier iterations.
Do not label distant trees, missing walls or a particular close surface as the
proven cause. The near-surface/coupled-factor explanation remains a hypothesis.

## Next discriminating step — proposed, not authorized or started

Use one candidate prefix with unchanged gain fix, Huber3, alpha0.1/beta0.1 and
the already verified125-second input. Retarget existing diagnostic machinery to
W297–300; the current hard-coded Livox1/Livox2 selectors do not cover this event.
Capture all iterations' pre-step poses, accepted pose/bias/velocity increments,
factor RHS and step contributions, range-binned support/turnover and Huber
weights, plus existing before/after-tail and incoming/outgoing state records.
Ensure angular contributions can be separated from translation in analysis.
No additional parameter sweep or candidate fix should be mixed into this check.

Before launch: review the diagnostic-only diff, freeze identities, select a fresh
external `livox4-ahrs-gain-trace` destination, and obtain explicit approval.
Require the diagnostic replay's first325 snapshot windows to match the existing
gain-only result exactly; otherwise do not attribute new trace behavior to this
event. Existing prefix takes about9.3 minutes; allow roughly10–20 minutes with
focused tracing, subject to measured overhead. Startup must remain included:
the current snapshot cannot restart odometry at W297.

The required question is which factor drives the first large angular increment,
whether correspondence support changes then, and how the gyro penalty is traded
against it. Existing final-state reconstruction cannot recover that iteration
history. Approval of this analysis does not authorize the proposed replay.

## Evidence and review

Reproducible analysis and JSON are archived externally at
`livox4-ahrs-gain-prefix/analysis-107s/`. Script SHA256:
`ad6761d5a4ca3363af55d023e2b34d6c18c8b3772b58386418db6c9e37ec206c`.
JSON SHA256:
`14f9caf14eb6543f34f2a09b91092e6ce43f9ea5d43420436602d7b9bd980914`.
JSON identifies source snapshots/logs/caches/IMU and verified raw-LAZ hashes.

The requested Codex research agent independently inspected code and reconstructed
the angular factor. It confirmed the main finding and required the occupancy
limitation, raw-LAZ provenance and distinct time origins above. No external-paper
claim is needed for these repository-specific findings. No repeated research
round or new mapping run was performed. Charlie remains at2/19 tasks; the
initiating-factor gate is still open. High effort remains appropriate for the
next causal attribution; routine replay monitoring can use medium.
