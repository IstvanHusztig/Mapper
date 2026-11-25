# Instrumented Livox1 results

2026-11-12. The authorized logging implementation and BOTH full Livox1 checks
are complete. No additional mapper runs or corrective algorithm changes.
Scratch source commit cb6ea83 on codex/phase0b-ndt-information; nothing pushed.

## Reproduction and validation

| Check | alpha1 | alpha0.1 |
|---|---:|---:|
| Mapper exit |0|0|
| Mapper time |760.16s|666.91s|
| Orchestration time |773.05s|680.59s|
| Pre-loop maximum difference from original |0.0m|0.0m|
| Final maximum difference from original |0.0m|0.0m|
| Merged LAZ |Bit-identical|Bit-identical|
| Traced iterations |3000|3000|
| Projection-reduced iterations |0|0|
| Rejected steps in traced intervals |0|0|

Each stage compares46,980 exactly timestamp-matched poses. All30 traced windows
contain100 iterations and incoming/outgoing carry records,246,390 records per
run. Payload dimensions/finiteness and actual-versus-recomputed assembly match
identities pass. Original raw files and exact archived TOMLs verified. Executable
bundles are immutable and fingerprinted; the build patch hashes exactly match
the committed4d1ea34..cb6ea83 diff. Original results preserved.

Release control:123 cases pass,3 skipped,3 known expected-failure assertions.
Alpha0.1 Release focused assembly/parallel/trace checks:3 cases,864 assertions.
Alpha0.1 Debug trace parity:1 case,816 assertions. The trace test covers active
IMU/gyro, both grids, serial/parallel, accepted and forced-rejected steps.
Independent Codex review (not Sonnet) found no blocking instrumentation issue.

## Why the slower Y progression persists

The direct evidence is an altered balance between scan matching and the
motion-model/carried-velocity terms. Consider a common world-Y velocity ramp:
translate node k by delta_v*(t_k-t_0) in Y and shift BOTH endpoint velocities by
delta_v. This keeps acceleration residuals essentially unchanged while testing
the scan and motion-continuity constraints along the observed separation.

At window107 (38.16s), iteration0, assembled directional normal-matrix values:

| Contribution | alpha1 | alpha0.1 |
|---|---:|---:|
| NDT |1,192,796.52|98,391.94|
| Motion edges |73,073.57|73,073.57|
| Carried start velocity |70,670.38|50,630.16|
| NDT / (motion + carry) |8.298|0.795|

These are the normal-equation convention, not the old finite-difference cost
stiffness diagnostic and not a marginal uncertainty or a scalar gain. Coupling
and different evolving maps remain relevant. Nevertheless they directly show
that the candidate's scan constraint is weaker than the combined continuity
constraints in this direction, while the control's scan constraint is much
stronger. At window108 the ratios are8.676 versus0.698; window110,7.507 versus0.666.

At candidate window107 iteration99, directional RHS terms are:

| NDT | Motion edges | Carried velocity | Accelerometer |
|---:|---:|---:|---:|
|-5969.716|+3697.354|+2270.484|approximately0|

Scan matching still asks for negative-Y progression; motion and carry oppose it
and almost balance it. End-Y velocity settles near-0.2543m/s versus control
-0.4908m/s. This is stronger evidence than simply observing a small end sigma:
the restoring terms and scan pull are directly present in the assembled system.
It is a balance in one tested direction, not proof of full optimizer convergence.
The slower end state then initializes the following windows, sustaining the lag.

Both runs follow a transient slowdown differently at windows103â€“106, before the
persistent5cm separation at38.324s. Control also makes abrupt corrections and is
not ground truth. Traced maps already differ at window100; this is not a frozen-
state ablation isolating the sole initiating event.

## Why the later failure is not simply accelerometer-driven

Large iteration0 accelerometer model improvements initially looked dominant.
Independent review showed an important confound: the solver initializes end
velocity equal to start velocity, so the first step adjusts that auxiliary state.
For candidate window151 the accelerometer RHS-dot-step end-velocity component
is about+1.112million, while translation components sum to-195,863. Its large
first-step number is NOT proof that it drives the pose drift. The common velocity
ramp above is approximately a null direction of the accelerometer factors.

Later iterations show scan-driven local updates that pay motion/IMU penalties.
For candidate window151, summing iterations1â€“99 local quadratic model changes
q=step^T H step -2 RHS^T step gives:

| Factor | Sum of local q |
|---|---:|
| NDT |-189,824|
| Accelerometer |+8,539|
| Motion edges |+38,452|
| Carried velocity |+16,805|

Window149 has the same qualitative pattern: NDT-192,130, accelerometer+731,
motion+32,543, carry+151. These are successive local models along proposed
unclamped steps, not changes in a complete nonlinear objective. They show that
later corrections cannot honestly be summarized as 'the IMU overwhelms LiDAR'.

The candidate's already-diverged motion and local map are being reinforced by
local scan updates while support deteriorates. At window149 indoor matches fall
25,766 to22,916 and outdoor25,227 to23,205 across the window. Final indoor matched
fraction at149 is61.55% versus control75.42%; at155 it is35.91% versus81.27%.
All30 corresponding source-point fingerprints match between runs; associations
and target-cell statistics differ. Hash differences establish difference, not
how many individual associations changed. Lower matched-only NDT cost can result
from losing support, so it is not a quality or objective-acceptance proof.

Prediction propagates the bad state: at154/155 the guess already moves roughly
1.4â€“1.65m per window and the correction is centimetric. The1m guard checks the
correction against that guess, not accumulated drift. At156 the5m/s velocity
carry guard resets the incoming mean to zero and covariance to1m/s sigma,
explaining the speed dip without repairing accumulated pose/map differences.

### Heading-change chronology refines the support warning

Actual last-pose rotation matrices show candidate window149 turns-0.414deg in
iteration0 to1, then+18.712deg from1 to99. Window151 turns+0.633deg then+14.236deg.
Thus the large turns mostly occur AFTER first-step end-velocity reconciliation.
At151, iterations10 to30 turn+14.923deg while indoor matches INCREASE25,884 to
26,217 and outdoor28,854 to29,597. During those steps the NDT local model favors
the joint updates while the listed IMU/motion/carry terms resist them.

Loss of support is therefore a warning in this run, not a necessary condition
for the runaway; a match-count-decrease-only rejection rule would miss this
interval. These joint-step contributions are not isolated yaw torques. Iteration99
pose is the state before the last tiny update. Both independent reviewer and
root reproduced the distinctions; no additional mapper run was used.

## Findings and limits

Supported mechanism: reduced NDT weight changes the scan/continuity balance,
the two runs take different motion estimates, the candidate carries its slower
state with narrow local covariance, and later scan updates/prediction continue
on diverged geometry with worsening support. A globally reduced weight is not
a reliable correction across these scenes.

Ruled out within the6000 traced iterations: degeneracy projection activation,
solver rejection, skipped-step behavior, and logging changing the result.
Post-processing is not the origin of the large failure. No sensor-damage or
Livox2-mirror explanation follows from this experiment.

Not uniquely isolated: which prior/noise model or correspondence/model decision
first makes the wrong physical choice, or which single code correction will
resolve both recordings. Existing traces sharpen the mechanism, but independent
truth and controlled ablations are absent. Do not promote a fix from the size of
one factor energy. No new mapping runs or algorithm changes are authorized by
these findings alone.

## Artifacts and next decision

All artifacts:
C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox1-instrumented/

- runs/alpha1 and runs/alpha0.1: complete clouds and snapshots; adjacent logs,
  archived TOMLs and verified identity/reproduction records.
- trace-alpha1.json and trace-alpha0.1.json: validated iteration summaries.
- trace-comparison.json and trace-comparison.png: compact comparison and plot.
- source-alpha*.patch/.json, bin/*/identity.json, CMake caches, build/test logs,
  runner.py, livox1_trace_analysis.py and compare.py: reproducibility evidence.

Next corrective design should address the balance and reliability of scan versus
continuity information, including support changes; do not simply lower global
alpha further or blame sensor hardware. The user decides the next correction or
additional ablation after reviewing this evidence. High effort remains useful
for that design; medium is sufficient for inspecting the completed report and
routine execution. No further runs are currently active.
