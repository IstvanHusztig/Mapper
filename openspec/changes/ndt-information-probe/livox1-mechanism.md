# Livox1 divergence: mechanism and evidence boundary

2026-11-12. User authorized existing-run/source investigation after switching to
Astra High. No new runs, replay, code changes, filtering or setting changes.
Source inspected is the clean recorded scratch commit
4d1ea3477271670355d4366c5295fa12cea76769. Logs are in the external phase0b
experiment runs/livox1/alpha1.log and alpha0.1.log. See livox1-divergence.md for
hashed inputs, exact time convention and independently replayed trajectories.

## Finding

The evidence supports a change in the balance between scan matching and the
carried motion estimate, followed by propagation of the divergent state through
successive predictions. It does NOT uniquely establish which numerical or
matching event first chooses the wrong solution. Control is not ground truth.

At the same frozen state alpha0.1 reduces NDT H/RHS/cost tenfold while keeping
motion/IMU terms unchanged. Real runs then have different states and local maps;
their logged NDT stiffness values must not be treated as a frozen tenfold ratio.

### 1. Onset is horizontal velocity disagreement, not immediate runaway

At window102 (36.36s) both runs have almost identical positions and Y end speeds
(-0.699 control, -0.690 candidate m/s). During windows103â€“106 they respond
differently to a slowdown/heading-change episode. Control itself has abrupt
corrections: window104 Y result move -0.012m versus guess -0.165m, then window105
-0.185m versus guess -0.017m. Candidate corresponding moves are -0.139/-0.209m
and -0.121/-0.090m. Thus there is no basis to call the control path exact truth.

By window107 (38.16s), candidate Y end speed is -0.254 versus -0.491m/s;
window108 is -0.292 versus -0.589m/s. Candidate predicts less Y displacement and
largely preserves that slower progression. The persistent position separation
crosses5cm at38.324s, grows mainly along world Y, and exceeds0.5m at40.822s.
Candidate heading corrections at windows107â€“113 are small, so a large new yaw
jump does not explain the initial persistent separation.

The logged least-constrained direction is approximately Y. However configured
projection thresholds are0.1/0.2, and final logged ratios here are about0.4â€“0.5.
Those final geometries would retain all directions; the0.45 diagnostic logging
cutoff is NOT a projection cutoff. Earlier per-iteration projection states were
not logged. Do not claim observed Y suppression by projection.

### 2. A divergent velocity is repeatedly used as the next prediction

run_odometry.cpp:201â€“228 prepares start velocity from the previous optimized
end state. Covariance is carried with noise added over the inter-window gap.
Lines237â€“244 retain end velocity/covariance and fold estimated acceleration
offset into the carried gravity vector. Lines1003â€“1004 initialize translation
from that velocity. scan_to_map.cpp:854â€“863 adds its inverse-covariance prior;
lines912â€“937 extract the next covariance from the undamped assembled system.

Candidate weak-axis prior sigmas remain around0.0044â€“0.0046m/s at onset despite
centimetres-per-second changes and large reported chi-square. The unchanged
motion-model stiffness is also present. This supports a tight, self-carried
state as a propagation mechanism. It does not independently calibrate covariance
or prove that the prior alone overpowers all scan information. Logged NDT
curvature is often larger and is measured along a different perturbation from
the full coupled solve. Both control and candidate show uncertainty mismatch.

### 3. Runaway adds heading/bias changes and is inherited by later guesses

Window149 (53.28s) candidate heading correction is18.258deg (control0.977deg).
Candidate residual Z gyro-bias estimate changes from -0.625 to -1.037deg/s;
these are fitted states, not evidence of sensor damage. Speed exceeds2.5m/s at
window151,54.00s. Acceleration-offset changes also grow later, but onset offsets
are only millimetres/s^2 in the rounded logs; they do not establish an initiating
acceleration fault.

At window154, candidate guess move[-1.423,0.491,-0.205]m becomes
[-1.397,0.471,-0.214]m. At155, guess[-1.648,0.668,-0.283]m becomes
[-1.644,0.654,-0.265]m. The already-large prediction is accepted with only small
corrections, and end velocity sigma remains0.00441/0.00473m/s. This directly
explains propagation without a fresh metre-sized correction at each window.

At window156 (55.80s), the previous-pose velocity exceeds the5m/s guard
(log5.35612). Optimized carried end speed also exceeds5m/s. The prior falls back
to the initial case; guess move drops to[-0.160,0.120,-0.010]m and end speed to
0.849m/s. That explains the temporary speed dip as a guard/reset, not restored
map accuracy. The preceding wrong positions/maps are not undone.

### 4. Existing acceptance logic does not certify a good fit

scan_to_map.cpp:90â€“96 increases LM damping on a schedule and rejected steps.
Lines970â€“1008 apply solved steps if every pose stays within1m of its motion-model
guess; there is no before/after total-objective decrease or gain-ratio test.
The bound is relative to an already-divergent guess, not a global drift bound.
Log: zero solver skips and zero big-jump rejections in the candidate. All windows
reach100 iterations. Small last deltas under increased damping are not proof
of convergence to a physically correct solution.

This is an existing vulnerability, not proof that a particular accepted step
increased total cost. LM factors and per-iteration term costs were not retained
(log_iteration_convergence=false). It would be incorrect to diagnose one exact
bad iteration from these end-of-window records.

## What the NDT profiles can and cannot show

Candidate windows107/108 report lower NDT-only costs at a nearby -0.05m shift;
window149 has negative finite-difference NDT velocity/yaw curvature. Those are
warning evidence of a locally irregular matching landscape, not a proof of a
wrong total-objective minimum. ndt_window_cost (scan_to_map.cpp:99â€“145) skips
unmatched/invalid cells and re-associates points at perturbed poses. It neither
penalizes lost matches nor includes all priors. Rigid profile shifts also move
the strongly anchored origin. Matched counts/identities are absent, and both
runs build different maps from their previous estimates.

## Alternatives and causal confidence

- Supported: tenfold relative weighting intervention; changed velocity/pose
  estimates; carry/predict propagation; later heading/bias/velocity runaway;
  speed guard explaining a temporary dip; failure already pre-loop.
- Plausible contributors, not separated: tight velocity/motion priors, changed
  associations/local-map feedback, fixed damping and missing objective-based
  acceptance, acceleration/attitude coupling and earlier carried-state history.
- Not established: a single defective sensor, mirror cause, active final Y
  projection suppression, loop closure as origin, or a globally correct alpha.
  Livox2 mirror annotations cannot be assigned to Livox1 without evidence.

## New-run boundary â€” not executed

To distinguish the initiating solver/matching event from propagated symptoms,
new diagnostic runs are needed: the existing snapshots retain final odometry
states, not historical per-iteration maps, residuals or accepted steps.

Proposed bounded next experiment: two fresh deterministic full Livox1 diagnostic replays
(alpha1 and0.1), preserving the original input/settings and100-iteration behavior,
with extra logging limited to windows100â€“115 and143â€“156. Rebuild maps from the
start, not from a final snapshot or a mid-walk cut. Record per-iteration matched
counts/identities, NDT and prior residual costs, state updates, damping, actual
projection factors, velocity/gravity/bias carry and solver acceptance reason.
Keep unmatched coverage separate from apparent cost reduction. Check replay
trajectory identity against the retained outputs before interpreting new logs.
No algorithm correction in this diagnostic step. Root source/settings unchanged.

Instrumentation implementation and these two runs require the next explicit
user decision. No replay or additional mapper run has been started.


## Independent review

Independent Codex reviewer (not Sonnet) confirmed the source paths, narrow
covariance carry/prediction mechanism, projection-threshold distinction and
window156 reset. Review also confirms covariance-before-update ordering has no
demonstrated rejection-related role here: no rejected steps were reported.
The final conclusion deliberately distinguishes established propagation from
unidentified initiation; no corrective algorithm is selected.
