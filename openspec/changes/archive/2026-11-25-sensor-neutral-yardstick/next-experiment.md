# Proposed phase0b: isolated NDT information response

User subsequently approved this nine-run plan on2026-11-12. Implementation and
evidence now tracked in ../ndt-information-probe/. Phase0a is complete.

One scratch-only multiplier alpha:1,0.3,0.1,0.03,0.01. Scale indoor AND outdoor
NDT Hessian/RHS contributions after point weights and before projection or
assembly. Scale matched-point diagnostic cost consistently. Never scale the
assembled system: that would scale IMU/prior terms too. Keep geometric normal
support unchanged. Record multiplier, scratch diff/build and executable hashes.
No shipped setting, calibration or accepted correction changes.

Validate alpha1 against unchanged control, Release tests and44_short regression;
use frozen synthetic cases for linear NDT cost/Hessian/RHS scaling and unchanged
priors/geometric support. Cover projected/nonprojected and indoor/outdoor paths.
Each multiplier needs fresh odometry; compiled alpha is absent from existing
snapshot-settings hashes, so never reuse another multiplier's snapshot.

Initial bounded collection,9 runs:

| Input | Alpha | Purpose |
|---|---|---|
| Livox2 0–60s | 1,.3,.1,.03,.01 | Primary response curve around early slide/bend |
| Whole Livox1 | 1,.1 | Preselected cross-check, not picked winner |
| 44_short | 1,.1 | Other-sensor/regression stress check |

Historical cost estimate40 minutes processing, plus builds, analysis and images;
revise from actual current timings before launch. Extension to all15 combinations
requires a separate decision if the initial contrast is ambiguous.

Use identical sufficiently high experiment-only wall-time limits for control
and interventions so machine load does not determine iteration count. Keep
use_norm/use_planarity/robust odometry off as in these settings. Existing
projection means diagnostic cost curvature is not the exact projected Hessian;
fixed LM damping changes relative strength when NDT weight changes. Neither
chi-square movement nor a sharper isolated run proves physically correct weights.

Read jump magnitude and carried sigma separately, eligible diagnostic coverage,
bends/speed/bias, convergence/skips, both trajectory stages, matched-region map
shape/revisits/sharpness and before/after views. No short-cut physical closure
verdict. Full matched perturbation and held-out evidence remain acceptance gates;
these9 runs diagnose responses and may yield a preliminary visual candidate only.
