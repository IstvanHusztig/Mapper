# Caller audit and separate AHRS gain correction

2026-11-13. User requested a thorough review of data and all nearby influences
on velocity/rotation. Charlie stays the larger-roadmap checkpoint. No recording
replay was performed. Independent code and existing-data reviews were used.

## Earlier onset localized

Approximate pre-LM reconstruction locates concentrated tilt growth in baseline
W298–302 (106.92–108.36 s), before speed runaway. At node 164, which the 15-node
tail replacement leaves untouched, optimized rotation relative to reconstructed
prediction changes by XYZ rotvec degrees:

| Window | Rotation change XYZ |
|---|---|
| 298 | [-0.440, 0.436, -2.464] |
| 299 | [-0.776, 0.929, 0.636] |
| 300 | [-1.120, 2.363, 0.730] |
| 301 | [-1.701, 1.824, -3.664] |

This rotation comparison does not use the fitted gravity. It still depends on
approximate reconstruction of original AHRS rotation matrices; full rotation
error is not rigorously bounded. First-node logged yaw corrections agree within
0.000285 degrees baseline / 0.000415 degrees gyro4x.

Relative AHRS/carried-gravity tilt rises from about 0.6 to 7.9 degrees over these
windows. Logged offset norms sum to only about 0.007 m/s² over W298–302; the
largest relevant bias update of 0.048 deg/s over 0.358 s cannot directly explain
multi-degree within-window pose changes. Earlier W50–250 horizontal disagreement
has median about 0.13 m/s², versus 1.33 over W306–320. Gyro4x reduces this local
tilt to about 4.6 degrees but still fails the full route. Thus the gravity
experiment addresses an amplification path, not demonstrated initiating matches.

## Caller findings

| Influence | Finding and disposition |
|---|---|
| AHRS gain adapter | Confirmed bug: caller requests gain zero outside 0.9–1.1 g, but adapter changed settings.gain directly while Fusion uses rampedGain. Corrected separately below. |
| AHRS startup | Direct gain assignment also left ramp slope at vendor-default 0.5 rather than configured 0.05. Corrected via public setter. Value-initialized Fusion state because vendor initialization reads initialising before Reset. |
| AHRS physical limitation | Magnitude gating does not identify all horizontal acceleration; AHRS tilt is not ground truth even after adapter correction. |
| Bias and tail | Tail applies bias increment, not full bias: no obvious double subtraction. Optimized tilt growth already exists before tail. |
| Velocity carry | Joint mean/covariance committed before tail; existing measured tail-vs-joint speed discrepancy was small. Accepted solve creates next prior. |
| Offset/scale | Legacy scale changes with carried gravity norm; candidate uses separate offset/fixed scale. Small local offsets do not explain observed tilt jump. |
| Degeneracy projection | Active thresholds 0.1/0.2 suppress NDT translation directions using unweighted normal information; rotation is not similarly protected. Candidate does not fix incorrect angular matches. |
| Solver acceptance | Successful solve plus translation distance from guess; no complete support-aware objective or angular bound. Known weakness, not new causal proof. |
| Covariance acceptance | Marginal covariance can be written before step acceptance; rejected mean can retain changed covariance. Livox4 reported all steps accepted, excluding this branch as its observed trigger. |
| Carry uncertainty | Pre-update undamped marginals omit full carried correlations. Formal velocity sigmas remain millimetres/second during growth; these are not calibrated confidence. |
| Map insertion | Uses post-tail poses, so accepted distortion can enter subsequent matching. Exact factor/support cause at W298–301 remains unresolved. |

## Narrow implemented correction

Main repo `FusionOrientationEstimator` now owns public Fusion settings and calls
`FusionAhrsSetSettings` on reset and gain changes. Public rejection values remain
in degrees; transformed internal thresholds are never fed back into the setter.
Convention behavior is preserved. This intentionally changes AHRS numerics and
needs recording validation; it has not been adopted as a demonstrated map fix.

Actual-adapter regression tests cover zero/restored/zero gain behavior, startup,
reset and NWU/ENU/NED against the vendor's public API. Focused standalone builds
of production adapter plus vendor sources pass Release (/O2 /fp:precise) and
Debug (/Od /RTC1): 2 cases, 26406 assertions each. The behavioral case compiled
against the original adapter fails two assertions, as expected. These are focused
adapter tests, not the full application suite or a completed mapping validation.
Tests are registered with the normal CMake test target.

At gain 0.05, native proof after warmup and two seconds of acceleration [0.5,0,1]g
with zero gyro/requested gain zero: original orientation changes 2.45195 degrees;
public setter changes zero. On Livox4 raw IMU the caller requests gain zero for
44261/183137 samples (24.17%), including 621/1999 (31.1%) in 106–110 seconds
relative to first IMU. These counts show relevance, not that it caused the onset.

## Separate experimental work and next gate

Scratch AHRS-gravity experiment stays default-off, uncommitted and unaccepted.
Its first Release build passed 5 cases/27 assertions; related legacy integration
and snapshot checks passed 6/71. Production multiwindow carry, candidate-specific
residual finite differences and full legacy equivalence remain incomplete. A
trace-schema rejection edit postdates that build. Do not use this binary as a
fully verified candidate, or mix the adapter correction into the same comparison.

Next recording proposal should isolate the proven gain correction first, with
frozen Huber3 weights and a prefix retaining original startup/initial map. A new
run still needs explicit user approval. Evaluate the W298–302 rotations and
later velocity buildup, not only endpoints. This record does not authorize one.

Evidence: `out/isolation_onset_audit.py`, `out/isolation_onset_*.json`,
`out/isolation_gain_requests.json`, `out/isolation_gain_probe/`, and
`out/fusion-adapter-{release,debug}/` (logs and actual regression binaries).
