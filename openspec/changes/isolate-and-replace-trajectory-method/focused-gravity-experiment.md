# Focused gravity experiment after Charlie

2026-11-13. User authorized focusing on the suspected calculation after saving
checkpoint Charlie. The larger interface/replacement roadmap remains at 2/19.
This detour is an isolated experimental formulation, not an accepted general fix.
No recording replay is authorized by implementation of this experiment.

## Question and candidate

The active always-on path subtracts carried world gravity from rotated raw
acceleration. A mismatch between estimated attitude and that gravity creates a
horizontal acceleration term which can propagate through the joint velocity
solution. Existing-data reconstruction finds an approximately 1 m/s² term in
Livox4's W301–320 buildup; neither AHRS nor the fitted gravity is ground truth.

The existing `world_acceleration` helper already removes gravity using AHRS
tilt for rest detection. The candidate applies that principle consistently to
the always-on predictor and joint accelerometer residual:

`q_k = f_k - g0 * A_k^T * ez`

`a_k = s0 * R_k * q_k - b_world`, with fixed `s0 = 9.80665 / g0`.

Here A is saved AHRS attitude, R is estimated map attitude, g0 is initial mean
acceleration magnitude, and b_world is accumulated acceleration offset in m/s².
Initialize b_world from the first-window mean s0*R*q, retaining startup rest
calibration. The window's additional offset e enters the residual as -e and is
carried via b_world += e, with the existing covariance/random walk. Never
normalize b_world as though it were a gravity vector.

Reuse optimizer algebra with a temporary corrected sample buffer: rotation
Jacobians must use R*q, not R*f. Preserve stored raw samples for rest detection,
exports and independent analysis. Prediction, optimization and diagnostic
residual reporting must use the same candidate convention.

## Scope and risks

Strict default-off scratch activation `DMAPPER_DIAGNOSTIC_AHRS_GRAVITY=1`.
Preserve Huber3 and numerical weights. Fail clearly on unsupported/missing
inputs; maintain method identity for snapshots so candidate state cannot be
silently reused as legacy. Keep numerical legacy behavior unchanged when off.

This transfers gravity-direction authority to AHRS and reduces the ability of
accelerometer residuals to correct map attitude. AHRS uses accelerometer feedback
and can mistake sustained physical acceleration for tilt. Its errors are
correlated with the input samples; existing residual weights are not newly
calibrated uncertainty. Treat this as a mechanism test, not proof of robustness.

## Work and gates

- [x] Review equation, Jacobian, offset sign and startup calibration independently.
- [ ] Implement opt-in predictor/optimizer/diagnostic paths and provenance guards.
- [ ] Verify stationary tilt does not create acceleration, actual acceleration is
  preserved, Jacobians match finite differences, offset carry is consistent,
  invalid activation fails, and the disabled legacy path remains unchanged.
- [ ] Build Release and run focused tests; independently inspect changes.
- [ ] Prepare a concrete bounded recording comparison and cost; request approval
  before any replay. Preserve prefix/state/map equivalence, not a raw cut with
  new initialization. Do not widen to full routes without approval.
- [ ] Present measurements and clouds before a user decision to keep or reject.

Implementation is in the existing scratch worktree; main repo owns this record.
Accepted changes later require the existing HTML documentation and broader
quality gates. Charlie remains the resume point if this detour fails.
