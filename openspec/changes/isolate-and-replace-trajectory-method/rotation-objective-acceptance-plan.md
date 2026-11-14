# Rotation and objective acceptance: prepared fix

Status: design prepared for implementation; **not implemented or replayed**.
User authorized preparation, retaining cell guards and refreshed matches. This
artifact is the concrete solver change, not a new verification skill. Existing
verify-run will consume its diagnostics. Charlie remains3/19; no new run approved.

## Why this needs a complete acceptance path

W301–302 commit8–9deg corrections because solve success and translation<1m are
the only acceptance gates. Refreshing individually valid cells does not establish
correct correspondence. An angle clamp alone can hide the symptom; the patch must
also evaluate a consistent fixed-support trial and preserve state on rejection.
Current diagnostic cost is explicitly incomplete, so it cannot be reused as a
full-objective acceptance score without completing the missing terms.

## Chosen candidate contract

Keep the guard and refresh behavior. Prepare a default-off acceptance setting in
the experimental worktree, preserving the current frozen executable as control.

1. At an outer iteration, freeze the admitted targets, map state, carried priors,
   motion measurements, factor support, and fallback anchors. Construct the usual
   joint system. Do not refresh matches during a trial/backtracking sequence.
2. Compute a proposal into private state. Scale every solved unknown together
   during bounded backtracking: pose translations/angles, start/end velocity,
   gyro bias and acceleration offset. Preserve the current additive-Euler update
   convention; measure angular distances from actual rotation matrices. Apply
   bias clamping before evaluating the actual trial state and cost.
3. Reject nonfinite proposals, existing translation-bound violations, excessive
   per-trial rotation, or excessive accumulated rotation correction relative to
   the immutable gyro prediction for this window, taking the maximum SO(3)
   distance over **all pose nodes**. Capture that prediction after intended
   carried-bias correction/translation prediction, before the first optimization;
   never overwrite it from accepted trial poses or handoff state. The latter prevents100small
   accepted steps from evading the safeguard. It limits correction to prediction,
   not physical turning already represented by the gyro.
4. Evaluate the same complete fixed-support merit at current and trial state.
   Accept only a finite, resolved reduction in physical fixed-support merit PLUS
   the current-base origin increment penalty (baseline penalty zero). This
   monotonicity is local to one frozen context, not across refreshed sets or
   changed fallback anchors. A roundoff-sized change is not proof
   of improvement. Exhausted backtracking returns rejection, not zero-delta
   convergence. Acceptance/rejection reasons are explicit.
5. Commit poses, velocities, bias, offset AND covariance atomically on acceptance.
   Rejection preserves every incoming value. Covariance initially remains the
   explicitly labeled pre-trial linearization covariance; do not misrepresent it
   as relinearized at the accepted/clamped/backtracked state.
6. Refresh valid matches only after an accepted state change. A rejected retry
   retains the same targets and anchors while increasing damping. Leave global
   window/map publication outside the trial. Repeated inability to accept must
   stop with an actionable result; do not silently output a predictor-only map.

## Objective definition and projection limitation

Complete merit includes robust fixed-target NDT, translation/gyro motion edges,
gyro bias prior, accelerometer factors, carried velocity prior, zero-velocity
priors, acceleration-offset prior, legacy continuity factors when active, and
end-velocity fallback when active. Extract residual evaluation beside the existing
assembly so tests can compare derivatives; do not maintain a second unrelated
handwritten model or read the diagnostic text back into the solver.

The end-velocity fallback differentiates only end velocity. Freeze its start
velocity anchor at the outer-iteration base state during trial evaluation.
Evaluating ||v_end_trial-v_start_trial||² would describe a different factor.

The origin contribution is an increment regularizer anchored at the current
first pose, baseline zero; it is not a prior to the recording origin. Its step
penalty must be explicit. LM damping is separate from physical merit.

NDT degeneracy projection modifies H/RHS. Ordinary nonlinear NDT cost is therefore
not, in general, the objective whose gradient generated the projected proposal.
Chosen treatment: retain the projection as a proposal mechanism and evaluate
actual unprojected fixed-target robust residuals as an **external merit safeguard**.
Do not claim textbook LM, model-consistent gain ratios, or guaranteed descent.
If the projected proposal cannot decrease this merit, reject/stall explicitly.
Do not silently disable projection or substitute a virtual projected cloud cost.
Finite-difference gradient agreement is required with projection=identity;
nonidentity cases require tests of the explicit safeguard/stall behavior.

## Experimental parameters to freeze before replay

Proposed configurable trial cap1deg and window correction cap3deg, measured as
SO(3) geodesic angles. These are provisional safety gates, not sensor-noise bounds
or evidence that3deg is a physically correct correction. Report every cap hit;
never treat a clamped/prediction-bound output as verified accurate.
Backtracking uses scales1,1/2,...,1/128 (eight trials). Failure returns a rejection
reason; permit six damping attempts total per outer iteration (initial plus
five retries, each with at most eight trial scales). Require merit decrease
exceeding 64*machine_epsilon*max(1,sum(abs(current factor costs)),
sum(abs(trial factor costs including origin penalty))). Test cancellation and
roundoff explicitly; this is an experimental numerical tolerance, not a tuned
recording threshold.

Convergence is separate from trial acceptance: a finite base state may terminate
as stationary only when the complete unprojected merit gradient, scaled by
explicit SI parameter scales (1m,1rad,1m/s,1rad/s,1m/s² respectively), has infinity
norm <=1e-6 cost units. This provisional numerical tolerance must be covered by
stationary and nearly-stationary synthetic tests; it is not a map-accuracy limit.
A small rejected step, saturated rotation cap, singular solve, or exhausted
budget alone is never convergence. If projection yields a zero proposal while
the full-merit gradient is not stationary, return blocked/rejected explicitly.

Include enabled state, limits, trial budget and policy revision in persisted run
identity. Disabled legacy settings/snapshot identities must remain unchanged.

## Files and implementation checklist

- [ ] `src/domain/scan_to_map.cpp/.h`: shared complete residual evaluation and
  fixed trial context, including fallback/regularizer semantics.
- [ ] Same domain: private coupled trial, finite/SO(3)/translation gates,
  backtracking, acceptance result, atomic state/covariance commit.
- [ ] `src/application/run_odometry.cpp`: accepted-iteration refresh, same-set
  retry and explicit failure/convergence handling; no unconditional refreshing
  or false-success delta on rejection.
- [ ] `odometry_config`, `optimizer_options`, `toml_io`, snapshot identity:
  default-off settings, validation, roundtrip and compatibility.
- [ ] Structured per-iteration logs: current/trial merit and factor components,
  actual scaled/clamped step, rotation from current/prediction, accepted scale,
  rejection reason, support identity, covariance linearization identity.
- [ ] Existing verify-run: recognize coverage/rejection summaries after logging
  format exists. This reports behavior; it does not implement acceptance.
- [ ] Release build plus meaningful synthetic acceptance tests and independent
  code review. Debug only if changed behavior warrants it.
- [ ] Freeze concrete candidate build/settings and request replay approval.

## Mandatory tests before recording replay

- Fixed-support nonlinear residual derivatives against assembled RHS with
  projection=identity, covering every active factor and fallback branch.
- Pure-rotation bad proposal with tiny translation cannot bypass rotation gates;
  repeated small increments cannot evade accumulated correction limits.
- Gyro-predicted physical turn is not rejected merely for its absolute heading.
- Full-state finite/equality preservation on rejection, including all covariance
  fields; accepted bias-clamped/backtracked state matches the evaluated state.
- Coupled scaling of pose, velocities, bias and offset; angle wrap near±pi.
- Merit-increasing proposal rejected; resolvable reduction accepted; no-op,
  NaN/Inf, incomplete objective and retry exhaustion not false convergence;
  a genuinely stationary full-merit gradient terminates normally.
- Same targets throughout trials; refreshing only at the next accepted outer
  state; nonidentity projection explicitly exercises merit rejection/stall.
- Disabled-path numerical parity and snapshot compatibility; enabled-policy
  snapshot mismatch, settings roundtrip and invalid-setting rejection.

## Evidence boundary and next gate

This is solver safety preparation, not a demonstrated cure for bad association
basins. A wrong correspondence basin may still reduce the merit; a cap may stop
processing without recovering the correct trajectory. Later validation needs the
31s translation region and108s angular region, with rejection diagnostics and
coverage, then independently approved broader indoor/outdoor/robustness checks.
No recording runs or production algorithm changes occurred during preparation.
Current high effort suffices for this design; reassess before implementation if
complete residual refactoring or mathematical consistency needs higher effort.

## Independent design review

trace_review (Codex, not Sonnet) reviewed the source and proposal. Incorporated:
projection-as-external-merit semantics; frozen end-velocity anchor; atomic
covariance/state; cumulative all-node rotation bound with immutable prediction;
separate stationarity detection; explicit origin penalty; six damping attempts.
No algorithm edits/builds/recording runs occurred during this preparation. The
OpenSpec validation checks document structure only; implementation and numerical
tests remain unchecked above. Do not report this plan as a tested fix.
