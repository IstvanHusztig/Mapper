# Rotation and objective acceptance: experimental implementation

Status: user authorized implementation; code and tests implemented in the scratch
worktree, Release validation passed; **not replayed**. Retains cell
guards and refreshed matches. This
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

- [x] `src/domain/scan_to_map.cpp/.h`: shared complete residual evaluation and
  fixed trial context, including fallback/regularizer semantics.
- [x] Same domain: private coupled trial, finite/SO(3)/translation gates,
  backtracking, acceptance result, atomic state/covariance commit.
- [x] `src/application/run_odometry.cpp`: accepted-iteration refresh, same-set
  retry and explicit failure/convergence handling; no unconditional refreshing
  or false-success delta on rejection.
- [x] `odometry_config`, `optimizer_options`, `toml_io`, snapshot identity:
  default-off settings, validation, roundtrip and compatibility.
- [x] Structured per-iteration logs: current/trial merit and factor components,
  actual scaled/clamped step, rotation from current/prediction, accepted scale,
  rejection reason, support identity, covariance linearization identity.
- [x] Existing verify-run: recognize coverage/rejection summaries after logging
  format exists. This reports behavior; it does not implement acceptance.
- [x] Release build plus meaningful synthetic acceptance tests and independent
  code review. Debug only if changed behavior warrants it.
- [x] Freeze concrete candidate build/settings and request replay approval.

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


## Implementation and review (2026-11-14)

The user subsequently authorized implementation, tests and independent review.
Implementation is in `C:/Repo/dMapper/out/ahrs-gain-worktree`, based on71e2fa5.
`step_acceptance.cpp` owns private trials and atomic publication. Shared residual
costs remain beside assembly in `scan_to_map.cpp`; objective-only mode returns
before covariance calculation/solve. Requested IMU/time-dependent factors fail
closed on malformed shapes/timing. Nonfinite and nonrigid rotations are rejected.
The existing NDT validity checks and between-iteration refresh remain unchanged.

New STEP_MERIT and STEP_ACCEPTANCE diagnostics report fixed support identities,
factor costs, trials/retries and terminal outcome. Candidate `tools/run_health.py`
includes acceptance coverage for the entire log, even when ordinary metrics are
time-filtered. It preserves exhaustion before a window summary, including W1.
The existing verify-run workflow uses this tool; no new skill was created.

Independent Codex reviewer trace_review (not Sonnet) found and prompted fixes for
snapshot-property hash placement, target identity, proposal-rejection visibility,
unnecessary dense objective probe allocation, stationary information reporting,
malformed factor inputs, and unfinished-window diagnostic retention. Tests were
strengthened for coupled covariance rollback, exact clamped/backtracked publication,
legacy velocity derivatives and a genuinely nonidentity projected-zero stall.
Final review found no further optimizer blocker; executed tests remain the gate.

Limits: this safeguard can explicitly stop without producing a cloud. The raw
full-merit stationarity rule does not recognize constrained optima at an active
bias clamp; these may report rejection. Accepted covariance is the base proposal
linearization, not recalculated for the accepted trial. Additional cost from
complete merit evaluation has not yet been measured on a recording. No robustness,
map improvement or general adoption is claimed. Current high effort is sufficient.


## Completed validation and frozen candidate

Implementation commit `1f0abc5`; deliberate serialization golden update in its
own commit `be898d6` (current scratch HEAD). Working tree clean. No push.

- Release dMapper and test executable built successfully. Existing vendored
  doctest warning C5285 only; no Debug build warranted by this numerical change.
- New acceptance tests: 8 cases /597 assertions, all passed. Includes coupled
  and legacy/fallback gradients, backtracking and bias clamp, full-state rollback,
  accumulated cap, physical heading across pi, malformed/NaN inputs, stationary
  outcome, projected-zero stall and roundoff rejection.
- Full CTest: passed in12.98s. Doctest146 cases passed,3 skipped;
  79,369 assertions include3 expected failures in the pre-existing consistency
  should-fail cases (not new failures). Initial CTest exposed only the three
  golden TOML comparisons; reviewed diffs add exactly the five default-off
  acceptance fields to each fixture, then full CTest passed.
- Python acceptance diagnostics: 8 tests passed, including an exhausted first
  window and an unfinished later window. Existing125s control log still yields
  the documented W97 eligibility ambiguity/INCOMPLETE result; the new checker
  does not turn historical missing evidence into a pass.
- Independent source review: no remaining optimizer blocker found. Review fixes
  and limitations are listed above. No recording, cloud, perturbation or golden
  recording equivalence check was run; those remain separately gated.

Frozen folder:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-step-acceptance-125s`.
`manifest.json` has status `prepared_not_run`, approvalfalse, exact argv/environment,
source identity and14 artifact/input SHA256 hashes. Executable SHA256:
`09f9ae113ab8a3712cd011385d1b0b03a71571257a12b524008d343e67854c88`.
Input LAZ/IMU hashes were rechecked against the existing125s control manifest;
no raw data were copied or recut. Only the five acceptance settings differ from
that control TOML. Huber3, cell guards, refreshed matches and all other settings
are retained. Validation logs and both checker scripts are frozen alongside it.

Replay approval requested for this125s prefix, checking31s translation and108s
rotation regions and completeness before cloud review. Control cost28.31min;
new overhead NOT MEASURED, provision roughly45�120min or longer and6GB. Rejection
may stop processing early, explicitly, without a finished cloud. No replay has
started. Map quality and robustness NOT MEASURED. Charlie remains3/19.
Routine monitoring can use medium; retain high for interpreting an unexpected
rejection or trajectory failure. No higher model/effort was required to implement.


User approved the prepared125s replay. Native process19688 launched once, with
manifest hashes/cache ownership/free-space checked and all unrelated DMAPPER_
environment overrides removed. Bundle manifest/run.json now record approval and
running status. Monitor this run; do not retry or start another without approval.
