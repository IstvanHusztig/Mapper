# Rotation and objective acceptance: experimental implementation

Status: user authorized implementation; code and tests implemented in the scratch
worktree. Original Release validation passed; approved replay stopped at W1.
Compatibility correction validated and approved replay running; no completed cloud yet. Retains cell
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


## First approved replay: early validator compatibility failure

Original candidatebe898d6 attempted once, nativeexit1 after23.262s, before the
first solve/window publication. No result cloud. Log SHA256
55bad049942866fa5496cac5dc69d76629538759588fc7c7dc795a64d243628e.
Fatal message `Nonfinite acceptance base state` conflated finite nonrigidity with
NaN. This was a newly introduced validation bug, not evidence about the sensor
or a measured flyaway. Preserve the failed bundle and do not retry automatically.

Code evidence: calculate_trajectory promotes Fusion Quaternionf to Quaterniond
without normalization; Fusion uses approximate inverse square root. Existing
imu_kinematics::rotation_of explicitly accommodates nonorthonormal AHRS matrices.
The zero carried bias path does not normalize initial poses. Existing control
snapshot's saved initial m_g has orthogonality Frobenius error1.4495253384062551e-5
and determinant error1.0249691962949115e-5, already above the new1e-8 check.
No raw replay is needed to demonstrate this incompatibility.

Correction: bounded representation tolerance0.01 for both Frobenius orthogonality
and determinant error, positive determinant, finite checks separate from rigidity.
This is an input-representation tolerance, not a trajectory accuracy limit. A
100,000-sample float emulation of the bundled normalization polynomial on squared
norm[0.5,2] gives q-norm-squared[0.99869454,1.00129914], worst rotation determinant
bound0.005215 and Frobenius bound0.007375. This sampled envelope motivates the
margin; it is not a proof for arbitrary malformed upstream matrices. Gross2I
scale remains rejected. Geodesic angles use existing rotation_of normalization
on temporary quaternion copies only. Stored states and immutable predictions
remain byte-preserved; no global AHRS or predictor normalization change.

Added actual Fusion float-conversion and saved Livox4 initial-orientation fixtures
for bounded representation, bit preservation and known relative-angle measurement.
The existing Euler update reconstructs exact rotations, so scale tending tozero
need not reproduce a nonorthonormal base matrix exactly; external merit can still
stall explicitly on this representation change. Do not conceal that limitation
or describe compatibility repair as a flyaway cure. Policy hash revisionv2.
Build/test/review of the correction in progress; no second replay authorized.


Independent compatibility review confirmed the bounded tolerance/temporary
normalization adjustment. It also exposed a separate representation distinction:
the old Euler assembly RHS and raw-matrix merit differ on nonorthonormal input.
The original nonzero-point regression correctly rejected instead of becoming
stationary; the revised tests preserve that fail-closed case and separately use
zero points/targets to verify stationary bit-preservation and geodesic measurement.
Stationarity now requires an orthonormal base (1e-10 numerical tolerance) or an
exactly zero nonnegative physical merit, plus the original small full RHS test.
This conservative condition passed independent review. Final build6 pending.
Python9checks pass, including actual first-window native failure returning
INCOMPLETE/exit2 with its cause. No second replay started or authorized.


## Compatibility correction validated; second replay approved

Scratch commit16e4ae7, clean. Release build6 complete;9focused cases/620assertions
pass; full CTest147cases pass,3skipped, existing3expected consistency assertions
remain (79,392assertions total). Python9checks pass. Independent narrow review
found no blocker in the bounded representation/angle/stationarity correction.
No Debug build or shipped-default change.

Fresh bundle `livox4-step-acceptance-125s-v2`, same125s input and identical settings
as the failed attempt. Executable SHA256
20ba76a2e8cf9d23bf9e3b0d07bfe180d01500f225f12ea6c252541c7abc98b0.
Input identities rechecked; frozen validation logs and checker scripts included.
User approved `Run the corrected replay`; process14700 launched once, with
prepared hashes, exclusive dMapper/cache ownership and free space checked. Native
run.json/manifest statusrunning. Monitor and collect; no third replay authorized.
Failed original bundle remains intact. Medium is sufficient for monitoring;
high is appropriate for interpreting any unexpected acceptance failure.


## Corrected replay outcome: bounded rejection at 108 seconds

Source 16e4ae7; native exit 1 after 2974.488 s (49.57 min). Completed
300/346 windows. W301 starts at log-time 108.00 s and accepts nine local
iterations; iteration 9 exhausts six damping attempts times eight scales on
prediction_rotation_bound. No snapshot or finished cloud was written. No third
replay authorized. Guards and refreshed matches remain enabled.

30,009 accepted updates pass the audit: finite resolved merit reduction, factor
sum, per-step/cumulative rotation limits, covariance label, and unchanged
source/target identities within each retry sequence. Maximum accepted step
0.977744883 deg; maximum accepted correction 2.999868063 deg. 108 trial
rejections: five per-step rotation and 103 cumulative rotation; one terminal
cumulative rejection. Final 48 trials all exceed the cumulative cap. Smallest
last trial step 0.000645466 deg would reach 3.000430104 deg. after=inf means
angle rejection skipped cost evaluation, not numeric overflow.

Last completed W300 yaw correction -2.775 deg; maximum completed end speed
1.24728 m/s. Control W301/W302 were -8.600/-9.095 deg. Candidate W301 has no
completed/exported result. Its nine accepted local updates are not a completed
map insertion; the rejected candidates are not committed (code/test evidence,
not a post-rejection full-state hash). No trajectory recovery is demonstrated.

W301 iteration 0 merit decrease 153765 is mainly IMU -151726.6 plus NDT
-2501.8, with motion +423.0. Do not attribute the first correction solely to
NDT. Iterations 1-8 have NDT as the only decreasing physical component on each
accepted coupled update. At i1: NDT -1162.77, motion +292.49, IMU +8.13,
velocity +49.78, bias +0.104; total reduction 812.26. At i8: NDT -5.72475,
motion +0.6160, IMU +0.0271, velocity +0.1035; reduction 4.97761, correction
2.999868 deg. Thus the tested merit rewards continued updates toward the cap.
These are coupled finite-cost changes, not isolated angular factor responses
or proof of physically wrong correspondences. Do not sum across refreshed
contexts as one objective. The 3-degree bound is provisional, not physical truth.

Independent Codex review trace_review confirmed these counts, angle gates,
cost interpretation and limits. Frozen external livox4-step-acceptance-125s-v2
contains run.json, run.log, run-health.json/txt and acceptance-analysis.json.
Run-health INCOMPLETE/exit 2; no map/overlay/robustness measurement possible.
Log SHA256 62f95ec01177f4f26c2599071c01537a5a1b8088ba8b48dfeb20120c0e54fb0c.
No push/default adoption. Charlie remains 3/19. No process remains running.

User now requests careful existing-run/code examination of where the cap is
exceeded, which values enter and why. Continue that investigation; no new replay
or algorithm change is authorized by this request. Current high is sufficient.
