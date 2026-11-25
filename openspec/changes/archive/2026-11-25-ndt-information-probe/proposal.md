## Why

Phase0a established a current-build18.67m pre-loop displacement from a1mm
decimation change. We need a controlled intervention to distinguish how scan
matching weight affects actual velocity jumps versus claimed uncertainty.

## What Changes

- Scratch-only compile-time NDT information multiplier, recorded in logs/build identity.
- Consistent indoor/outdoor Hessian, RHS and diagnostic cost scaling; unchanged priors.
- Nine approved diagnostic runs with stage-specific measurements and preliminary views.

## Capabilities

### New Capabilities
- `ndt-information-experiment`: isolated, reproducible controlled weighting experiment.

### Modified Capabilities
None. No shipped setting or accepted algorithm change.

## Impact

Scratch checkout C:/Repo/dMapper/out/phase0b-worktree on
codex/phase0b-ndt-information, based on8d230a1. Root OpenSpec is the planning
source of truth. New dependencies none. Application changes stay in scratch.
User explicitly approved the nine-run plan2026-11-12; no additional confirmation
needed for its implementation/tests/runs. Candidate acceptance/push remain separate.

Follow-up explicitly authorized: diagnostic logging plus two full Livox1 replays
(alpha1/.1) to inspect the onset/runaway without changing settings or algorithm.

User subsequently approved next-step-design.md: finite motion XYZ information beta0.1 with NDT alpha0.1 fixed, gated tests/review and two full-start Livox1/Livox2 first60s runs. No shipped defaults or acceptance changes.

On November13 the user approved the two isolated W754 checks described in
full-livox2-cell-attribution.md: exact control, then a three-outdoor-cell ablation
only if control reproduction succeeds. The latest instruction is to finish these
checks, save the state and stop for later continuation. No full replay or fix is
authorized by this step.

The user then resumed work and explicitly authorized the next isolated exact
W754 control plus Huber3 candidate after preparation/review. See
full-livox2-robust-design.md. No additional full-recording replay is included.

After the successful Huber checkpoint, the user explicitly authorized the fresh
full Livox2 Huber3 replay and continuing toward the goal until new information
is required. Preserve alpha/beta/settings, isolated build/provenance and review
gates. This supersedes the prior pause/new-run approval boundary for the authorized
continuation; it does not authorize pushing or accepting survey accuracy.

## Retained Huber3 postprocessing attribution (2026-11-13)
Full Huber3 mapper completed; see full-livox2-huber-full-results.md. Structural
comparison is verified, while W798 conditional-uncertainty eligibility remains
unresolved and original needs_review/health exit2 are preserved. No acceptance.
User authorized continuation to the goal. Next bounded check in
full-livox2-postprocess-attribution.md reuses exact retained odometry, captures
afterloop and each unchanged consistency pass, and requires exact final
trajectory/cloud reproduction before attribution. No full odometry replay,
shipped settings change, or silent experimental snapshot-guard bypass.

## Exact postprocessing attribution complete; native stage exports next
Phase17 replay exit0,1127.06s; all64numericfiles exact,12stage snapshots verified.
Independent binary-parser review separatesLCmax10.4697m(global)/5.3631mvertical
fromconsistency densepath333.228->408.553pass1->892.159pass10. This isnotphysical
travel or proofwhichcorrectionisright. See full-livox2-postprocess-attribution.md.
Usercontinuationauthorizationcoversboundedstagecloudexportswithoutoptimization:
firstsavedfinalsnapshot mustreproduce64numericfiles, thenLCandodometrystages;
freshcache/originalraw preserved. Phase18 preparesstandaloneexporter/runner.
No fullodometry/newnumericalcandidate, noacceptance. Mediumsufficientforexports/
measurements; usernotified, flagbeforeHigh correctiondesignifuserswitchesdown.


## Existing-data route/scene interpretation (phase19)
The saved-stage geometry is complete and reviewed; phase18 did not fix or accept
full-route accuracy. Prioritize the remaining large odometry-stage displacement.
Before any new optimizer execution, inspect bounded beginning/end scene geometry
and relate raw sensor geometry/estimated motion to known route context. A final
indoor cluster is displaced roughly62m from the start; this is endpoint separation,
not physical closure error without route evidence. User was asked whether Livox2
ended in apartment56, another indoor location or outside. No timestamp recall.
A small bounded chunk-view script is prepared/reviewed for first/last60s; translation
only local panels are shape inspection, never a physical alignment or accuracy
claim. Reuse retained outputs and preserve all source-coverage caveats.


## Route confirmation received (2026-11-13)
User confirms fullLivox2 finished in apartment56, sameapartment asstartWohnen.
No longer waitingforrouteinfo. Exactendpointposition/heightnotknown;62m is
estimatedseparation, notexactsurveyedclosureerror. Grossapartmentmisplacement
is nowconfirmed. See full-livox2-return-diagnosis.md. Continue existing-data
orientation/translation diagnosis withTerra sourcecontracts andindependentreview;
AHRS angles are notpuregyro; always-on processing enables rest checks and a later moving-window acceleration integration despite disabled generic prediction flags.
No newmapperrun currentlystarted; no acceptance.


## Focused online gyro-bias ablation (phase20)
Userconfirmedreturntoapartment56; existingorientationanalysis showslargeAHRS
relative departures by149-207s, beforelatevelocityrunaway. Independentreview
supports onefullstart use_gyro_bias_estimation=false test, keepingstartupbias,
Huber3/alpha.1/beta.1/allotherparsedsettings andexistingfrozenexecutable. No build;
tracingoff, normaldiagnostics retained. See full-livox2-return-diagnosis.md for
mechanism/applicability/acceptance limits. Stop diagnostictool expansion and
prioritize thisboundedcorrectiontest peruser's token-cost feedback.
