## Context

See proposal and sensor-neutral-yardstick/readiness.md. Isolated scratch checkout;
main application and shipped TOMLs remain unchanged.

## Goals / Non-Goals

Measure response to NDT information scale, not choose final physical weights or
claim a robust candidate from single runs. No snapshot reuse across multipliers.

## Decisions

Compile-time alpha in1,.3,.1,.03,.01. Scale both indoor/outdoor point Hessian/RHS
after existing weights, before degeneracy projection/assembly. Scale
ndt_window_cost consistently. Alpha1 bypasses extra arithmetic. Never scale the
assembled system or geometric sum(n*n^T); covariance then changes naturally.
Record alpha, diff and executable/input/config hashes. Refuse unsupported robust,
norm or planarity paths in non-control experiment runs. High wall-clock cutoff
uniform across controls/interventions; inspect iteration and skip events.

Run matrix: Livox2 first60s all five alphas; wholeLivox1 alpha1/.1;44_short
alpha1/.1. Historical estimate40minutes plus builds/analysis; latest Livox1
control measured6m34s. Preserve every cloud/snapshot/log. Read velocity jump and
sigma separately, chi-square coverage, bend/speed/bias, geometry/revisits/sharpness.
Use matched physical map regions and show preliminary before/after views.

## Risks / Trade-offs

Projection makes diagnostic cost curvature differ from the projected Hessian.
Fixed LM damping changes relative strength. Existing outdoor norm mismatch is
out of scope; keep norm/planarity off. Numerical sensitivity may defeat exact
control reproduction: report it, do not quietly replace baseline. Full-family
and held-out validation remain required before candidate acceptance.

## Existing-run follow-up (user authorized November12)

Localize Livox1 alpha0.1 versus alpha1 in time using retained snapshots, final
trajectories and logs. Distinguish first relative differences, sustained growth
and later runaway. No new mapper runs, filtering, setting changes or root-cause
acceptance. Notify the user before any step requiring higher effort.

User additionally authorized causal investigation of existing runs/source after
switching to Astra High. No new runs may start without advance notice and a
separate decision; distinguish observed propagation from an identified trigger.


## Authorized instrumentation and replay

User approved logging implementation and two full Livox1 runs (alpha1/.1).
Extend the existing scratch checkout; no correction or shipped-setting changes.
Trace only windows100–115 and143–156, using an explicit opt-in switch. Preserve
solver arithmetic, accumulation order and parallel ownership. Record per-iteration
matched support, projection factors, factor contributions, state/carry, damping
and acceptance. Clearly label partial cost diagnostics; unmatched support is
separate. Unit parity and exact retained-run trajectory comparison are gates for
using traces as causal evidence. Build immutable executable bundles and run
sequentially against the original raw Livox1; no snapshot reuse or mid-walk cut.

Artifact root: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/
livox1-instrumented/. Root OpenSpec remains authoritative. Preserve old artifacts.
Current high effort is useful through trace review and interpretation; medium
suffices for mechanical waiting, but do not silently change user model settings.

## Approved finite translation-prior follow-up
User approved next-step-design.md and its two-run matrix. That document governs the isolated beta experiment and required synthetic gates. Prior completed tasks remain historical; section7 tracks the new work.

## Full Livox2 follow-through
After visually liking the new short Livox2 cloud, the user explicitly selected launching the full run. Use the same frozen alpha0.1/beta0.1 binary and exact archived settings on original raw data/livox/2. Disable only optional detailed trace (verified behavior-preserving); retain ordinary diagnostics and snapshots. New external output full-livox2-motion-prior. No new tuning or defaults. User will consider recording a new Livox1 after inspecting this full result.


## Authorized existing full Livox2 diagnosis
After switching to High, the user authorized investigating the retained full run and documenting it in OpenSpec, with advance notice before new runs. Scope is existing log/snapshot/final trajectories/raw IMU and primary-source research only. See full-livox2-jump-diagnosis.md for reviewed findings and proposed replay. No new mapper runs, builds or fixes are authorized by this investigation.


## Authorized surrounding-cloud inspection
User requested Python analysis of surroundings and authorized reusable diagnostic tools. Inspect existing raw sensor-frame range/short snapshots and transformed final-cloud intervals; do not rerun SLAM. Record time/frame conventions, identity and scene-inference limits. See full-livox2-cloud-context.md. tools/cloud_context.py is a read-only-input offline analysis tool; artifacts stay in the external experiment directory.


## Approved full Livox2 focused replay
User explicitly approved focused logging and one full original Livox2 replay after the close-scene inspection. Scratch instrumentation only: separate DMAPPER_TRACE_LIVOX2 opt-in covers windows408-416,730-780,830-885 (116windows), retaining pre-onset lead-ins and both near-surface episodes. Preserve alpha0.1/beta0.1, exact archived TOML and original raw2; no snapshot restart/cut, automatic extra run, filtering or fix. Extend read-only traces with node times/IMU samples and prediction/handoff poses, per-range match support/geometry/turnover, actual projected NDT step contributions by range and final iteration poses. Existing factor/carry/acceptance traces remain. Release trace parity/conservation and full recorded reproduction are required gates. New external artifact root full-livox2-trace/. Notify when High is no longer required for mechanical execution; causal interpretation still benefits from High.


## User stop before replay launch
User explicitly requested stopping before a new run. No Livox2 replay has started. Scratch logging commit5eb882e is built; parity/conservation test passed1668assertions, fullRelease reproduced the same existing assertion failures. Frozen binary is in full-livox2-trace/bin (SHA256 d7915fba00a131f032d0a620fec2f11ae99b5f9bda291e2f8b8d213f61db7673). Runner out/full_livox2_trace_replay.py has not launched and still needs review fixes for bundle inventory, rich trace completeness, archive overwrite protection and capture-error child handling. Do not launch until the user explicitly resumes. Tasks11.2-11.4 remain unfinished; no automatic retry or processing.


## Preparation complete; replay still stopped
See full-livox2-replay-readiness.md for finalized runner safeguards, ten synthetic tests, final independent review and archived preparation manifest. Tasks11.1/11.2 complete; replay/reproduction/causal analysis remain pending explicit user resume. User subsequently objected to standalone HTML presentation overhead and prefers reusable Python diagnostics suitable for later dMapper integration. Do not extend the HTML viewer; no replacement implementation is implied by that feedback.


## Full replay explicitly resumed
User explicitly authorized starting the single full Livox2 replay, monitoring through completion and documenting in OpenSpec. This supersedes the prior launch stop only for this one unchanged replay. Medium suffices for execution and mechanical validation; notify before High is needed for causal interpretation. OpenSpec documentation is a standing requirement and does not need repeated user reminders.


## Full Livox2 traced replay completed
Single authorized replay completed overnight2026-11-12/13: mapper exit0, collected_complete,3993.63s. All63 output hashes reproduce retained full run exactly;116 windows/11600 iterations/23200 assembly identity checks complete, zero mismatches. Trace2,497,193,543 bytes. Run-health triggers unchanged, not an accuracy pass. See openspec/changes/ndt-information-probe/full-livox2-trace-results.md. No jobs remain. Task11.3 complete;11.4 causal analysis pending High. Use streaming Python on existing trace; no additional run required now. User wants quiet monitoring and standing OpenSpec updates.


## Existing-evidence map reconstruction investigation
User authorized continuing analysis. A standalone diagnostic helper may reconstruct the pre-W754 map from retained post-handoff snapshot poses and point caches using unchanged C++ map-update arithmetic. This does not authorize a new odometry optimization run or settings intervention. Require exact source-order, matched-cell and matched-cell-geometry hashes for both grids at all selected iterations before trusting per-return attribution. Fail closed on reconstruction mismatch. Fresh external outputs only; no recording/cache/bundle mutation.

## Authorized isolated W754 checks (2026-11-13)
User switched High and approved the two checks in full-livox2-cell-attribution.md.
Restore exact traced pre-iteration state and priors; use the unchanged production
optimizer and schedule against the independently validated historical map.
Require exact control state/decision reproduction across all100 iterations before
one restart omitting the three named outdoor cells. No next window, full replay,
settings change or shipped fix. Fresh external artifacts and frozen provenance.
User then requested finishing this work, saving state and stopping for later.
Section13 tracks these checks; do not archive the broader investigation.
## W754 causal checks completed; user resumed correction design (2026-11-13)
Exact isolated control reproduced all15,400 trace records/100 iterations. Omitting
only3 outdoor cells reduced estimated endpointtilt15.1886->2.3260deg and endpoint
speed0.8899->0.3555m/s; initial outdoor support loses11 terms only. Both100accepted
iterations, projection scales1. Independent Codex review passed. This confirms
collective causal influence inW754, not physical accuracy or correctedfullcloud.
Report: openspec/changes/ndt-information-probe/full-livox2-w754-ablation.md.
External artifacts: full-livox2-trace/optimizer-w754-control-ablation/ under the
approved experimentroot. Scratchcommit7f99b29; Release/Debugbuilds pass. Nojobs.
User first requested finish/save/pause, then explicitly said don'tstop/goon.
Continue generalcorrection design fromthischeckpoint withsource/primaryresearch;
High stilluseful. Noadditionaloptimizer/fullreplay launched orspecified yet.
Do not generalize hardcodedcellIDs intoashippingfilter. Keepallresults.
## Authorized W754 Huber diagnostic (2026-11-13)
User resumed and explicitly approved the proposed exactcontrol+Huber3 check after
build/tests/review. See full-livox2-robust-design.md. Scratch-only runtimeoption
(default0) in bothgrids, consistent H/RHS/robustcost, allmapcells retained; no
shippedsetting. No fullreplay or extra kernel/scale/checkpoint authorized.
## Huber preparation gates and launched scope
Scratchcommit c4b3b6f implements diagnostic-only Huber3, disabledbydefault.
Release/Debug focused3cases469assertions passed each. FullRelease127cases126pass,
1knownunexpectedcorridorfailure,3skipped; allfour printedassertionvalues match
retainedbaseline exactly. Independent Codexsource/testreview passed. User gave
explicitgo; exactcontrolthenHuber3 launched into full-livox2-trace/optimizer-w754-huber3/.
FrozenexeSHA eb4eedf4a4e9678d0a05fdb8f183a16ff7bfab78b0550414620c7f16cac6f6d7.
No fullreplay. Terra6.1 research challenge saved in full-livox2-robust-design.md.
## Huber3 isolated W754 result (2026-11-13; latest)
Approved exactcontrol+Huber3 completed, exit0,136.93s. Controlall15,400records exact;
candidate100acceptediterations/allprojection1, allcells retained and identical
initialmatches. Tilt15.1886->2.1480deg, endpointvelocitynorm0.8899->0.3739m/s.
Finalsupport13501/12514 ->16666/14705 indoor/outdoor. Initial meanweights~.922/.936
hide stronger directionalinforeduction (~28%/42% smallest summedinfoeigenvalue).
Independent Codex reviewpassed; fullcloud/physicalaccuracy/robustness NOTMEASURED.
See openspec/changes/ndt-information-probe/full-livox2-huber-results.md and
full-livox2-robust-design.md (includesrequestedTerra6.1researchchallenge).
Artifacts full-livox2-trace/optimizer-w754-huber3/ underapprovedexperimentroot.
Scratch c4b3b6f; defaultdisabled diagnostic runtimefield, noTOML/GUI exposure.
Release/Debugfocused3/469pass; fullRelease retainedidentical knowncorridorfailure.
Nojobsremain. Proposednext: opt-in freshfullLivox2 Huber3 run, keepingalpha/beta
andsettingsfixed. Useraskedforauthorization; nofullrunlaunched. Honor anylater
reply. Highusefulforinterpretation; Mediumsufficientforroutinepreparation/waiting.
## Latest authorization supersedes pending full-run question (2026-11-13)
User: "Go ahead until you achive the goal or need nee info from me", and useTerra
researchwhenneeded. FullLivox2Huber3 is nowapproved; do notaskagain. Continue
necessary bounded diagnosis/validation towardusablefullcloud, preservequality
andprovenance, report meaningfulchanges. No push/shippingacceptance implied.
Activepreparation: historical_measurements owns scratch application-level
DMAPPER_DIAGNOSTIC_NDT_HUBER3=1 optin (unsetdisabled) andbuilds/tests; provenance_audit
owns rootout/full_livox2_huber_replay.py +synthetictests. RootownsOpenSpec,
externalfreeze/launch andanalysis. snapshot_format_review reviews beforelaunch.
Terra6.1 researchcompleted, findingssaved. No fullcandidate processlaunchedyet.
Nextartifactroot full-livox2-huber3/ underapprovedphase0bexperimentroot.
NeedfulltraceH3coverage, finite/conservationchecks, raw/settings/bundleidentity,
health/stage/mapcomparison; changedcandidateoutputsareexpected, noexactfullrun
reproductionrequirement. Rootresultcommitpending afterthisentry.
## Full Livox2 Huber3 launched (latest, 2026-11-13)
Frozen scratchcbca36d fullcandidate launched from external full-livox2-huber3/.
Rootunifiedexecsession46859 monitorsarchived full_livox2_huber_replay.py; livePID
andstatus inrun.json, quietwindowlogs inmonitor.log. No competing mapper/cachejob.
Originalraw2/settings alpha.1beta.1 unchanged; childenvH3=1, focusedtrace=1.
ExeSHA101881d74dfcb4369103d070701b003e5a83bdf4b95538aa3cc0a0114f997287.
RunnerSHA2bbb0fc664381055516bdd2cc67ab9ca67bcfe68202cbc5ec63a8475a5bb2e27.
Build/test/reviewgatespassed exceptidenticalknownbaselinecorridorfailure;18runner
tests pass. See full-livox2-huber-replay.md. Keepwatchingthroughcompletion,
thenvalidateodometry/finalstagesandmap. Do notstartadditionalprocessingonraw2.
Userhasauthorizedcontinueuntilgoal/newinfo; nofreshpermissionneededforthisrun.
UsertoldMediumis sufficientforprocessing; iftheyswitchdownnotifybeforeHighcausalwork.
Allagentscurrentlyfinished; historical_measurementsownscompletedsourcechanges,
provenance_auditcompletedrunner, snapshot_format_reviewcompletedlaunchreview.
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

## Audit continuation (2026-11-13)
User resumed code/cause work after the Sol6.1 audit. The stop boundary is lifted
for this investigation; no new mapper replay has been launched. Phase21 targets
the earlier W415-416 rotation bend using existing Huber3 trace, snapshot and raw
IMU. Raw-gyro local integration agrees with AHRS within0.09deg, whereas optimized
rotation differs by about9deg. NDT supports these accepted updates against the
motion prior; exact cell attribution is pending. Reuse existing reconstruction
with unchanged map/source gates. Origin regularizer units defect is real but
first-node movement<0.00011deg excludes it as the dominant cause here. Preserve
all earlier diagnostic gaps and failed bias-off records. No production fix or
full-route acceptance yet. User requests very concise output.
## Authorized bounded correction (2026-11-13)
User: fix what you found. Implement/test all-cell Cauchy NDT influence correction
in scratch, preserving IMU and bias. See cauchy-correction.md and phase22 tasks.
Input-integrity check found no decoding/nonfinite/local timestamp failure;
reflections, calibration and internal synchronization are not thereby certified.
First gate is exact Huber3 W415 control followed by one Cauchy3 checkpoint.
No shipped-default change, full replay or broad parameter search authorized by
this design alone; respect the existing tell-before-new-run instruction.