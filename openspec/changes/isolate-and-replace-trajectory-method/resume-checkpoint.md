# Charlie â€” roadmap checkpoint before a focused fix

Checkpoint name: **Charlie**. Saved 2026-11-13 at the user's request. The larger replacement roadmap is
parked for a possible focused-fix detour, not abandoned or complete.

## Progress and resume point

2/19 tasks complete: 1.1 evidence inventory, 1.2 independently reviewed
state-transition trace. Resume at 1.3: exhaust bounded existing-data attribution
of the W295â€“301 attitude/gravity change, then prepare a minimal capture only if
needed. Task 1.4 requires the reviewed isolation decision and concrete interface
contract before any extraction. No interface or replacement algorithm exists yet.

Remaining stages: isolate -> extract legacy with numerical equivalence and saved
method selection -> research/select one candidate -> implement opt-in -> validate
indoor/outdoor quality and perturbations -> user adoption/removal decision.

## Findings to carry forward

- Livox4's incoming prediction explains most displacement growth at W301â€“320,
  but the joint optimizer creates outgoing velocity increments subsequently
  carried into prediction. W320 net accepted end-velocity change is approximately
  [0.282,0.355,0.008] m/s. This is a feedback path, not proof of a defective update.
- Approximate reconstruction shows persistent horizontal attitude/gravity
  disagreement of about 1 m/sÂ² during buildup. See attitude-gravity-lead.md for
  equations, scale provenance and uncertainty. AHRS is not ground truth and the
  gravity decomposition fits predicted displacement; initiating cause unresolved.
- W321 is an explicit velocity-prior reset; W397 is a separate large angular
  correction. The translation-only acceptance guard does not catch both forms.
- Snapshot v1 overwrites motion_model_poses; it is not an exact odometry restart.
  Prefer validated bounded checkpoints to repeated full routes, preserving
  incoming state and map. Raw cuts reinitialize the estimator.

## Controls, files and code

Root branch `remove-gated-accelerometer`; last analysis commit `b416fc6`.
Scratch `out/phase0b-worktree` at `e79125c` restores pre-floor tree `c52b25f`.
Frozen Huber3 reference remains `cbca36d`, alpha0.1/beta0.1. No new numerical
code or shipped defaults changed during this roadmap. Preserve unrelated dirty
files; consult evidence-inventory.md for source/configuration identities.

Livox4 gyro4x completed: final endpoint separation 44.394 m versus reference
36.717 m. User judged its cloud a mess; no adoption. Residual floor was explicitly
rejected and removed; failed Cauchy/bias-off evidence also remains relevant.

Generated analysis was copied with SHA-256 verification to:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/trajectory-method-checkpoint-2026-11-13/`.
Its manifest covers the orientation probe source/result and both displacement
decomposition JSON files. Original complete run bundles remain in place.

## Detour boundaries

The narrow Fusion AHRS gain-setter correction was implemented and tested in a
separate gain-only build. The two approved Livox4 prefix runs completed; see
`livox4-gain-prefix-comparison.md`. Control first325 snapshot windows exactly
match the original full run. Candidate maximum speed fell5.086->1.260 m/s, but
maximum yaw correction increased3.480->16.959 degrees atW298/106.92 s. This is
not an accepted flyaway fix. Both clouds are ready for visual review. The separate
AHRS-gravity-model experiment remains parked and was not included.

Existing-data high-effort investigation completed: `gain-jump-107s-analysis.md`.
AHRS increments remain nearly identical, but the joint optimizer creates about
17.9 degrees of correction by untouched node164 at W298, strongly violating the
active gyro factor. Translation-only acceptance permits the bend; exact initiating
factor/match is still unknown. Research agent independently confirmed the findings.
The proposed next step is diagnostic-only logging for W297â€“300 and one identical
125-second candidate prefix with exact equivalence against the saved gain-only
run. It is NOT authorized or launched. See the analysis note for scope and limits.
Do not mark larger roadmap tasks complete. New mapping/checkpoint runs still
require explicit user approval.

Before the bypass or trace, user requested optimizer code research. Completed
`optimizer-code-and-methods-audit.md`: gyro signs and carried-bias handling are
consistent; acceleration angular Jacobian formula checks agree. Known C37
origin-angle units defect is real but does not explain downstream W298 rotation.
Midpoint acceleration quadrature differs from exact time weights, with small,
similar local discrepancies in both runs. Independent research compared Ceres,
FAST-LIO and LIO-SAM/GTSAM; no replacement chosen and no new run authorized.
Priority remains identifying the first rotation-growing update and its changing
support, rather than bundling minor corrections or switching off the optimizer.
Commits authorized, no push. Keep OpenSpec updated; use existing Python tools.
Latest user-confirmed model/effort: Astra high. Tell before needing a change;
medium is sufficient for routine execution once numerical analysis is settled.

After the detour, record its evidence here and resume the remaining tasks or
explicitly revise the roadmap with the user. Do not restart the prior research
or rerun rejected candidates as new ideas.


## Latest: authorized rotation-factor trace running

User explicitly authorized tracing the first bad rotation update. Scratch
`f400479` adds diagnostic-only W297–300 logging and passed Release parity/Huber
checks (5 cases,2159 assertions). One existing125s prefix replay launched to
external `livox4-ahrs-gain-trace`; monitor its run.json and native run.log.
Do not launch a second run. Require identical346 snapshot window payloads to
existing candidate before attribution. Critical provenance correction: previous
control log alpha0.1, candidate alpha1 and build beta1. Previous gain-only causal
attribution withdrawn; this trace reproduces candidate1/1 intentionally. Details
and pending evidence in `livox4-rotation-factor-trace.md`. High remains appropriate.


## Latest: rotation trace completed and numerical driver localized

Single authorized replay completed exit0 in607.476s; all346 snapshot payloads
exact, all400 trace iterations valid. At W298 iteration0, node164 bends-0.855775°:
NDT net-0.855777°, principally2–5m response-1.064728°, near net+0.208951°.
Beyond5m matches zero throughoutW298; projection identity. Repeated accepted
updates reach-17.917741°. Fixed-match NDT cost always falls, but rematched cost
rises40/100 times; simple fixed-cost guard is insufficient. Reviewed independently.
Specific physically incorrect cell/surface and upstream map/model cause unresolved.
Previous candidate/control comparison confounded: controlalpha.1, candidatealpha1/
beta1; gain-only causal claims withdrawn. See livox4-rotation-factor-trace.md.
No fix/bypass/new run authorized beyond the completed diagnostic. Archive and
scripts live in external livox4-ahrs-gain-trace; scratch source commitf400479.

Roadmap update: task1.3 complete via exact sequential-prefix fallback;3/19.
Task1.4 interface contract remains open. Do not infer reusable full-map restart
support from this diagnostic capture.


## Latest: W298 cell-level cause of the rotational pull

Existing-data native reconstruction matches all source/association/target-cell
hashes for iterations0,1,2,10,99 in both grids; no SLAM replay or optimizer solve.
See livox4-ndt-cell-audit.md. Coarse dense cell1892points admits32 currentmatches
90.9–368.4mm off its3.162mm narrow-axis model; mean correction along that axis
only0.216mm, so known first-pointmean does not explain that discrepancy.
Another coarse cell has2 returns20usapart, no normal, yet admits213 matches.
Containing-voxel lookup plus inverse/viewpoint checks lacks support/association
validation; Huber retains influential incompatible residuals. These2cells are
55.15%net2–5m yaw-work, not endpointangle attribution. Independent review done.
Physical reason for initialgeometry disagreement remains open. Nextdesign target
is cell eligibility/correspondence consistency, not an unapproved whole-solver
swap or revival of rejected floor/statistics tests. Charlie3/19 unchanged.
Scratch inspection commitb892823; external livox4-ndt-cell-audit has frozen tools,
validatedcells, cell-ranking-v2.json/history-v2.json and plot. No fix/newrun approved.


## Latest: local NDT match guard implemented; replay awaiting approval

User authorized the local cell-validity/correspondence-consistency fix. Candidate
commits 426b8ed / 1268e5a in out/ahrs-gain-worktree; default off, shipped root
settings unchanged. Admission min6 cell points, SPD/finite/inverse validation,
model-relative residual<=6; >=6 unique source returns required. Targets and
membership fixed for entire window; diagnostics use same set. Unsupported window
stops before optimization/map carry; zero-normal supported volumes allowed.
Release+Debug7 newcases206 assertions pass each;8 legacy fixtures byteexact;
fullRelease137 cases accepted (3 pre-existing expected-failure assertions),3 skipped.
Independent review complete. See ndt-match-guard-implementation.md in the active
OpenSpec change for scope, risks and exact paths. External livox4-ndt-match-guard
bundle prepared_not_run, approvalfalse. Next proposed125s prefix alpha1/beta1/
Huber3,10-20minutes; STOP before launch. No replay/cloud-quality verdict yet.
Charlie3/19 unchanged. Medium enough for mechanical monitoring; high for analysis.


Latest approved NDT guard replay completed exit0 in524.17s,346windows.
Cloud external livox4-ndt-match-guard/result/scan_lio_merged.laz. Maxbend1.8vs17deg
but maxspeed5.40vs1.26m/s;116 excessive-speedwindows. Not a successful fix.
See ndt-match-guard-implementation.md. No more runs authorized; high needed
for numerical diagnosis. Charlie3/19, no adoption.


Latest existing-data guard diagnosis: first clear departure W87/30.96s, not
W122/43.56s. Nearly same predictions; reference corrects93mm, guard8mm despite
22,988 admitted sources. Larger outgoingvelocity carries forward. Frozen
prediction-selected associations are strongest supported mechanism; occupancy,
residualgate and freezing remain confounded. Attitude contribution measured,
net gravity leakage unproven (missing preLM/gravity states). Independent review
complete. See ndt-guard-early-drift-analysis.md. No newcode/build/optimizer/replay.
Next possible trace W84-102 requires explicit run approval. Charlie3/19.


Latest W87trace: scratch9635964; external livox4-guard-w87-trace nativeexit0
214.877s137windows, but first130ALLdiffer -> identity_failed. Preparationcut0
removed491leadingLiDARreturns beforefirstIMU, shiftedseed/windowstart~4ms.
Localfactoranalysis190iterations6assemblyalternatives internalchecks pass; NDT
dominantconditionaltranslationresistance, but originalcause NOTconfirmed.
Correctedfolder livox4-guard-w87-trace-corrected prepared_not_run approvedfalse,
start-10duration60preservesleadingdata; first200kLiDARrecords+IMUprefixexact.
STOP before additional replay; obtain approval. See ndt-guard-w87-trace.md.
High sufficient; Charlie3/19 unchanged.

Corrected W87 trace completed with user approval: external
livox4-guard-w87-trace-corrected, native exit 0, 211.590 s, 137 windows;
first 130 window payloads EXACT against original guard. Factor audit passes.
NDT supplies 89.64% conditional curvature; motion/velocity gradients balance its
small corrective push. Refreshing guarded matches increases local NDT push 5.62x
at i99 with similar curvature. Admission also reduces push. No alternative solve,
fix adoption or new replay. See ndt-guard-w87-trace.md corrected section.
Next: propose refreshing between linearizations, preserving cell validity and
consistent trial associations. Ask before algorithm change/new runs. High sufficient.
Charlie remains 3/19.


Iteration-refresh candidate implemented and committed in scratch as 71e2fa5.
Valid matches refresh each iteration; cell/support guards unchanged; each solve
keeps immutable targets. Enabled snapshot policy hash rejects frozen-guard state.
Independent review clear. Release build and 18 tests / 2436 assertions pass.
See ndt-iteration-refresh.md. NO new recording replay started. Next candidate
replay needs approval; 50 s diagnoses early drift, 125 s also covers rotation gate.
Current high sufficient; Charlie remains 3/19; no shipped adoption.

Approved iteration-refresh 50s replay completed: native0, 137 windows, 577.356s.
Max speed2.992->1.100m/s; >2.5 crossings8->0; W87 correction8.18->13.49mm.
Still1.583m max disagreement with unguarded same-time reference (not truth).
W97 missing chi2 warning resolved by exact trace ratio.4503025 above cutoff;
raw warning retained, collection-review.json records review. Cloud and comparison
in external livox4-iteration-refresh-50s. Independent review done. No full-route/
107s/robustness/map-accuracy verdict. No further run authorized; high sufficient.
See ndt-iteration-refresh.md. Charlie3/19, no adoption.

125s refreshed replay completed, native0,1698.819s346windows; first130 exact50s.
W298 rotation -2.247deg, but W301/302 -8.600/-9.095deg andW312+5.874deg.
Rotation gate NOTpassed; do notclaim samecause moved. Maxspeed2.404,zero>2.5,
versusfrozen5.402/116crossings; rises atprefixend. Detailedtrace297-300only,
400iterationserrors[]. Candidate9loops/max6.16m correction. Cloud+chart ready
external livox4-iteration-refresh-125s. OpenSpec ndt-iteration-refresh.md has
fulltable/identities/limits; independentreviewdone. No further run authorized.
Next existing-data focusW301â€“302; newtrace requiresapproval. High sufficient.
Charlie3/19; noadoption. User likedshortcloud, notyetreviewed125scloud.

Existing-data108s investigation completed: see ndt-refresh-108s-investigation.md
and external refreshed125s/event108-analysis.json. Concrete known weakness:
solver success +translation<1m accepts angles without objective/rotation check.
W301/302 changing support/turnover supports association-feedback hypothesis;
initiating factor NOTcaptured (traceendsW300). No staleownership defect, no
rejectedsteps/maprebuilds, savedpitch farfromsingular; biasincrementalone too small.
Independentreviewcompleted. No newrun/build/algorithm edits. Proposed nextdesign
objective-consistent rotation-aware acceptance; newcapture requiresapproval.
Highsufficient, Charlie3/19. User askedbestrecentfixes: Huber3+validity+refresh
usefulcandidates but no completeoutdoorfix;125sangular gate stillfails.


Acceptance fix PREPARATION ONLY: rotation-objective-acceptance-plan.md now has
independently reviewed concrete contract and implementation/test checklist. No
algorithm code/build/replay was performed. Need complete residual evaluator,
private coupled trials, all-node step+prediction-relative rotation bounds,
fixed-support external merit (projected proposal caveat), atomic covariance,
and distinct stationarity/rejection. Experimental1deg/3deg caps,8scales,6damping
attempts and numerical tolerances are proposed, not validated. Existing verify-run
will report acceptance; fix belongs in optimizer, not a new verify skill.
High sufficient for preparation; reassess implementation. Charlie3/19.


## Latest: acceptance implementation tested, replay approval pending

Scratch `1f0abc5` implements fixed-support full-merit/rotation acceptance with
atomic coupled state/covariance; `be898d6` updates only serialization fixtures.
Cell guards and refreshed matches retained; acceptance defaultoff. Independent
review clear after fixes. Release full suite146passed/3skipped (existing3 expected
assertion failures), new8cases/597assertions and8Python checks pass. See
rotation-objective-acceptance-plan.md for evidence/limitations. External bundle
`livox4-step-acceptance-125s` is prepared_not_run, approvalfalse; input identities
verified. Explicit125s replay approval requested, not received yet. No new cloud.
Charlie3/19 unchanged. Medium enough for monitoring; high for unexpected diagnosis.


User approved the prepared125s replay. Native process19688 launched once, with
manifest hashes/cache ownership/free-space checked and all unrelated DMAPPER_
environment overrides removed. Bundle manifest/run.json now record approval and
running status. Monitor this run; do not retry or start another without approval.


Latest: first approved acceptance replay exited1 in23.262s atW1 before solve:
new1e-8 rigid validator incompatible with finite Fusion approximate-quaternion
matrices. Existing control m_g error1.45e-5 confirms incompatibility. No cloud.
Bounded representation/normalized-angle-only correction and regression fixtures
implemented; build5/test pending. Preserve failed bundle; no second replay approved.
See acceptance-plan failure section. Charlie3/19. High sufficient.


Latest state: compatibility correction16e4ae7 tested (Release147cases pass,
9focused/620assertions,9Python checks), reviewed, committed. Fresh external
livox4-step-acceptance-125s-v2 approved by user and running PID14700. Same settings/
input as failed first attempt. Monitor/collect this run; no third replay approved.
No cloud yet. Failed original attempt retained. Details in acceptance plan.


Latest: corrected16e4ae7 replay stopped W301/108s iteration9 after2974.488s,
300/346windows completed. Last accepted correction2.999868deg;48final trials
exceed3deg. Guard works, recovery not achieved; no cloud/snapshot. All30009
accepted steps pass merit/angle/support audit. Evidence in external v2
acceptance-analysis.json/run-health. User requests careful existing-data/code
analysis of incoming values and why the cap is exceeded. No new replay or code
change authorized. Charlie3/19. See acceptance plan outcome.
