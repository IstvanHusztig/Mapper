# Phase0b evidence, 2026-11-12



## Implementation and checks



Scratch base8d230a1, branch codex/phase0b-ndt-information, checkout

C:/Repo/dMapper/out/phase0b-worktree. Source diff retained at

out/phase0b-2026-11-12/experiment.patch. Independent Codex source review (not

Sonnet) found no blocking issue after adding active accelerometer/velocity-prior

fixtures. Both grids scale NDT Hessian/RHS before projection, diagnostic cost

scales consistently, priors/geometric support remain unscaled; alpha1 bypasses

added arithmetic. Snapshot restoration is refused in the scratch CLI.



Alpha1 Release build succeeded. Unit suite:122 cases passed,3 skipped; three

expected-failure assertions retain their existing doctest classifications.

Eight frozen fixtures cover both grids, projection on/off and accelerometer

on/off. Alpha1 self-check verifies nonvacuous hierarchy/projection and zero NDT

extra-variable blocks. Cross-build proportionality checks remain in progress.

Debug verification is underway because CMake compile definitions changed.



## First control:44_short, alpha1



Exit0; mapper147.32s, orchestration153.46s. Preserved output:

out/phase0b-2026-11-12/runs/44_short/alpha1/scan_lio_merged.laz.

Executable SHA256:

2e1f6e103a60d2f8de25b024463b61c17e3c68c0026c862b19f69d88bd5165c8.

Full identity/hashes/diagnostics in sibling alpha1.json; log in alpha1.log.



Golden manifest comparison: all10 non-parameter baseline files identical,

including cloud and trajectories. The only differences are parameter TOML and

the additional snapshot. Parsed TOML comparison with44_short_baseline confirms

exactly three changed keys: log_window_diagnostics false->true,

save_odometry_snapshot false->true, real_time_threshold_seconds10->3600.

Odometry/final each25200 samples. No logged failure lines.



Diagnostic completeness remains false: rounded eligibility leaves conditional

chi-square coverage unresolved. This is explicitly retained, not a map verdict

or a reason to discard the run.44_short remains an old-sensor regression/stress

check with known timing defects; it does not calibrate or assess Livox health.



## Current collection



Livox2 0â€“60s alpha1 processing. Remaining approved runs follow after build and

equation checks. No candidate accepted. Seven of nine runs use Livox; two use

44_short. All processing is fresh Release odometry, never snapshot restoration.

All controls/interventions use the same3600s per-window wall-clock cutoff.



## Approved output relocation



User requests experiment data/results outside the repo to avoid confusion. After

active phase0b jobs finish, relocate to

C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/, verify hashes and

update links. Preserve historical run-time identity paths with a relocation

manifest rather than rewriting provenance. Original raw recordings stay unchanged.



## Livox2 control and cross-build check update



Livox2 first60s alpha1 completed exit0, mapper369.75s/orchestration379.91s.

All165 windows have heading/speed/bias; conditional chi-square16/165 with no

missing required evidence. Bends>3/>10deg20/3, max23.732deg; peak2.742m/s;

residual bias max[.463,.939,.619]deg/s. Median jump41.99mm/s, carried sigma3.75mm/s;

conditional chi-square median181.55,p9515861.4. These are diagnostic readings,

not sensor-health or accuracy verdicts. All windows logged100 iterations.



Alpha0.3 Release frozen/serial-parallel tests pass. Cross-build NDT H/RHS/cost

proportionality and exact geometric-support checks pass in8 fixtures. Initial

prior-RHS componentwise tolerance failed on cancellation (~1e-11 in small rows).

Independent review confirmed intermediate IMU additions of~2e5–4e5 cancel to

~1e-6, while largest prior RHS is3.17e8. Replaced invalid final-component-only

roundoff bound with32eps times RHS infinity-scale (~2.25e-6), retaining exact

extra-variable RHS equality and negative controls that deliberately scale priors

and must fail. This is norm-scaled invariance, not exact equality of tiny

cancellation residues. Detailed errors/tolerances retained in scaling-alpha0.3.json.



Output-local matched_clouds.py freezes reference return indices before reading

interventions, verifies complete ordered timestamp/intensity sequences and

retains displaced points without candidate clipping. Synthetic0.8m shift test

retained2417/2417 patch observations, measured0.8m shift and100% outside band,

while centered thickness remained unchanged. Actual controls extracted136 patches

on44_short atz0 and114 onLivox2 atz+.5. Index identity relies on unchanged input,

sort implementation and export ordering; duplicate timestamp/intensity tuples

are not independently unique source IDs. Fits are not held-out accuracy evidence.



## Livox1 control



Exit0; mapper682.31s/orchestration701.91s under concurrent build/Debug-test load.

Complete required diagnostic evidence. Both46,980-sample odometry and final

trajectories match phase0a current baseline exactly: max displacement0.0m.

Only effective config difference is the experiment wall-clock cutoff10->3600s.

This confirms the control arithmetic on Livox1 as well as44_short. Background

compiler/test priorities subsequently lowered to favor mapper processing.

Alpha0.1 and0.03 also pass8-fixture scaling and serial/parallel checks.



## Debug intervention suite finding



All four non-control Release builds passed scaling/serial-parallel checks.

Full Debug alpha0.1 suite ran1078.07s:121/122 cases passed,3 skipped; one actual

failure beyond existing expected failures. In the degeneracy-aware NDT corridor

fixture, residual cross-wall error17.9039mm exceeds7.5mm (initial error30mm).

The suite uses shipped-weight accuracy expectations. Do not dismiss this as an

implementation bug or silently loosen its threshold: it is a synthetic downside

of this intervention pending control verification. Alpha1 Debug control suite

is now building/running. This diagnostic experiment does not accept alpha0.1.



## User visual feedback



User opened/referenced Livox2 alpha0.3 merged cloud and reported it

"starts looking good". Record as positive preliminary visual feedback for that

single result, not acceptance of a correction or stability claim. Approved

cross-checks remain Livox1/44_short alpha0.1; no alpha0.3 extension is authorized

by this observation alone. Complete the agreed nine before proposing next work.



User subsequently supplied an annotated CloudCompare top view of Livox2 alpha0.3,

marking two wall junctions with remaining drift/misalignment. Screenshot preserved

unchanged at out/phase0b-2026-11-12/user_feedback/livox2-alpha03-marked-drift.png.

This qualifies the earlier positive impression: residual local defects remain;

no visual acceptance or fixed-cloud claim. Compare these junctions across weights,

retaining translation/heading/post-processing alternatives until localized evidence.



User additionally requested the house-plan overlay. Existing plan verified at

C:/Users/User/Downloads/dLidar/reference/haus47_LG_plan.pdf (one scanned1:50 sheet).

Existing44-specific overlay anchors cannot be blindly transferred to Livox2.

Fit/check a single control-derived rigid alignment, fixed scale, and reuse it

across candidates; retain alignment ambiguity rather than claim survey accuracy.



## Debug control verification



Alpha1 Debug ctest passed (814.37s), after alpha0.1's one weight-dependent

corridor accuracy failure. Release alpha1 suite and all Release frozen/parallel

checks also pass. No thresholds were relaxed to hide the alpha0.1 failure.

CLI guard smoke checks refused snapshot restoration and unsupported robust mode

before reading a nonexistent input folder; no output folder was created.



## Overlay alignment gate



Control early0–15s all-heading plan fit has competing plausible placements;

initial selected and alternate headings have similar line-distance costs and a

search-boundary warning. This does not establish correct apartment placement.

User asked asynchronously for Livox2's start room to disambiguate; alternative

visual overlays are retained and unconfirmed registration is not called accuracy.



User resolved Livox2 start location: **apartment56, Wohnen room**. Constrain

registration origin to that room, not the historical44 origin assumption;

check neighboring walls and freeze a single alignment for all candidates.



Additional user visual feedback: Livox2 alpha0.1 "looks good no double walls".

Record positive visual observation for this recording, alongside the independently

verified failed Livox1 alpha0.1 cross-check. Do not conflate good Livox2 appearance

with cross-recording robustness, plan accuracy or an accepted global setting.



## Completed collection and user reflection annotations

All nine approved mapper runs completed with exit0. Artifact verification
rehashed nine run records and seven analysis directories. Historical previews
are explicitly excluded from final comparisons.

The user identifies the blue-box returns beyond the apartment (upper stairwell
and lower terrace regions in the overlay) as window/mirror reflections. The
Schlafzimmer mirror is approximately at plan x25.6m, y14.9m, near the upper wall
beside the entrance. The red rectangle supplies approximate location, not
measured mirror dimensions or orientation. Annotated originals are preserved as
user_feedback/livox2-alpha01-reflection-regions.png and
user_feedback/livox2-alpha01-mirror-location.png in the experiment folder.

These user-identified reflections are not standalone evidence of drift. No raw
points were deleted and numerical measurements were not silently masked.
Whole-slice statistics may include reflective returns. Physical wall/revisit
agreement needs separate interpretation; the annotations do not establish that
all previously circled defects are reflections. Ray-level attribution is NOT
MEASURED; plan alignment is preliminary, not surveyed accuracy.

User's latest assessment: otherwise this is a step forward, following the
alpha0.1 Livox2 observation of no visible double walls. Record positive local
visual progress, not acceptance of a global setting: Livox1 alpha0.1 still has
substantial deformation in the retained cross-check.


## Final independent review and delivery

Independent Codex reviewer (not Sonnet) verified all nine output manifests,
frozen configs and current script identity in the three final map reports.
Review retained the44_short control uncertainty-coverage gap and corrected the
summary to avoid claiming uniformly sharper walls: Livox2 local XY-PCA p50/p90
25.63/87.55mm becomes22.15/94.99 at0.3 and22.50/90.73 at0.1. 44_short has135
valid patches of136, with18.05/48.34 becoming22.32/54.23mm; no robustness verdict.
Livox1 alpha0.1 deformation remains a failed cross-scene sanity check. Livox2
0.03/0.01 also show broad dynamic deformation; lower chi2 is not a quality gate.
No single global multiplier or sole causal mechanism is established.

The reviewer recommends first localizing divergence in existing outputs and
separating user-attributed reflection groups from persistent physical-wall
misalignment, retaining included/excluded diagnostic views. If pursuing0.3,
the missing bounded cross-checks are Livox1 and44_short at0.3, then perturbation
testing; these additional runs are proposed, not authorized or launched.

After all writers finished, moved experiment artifacts and the Livox2 cut to
C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/.
Relocation verified all721 files by SHA256 and size before/after; manifest is
relocation-manifest.json in that folder. Original raw recordings unchanged.
Historical provenance paths intentionally remain unchanged; use the manifest's
root mapping. Scratch source worktree remains in the repository. No files deleted.
Detailed readings are in results.md. Phase0b's bounded diagnostic work is complete;
full-route validation, robustness and global setting acceptance remain open.


## Existing-run Livox1 localization completed

User authorized this follow-up without new runs. Medium effort sufficed; no
higher-effort step was started. See livox1-divergence.md and external experiment
livox1-divergence/localization.json, timeline.png and retained script.
Independent Codex reviewer (not Sonnet) replayed the thresholds and logs:
early >5cm separation13.580s recovers to2.35mm at30s; persistent >5cm begins
38.324s, >0.5m40.822s; sustained >1m52.246s; >2m53.412s; >5m55.160s.
Candidate window149 at53.28s bends18.258deg; first speed>2.5m/s is window151,
54.00s. This separates transient early disagreement, persistent horizontal
separation and later runaway/vertical collapse. All times relative to first
retained pose (recording offset0.37497177s). Large failure exists pre-loop;
root cause remains unestablished. Existing input/output artifacts unchanged.


## Existing-run mechanism investigation and new-run boundary

User switched to Astra High and authorized causal investigation without new
runs. See livox1-mechanism.md. Independent Codex reviewer confirmed velocity
carry with narrow local uncertainty and inheritance of large motions by the
next prediction; the5m/s reset explains the55.8s speed dip. Final geometric
ratios exceed the0.2 projection threshold, so weak-axis logging is not proof of
projection suppressing Y. Matched-only cost profiles do not identify a total-
objective minimum or the initiating failure. Source has scheduled damping and
no objective-decrease acceptance check, but retained logs do not prove a
particular cost-increasing step. No sensor fault or single root cause established.

Reached the evidence boundary: proposed two instrumented full Livox1 replays
(alpha1/.1) with focused windows100–115 and143–156. Need per-iteration matches,
projection, factor contributions and carry states; verify trajectory identity
before conclusions. No instrumentation changes or new runs launched. Await user
decision before those new runs. Existing source and all results preserved.


## Instrumentation/replay authorization and preparation

User approved logging and the two full Livox1 alpha1/.1 runs. Scratch source
instrumentation is in progress; separate read-only reviewer assigned. Root runner
checks original artifact manifests, exact archived TOML identity, raw input hashes,
pre/post build source identity including untracked files, immutable binary bundle
identity, completed trace coverage, strict stage matching and cloud hashes.
Artifact output remains external under livox1-instrumented/. At preparation,
no dMapper process active and drive had approximately136.8GB free.

Diagnostic cost scope is explicit: NDT rematched cost omits unmatched penalties;
origin anchoring is an incremental zero-RHS regularizer, so a fabricated complete
nonlinear total objective will not be reported. Actual active residual costs and
factor normal-equation contributions are recorded with those limits. High effort
still useful for instrumentation review/causal interpretation; user informed.


Instrumentation frozen in four tracked scratch files (scan_to_map.cpp/.h,
run_odometry.cpp, scan_to_map_test.cpp), no new sources or CMake changes.
Independent reviewer found no blocker: unchanged solver arithmetic/order,
point-owned trace writes, enabled-window gating, explicit incomplete costs and
matched cell geometry fingerprints. Added synthetic traced/untraced parity with
both grids, gyro/IMU states, serial/parallel and accepted/rejected steps. Release
control build/test underway. No real replay launched until those checks pass.


Release alpha1 full suite passes123 cases,3 skipped, with3 pre-existing expected-
failure assertions reported by doctest as successful expected failures. New
trace parity test passes. Frozen alpha1 bundle built and its full Livox1 replay
started from original raw input and exact archived TOML. Alpha0.1 bundle building
separately; no simultaneous jobs on the recording. Trace parser retains64-bit
fingerprints as integers and checks payload dimensions/finiteness and coverage.


Alpha0.1 Release frozen assembly, parallel equivalence and trace parity checks
pass3 cases/864 assertions. Both immutable Release bundles ready. Focused Debug
trace parity check launched for new state/indexing diagnostics. Independent
actual-log schema review of control window100 passes:100 iterations plus both
carry phases,8213 records, finite correctly shaped payloads, zero actual-match
reconstruction mismatches. No interpretation before full reproduction gate.

Focused alpha0.1 Debug trace parity passes1case/816assertions. Both Release
bundles and Debug instrumentation validation complete. Real control replay
continues; first106 complete window headers exactly match original diagnostics.


Instrumented alpha1 replay completed: mapper760.16s, orchestration773.05s;
strict pre-loop and final comparisons each46,980 poses, maximum difference0.0m.
Merged cloud SHA256 equals retained control exactly. Trace payload validation
passes all30x100 iteration groups plus both carry phases. Run record status
verified/evidence_complete. Alpha0.1 replay now active, sequentially after control.
Control trace summary generation runs separately from mapper on read-only logs.


## Two instrumented checks complete and reviewed

Both replays exactly reproduce46,980 pre-loop/final poses and bit-identical
merged clouds. Alpha0.1 mapper666.91s/orchestration680.59s. All6000 focused
iterations have full projection retention and applied steps. Source commit
cb6ea83 exactly matches both built source patches; nothing pushed.

See instrumented-results.md for the final diagnosis and limits. Directional
Y-ramp evidence directly shows scan pull opposed by motion/carry (candidate
window107 final RHS -5969.716,+3697.354,+2270.484). Its initial directional
NDT/(motion+carry) H ratio0.795 compares with control8.298. Do not read this as
a marginal gain or isolated same-state ablation.

Independent review corrected the first-step acceleration interpretation: much
of its large energy comes from adjusting the auxiliary end velocity. Later
large yaw changes are favored by NDT local models; support can increase during
those turns, so match loss is not a necessary mechanism. Both review and root
verified this from the retained traces. No sensor fault, projection trigger,
logging effect, or single validated corrective algorithm is claimed.

All18 tasks for the authorized experiment and follow-ups complete. Analysis
scripts, source patches, build/test logs, run manifests and plots are retained
under external livox1-instrumented/. No further mapper runs are active. High
still useful for next corrective design; medium sufficient for reading/reporting.
