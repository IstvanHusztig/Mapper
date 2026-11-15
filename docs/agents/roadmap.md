# dMapper roadmap — current status

Updated 2026-11-12. This is the current status view, superseding stale decisions
in docs/sensor-neutral-retune-plan.md while retaining its historical evidence.
Goal: a clean, precise, stable point cloud, with sensor-grounded settings and
no regression in geometry, revisit consistency or sharpness. Direction A was
selected by the user; L2 numerical results may change only with quality evidence.

| Stage | Work and exit evidence | Current status |
|---|---|---|
| Agent takeover | Shared instructions, native Codex/Claude OpenSpec workflows, verification skills, review process | Setup validated; Claude login remains a separate authentication issue. Setup changes remain uncommitted. |
| Removal prerequisite | Remove obsolete accelerometer modes; preserve other results; Release checks and golden manifests | Done, three commits e196692/e0b29ab/8d230a1; not pushed. |
| 0a: trustworthy measurements | Inputs, agreed criteria, reliable tools, reference provenance, perturbation evidence | Complete: sensor-neutral-yardstick20/20 tasks. Fresh Livox1 base/1mm decimation check confirms18.67m pre-loop and18.46m final displacement; ready for bounded diagnosis, not candidate acceptance. |
| 0b: diagnostic weighting experiment | Scratch information multipliers1/.3/.1/.03/.01 on short inputs; separate velocity jump from claimed sigma; compare cloud geometry and stability | Completed diagnostic batch: all nine runs and independent review. Livox2 .3/.1 show local visual progress; .1 fails Livox1 cross-check. No global candidate accepted. Results: openspec/changes/ndt-information-probe/results.md. |
| 1: sensor measurement/calibration | Static noise, bias, scale, axis alignment, lever arm/sign, internal timing with uncertainty | Partial: static/turn collection and analysis exist. Turn registration ambiguous; calibration not accepted. Healthy Livox evidence is distinct from calibration certainty. |
| 2: sensor-neutral implementation | Durations rather than sample counts, time-scaled translation uncertainty, point interpolation, remaining IMU-front-end defects; synthetic rate/density invariance and44 guard | Pending. Reconcile existing D1/D2/D5 work before implementing; do not redo completed changes. |
| 3: odometry corrections | Evidence-driven information model, velocity covariance, gyro/bias model, step acceptance/unmatched points, cell statistics | Pending. Likely main corrective phase if0b alone is insufficient; depends on adequate phase1/2 evidence. |
| 4: geometry and filtering | Cell/window/range/decimation choices, stricter rejection of uncertain returns, reflectivity normalization | Pending. One family at a time after structure/weights are sound. |
| 5: loop closure and consistency | Revalidate matching/gates/corrections and downstream44 investigations with improved odometry | Pending. Correction size alone is not a pass/fail rule. |
| 6: validate and ship | Full-recording robustness,44 s0/s1/s2 checks, before/after clouds, user visual acceptance, settings/baselines/docs | Pending. Push only on explicit authorization. |

## Flyaway and passage-stall findings and next steps (2026-11-15, Claude; latest, supersedes the next-work list below)

Branch `honest-degeneracy-measure` (local, not pushed). Evidence: `openspec/changes/isolate-and-replace-trajectory-method/checkpoint-mamba.md`.

**Confirmed cause of the Livox 4 wall offset:** the 10 m walk through the narrow passage between the buildings
(100-110 s) is measured as 5.3 m: dMapper's carried speed decays from 1.15 to 0.13 m/s while the user walks steadily.
The user confirmed the route and the true positions on the plan.

**Code findings (reviewed against the code; none measured on a recording yet):**

| # | Finding | Effect |
|---|---|---|
| M1 | The LM loop never checks the cost; a step that raises it is applied | A window can slide in a wrong direction. Fix ported: `lm_step_check` (off by default) |
| M2 | A point in no NDT cell costs nothing | Sliding points off the map looks better. Fix ported: `ndt_unmatched_point_cost` (off; cost 16 tested on 44 in October). Unit test: 16 is too small, a window 500 m off the map costs 96,000 against 337,000 for 8 cm off. Choose it from the per-point cost of a correct match |
| M3 | One cell per point, cell mean = its first point (default statistics) | Target jumps at 15 cm cell borders; likely part of the run-to-run chaos on 44 |
| M4 | The only guard is "no node more than 1 m from the guess"; the guess comes from the carried speed; no speed plausibility check | A runaway guess is never caught; large corrections are blocked and the window ends on its guess |
| M5 | Above max_prior_velocity_mps (5 m/s) the carried speed is replaced by zero with sigma 1 m/s | A jolt to standstill, not a recovery |
| M6 | Every window goes into the map unchecked | One wrong window corrupts the map; the next windows confirm it |
| T1 | No setting for an IMU-LiDAR time offset (44 was restamped outside dMapper) | Every turn disagrees when a recording lags |
| T2 | Points get the pose just before them (no interpolation); dropped if more than 10 ms after it | An IMU gap silently drops points |
| T3 | First AHRS step assumes 200 Hz, a repeated timestamp 2 ms | Minor |
| G1 | AHRS tilt correction is off for every sample outside 0.9-1.1 g (about 25-31 % while walking) | Tilt corrected from the quiet phase of each step only: a possible steady tilt error (hypothesis) |
| G2 | e41bd08 made the gain switches take effect | Fixed; it changed the numbers and the baselines were not refreshed (see step 0) |
| D1 | A window's internal speed profile is fixed once from the carried speed (motion model sigma 1e-4 m per edge) | A wrong carried speed is baked into the whole window |
| P1 | The default degeneracy measure uses each cell's first-3-point normal | Reads a smooth passage as 0.15-0.27 (exact: about 0.005), so the projection stays off. Fix (a) `degenerate_projection_all_point_normals` written, off by default |
| P2 | The carried velocity covariance includes the NDT's in-plane pull | The slowed speed is handed on as known to 5 mm/s. Fix (d) `carried_covariance_normal_information_only` written, off by default |

**Tried on Livox 4 (125 s prefix, one run each):** exact cell statistics, projection 0.3/0.5, accelerometer floor 0.02,
and (a): none fixes passage and gallery together. (a) runs away (62 m off by 100 s): without the in-plane pull the
IMU carry alone holds the passage axis, and it drifts.

**Next steps, in order:**
0. Baselines: refresh 44 / 44_short for e41bd08 (in progress), so bit-identity checks work again.
1. Step check on its own: `lm_step_check` with a well-chosen `ndt_unmatched_point_cost` (M1, M2) on Livox 4 and 44
   s0/s1/s2, 45f, `/fp:fast`. Expected: fewer runaways (it stopped the `/fp:fast` runaway on 44 in October), not the
   passage itself.
2. IMU carry (c): estimate accelerometer bias and tilt over many seconds while the LiDAR sees, carry them with an honest,
   growing uncertainty through blind stretches; check G1 (biased tilt correction) as part of it. Design work (Fable).
3. With (c) in place: (a) honest degeneracy measure and (d) honest carried covariance, then the passage again.
4. Guards: speed plausibility instead of the 5 m/s reset (M4, M5), a check before a window enters the map (M6).
5. Smaller: IMU-LiDAR time-offset setting (T1), pose interpolation for points and a warning for dropped points (T2),
   real dt for the first AHRS step (T3).
6. Each step judged with tools/map_quality.py on 44 s0/s1/s2 (noise floor), Livox 4 against the user's marked
   positions, and the stairwell of 44 (the opposite failure: runaway without the in-plane pull).

## Next concrete work

The nine-run batch, two logging replays and two finite motion-prior checks are
complete. OpenSpec ndt-information-probe28/28 tasks (full run, existing-run diagnosis and surrounding-cloud inspection completed). Scratch implementation
commit16807ac; no shipped defaults changed or pushed. See motion-prior-results.md
and slam-research.md in that change for measured evidence and Terra research.

- New Livox2 alpha0.1/beta0.1 looks good to the user; selected wall spread is
  about22/91mm, similar to the previous alpha0.1 result. Revisit tail is higher;
  no perturbation/acceptance claim.
- Livox1 gross runaway is absent in the candidate, but the user reports doubled
  walls and selected geometry remains worse than alpha1. Not a fixed final cloud.
- User wants to stop tuning investigation and process full Livox2 using the
  promising candidate, then record a new Livox1 if the full result looks good.
  Full run completed and exported; user reports flyaway outside the building.
  High-effort existing-run diagnosis completed and independently reviewed:
  disruption around268.56–269.64s, sustained speed growth from271.08s;
  first speed-trigger window starts272.88s, peak endpoint estimate7.647m/s.
  The large excursion exists pre-loop; inherited rapid prediction can pass the
  correction-relative guard. Initiating factor remains unresolved. See
  openspec/changes/ndt-information-probe/full-livox2-jump-diagnosis.md.
  Proposed next evidence: one exact diagnostic replay, about50minutes plus
  trace overhead, requiring fresh authorization. No new run or fix is authorized.
- User requested surrounding-cloud inspection first. Raw scene analysis now
  shows close rectangular surface arrangements before both escalations;
  99.989% of returns are within5m at265–268s. This narrows the investigation to
  changing nearby geometry/visibility but does not prove location or cause.
  See full-livox2-cloud-context.md and external cloud-context/ extracts. The
  new reusable tools/cloud_context.py passed a small coordinate/CLI smoke check.
- Full-recording visual success remains separate from robustness/regression and
  shipment gates. Preserve the user's mirror/window feedback.

Artifacts: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/
motion-prior-probe/. Both short-batch and full-recording processing have finished; no jobs remain. Existing relocated
artifacts and manifests remain intact. Medium suffices for full-run execution;
notify before High is needed for new numerical design or causal interpretation.

## When a better cloud becomes visible

Tool completion does not improve point coordinates. The first candidate visual
comparison is phase0b if an experiment changes the map beneficially. Reliable
final improvement may require phases2–5; no defensible completion date exists
until the diagnostic experiments identify the necessary corrections. Full
acceptance is phase6, after metrics and the user's inspection agree.

## Agreed evaluation and boundaries

- Building planar-wall robust spread targets: median20mm,90th percentile60mm.
- Short-input perturbation maximum0.10m; full-route0.3m/100m of a frozen reference.
- Pre-loop drift: investigation ceiling1m/100m, desired target0.3m/100m.
- Physical return position/height residual0.10m only with independent endpoint
  evidence and adequate reference uncertainty; returning to the room is insufficient.
- Compare empirical envelopes; overlap is inconclusive. Geometry, revisits and
  sharpness are separate. Diagnostic triggers are not accuracy verdicts.
- No absolute cave-wall criterion or cave-readiness claim without suitable cave
  recordings and independent references. A future map-quality report/learning
  work is separate; no unrequested ML implementation.

Source of truth: openspec/changes/sensor-neutral-yardstick/tasks.md and
criteria-proposal.md. Other existing OpenSpec changes remain separate and are
not automatically closed or archived. Research/review agents are used where useful.
Notify the user before a step needs a higher model or effort. High remains useful
for the proposed instrumentation design and causal trace interpretation; routine
replay monitoring is suitable for Medium. The user subsequently authorized one
replay, then explicitly stopped before launch. Current permission covers timeline
and runner preparation; launch awaits an explicit resume instruction.

## Retained-run timeline delivery

The offline synchronized timeline is built from the existing full Livox2 run:
`full-livox2-motion-prior/timeline-v3/index.html` under the experiment root.
All input/output hashes and LAZ links passed; scripted interaction checks passed.
Real-browser visual verification remains unavailable because the browser tool
blocked local file access. No new SLAM run or point-coordinate change occurred.
Preparation status and gates: OpenSpec `full-livox2-replay-readiness.md`.


## Full Livox2 traced replay completed
Single authorized replay completed overnight2026-11-12/13: mapper exit0, collected_complete,3993.63s. All63 output hashes reproduce retained full run exactly;116 windows/11600 iterations/23200 assembly identity checks complete, zero mismatches. Trace2,497,193,543 bytes. Run-health triggers unchanged, not an accuracy pass. See openspec/changes/ndt-information-probe/full-livox2-trace-results.md. No jobs remain. Task11.3 complete;11.4 causal analysis pending High. Use streaming Python on existing trace; no additional run required now. User wants quiet monitoring and standing OpenSpec updates.


## Traced onset analysis completed
See full-livox2-trace-analysis.md: W754(271.08s) accumulates15.189deg estimated tilt change during100 accepted iterations, projection scales1. Signed joint-step evidence favors NDT, specifically minority5-15m returns over iterations1-99; near returns oppose in aggregate. Same-sample rotated acceleration changes substantially; independent raw-trace/time-weighted check confirms. This identifies a supported attitude/acceleration propagation mechanism, not surveyed tilt error or proof far returns are wrong. Association/geometry cause remains unresolved. No fix/new mapper run. Python streaming summary and tables archived externally. Independent Codex review incorporated. High remains useful for follow-up causal/design work.


## Exact cell attribution and insertion history complete
Existing-data map-only helpers (scratch9653ae0,d87b89f) reproduced W754 associations and numerical contributions exactly/withinroundoff; no optimizer or full replay. Atit20 three high-occupancy coarse-grid cells dominate favorable far-range joint-step terms. Histories builtW623-633, no rejectedviewpointupdates, thinmapgeometry; currentmatches have verylarge distances undercellcovariance. Notproofreflection/physicalwrongness. See openspec/changes/ndt-information-probe/full-livox2-cell-attribution.md. Proposed next two isolatedW754 optimizerchecks: exactcontrol thenthree-cellablation, notexecuted; awaitusergo-ahead understop-before-newruns. Highremainsuseful.

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
## Full Huber3 completed; postprocessing attribution in preparation (2026-11-13)
No mapper job remains. Full run4076.10s exit0; all11600focusediterations and
23200assembly checks passed. Original run.json remains needs_review due solely
to W798 printedratio.450 conditionalchi2 eligibility ambiguity (untraced).
Separate reviewed-collection.json verifies artifact integrity/stage correspondence/
82,688,088 return sequence; evidence/diagnostics complete remainFALSE, health exit2.
Never silently promote that gap to PASS. Full outputs and analysis_scripts/views/
matched-first60/motion-830-950 are under full-livox2-huber3 externalexperimentroot.
Newcloud link delivered. Fullresults report in OpenSpec; independent Codex review
confirmed oldverticalexcursion absent, later W915speed5.154/W916fallback, mixed
firstminute patchstatistics, and substantial fine-scale postprocessingvariation.
User asked optional whole-route endpoint question; no reply yet, no timestampriddle.
Standingauthorization: continue untilgoal/newinfo, Terrawhenuseful. Terra6.1
source review identifies consistency aspossiblecause, notproof. Phase17 planned
in full-livox2-postprocess-attribution.md: exact retainedsnapshot postprocessing
replay, capture afterloopclosure and eachof10consistencypasses, require exactfinal
trajectory/mergedcloud reproduction beforeinterpretation. No fullodometryrerun.
historical_measurements owns scratch standalone logger/progress-observer driver
andbuild/tests; nocorehook orCLIguardweakening planned. Root ownsOpenSpec/runner
coordination/freeze/launch; independentreview beforeexecution. No phase17run yet.
Highstillusefulforcausal/designwork; routineexecutionMediumsufficient.

## Postprocessing-only exact-control replay launched (latest, 2026-11-13)
Root session41934 runs frozen postprocess-attribution/full_livox2_postprocess_replay.py.
LivePID/status in its run.json (initialPID34076), parent full-livox2-huber3.
Driver scratchf18353e, exeSHA b8bcde8ad91f78320226fc1bbfe5cdbbfbb56804f274596599971be18d2c85fa.
RunnerSHA f1786e50fd169706c64d8170070449d05670160f8ca3b2086fe8dc1b7cd10479.
OneunchangedLC+10consistency replayfrompinnedcandidateodometry; NOodometryoptimizer.
Fresh workspace/cache andnopreview protectoriginalraw/cache. Captures12snapshots;
restoredsnapshotexact beforepost, allfinalnumericfiles exactbeforeattribution.
Release/Debugselftests+12runnertests+independentreviewpass. Nootherprocessing/build.
Monitorquietlythroughcompletion, inspectfailureswithoutautoretry. Source/results
reportOpenSpec full-livox2-postprocess-attribution.md; tasks17.1done17.2/17.3pending.
historical_measurements nowpreparesrootout sequentialstageanalysisscript only;
no further scratch changes/builds permittedwhilethisruns. Rootownsmonitor/finalanalysis.

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


## Saved state: phase18 and19 analysis complete, route detail pending
No mapper, exporter, build or analysis process remains active. Stage18 exports
and comparison complete; room_shapes_v2 session82785 completed successfully.
All task sources preserved externally; failedv1 source retained. Source/outputs:
full-livox2-huber3/stage-clouds/analysis_scripts and analysis/ and room-shapes/.
Independent Codex review (notSonnet) verified stage results, factor research,
and physical interpretation limits. Terra6.1 was used for focusedsource/timing/
LC research. No numerical fix selected from these measurements.
User async question pending: didLivox2 endbackinapartment56, otherindoor, oroutside?
About62m is estimated endpoint displacement, not closureerror withoutthisfact.
Root requested this afteractualroomviews; no precise timestamp recall needed.
Next diagnostic: accumulatedorientation versusavailableIMUincrements andscene
constraints near318�330s; notindependentpositiontruth. Exactgravity/offset/scale
statesafterW885 absent, W798uncertaintygap retained. Do notinventfactor replay.
Fullcloudgoal remainsunfinished/unaccepted; no push/archive/shippedsettings change.
Medium sufficientfor completedexports/measurements; notifybeforelaterHighdesign.


## Route confirmation received (2026-11-13)
User confirms fullLivox2 finished in apartment56, sameapartment asstartWohnen.
No longer waitingforrouteinfo. Exactendpointposition/heightnotknown;62m is
estimatedseparation, notexactsurveyedclosureerror. Grossapartmentmisplacement
is nowconfirmed. See full-livox2-return-diagnosis.md. Continue existing-data
orientation/translation diagnosis withTerra sourcecontracts andindependentreview;
AHRS angles are notpuregyro; always-on processing enables rest checks and a later moving-window acceleration integration despite disabled generic prediction flags.
No newmapperrun currentlystarted; no acceptance.


## Active focused correction test (latest)
User confirmed end apartment56. User also objectedtokenwastewithoutfixedcloud;
keepworkfocusedonthisoneablation,nofurtherdiagnostictoolbranches.
Rootsession92411 launched no-online-bias runner(preflightthenfullrun), newBASE/
full-livox2-huber3-no-online-bias. RunnerSHA d1a1f1a8ea6f0cecc337de354882cd6d6b88467ddd868bf41d60552744ad2297.
Existing frozenH3exe reused; exactlyuse_gyro_bias_estimation=false, startupbias
unchanged, tracingoff. Allsource/tests/reviewgatespassed; rootcheckednopriorjobs.
Watchrun.json/run.log throughcompletion; noautoretry. NormalCLIwritesderived
raw/cache+preview, recordedbeforeafter; physicalrawfiles andretainedresultsfixed.
Healthtoolunconditionallyexpectsbiasrows: preservetimes/coverage/rawexit2,
labelbiasnotapplicableandreportothergaps; noPASSwaiver. Comparepreloopreturn,
earlyorientationdeparture,actualapartmentoverlap. Mediumsufficientforrun.
Orientationcheck completed7s: hugeAHRSrelative azimuthdepartures by149-207s,
end~-107.7deg, notattitudetruth. Resultreturn-orientation/analysis.json.
IMPORTANT correctedresearch: always_on enablesrestchecking ANDlatermovingguess
integrate_window_translations despitegenericIMUpredflagfalse; fullcallchain
infull-livox2-return-diagnosis.md. No furtherresearchagentcurrentlyactive.
