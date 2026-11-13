## 1. Implementation and control
- [x] 1.1 Create isolated checkout and record approved scope/base revision.
- [x] 1.2 Implement consistent scratch multiplier and logging/guards; independently review both grids and priors.
- [x] 1.3 Validate frozen synthetic scaling, serial/parallel behavior and Release tests.
- [x] 1.4 Establish alpha1 control and44_short numerical regression before interventions.

## 2. Collection
- [x] 2.1 Build/fingerprint approved multipliers, freeze effective configs and input/cache ownership.
- [x] 2.2 Collect five Livox2 first60s runs and alpha1/.1 on Livox1 and44_short; preserve all artifacts and failures.

## 3. Analysis
- [x] 3.1 Report jump/sigma separately, uncertainty coverage, motion/bias and solver events for all9 runs.
- [x] 3.2 Produce matched-region map measurements and preliminary before/after views with coverage limitations.
- [x] 3.3 Obtain independent evidence review, update roadmap and present diagnostic findings without accepting a candidate.

## 4. User-requested delivery additions
- [x] 4.1 Produce house-plan overlays with one frozen control-derived alignment, disclose registration ambiguity and preserve annotated residual-defect feedback.
- [x] 4.2 After active jobs finish, relocate experiment data/results to the approved dLidar experiments folder, verify hashes and update delivery links.

## 5. User-authorized existing-run localization
- [x] 5.1 Locate Livox1 first divergence and later runaway from retained odometry/final trajectories and logs; independently review and document limits, without new mapper runs.

- [x] 5.2 Investigate Livox1 divergence mechanism from existing logs and recorded source; document supported mechanism, alternatives and the boundary requiring new runs.


## 6. Authorized two-run instrumented Livox1 replay
- [x] 6.1 Implement opt-in diagnostic-only tracing in scratch checkout and independently review arithmetic/order preservation.
- [x] 6.2 Build/fingerprint alpha1 and alpha0.1 bundles; run appropriate tests including trace parity.
- [x] 6.3 Run two full Livox1 replays sequentially on unchanged input/configs, preserving all traces and output identities.
- [x] 6.4 Verify exact pre-loop/final trajectory and cloud reproduction before interpreting traces; report any mismatch.
- [x] 6.5 Analyze focused windows, obtain independent review, update OpenSpec/roadmap with evidence and remaining causal limits.

## 7. Approved translation-prior experiment
- [x] 7.1 Implement finite XYZ residual information multiplier with focused assembly/parity/support tests.
- [x] 7.2 Complete Release gates and independent source review; freeze executable provenance.
- [x] 7.3 Run approved full Livox1 and Livox2 first60s candidates sequentially.
- [x] 7.4 Verify logs, stage trajectories, matched map measurements and before/after views; independently review evidence and report limits.

## 8. User-approved full Livox2 visual check
- [x] 8.1 Verify original full Livox2 and launch the identical frozen alpha0.1/beta0.1 build with unchanged archived settings.
- [x] 8.2 Verify completion/log/snapshot/output identity and deliver full cloud for user inspection; no extra tuning runs.

## 9. User-authorized existing full Livox2 diagnosis
- [x] 9.1 Locate onset and propagation using existing log, pre-loop snapshot, final trajectories and raw IMU; preserve diagnostic tables and plots externally.
- [x] 9.2 Independently review mechanism and research alternatives, document user flyaway feedback and evidence limits, and identify any proposed replay before requesting authorization.

## 10. User-requested surrounding-cloud investigation
- [x] 10.1 Inspect raw range/return context and time-selected exported cloud around the first and second escalations, preserving new external views/extracts without rerunning SLAM.
- [x] 10.2 Review scene interpretation, document its limits and provide useful CloudCompare extracts; retain the new-run authorization boundary.

## 11. Approved single full Livox2 diagnostic replay
- [x] 11.1 Extend opt-in scratch tracing for close-scene transitions, predictors, per-factor contributions, match turnover/range geometry and final states; independently review behaviour preservation.
- [x] 11.2 Run appropriate parity/tests, freeze the alpha0.1/beta0.1 executable and unchanged settings, and review the one-run runner.
- [x] 11.3 Execute exactly one full original Livox2 replay, preserve provenance and verify pre-loop/final trajectory and cloud reproduction against the retained run.
- [x] 11.4 Diagnose onset and propagation from traced factors/associations with scene context; independently review, document findings and notify the user when High is no longer needed. Initial traced diagnosis is in full-livox2-trace-analysis.md; physical association correctness remains unresolved and High remains useful for follow-up causal/design work.

## 12. Existing-evidence investigation of influential range group
- [x] 12.1 Quantify matched-cell information, residuals and support changes at754; independently review range/geometry interpretations without altering the mapper.
- [x] 12.2 Establish whether exact historical point-to-cell associations can be reconstructed from existing snapshots/recording; document the remaining evidence boundary before any new mapper run. Standalone map-only reconstruction passed exact source/association/geometry and numerical gates; no optimizer invoked.
- [x] 12.3 Trace insertion history of the three influential coarse-grid cells and independently review what that history establishes before proposing an intervention. Bitwise history gates and independent review passed; proposed frozen-window control/ablation awaits user authorization.

## 13. Authorized isolated W754 checks and saved handoff
- [x] 13.1 Prepare and independently review a standalone W754 optimizer driver restoring exact traced state and reconstructed maps; build appropriate configurations and freeze provenance. Scratch7f99b29; Release/Debug passed; independent Codex execution review passed.
- [x] 13.2 Execute the unchanged 100-iteration control and require exact traced state/decision reproduction; stop the intervention if the control gate fails. All15,400 records reproduce exactly.
- [x] 13.3 Only after the control gate passes, execute one identical restart omitting outdoor cells 77310918653, 73015885821 and 81605885949; report tilt, velocity and support without claiming a full-cloud fix. Both100 iterations; tilt15.19 to2.33deg, speed0.890 to0.356m/s. See full-livox2-w754-ablation.md.
- [x] 13.4 Independently review evidence, save OpenSpec/results/next steps and commits, then stop as requested by the user. Independent Codex source/output review passed; resumable state in full-livox2-w754-ablation.md. User subsequently resumed correction-design work; no additional run started.

## 14. Resumed general-correction design
- [x] 14.1 Review source and primary research for general treatment of the influential associations; distinguish principled models from diagnostic heuristics.
- [x] 14.2 Specify a bounded correction and validation proposal, obtain independent review and document the new-run boundary before any additional optimizer execution. full-livox2-robust-design.md; independent Codex recommendation review passed. User explicitly authorized its control+Huber3 check.

## 15. Authorized W754 Huber diagnostic
- [x] 15.1 Implement scratch-only disabled-by-default Huber weighting consistently in both grids and diagnostics, with synthetic tests and independent review. Release/Debug focused3cases469assertions pass each; fullRelease127cases126pass1knownfailure, allfour printedfailurevalues identical to retained baseline. Independent Codex source/test review passed.
- [x] 15.2 Freeze provenance and execute exact W754 control, then only after its exact gate one identical restart with Huber3 and all map cells retained. Exact15,400controlrecords; candidate100iterations;136.93s; allinput/bundleidentitygates passed.
- [x] 15.3 Review candidate states, support/information and limitations independently; document results and propose further validation before any additional run. full-livox2-huber-results.md; independent Codex evidence review passed. User subsequently authorized full replay and continuing toward the goal.

## 16. Authorized fresh full Livox2 Huber validation
- [x] 16.1 Prepare explicit scratch full-odometry Huber3 opt-in, test unchanged disabled behavior and activation, freeze the Release bundle and independently review runner/source safeguards. Scratchcbca36d; focusedRelease/Debug4cases491assertionspass, fullRelease unchangedknownfailure;18runner tests pass; independentreviewpassed;13-filefrozenbundle verified.
- [x] 16.2 Execute fresh full original Livox2 with Huber3, alpha0.1/beta0.1 and unchanged settings; monitor trajectory/solver behavior, preserve trace and verify completion/provenance without expecting identical candidate output. Mapper exit0; structural comparison verified separately; original strict diagnostics remain incomplete only at W798 rounded eligibility boundary, never marked PASS.
- [x] 16.3 Compare odometry/final trajectories, onset and later excursions, support and map geometry; independently review findings and provide useful new cloud artifacts for inspection. full-livox2-huber-full-results.md; independent Codex review, mixed first-minute geometry, later speed escalation and postprocessing variation retained; no acceptance.

## 17. Retained-snapshot postprocessing attribution
- [x] 17.1 Prepare a standalone exact-provenance snapshot driver with observational stage/pass captures, appropriate tests and independent review; retain normal experimental CLI safeguards. Scratch f18353e; Release/Debug observer selftests and 12 runner tests passed, independent review cleared, frozen bundle verified. No core numerical changes.
- [x] 17.2 Freeze provenance and execute one unchanged postprocessing replay; require exact final trajectory/cloud reproduction before interpreting intermediate states. Exit0,1127.06s,all64numericfinalfiles exact and12stage snapshots validated; original raw/cache/preview preserved.
- [x] 17.3 Attribute observed variation by stage/pass, inspect affected geometry, independently review and document the next correction or unresolved evidence boundary. Stage attribution and independent binary-parser review are complete; affected stage-cloud geometry and independent review completed in phase18. Large odometry placement remains the next evidence boundary; no correction accepted.

## 18. Native exports of retained stages for geometry comparison
- [x] 18.1 Prepare/review a standalone native snapshot exporter and runner with exact final-control reproduction before earlier-stage exports; no optimizer invocation or normal CLI guard changes. Scratch4f8af9a; Release/Debug selftests and9runner tests pass; independent Codex review cleared native I/O and complete source/trajectory coverage gates.
- [x] 18.2 Export final-control, after-loop-closure and odometry clouds into fresh external folders; verify snapshot/input identity and matching exported source sequences.
- [x] 18.3 Compare actual stage geometry and coverage, independently review findings, deliver meaningful new clouds and record the remaining physical-reference/correction decision.


## 19. Remaining odometry placement: existing-data scene interpretation
- [x] 19.1 Inspect beginning/end room shapes from retained clouds, with source coverage and no physical identity assumption; relate to route confirmation if available. Bounded chunk assumption rejected for165postlastpose returns; full merged v2 completed with explicitcoverage. User subsequently confirmed return to apartment56; see full-livox2-return-diagnosis.md.
- [x] 19.2 Document the remaining evidence boundary and next focused odometry diagnostic, independently review before any new optimizer execution or correction selection.

- [x] 19.3 Reconcile the confirmed return to apartment56 with the scene views before interpreting closure or selecting a route-dependent correction; continue focused orientation/translation diagnosis under the recorded state-coverage limits.


## 20. One online gyro-bias feedback ablation
- [x] 20.1 Prepare/review a minimal replay using the existing frozen executable and exactly one parsed setting change; preserve startup calibration and diagnostic applicability.
- [ ] 20.2 Execute one fresh full Livox2 run, monitor, preserve outputs and verify input/build/source coverage without automatic retries.
- [ ] 20.3 Compare early orientation departure, pre-loop apartment return and final cloud geometry against retained Huber3; independently review and report the correction decision.

## 21. Resume from independent audit: early rotation failure
- [x] 21.1 Locate W415–416 orientation departure using retained trace, original raw gyro and saved snapshot; independently review factor signs, local rotation checks and causal limits.
- [x] 21.2 Reuse the existing map reconstruction tool at W415, preserving exact source/association/geometry and quadratic-contribution gates; identify responsible correspondence contributions without running odometry. V2 passed all gates; earlier W754-only history-check failure preserved.
- [x] 21.3 Execute one exact-control-gated W415 three-cell causal check and document the supported mechanism and correction decision. All100 control iterations reproduced exactly; three-cell omission reduced9.142869°→2.656545°. See early-heading-cause-audit.md. This diagnoses a local causal contributor; no general fix or full-route acceptance.

## 22. General correction for excessive NDT influence
- [x] 22.1 Check original input integrity around the failure and independently review a general bounded-influence correction; cauchy-correction.md.
- [x] 22.2 Implement mutually exclusive Cauchy opt-in in both NDT grids with matching cost/trace, preserving Huber/disabled behavior; validate meaningful Release tests and independent code review. Seven diagnostic cases/817 assertions pass; full suite130/131 with the same existing17.9039mm corridor failure; Sol6.1 review found no blocking issue.
- [x] 22.3 Run exact Huber-control-gated, all-cell W415 correction check; measure gyro consistency and collateral effects, document and commit without claiming full-route acceptance. Resumed checkpoint exit0; exact100-iteration control; raw-gyro disagreement9.188469°→1.639269°, all cells retained; Sol6.1 result review complete. cauchy-correction.md records collateral effects and remaining route/robustness gates.
