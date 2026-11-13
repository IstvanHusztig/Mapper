# Tasks

Applying this change starts with bounded existing-data analysis. It does not authorize any processing/checkpoint run or a positive shipped default. All tasks begin unchecked; historical research is evidence, not newly completed implementation.

## 1. Freeze references and resolve the remaining empirical decision

- [x] 1.1 Record implementation branch/base, isolated checkout, frozen Huber3 binaries/TOMLs, original Livox2/3 hashes and established W415/W754 evidence; verify a written manifest against files without running odometry.
- [x] 1.2 Produce a bounded later-failure decision from existing W904–916 logs/snapshots, separating observation from absent detailed trace; verify the report identifies any exact missing state/capture and does not repeat the established W415 ablation.
- [x] 1.3 Analyze declared stationary Livox3 surfaces/ranges and repeated support to justify one empirical residual-floor value or predeclared narrow range; deliver data/uncertainty/coverage evidence, distinguish noise from map/pose error and unsupported range/incidence extrapolation, and check whether that range affects the known influential cell widths. Stop numerical application if unjustified or inconsistent with the proposed mechanism; do not claim unique noise calibration or enlarge the range by endpoint tuning.
- [x] 1.4 Record independent review of the selected first candidate and the failure-mechanism evidence; verify no silent switch to map-statistics/projection/prior changes and update this design before any alternate branch.

## 2. Experimental residual confidence floor and focused verification

- [ ] 2.1 Add finite nonnegative metre-valued TOML load/save support with default zero and propagate to odometry options only; verify omitted/zero equivalence, positive round trip, invalid-setting rejection (including squared-sigma overflow/underflow), snapshot settings-hash compatibility and unchanged effective shipped defaults in settings tests. Distinguish numerical equivalence from saved-TOML byte changes.
- [ ] 2.2 Implement the isolated eigenvalue floor for accepted SPD cell residual covariance, preserving stored cell statistics and legacy eligibility; verify thin-cell bounds, floor rotation equivariance on a fixed eligible set, separate orientation-dependent legacy eligibility, invalid/near-singular handling and exact disabled-path arithmetic in focused numerical tests.
- [ ] 2.3 Use the same effective information in both grids' cost/Hessian/RHS, Huber weights and existing diagnostic contributions; verify cost/weight/assembly consistency, the mixed-axis reweighting counterexample, joint-state uncertainty/carry and no direct floor application in loop closure/consistency with synthetic room/corridor/open-plane and wrong-cell tests. Preserve prior models/parameters and controlled initialization; downstream estimates and outputs may differ.
- [ ] 2.4 Record startup activation/value/scope and diagnostic skip reasons; verify logs identify the option and focused Release tests pass, with existing full-suite failures separately preserved; document the experimental parameter and rollback in OpenSpec. Add Debug checks only where configuration-dependent behavior warrants them.

## 3. House-plan registration evidence, independently of algorithm changes

- [ ] 3.1 Reconcile the original room-only and broader structural overlays, their fixed source regions/identities and fit versus held-out metrics; verify an audit artifact preserves the approximately2.93-degree registration difference and mixed residuals without claiming a solved tie.
- [ ] 3.2 Identify two distinct corresponding structural anchors and plan/as-built correspondence if possible; verify one fixed-scale rigid transform across visits with held-out residuals, or record provisional/ambiguous status and the exact missing anchor information. Do not force a fit or block independent map measurements.
- [ ] 3.3 Deliver the before/after overlay and coverage/mirror/roof/fixture limitations using preserved source hashes; verify no point coordinates, per-visit transforms or scale were changed, and update OpenSpec with the user's registration review.

## 4. Authorized checkpoint and longer-segment evaluation

- [ ] 4.1 Prepare a concrete exact-control/candidate experiment manifest and estimated runtime, and obtain explicit user authorization before launching; verify approval is recorded and the runner refuses existing output paths/no automatic retry.
- [ ] 4.2 After approval, reproduce the frozen Huber W415 control before one enabled candidate check; verify full declared control equivalence, fixed-state signed translation/rotation influence and robust weights, then evolving support/motion and bias/velocity/offset estimates and finite/SPD covariance/carry with declared units. Do not rank by cost reductions across different sigmas; document independent review without claiming full-route success.
- [ ] 4.3 Prepare a longer failure segment with control-equivalent incoming map/state, identifying any additional capture separately; obtain approval for that run and verify reproduction, cumulative motion, support and state-carry results. If required state is unavailable, deliver the missing-evidence report rather than a cold-start substitute.
- [ ] 4.4 Review the longer-segment results against predeclared gates; verify failed or unsupported candidates stop here and a survivor's next experiment is proposed before any full replay.

## 5. Full-route preservation, robustness and promotion gates

- [ ] 5.1 Obtain separate authorization and freeze manifests for full Livox2 and Livox3 preservation replays; after approval verify original input/config/build identities, native completion, diagnostics and source-selection coverage with atomic collection records.
- [ ] 5.2 Compare odometry/final trajectories and matched physical geometry, revisit consistency and sharpness with preserved coverage/reflections; verify an evidence report and actual clouds are delivered for user inspection, with no cross-recording index comparison or endpoint-only verdict.
- [ ] 5.3 Propose/authorize the agreed perturbation and historical regression checks before execution; verify each retained member against the sensor-neutral evaluation contract, separate both stages, retain failures and report unresolved overlap/non-regression honestly.
- [ ] 5.4 Review all evidence independently and with the user; verify explicit adoption, rejection or unresolved status. Keep the feature default disabled absent acceptance, and document accepted behavior/settings/limits in existing sensor-neutral HTML docs with detailed recording results linked in OpenSpec before finalization.
