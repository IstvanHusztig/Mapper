# Tasks

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Superseded by the region agents and the Livox 4 fixes; nothing in the code; spec not synced.**

Latest design-only detour: proposed W214 restricted recovery and its checklist
are in `translation-recovery-fix.md`. The user requested preserving weak-direction
and rotation guards, then authorized implementation, tests and independent
review. Implementation is complete in experimental commit a27fa33, with full
Release and focused Debug validation. Recording validation remains pending.

Latest diagnostic detour: user authorized repairing checked-solver tracing and
preparing reusable W301 cases. See `solver-case-capture.md` for the local
implementation/test checklist and `108s-boundary-stall-analysis.md` for evidence.
No new recording or captured-case replay is authorized. Charlie remains 3/19.

Current local detour: the user authorized the NDT match guard implementation in
`ndt-match-guard-plan.md` (follow-up: `ndt-iteration-refresh.md`). Its own checklist tracks preparation/testing; it does
not complete the interface/replacement tasks below. Charlie stays 3/19. Stop
before any new recording or captured-window replay.

Planning only. Apply starts with group 1, not an algorithm change. Each new
processing/checkpoint run needs explicit approval of its concrete scope. The
separately authorized stronger-gyro Livox4 replay does not authorize other runs.
Groups 2 and 4 require the preceding evidence/design gates. Default adoption and
legacy removal remain separate user decisions; no push is authorized.

## 1. Isolate the responsible computation

- [x] 1.1 Freeze the evidence/control inventory, including source/configuration identities, existing synthetic checks and rejected candidates; verify the inventory links to preserved artifacts and distinguishes shipped defaults from frozen Huber3.
- [x] 1.2 Trace Livox4 W301–320 prediction growth separately from W395–410 angular corrections and the Livox2 events; deliver a state-transition table naming calculations, observed values, upstream uncertainty and missing pre-LM state, independently reviewed against existing data.
- [x] 1.3 If existing data cannot isolate the initiating transition, prepare the smallest prefix capture and bounded failure-window replay, including incoming state/map, identity checks, sequential-equivalence test and cost. Obtain explicit run approval, then collect only that scope; never treat a raw cut or downstream-only snapshot as an equivalent restart. If faithful restart is unavailable, document the constraint and the minimum prefix cost.
  - Completed through the approved sequential-prefix fallback: unchanged candidate,
    incoming state/matched-map identities,400 iterations and346 exact snapshot
    payloads. See `livox4-rotation-factor-trace.md`. No standalone restart or
    reusable full-map checkpoint is claimed; specific offending cells remain open.
- [ ] 1.4 Deliver the reviewed isolation decision and concrete interface contract covering the smallest coherent coupled state; verify the note distinguishes proven cause from guard weakness and defines units, frames, lifecycle, failure and commit semantics before extraction.

## 2. Extract and preserve the current method

- [ ] 2.1 Move the identified component behind the agreed interface without numerical changes; add lifecycle/state-ownership and failure-contract tests and document the boundary, verifying existing focused tests and controlled legacy fixtures remain exact.
- [ ] 2.2 Add stable per-run TOML selection with missing-key legacy default, shared CLI/GUI wiring and clear unavailable-method errors; verify round-trip/default/unknown-ID and both-front-end tests, and document selection through the existing Load parameters workflow.
- [ ] 2.3 Persist method/configuration/state identity and define historical legacy snapshot compatibility; verify matching restart, cross-method/configuration/version rejection and legacy compatibility tests, with format documentation beside the implementation.
- [ ] 2.4 Obtain approval for required recording equivalence checks, then compare extracted legacy against frozen controls and the project's 44_short/full-44 golden requirements; publish numerical equality, provenance-only differences and known test failures separately before candidate implementation.

## 3. Select a research-grounded replacement

- [ ] 3.1 Reuse the existing Terra report and failed experiments, then research primary papers/original implementations against the isolated mechanism; deliver a focused comparison of assumptions, observability, Livox/cave applicability, dependencies, licensing and evidence gaps.
- [ ] 3.2 Select one candidate in a reviewed design decision with equations, expected discriminating effect, rejection criteria and interface fit; verify every material claim has primary evidence or is labeled a hypothesis, and revise this plan before coding if the seam must change.
- [ ] 3.3 Freeze the candidate evaluation protocol and physical regions using the agreed quality contract; verify indoor preservation, outdoor transitions, historical regression, perturbation family, coverage, odometry/final stages and explicit run-approval gates are specified without tuning thresholds to candidate results.

## 4. Implement one opt-in candidate

- [ ] 4.1 Implement the reviewed candidate as a separate method with isolated dependencies and no default change; add math/state/failure tests for its documented assumptions and verify the legacy control still matches, documenting unsupported operations and diagnostic meaning.
- [ ] 4.2 Add candidate configuration validation, provenance and restart behavior; verify round-trip, malformed-setting, cross-method-state and method-removal compatibility tests, and document how users select the experiment.
- [ ] 4.3 Exercise the isolated mechanism using existing valid fixtures or a separately approved captured-state check; publish the predefined measurements and counterevidence, and pass the local gate before requesting full-route validation.

## 5. Validate map quality and robustness

- [ ] 5.1 Prepare concrete indoor/outdoor replay commands, frozen inputs/settings/builds, runtime/disk estimates and fresh external result folders; verify approval and cache/executable ownership before each new processing scope.
- [ ] 5.2 Collect the approved indoor Livox3 and outdoor-transition/full-route Livox2/4 comparisons; verify complete diagnostics and artifacts, then publish geometry/revisit/sharpness/coverage and actual endpoints for odometry and final stages with clouds for the user's visual review.
- [ ] 5.3 Run separately approved perturbation and historical 44 s0/s1/s2 gates; verify all members remain visible, report empirical envelopes under the agreed contract and label incomplete or overlapping evidence accurately rather than declaring non-regression.

## 6. Decide adoption or removal

- [ ] 6.1 Present the reviewed evidence and obtain the user's keep/reject/default decision; record the decision and unresolved limits in OpenSpec, with no automatic default switch or legacy removal.
- [ ] 6.2 Apply only the chosen disposition, update the existing sensor-neutral HTML documentation and method guidance for accepted behavior, or remove rejected implementation/exclusive dependencies while preserving evidence; verify retained-method behavior and actionable removed-ID errors. Legacy removal requires explicit separate authorization.

Acceptance-fix implementation: `rotation-objective-acceptance-plan.md` specifies the
complete residual/trial/atomic-commit work and synthetic gates. Its checklist tracks the authorized scratch implementation and validation.
Complete fixed-support merit is now separate from the old partial diagnostics.
No new recording run is authorized. Broader roadmap remains3/19.

Local acceptance checklist now complete for implementation/preparation: source
1f0abc5, golden fixtures be898d6, Release146test cases passed and8Python checks.
Prepared125s replay remains approval-pending; no map validation or broader
interface milestone is complete. Details in rotation-objective-acceptance-plan.md.


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


Latest authorization: split checked proposal/trial/commit and fix the stale end
velocity seed plus lost successful damping. Keep guards and refresh. Track this
local work in acceptance-seed-damping-fix.md; no new recording replay authorized.
The broader Charlie interface roadmap remains3/19.


Latest completed local fix: scratch ba0268c (clean) splits proposal/evaluation/
commit, seeds the free end velocity from the position integrator in the enabled
path, and carries accepted damping to the next iteration. Guards, refresh and
rotation/merit limits retained; disabled processing unchanged; enabled snapshot
policy v3. Release151cases pass (3skipped), focused Release+Debug14cases/997
assertions each, Python9pass. Independent review completed and its findings
addressed. See acceptance-seed-damping-fix.md for artifacts/limits. No new
recording replay, cloud, push or adoption. Stop for approval before any run.
Charlie3/19. Medium sufficient for preparation/monitoring; high for analysis.

Current authorized local detour: `translation-recovery-fix.md` addresses the
saved W301 i51 loss of feasible translation descent while preserving all guards.
The broader interface tasks above remain unchanged; no recording replay approved.
