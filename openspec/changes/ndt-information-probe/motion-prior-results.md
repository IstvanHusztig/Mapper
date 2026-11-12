# Motion-prior experiment execution

## User visual feedback

After opening the newly exported motion-prior-probe/runs/livox1 cloud, the user
reported "has dobled walls". This is a remaining visible defect, not a clean
final result. Location/magnitude and change relative to the previous cloud are
not yet measured. Preserve the observation alongside quantitative comparisons;
do not infer success from diagnostics or local sharpness alone. This does not
cancel the already approved Livox2 check or authorize further experiments.

After the new Livox2 candidate link was delivered, the user said "it looks realy
good". Context identifies the new alpha0.1/beta0.1 Livox2 cloud. Record positive
visual progress separately from unresolved Livox1 doubled walls and quantitative
validation; this is not authorization to ship or skip robustness checks.

User approved next-step-design.md implementation and both runs on2026-11-12.
This is an execution record, not candidate acceptance.

## Implementation and review

Scratch checkout out/phase0b-worktree, starting at cb6ea83. Default beta1;
candidate alpha0.1/beta0.1. Only active-accelerometer XYZ motion observation
weights change, including attitude coupling and diagnostic cost. Main application
source/settings unchanged. Independent Codex review (not Sonnet) found no remaining
implementation/runner blocker, conditional on passing the gates below.

Implementation committed as16807ac. Root verified its binary git diff from
cb6ea83 exactly matches the frozen source.patch. Frozen candidate executable
SHA256:8e9fe49f3033570d74d8e65167129409155be220584c143a546158205c604150.

Review corrected the new fixture's use of the assembly-only early return and
added runner failure-line, snapshot/log coverage and active-multiplier checks.

## Gate progress

- Alpha1/beta1 full Release suite:124 cases passed,3 skipped,50765 assertions;
  three assertions in existing expected-failure cases, overall SUCCESS/exit0.
- New frozen finite-difference XYZ assembly and sparse-support fixture passed.
- Alpha0.1/beta1 and alpha0.1/beta0.1 full Release suites:123/124 passed,
  identical sole unexpected failure at scan_to_map_test.cpp:695, corridor
  across error0.0179039m versus0.0075m limit. This fixture has acceleration
  disabled, so beta is inactive; root verified exact assertion/value. The three
  expected-failure assertions also match. Classified as preexisting, not a new
  beta regression, not a passing full suite or relaxed accuracy requirement.
- Both same-alpha focused Release sets:3 cases/923 assertions passed.
- Candidate focused Debug:2 cases/875 assertions passed. First Debug attempt
  failed with Windows MSVC C1902 database-manager errors before tests; host
  retry succeeded, original failure log preserved.
- Eight frozen cross-build cases: NDT/cost/geometric information exact in all;
  inactive full H/RHS exact in four; active systems change as intended in four.
  Finite-difference XYZ residual/cost/attitude coupling and sparse no-match
  finite-covariance checks passed. Evidence: scratch out/motion-prior-gates/.
- Control reuse basis: beta1 unchanged compile-time arithmetic and regression
  fixtures, plus previously exact diagnostic replays. No new recorded beta1 run
  or claim of cross-build executable identity.
- Both retained controls' input/settings/log/output hashes verified before work.

Root cleared launch after independent review and the above gate classification.
Both approved mapper runs exited0 and completed export, integrity validation and
map comparison. Outputs are in
C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/motion-prior-probe/.

## Livox1 completed readings (independently reviewed)

Mapper exit0, wall656.98s; complete261-window diagnostics, snapshot/log and
identity checks, all3000 traced weights active. First candidate cloud exported.
Livox2 mapper also completed (wall544.52s) with complete validation.

| Measure | alpha0.1/beta1 | alpha0.1/beta0.1 | alpha1 context |
|---|---:|---:|---:|
| Maximum logged speed m/s |9.908|1.016|not tabulated here|
| Heading corrections >3deg / >10deg |50/16|10/1|not tabulated here|
| Local XY PCA MAD p50/p90 mm |143.94/885.07|30.29/116.30|19.50/66.14|
| Frozen-normal MAD p50/p90 mm |102.81/3346.28|35.06/120.98|23.32/73.28|
| Same-side pooled revisit p90 m |14.468|0.182|0.090|

Frozen111 patches,109 valid local-PCA measurements,767 correlated visit pairs.
Unique selected returns330018/14747244 (about2.24%), at world z0.50+/-0.05m.
All source sequence/region checks passed. Local PCA removes orientation effects;
neither this fitted-point metric nor the selected narrow slice proves whole-cloud
accuracy. User-reported doubled walls remain a visible defect; these aggregates
cannot locate or explain all of them.

Candidate pre-loop trajectory differs from alpha1 by at most1.418m, versus the
former alpha0.1 result's31.147m; this is relative displacement, not truth error.
Maximum candidate displacement from beta1 is31.174m pre-loop/31.236m final.
The previous gross runaway is absent in this single run, but remaining defects
and worse selected-region readings than alpha1 preclude calling it a fixed map.
No perturbation envelope or candidate acceptance is implied.

### Focused trace comparison

Independent reviewer reproduced trace-analysis-v2/comparison.json values. At
window107 end-Y velocity changes from-0.2543 to-0.4586m/s; directional
NDT/(motion+carry) information from0.795 to2.476. Candidate final directional
RHS contributions are NDT-3248.20, motion+697.32, carry+2550.48. Later actual
yaw changes (iteration1 state to iteration99 state) at149/151 are0.067/0.663deg
versus18.712/14.236deg. All30 source fingerprints match, with no projection
activation or step rejection in either run's3000 traced iterations.

This supports motion-prior strength contributing to the observed feedback.
Maps, associations, state and covariance evolve differently: the ratios are not
universal thresholds or a frozen-state causal isolation. No unique cause or
physical accuracy follows; doubled walls and other geometry defects remain.

## Livox2 completed readings (independently reviewed)

Complete165-window diagnostics and identity/snapshot checks; all3000 trace
weights active. Frozen114 regions,152 correlated visit pairs,217292 unique
selected returns of9551006 (about2.28%). Original alpha1-derived region indices
and definitions match exactly; all ordered source sequence checks pass.

| Measure | Previous alpha0.1/beta1 | New alpha0.1/beta0.1 |
|---|---:|---:|
| Local XY PCA MAD p50/p90 mm |22.497/90.734|22.214/90.784|
| Frozen-normal MAD p50/p90 mm |26.363/99.182|23.024/103.436|
| Same-side pooled revisit p90 m |0.308|0.342|
| Maximum logged speed m/s |0.691|0.900|
| Heading corrections >3deg / >10deg |4/0|3/0|

Maximum displacement versus previous same-alpha result is0.3454m pre-loop and
0.3444m final, not error against truth. Local spread is essentially similar;
the observed revisit tail is higher. No perturbation evidence establishes a
repeatable improvement or regression. User's positive visual feedback is retained,
but this is not a globally accepted setting or a finished Livox1 correction.

The new plan overlay reuses the retained apartment56/Wohnen control fit, verified
against its historical hash, with no candidate refit. Root inspected the output.
The existing user-attributed mirror/window returns remain unfiltered; the plan
is preliminary visual alignment, not surveyed accuracy.

## Completion and user direction

Both approved checks, map views, trace comparison and independent Codex reviews
are complete. Terra research is in slam-research.md with primary sources and
explicit corrections/limits. Implementation commit16807ac; no shipped setting
change or push. Full Release baseline and focused Release/Debug gates passed;
the candidate full suite retains the identical documented alpha0.1 corridor
failure. OpenSpec validation is required before final documentation commit.

User requested stopping further tuning investigation after these checks and
computing full Livox2 with the promising candidate, then recording a new Livox1
if the full Livox2 cloud looks good. User explicitly selected "Launch the full run now"; full-recording execution
is tracked separately in task section8. Do not start another
diagnostic sweep. Full-recording execution is distinct from shipment acceptance.

Artifacts under the external root: runs/livox1, runs/livox2_0_60, maps/livox1,
maps/livox2_0_60, maps/livox2_plan_overlay, trace-analysis-v2, gate_logs and bundle
identities. Prior artifacts remain intact. No mapper jobs remain from this batch.

User clarified priority: if full Livox2 looks good, then locate where and why Livox1 goes wrong using the same candidate. Do not assume a new Livox1 recording is the immediate next step or change settings before that decision.

Full Livox2 launched after independent runner review. Output: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-motion-prior/result/. Original raw2, exact archived TOML and frozen candidate; detailed iteration trace disabled only. Normal diagnostics and snapshot remain enabled. Full-input health exit0 with one handheld-noise warning, no timing/gyro-offset failure. Run completion/validation pending.

User requested live trajectory-jump monitoring before export. External monitor/ snapshots preserve readings. First >10deg heading correction: window414, recording148.68s, -11.192deg. At monitorwindow476/t171s no estimated/end or successive-start speed >2.5m/s, no result-guess >1m, and no guard/warning lines. This is a heading diagnostic, not proof of position teleport; run continues unchanged.

Live full-run monitoring detected the requested positional/speed failure: first reported speed >2.5m/s at window759/t272.88s (2.617m/s), peak5.242m/s at771/t277.20s; successive-start displacement speed first >2.5 at760/t273.24s, peak4.923m/s at772/t277.56s (about1.772m/.36s). No result-guess >1m or logged guard at snapshot784. User notified immediately; full run continues unchanged to preserve evidence, no new correction or rerun. High needed before causal diagnosis, routine monitoring remains sufficient at Sol Medium.

Further live event: window870/t312.84s estimated speed7.647m/s;871/t313.20s successive-start speed6.126m/s (~2.205m/.36s). At874/t314.28s15 node updates would exceed1m fromguess and were not applied; result-guess norm.998m. User notified; guard is not evidence of recovery. No causal interpretation or tuning while current run executes.

User explicitly requested stopping at the higher-effort boundary: finish current run, export and basic checks, then stop before causal diagnosis and ask user to switch to High. Do not start a new corrective investigation or additional run automatically.

## Full Livox2 completion: stop before diagnosis

Mapper exit0, wall2998.45s (~49m58s), collected_complete/evidence_complete true.
All1517windows covered for speed/heading/bias;831conditional uncertainty rows.
Odometry/final snapshots contain273060poses each. Original raw-file/settings/
frozen-binary hashes, snapshot/log coherence and output identities verified.
No input copy, rebuild, tuning or detailed trace; only this single full run.

Descriptive diagnostics:93estimated-speed crossings above2.5m/s, maximum7.647;
84heading corrections above3deg,14above10deg, maximum19.925deg. Native summary
records29big-jump node updates rejected,0solver skips and2map rebuilds. The
run-health tool's zero warning count does NOT mean no jump guards.64loops,
maximum loop correction2.56m; no universal loop-size accuracy criterion applies.
Five diagnostic trigger categories; integrity completion is NOT map accuracy.

First positional/speed warning at272.88s, larger event at312.84-313.20s;
retained monitor snapshots identify exact windows. These are descriptive readings,
not a causal explanation or proof of physical trajectory error magnitude. No
full-map accuracy/robustness comparison was performed; user inspection is pending.

Final cloud:
C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-motion-prior/result/scan_lio_merged.laz
Companion evidence: run.json, run.log, run-health.txt, preflight.json, imu-health.txt,
monitor/ and the preserved runner in the parent output directory.

All authorized processing is finished; no jobs remain. User received the cloud
and was told to switch to High before diagnosis. Stop here as requested. The
condition 'if full Livox2 is good, investigate Livox1 next' is not established by
successful export; full Livox2 has the recorded instability. No new runs or
corrections are authorized by this completion record.
