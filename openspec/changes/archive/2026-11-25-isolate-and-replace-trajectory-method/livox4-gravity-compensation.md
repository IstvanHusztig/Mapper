# Original Livox4 gravity-compensation experiment

2026-11-15. User authorized continued fixes, tests and replays without approval
stops. Preserve evidence and separate run folders. No map-quality acceptance or
default adoption implied. Charlie remains 3/19. User replay settings belong in
`settings/livox-vertical.toml` with the matching executable and persisted Huber3.

## Existing-data evidence

Original control: cbca36d, alpha/beta0.1, Huber3, external experiments folder
`livox4-indoor-outdoor/huber3-baseline`. Rejected comparison: a27fa33 at
`livox4-w214-restricted-125s`; do not stack this experiment onto that solver.

The exact same 27,299 user-marked returns were reprojected through saved poses.
All346 common window timestamp arrays match; independent review checked each
selected return's pose index against its time. Early15s position disagreement
is at most0.98mm. One preliminary fixed plan alignment is used.

| Original observation | Reading |
|---|---:|
| Marked returns111-114s, median plan y |12.76m (137 returns)|
| Marked returns114-116s, median plan y |2.92m (2284 returns)|
| User-indicated approximate wall line |y=10.9m|
| Maximum boundary displacement W301-321 |8.63mm|
| Within-window displacement W320 / W321 |1.749m /0.054m|

The speed reset at W321 is not a position teleport: drift has accumulated
before it. Post114s cross-configuration marked-point disagreement is dominated
by translation, not rotation of the ray. Selection is sparse before114s and may
mix surfaces; the plan alignment is unsurveyed and has competing hypotheses.
This does not identify the first physical error in the entire route.

Yaw-independent saved-map versus AHRS tilt rises within W298-301. W298 starts
at0.605deg and ends1.203deg; subsequent endpoints2.430,5.079,7.553deg. W301
starts108s. Existing original-control reconstruction also shows node164 (before
tail replacement) world-X/Y rotation corrections [-.440,.436],[-.777,.929],
[-1.120,2.363],[-1.701,1.824]deg. Tail replacement alone cannot explain this.

Independent tilt-only sensitivity, retaining heading, gives mean horizontal
acceleration difference1.31m/s² across W306-320, almost entirely explainable
by gravity direction. These saved poses are post-handoff; exact pre-LM attitude
and evolving gravity carry were not saved. AHRS can itself be wrong. This is
evidence for a focused experiment, not actual predictor acceleration or a cure.

Python artifacts: original run's `existing-wall-check/measurements.json`,
`motion-decomposition.json`, `marked-wall-comparison.png`,
`wall-motion-timeline.png`, and `tilt-sensitivity/measurements.json`, with
reproduction scripts. Reviewer: seed_damping_crosscheck (Codex, not Sonnet).

## Reviewed candidate

Worktree `C:/Repo/dMapper-worktrees/livox4-ahrs-gravity`, branch
`codex/livox4-ahrs-gravity`, starts at cbca36d. The original always-on predictor
and joint factors use `scale*(R*f-g)`. Elsewhere `world_acceleration` removes
AHRS gravity in body coordinates. Apply that reference consistently:

`u=f-g0*R_AHRS^T*z; a=scale*R*u-carried_world_offset-window_offset_increment`.

Freeze startup gravity magnitude/scale. Carry world offset separately, adding
the solved increment once; retain covariance semantics without folding into
gravity. Use centered u in the attitude Jacobian too. A residual-only change
would preserve erroneous gravity-sized derivatives. Keep disabled legacy
numerics unchanged. Expose opt-in TOML and Huber3 in both CLI/GUI. Do not change
weights, gain, cell support, match policy, tail policy or solver acceptance.

Limitation: AHRS tilt can absorb real acceleration; this candidate may prevent
velocity feedback yet leave rotation or other map errors. It is not adoption.

## Checklist and rejection gates

- [x] Existing-data localization and independent evidence/design review.
- [x] Implement consistent predictor/factors/Jacobian/offset carry and config.
- [x] Tests: gravity-only tilted pose, nonzero-motion finite differences,
  two-window offset carry, legacy equivalence, config/hash compatibility.
- [x] Independent implementation review and Release build. Execution results below.
- [x] Freeze executable and livox-vertical.toml together; verify alpha/beta0.1,
  Huber3, input identity and cache ownership before running.
- [x] Test existing125s prefix from original start. Inspect early indoor geometry
  and W298-325 tilt/speed/wall placement/coverage; record runtime. A short result
  can reject the candidate but cannot certify the map.
- [ ] If local geometry survives, complete Livox4 and Livox3 indoor comparisons.
  Report odometry/final separately. Endpoint closure alone is insufficient;
  perturbation and visual gates remain required for robust/adopted improvement.

If this hypothesis fails, do not add another recovery layer. Preserve the
failure and reassess the original estimator or the saved replacement roadmap.

## Implementation and preflight

Worker `gravity_candidate` used Sol6.1 medium in the isolated worktree;
`seed_damping_crosscheck` independently reviewed source and native tests.
Review corrected startup scale to match original norm(mean(R*f)), added
effective-NED rejection, persisted effective Huber configuration/hash and a
moving-predictor test. These were resolved before freezing recording binaries.

Nominal Release suite:131 passed,3 skipped,3 pre-existing expected-failure
assertions. Matched alpha/beta0.1 Release focused tests:10 passed/159 assertions.
Matched full suite:131 passed,1 failed,3 skipped. The additional degeneracy-aware
NDT blind-direction threshold failure was reproduced with the historical0.1
binary: both report17.9039mm across against7.5mm threshold; along metrics also
match. It is not a new candidate regression. Logs are retained in the worktree's
build evidence and copied to the frozen run below.

Actual matched dMapper SHA256:
`7f0f4a87a4085f741fe0f616ba8478c404bb596a3514eb70e22a6b28806beba7`.
Candidate livox-vertical TOML SHA256:
`62f4725a7f33481163bb8a7efc91ca23c2947f3580d5b16bb0a9f35982921198`.
Both CMake cache and compiler definitions were checked at0.1/0.1. Fresh CMake
defaults were1/1 and were corrected BEFORE freezing any replay; the nominal
binary is not the recording candidate.

Frozen external folders are `livox4-ahrs-gravity-control-125s` and
`livox4-ahrs-gravity-125s`. Each contains its own binary, source patch/identity,
CMake cache, `livox-vertical.toml` and matching GUI default settings. Only the
compensation bool differs between these controls. Against original archived
settings, the only additions are that bool and persisted Huber3; no hidden
DMAPPER environment options are inherited. The sequential runner requires exact
first325 whole snapshot window payloads against the original full control before
starting the candidate. Source/raw caches stay in their existing locations.

Reviewed implementation committed as `cd212ba88cad5a81be30d5c60d6e143d09e75b2d`
in its isolated branch, worktree clean. Frozen manifest retains the cbca36d base
plus the exact patch and file hashes, equivalent to this commit. Sequential
control-then-candidate helper started (parent PID31180); no concurrent recording
jobs. Sol6.1-medium worker monitors. The user explicitly authorized this scope.

Control completed exit0 in567.21s native processing. Identity gate passed:
325/325 complete window payloads byte-identical to the original full Huber3,
including poses, times, samples and joint state. Run-health exit2 retains the
pre-existing coverage limitation; identity is not an accuracy verdict. Candidate
started automatically after this gate, PID13168. No candidate result yet.

Per user's request, root `settings/livox-vertical.toml` now exactly matches the
frozen candidate TOML. Matching executable is the candidate run's
`bin/dMapper.exe`; root's older executable does not implement these options.
Previous root profile is preserved as `livox-vertical-before-candidate.toml`
inside the candidate folder. This is experimental selection, not adoption.

## Outcome: does not fix the map; no full replay

Candidate native processing547.04s (563.05s helper elapsed), exit0,346windows.
Peak speed falls5.086->2.220m/s, but largest yaw correction increases3.480->8.697deg.
First60s position disagreement reaches0.773m. Same23,442 late marked returns
remain7.754m off the user line in odometry and7.285m after postprocessing.
The plan comparison visibly retains the misplaced wall and changes its angle.
No full candidate or indoor replay is justified by this result. Independent
review confirmed this decision; lower speed is not a map fix.

Artifacts remain under `livox4-ahrs-gravity-125s/quality/` and `quality-odometry/`.
Root `livox-vertical.toml` is now the verified control profile (compensation=false,
persisted Huber3) so this unsuccessful compensation is not selected there.

## Bounded original-factor trace

Next step is measurement, not another algorithm layer: port only the107-line
f400479 diagnostic additions onto cd212ba, compensation=false, alpha/beta0.1,
Huber3. Earlier f400479 data were from alpha/beta1 and a gain change, and cannot
attribute the original first bad update. Trace W297-301, all100iterations:
W297 is pre-onset; W298-301 contains the rising tilt. Source reviewer confirms
production assembly/update/covariance unchanged. Shared-H responses are not
counterfactual factor-removal runs or proof of physical correspondence validity.

Disable downstream loop closure, consistency and merged export for this
diagnostic only. Require all346 complete snapshot payloads byte-identical to
the newly verified125s control before analyzing factor responses. Then require
500 unique iterations, applied updates, no dropped matrix writes, association
identity and additive/linear reconstruction. Expected trace size~300MB. Preserve
3600s window limit. Own result folder: `livox4-original-tilt-trace-125s`.

Existing Python reduction now accepts a bounded window range and reports XYZ
rotation responses plus yaw-independent tilt change from each window's seed.
Rechecked on historical400-iteration trace: all original fields unchanged and
no reconstruction errors. This reuses existing tools rather than a new viewer.

### Original trace result (2026-11-15)

Trace-only source402e1247, matched alpha/beta0.1, compensation=false.
Native437.76s; helper449.293s. All346 complete snapshot payloads exactly equal
control. All500 W297-301 iterations reconstruct native updates, with no rejected
steps, dropped writes or assembly-association mismatches. Maximum full-step
reconstruction error1.29e-13. Evidence: external
`livox4-original-tilt-trace-125s/{identity.json,factor-analysis.json,tilt-factor-summary.json}`.

Independent reviewer seed_damping_crosscheck confirms the original onset is NDT:
node164 W298i0 XY step0.349782deg, NDT response0.349971deg, accelerometer
-0.000189deg along that step; W299i0 step0.657249deg, NDT0.657321deg.
At W300i2, NDT0.207586deg, carried velocity0.072763deg and accelerometer0.041423deg
combine into0.320100deg. This separates initial geometry forcing from later
velocity/IMU amplification; it does not establish physical correspondence error.

W298i0 near-range contributions:0-2m0.218304deg,2-5m0.131667deg; farther ranges
zero. Angular-only NDT RHS opposes XY tilt (-0.13874deg); translation RHS coupled
through the shared Hessian contributes+0.48871deg. W299 differs, so a blanket
angular penalty is not justified. These are shared-H responses, not factor-removal
experiments. Frozen robust cost decreases throughout W298: an acceptance cost
gate alone cannot detect this first bad tilt.

Next bounded existing-data check reuses the native map/cell inspector for original
W298 selected iterations and W299i0, requiring source, association, geometry,
count and cost identities. Prior alpha1/gain bad-cell findings are not substituted
for this original-control evidence. No new processing replay required for this
inspection, no further solver layer selected. Full gravity candidate remains
unjustified by its failed geometry. Charlie remains3/19.

Focused next candidate: `livox4-original-match-admission.md`. Exact native original-control cell inspection completed and independently reviewed. Implement only per-iteration min6/SPD/residual6 admission on alpha/beta0.1 Huber3, preserving original solver and map statistics. Earlier alpha1 guard trials are not this comparison. Tests/review and fixed-prefix geometry must precede any full candidate.
