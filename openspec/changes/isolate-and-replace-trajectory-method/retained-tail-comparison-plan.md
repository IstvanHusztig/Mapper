# Retain the optimized tail: reviewed comparison

2026-11-14. High-effort follow-up to `tail-factor-check.md`. Preparation only;
no new recording or captured-case replay. Charlie remains 3/19.

## Decision and evidence boundary

Recommend one configuration-only comparison with `handoff_tail_nodes = 0`.
Keep the accepted optimized poses instead of replacing the last15 poses after
the solver commits its velocity and covariance. The existing executable
supports this; no source change or rebuild is needed.

Four saved W301 calls prove that replacement can invalidate the fitted
end-velocity factor (~0.026 m/s added residual). They do not establish how much
of the real W313 jump or displaced exterior is caused by it. Removing replacement
changes subsequent poses and map from W1; this is a handoff-policy comparison,
not an isolated W312 intervention or a proven map fix.

## Coupled-state audit

Paths below are relative to `out/ahrs-gain-worktree`, diagnostic HEAD9987917;
the frozen recording executable is numerical source d213b75.

- `src/domain/window_steps.cpp:198`: zero tail nodes make replacement an exact
  no-op. Otherwise it reconstructs translation and rotation without updating
  velocity/covariance or rechecking solver acceptance.
- Bias estimation remains active. `src/domain/scan_to_map.cpp:934` subtracts
  the estimated bias update in the motion factor; the application carries bias
  at `src/application/run_odometry.cpp:822`; `apply_gyro_bias_prior` at
  `src/domain/window_steps.cpp:141` corrects the next window's initial rotations.
  Do not apply that bias again to the optimized tail. The candidate gives up
  exact IMU rotation reconstruction there; fitted poses can retain LiDAR bends.
- At `run_odometry.cpp:1106`, geometric velocity uses the previous last10-node
  span. In this TOML it advances the next first pose across the sample gap.
  At line265, valid carried optimized end velocity wins for the new start prior,
  with covariance grown over that gap; moving-window integration then starts
  from that prior at line1160. These remain distinct velocities. Rest detection,
  invalid/missing carry and the5 m/s guard keep their existing behavior. The
  legacy geometric-velocity factor is off while the accelerometer factor is
  active (`scan_to_map.cpp:1098`).
- The raw cross-window IMU increment still places the next first pose. Bias
  correction anchors that pose and corrects later increments. This boundary
  approximation is shared by both policies, not fixed here.
- Velocity covariance is an undamped base-linearization marginal block
  (`scan_to_map.cpp:1396`), preserved during translation recovery. Retaining
  accepted state avoids another pose-only mutation; it does not recompute a
  final-pose posterior, restore cross-window correlations or calibrate chi-square.

An inertial tail rewrite would need an explicit acceleration, bias, terminal-state
and uncertainty model plus acceptance checks. Copying geometric slope into end
velocity drops the inertial terminal correction; reordering carry copying alone
does not update that velocity. Defer these alternatives pending the simpler test.

## Frozen candidate, not started

External base: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/`.

- Candidate `livox4-retained-tail-125s/`; completed control
  `livox4-translation-recovery-125s/`; verified existing input
  `livox4-ahrs-gain-prefix/input`. Same125-second prefix, expected346 windows.
- All four executable/DLL files are byte-identical to control. Executable SHA256:
  `614765CC2B309DB8498EC1F97A79BD3316C8AD138BA36AAC988C934A2E542C0B`.
- Parsed TOML has exactly one change: `[gyro_bias_estimation]`
  `handoff_tail_nodes = 15` becomes `0`. Settings SHA256:
  `8359C8F5B693E7839D83CB678281B46945FE75FEF84B865088FDC367CD26E7A0`.
- Huber3 remains `DMAPPER_DIAGNOSTIC_NDT_HUBER3=1`; clear other diagnostic
  overrides. Keep guards, refresh, recovery, bias, alpha/beta and postprocessing.
- Manifest holds verified input/binary hashes, `approval=false` and
  `recording_replay_started=false`; result directory absent. CLI arguments after
  approval: `<input> <candidate/settings.toml> <candidate/result>`.
- Estimate45-120 minutes; control took63 minutes. Recheck disk, cache/executable
  ownership and hashes at launch. CIM process inventory was access-denied during
  preparation: ownership remains unverified and must be resolved before launch.
  No old snapshot reuse: this setting participates in configuration identity.

## Predeclared checks

1. Require successful346-window completion and equal timestamps/node counts.
   Audit1-degree step/3-degree prediction-relative rotation limits and fixed
   merit acceptance. Early trajectory equality is not expected after changing W1.
2. Compare W87(~31s), W297-313(~107-112s) and through125s. Use snapshot/log
   terminal geometric and optimized velocities, next start velocity, gap motion,
   bias/end attitude, recovery count, speed and jump magnitude/covariance.
   Diagnostic eligibility may change; fewer weak-axis rows are not improvement.
   Actual final pre-tail factors in control were not captured: do not fabricate
   them from the replaced snapshot tail.
3. Use existing run-health and equal-timestamp trajectory checks, separating
   odometry/final stages and postprocessing displacement. Freeze physical wall
   regions/extraction from control before examining candidate metrics. Report
   geometry, revisit offsets, sharpness and coverage separately; supply both
   clouds for the user's look. The screenshot rectangle is not registered to
   a precise physical/time region yet.
4. No full-route closure verdict from this prefix. Robustness, wider indoor/outdoor
   validity and recording44 gates remain NOT MEASURED. One pair tests handoff
   consequences; it cannot establish general improvement or default adoption.
   Additional runs still need separate approval.

## Verification and independent review

Reviewer `seed_damping_crosscheck` independently inspected source and evidence,
read-only, with no delegation/runs. It supports this one-setting comparison and
highlighted the two velocity paths, loss of IMU tail reconstruction and covariance
limits incorporated above. Its review did not establish a W313/map fix.

Existing Release tail and bias-prior tests:2 cases/299 assertions passed, log
`out/retained-tail-unit-check.log`. Parsed TOML diff and all binary identities
checked; source unchanged. No build or Debug run needed. Medium is sufficient
for preparation/monitoring; request High before unexpected numerical diagnosis
or choosing production policy. No replay launched or default changed.

## First launch

The user approved the replay on medium. The frozen files passed hash checks,
no dMapper process was present before launch, and about 80 GB was free.
PID 31664 loaded the settings and point cloud. The sandboxed process check
gave a false negative during loading. On the user's retry approval,
the retry preflight detected an active dMapper process and did not launch a
second one. An elevated process inventory confirmed that original PID 31664
was still running the candidate and writing solver logs at W24. The original
manifest's `running` state was correct at that check.

## Window 214 stall

The original replay completed 213 windows. Stderr records `Fatal error: Step
acceptance exhausted at window 214 iteration 9: merit_not_decreased`. W214 starts
at 76.68 s on the elapsed recording clock. No cloud was exported. The launch
did not capture an exit code.

- W214 accepted iterations 0–8. At iteration 9 the fine grid dropped one
  association and the coarse grid was unchanged. All 48 subsequent trials used
  the same frozen source and target hashes. Across six damping values (5 to
  500,000) and eight scales (1 to 1/128), the evaluated merit rose by 0.000000396
  to 0.040239 from 189231.251487. The strict-decrease tolerance was about
  0.00000000269. Max step rotation was 0.001173° and max prediction-relative
  rotation was 0.689180°, both inside the 1°/3° guards.
- First full-scale trial: NDT cost rose 0.509524 while motion, velocity and
  IMU fell by 0.380016, 0.066534 and 0.026096 respectively. The origin penalty
  added 0.003332. The NDT increase also dominates the smaller trials. The
  refresh changed the objective before the trials, but the logs cannot isolate
  the removed match as the cause of the failed search direction.
- `step_acceptance.cpp:63` evaluates unprojected merit, whereas the proposal can
  downweight weak LiDAR translation directions (`scan_to_map.cpp:824–842`).
  `step_acceptance.cpp:387,423–425` enables translation recovery only if at
  least one valid ordinary proposal had identity projection, the frozen bias
  is within bounds and base rotation is within budget. All six proposals reached
  trial evaluation. Iteration 8 committed a bias clamped to the same bound and
  a 0.689156° correction. Recovery was therefore skipped because the proposals
  used active projection, inferred from the code and absent recovery rows.
  Exact projection eigenvalues and proposal vectors were not logged.
- Independent reviewer `seed_damping_crosscheck` confirmed the gate deduction
  and cross-checked the bias using the logged prior uncertainty and bias cost.
  With a positive definite prior, the implied bias is below 0.13°/s per axis,
  well within the 20°/s limit. Adjacent-window information ratios cannot stand
  in for the missing W214 iteration 9 projection eigenvalues.
- Tail-15 control passed W214 iteration 9 on its first trial. Rounded logged
  window-start positions were already 7.7 cm apart at W94 (33.48 s), 1.15 m
  at W100 (35.64 s) and 1.53 m at W213 (76.32 s). These are differences
  between runs, not absolute errors. W214 identifies the fatal failure in the
  search and recovery policy, well after the trajectories diverged.

The logs establish the failed descent and skipped recovery. They do not resolve
whether the projection alone reversed the descent, or whether a feasible
translation-only step exists that respects the weak-direction policy. Evaluating
that requires the saved W214 state, matches and derivatives, which this replay
did not capture. No new processing or algorithm change was performed.
