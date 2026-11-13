# Approved short trace of the guard's first translation departure

User approved the short trace around 31 seconds. Current high effort suffices.
Use the existing experimental worktree, preserve alpha1/beta1/Huber3 and all
guard settings, and process one 50-second prefix from the original beginning.
This is a sequential prefix, not a cold restart at 31 seconds. No ablation
trajectory or further replay is authorized. Charlie remains 3/19.

New opt-in diagnostic selector `DMAPPER_TRACE_LIVOX4_EARLY=1`: context/state
W84–102; factor traces at iterations 0,1,2,10,99, with all 100 iterations of
W87. Existing diagnostic factors expose directional curvature/RHS and each
factor's response under the same joint Hessian. Numerical processing unchanged.

At W87 iterations0/99 only, assemble three models on private copies of the same
candidate state and map: actual frozen matches, newly admitted matches, and
legacy rematching. The existing assembly-only probe returns before solving,
covariance calculation or state commit. Compare actual-factor sparse-H hashes
and full RHS against the observational frozen assembly. Alternative models are
local calculations, not successful counterfactual trajectories or reconstructions
of the reference map. Do not interpret non-comparable matched-only costs as an
acceptance test. Unsupported alternatives must not be silently replaced by zeros.

Require exact equality of the first130 odometry snapshot window payloads against
the completed 125-second guard replay before causal interpretation. The time
limit remains3600s/window. Validate trace completeness, factor-response sums,
assembly hashes and RHS, and source/match identity. Use an explicitly described
translation ramp when examining resistance toward the rounded reference
correction; saved post-tail poses are not the original pre-LM reference state.

Independent read-only design review: trace_review, Codex (not Sonnet). No
whole-algorithm fix is bundled. Fresh external folder:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-guard-w87-trace`.
Estimated processing3–8minutes plus diagnostic build time. Preserve input,
settings, source, executable, runtime DLL and result identities; no retry on
identity or numerical-equivalence failure. Report actual runtime and limitations.


## Collected trace and failed reproduction gate

Scratch diagnostic commit9635964; Release12cases/2365assertions pass (guard,
Huber and trace parity). No Debug build: observational arithmetic/logging uses
existing probe infrastructure on ordinary scoped copies, with no new threading
or persistent lifetime mechanism. Native replay exited0 after214.877s,137windows.
All first130 snapshot payloads differ: run.json is **identity_failed**. No automatic
retry was performed. Therefore this is not exact evidence for the original W87.

The preparation mistake was `cut_recording ... 0 50`: zero starts at the first
IMU sample (0.00246995s), not the first LiDAR point (0s). This dropped491leading
LiDAR returns,490of which pass the production startup filter. The60001-point seed
ends at0.357340s instead of0.354030s. The first saved pose shifts from0.35440164s
to0.35866811s, about two IMU nodes. W87 rawstart similarly shifts31.31450537s to
31.31869725s. Initial IMU rows match. Timestamp ties are not the explanation:
the first150000raw LiDAR timestamps are unique. Independent review confirmed
this preprocessing mismatch. The differences begin before tracing activates.
The cutter itself behaved as documented; the requested cut interval was wrong.

## Local factor evidence: altered input only

`factor-analysis.json` explicitly records `original_run_reproduced=false`.
Internal checks pass:190actual traced iterations,6alternative assemblies; all
frozen-alternative factor Hessian hashes and complete RHS match production at
W87i0/i99; factor-response reconstruction and dropped-write checks pass.
These checks validate the local observations, not original-run equivalence.

The descriptive direction is an endpoint displacement(-0.061,+0.059,-0.006)m,
the difference of the earlier rounded reference/guard corrections, expressed as
a constant velocity ramp over0.36s. Both velocity variables change with the ramp;
attitude/bias/offset increments remain fixed. It is not ground truth, an actual
reference-state replay, or the fully reoptimized minimum. For each factor,
report curvature d^T H d, linear push b^T d, and q=d^T H d-2b^T d.

At iteration99:

| Factor | Curvature | Linear push | Local quadratic cost change q |
|---|---:|---:|---:|
| NDT scan matching |71001.81|726.27|69549.27|
| Motion edges |4037.56|-379.72|4796.99|
| Carried start-velocity prior |3916.88|-345.89|4608.65|
| Accelerometer residuals |approximately0|approximately0|approximately0|

NDT supplies about89.9%of curvature and88.1%of the positive local cost increase
in this direction. It is the dominant local resisting term, not the gyro prior.
The near-zero accelerometer curvature here is specific to a constant-velocity
ramp; it does not mean accelerometer coupling is unimportant in other directions.
At iteration0 NDT also supplies the initial correction response (about7.3mm
projected along this direction). The factor supports a small correction while
strongly resisting a larger one; it is not simply pushing the wrong way.

Same-candidate-state alternative NDT linear push:

| Iteration | Frozen | Fresh admission | Legacy rematching |
|---|---:|---:|---:|
|0|13883.20|13883.20|23160.01|
|99|726.27|3976.67|13382.98|

At i99 fine/coarse match counts are17715/19063(frozen),18295/19115(refreshed),
21279/20559(legacy). Fixed and refreshed models agree at i0 as expected. Later,
refreshing increases corrective push; admission filters also suppress part of
legacy's push. Thus neither freezing alone nor cell filtering alone has been
isolated as the complete original cause. Per-source gained/lost/reassigned counts
are not exported; aggregate counts and association hashes establish change only.
No alternative model was solved; all use this candidate's map, not the reference
map. No claim is made that replacing it would recover the correct trajectory.

## Corrected preparation; approval still required

Prepared only: external `livox4-guard-w87-trace-corrected/manifest.json`, status
prepared_not_run, approvedfalse. Same frozen diagnostic executable/settings/DLLs.
Cut uses start-10,duration60, preserving all leading LiDAR while keeping the same
50.00246995s end. Native cutter verification passes; the first200000LiDAR records
match the original exactly and all25001IMU header/data lines are an exact prefix.
The manifest hashes corrected inputs, original inputs, reference snapshot/log,
executable/settings/DLLs and cutter. No dMapper run was launched in this folder.

The safest fallback is unchanged125sinput; the prepared corrected50sprefix still
must pass the first130-window exact gate. Request explicit approval before either
additional replay. Current high effort is sufficient. No algorithm/default change
or adoption. Charlie remains3/19. Preserve failed attempt and its evidence.

## Corrected replay — exact reproduction confirmed

User explicitly approved “Run corrected trace”. The prepared corrected bundle
was run once, without algorithm/settings changes. Native exit 0; 211.590 s wall
time; 137 windows. All first 130 odometry window payloads match the original
125-second guard run **exactly**. `run.json` status is `validated`; the original
preparation manifest remains an immutable record of the earlier unapproved state.
The failed first attempt is retained separately.

External bundle: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-guard-w87-trace-corrected`.
Log SHA256: `d33dcc6c18f7817d6417536575e46e61623ac35e3d715c9293078ef7a6825194`.
Snapshot SHA256: `319223671b21fe67ca0a0283d35ae9093b8e6daa812edf2b1a57cc6b3d6f53a6`.
`factor-analysis.json` records original_run_reproduced=true, no internal errors,
190 actual iterations and six alternative assemblies. Frozen alternative hashes
and full RHS match production at both sampled iterations.

### What actually blocks further correction

W87 (30.96 s log-relative time) makes a total endpoint correction
(-0.00440318, +0.00665278, -0.00181734) m, approximately 8.18 mm.
The first iteration already supplies about 7.57 mm; by iteration 10 the next
endpoint step is only about 7 nanometres. The factors nearly balance under the
guarded local model. Late small steps also reflect scheduled damping; they do
not prove a global optimum. The update is executed, not skipped.

For the same descriptive fixed-attitude velocity ramp defined above, iteration 99:

| Factor | Curvature | Linear push | Local quadratic cost change |
|---|---:|---:|---:|
| NDT | 70814.93 | 696.76 | 69421.41 |
| Motion edges | 4043.23 | -374.84 | 4792.90 |
| Carried start-velocity prior | 4141.74 | -321.29 | 4784.31 |

NDT supplies 89.64% of directional curvature. However, **curvature and the
opposing gradient are different questions**: NDT still pushes toward a small
correction; the motion edges and carried velocity prior oppose it. At iteration
10 the shared-H endpoint responses projected along this direction are NDT
+2.234 mm, motion edges -2.283 mm, accelerometer +0.148 mm and carried velocity
prior -0.099 mm (other terms complete the near cancellation). These are additive
RHS responses under the same joint damped Hessian, not factor-removal experiments.
Thus do not say NDT alone pushes backward or gyro alone blocks the solve.

All three translation-projection scales are 1 at the inspected iterations
0, 1, 2, 10 and 99. Weak-axis projection is not suppressing this correction there.
Near-zero accelerometer curvature remains specific to this constant-velocity
ramp; it does not rule out inertial/attitude coupling in the full system.

### Why the guarded NDT model loses corrective pull

Same state and map; no alternative trajectory was run:

| Iteration | Frozen NDT push | Refreshed admission push | Legacy rematching push |
|---|---:|---:|---:|
| 0 | 11325.35 | 11325.35 | 19167.64 |
| 99 | 696.76 | 3918.16 | 11945.09 |

Refreshing valid associations at iteration 99 increases the corrective linear
push 5.62-fold, with almost unchanged directional curvature (70569.12 versus
70814.93). Fine/coarse matches change from 17640/19335 to 18200/19390; legacy
uses 21048/20863. Source identities and assembly checks pass. This isolates a
local effect of whole-window freezing; the initial admission policy also reduces
push, as shown at iteration 0. It does not prove which rejected matches are good,
or that legacy's much larger push is accurate or safe.

The supported mechanism is: prediction-selected, whole-window frozen NDT targets
have too little corrective pull in this state; motion/velocity constraints balance
that small pull after an 8 mm correction, leaving excess outgoing velocity to
feed the next prediction. It is a local mechanism, not proof of a successful fix
or of the entire later flyaway's cause. The chosen direction uses rounded
reference differences, not surveyed truth; alternative models use the candidate
map and were assembled only, not solved.

### Run diagnostics and next decision

Coverage 137/137; no missing required diagnostics. Maximum heading bend 1.783 deg;
maximum speed 2.992 m/s; eight speed crossings above 2.5 m/s (first W122 at
43.56 s). Conditional velocity chi-square median/p95 827.7/2517.32 over 34 eligible
windows. This faithfully reproduces the drift; it is not an improved map. No
map-accuracy, physical closure or robustness verdict is made for this prefix.

Recommended next design, not implemented: retain cell-validity checks; refresh
admission/correspondences between outer linearizations, freezing only for an
individual trial/acceptance comparison. Do not simply weaken the gyro or velocity
prior. A subsequent controlled test must separate refresh from admission changes
and retain the original rotation-jump gate. New algorithm changes and any further
recording replay require user approval. Current high effort is sufficient;
Charlie remains 3/19.

Independent corrected-run evidence review: trace_review (Codex, not Sonnet)
confirmed the exact reproduction and bounded mechanism. Review explicitly
separates conditional stiffness from gradient cancellation and cautions that
late small steps include scheduled damping. All full-ramp alternative quadratic
cost changes remain positive; larger corrective push does not prove recovery.
Changing matched-only costs must not be used as an objective acceptance test.
