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
