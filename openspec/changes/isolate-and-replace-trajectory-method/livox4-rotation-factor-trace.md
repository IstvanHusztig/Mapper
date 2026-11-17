# Livox4 first bad rotation: diagnostic trace

## Authorization and scope

User explicitly requested: “Go trace which factor causes the first bad rotation
update, log it, trace it!” This authorizes the previously proposed single
125-second prefix diagnostic replay, with unchanged executed candidate settings.
No parameter sweep, bypass, numerical correction or full recording replay.
Current high effort is sufficient. Charlie's larger interface/replacement remains
parked at 2/19 until capture/evidence gates are complete.

## Provenance correction

During preparation the saved candidate log reports `NDT_INFORMATION_EXPERIMENT
alpha=1`, while the control reports `alpha=0.1`. Candidate source fc73404's build
cache and Ninja definitions specify alpha=1 and beta=1. Earlier reports wrongly
called the comparison gain-only and described the candidate as alpha0.1/beta0.1.
Withdraw causal attribution of candidate/control differences to the AHRS gain
fix alone. Preserve both original runs and their observed results.

The diagnostic replay reproduces the actual failing candidate: alpha1/beta1,
Huber3, original candidate TOML/runtime DLLs, input prefix from the beginning.
It does not silently change weights to the earlier intended configuration.
Frozen input and candidate snapshot identities are checked against the original
preparation/collection records. No separate recording verification is needed for
these identical, previously verified input bytes.

## Logging and validation

Enable `DMAPPER_TRACE_LIVOX4=1` only for W297–300, every iteration. Reuse existing
pose/state, factor RHS, range support, Huber weights and match-turnover records.
Add:

- Each factor's angular/other step push and quadratic terms, including cross terms.
- Responses `H_damped^-1 b_factor` using an independent diagnostic factorization;
  record every vector and reconstruction error against the actual solver step.
- Motion translation-row versus gyro-row RHS responses and NDT range-bin responses,
  with angular-RHS-only responses to distinguish direct torque from coupled forces.
- Nonlinear Huber cost before/after with fixed pre-step cells, split by grid/range.
  This is unprojected cost, not an objective-equivalence claim when projection acts.
- Release dropped-write count. Production accumulation, solver, state and acceptance
  are unchanged. Shared-H responses are not counterfactual factor-removal runs.

Expected diagnostic cost 10–20 minutes; existing settings allow 3600 s/window.
Fresh destination: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-trace`.
Input is reused from `livox4-ahrs-gain-prefix/input`; no duplicate recording copy.
Require exact snapshot window payload equality with the existing candidate for
all 346 windows (stronger than the proposed first325 gate). Verify 100 iterations
in each traced window, finite responses, additive reconstruction and complete
run-health coverage. No automatic retry if trajectory or identity gates fail.

Independent Codex reviewer checked diagnostic algebra and code before launch,
found no blocker, and independently caught the control/candidate alpha mismatch.
Final attribution still requires review of collected evidence.

## Status

Scratch logging commit `f400479` builds successfully in Release. Existing trace
parity and Huber checks: 5 cases, 2159 assertions passed. Debug not required for
these observational additions; no numerical or threading change was introduced.
Two premature test invocations encountered the executable locked by its ongoing
link step; neither executed tests. The post-build invocation above passed.

The approved replay is running under `tools/run_livox4_rotation_trace.py`;
external manifest and run.json preserve command, identities, environment and PID.
No automatic retry. Native completion and exact equality remain pending.


## Completed result and isolation finding

Native exit 0; wall time 607.476 seconds. All 346 odometry snapshot window payloads
are byte-identical to the previously saved candidate, including the failure.
All 400 expected traced iterations (W297–300,0–99) are present. 90,076 extracted
records match the final native log exactly; helper identities unchanged.
Independent Codex review confirmed the algebra and evidence, with the coupling
and provenance limitations below. No fix or second replay was performed.

**The initiating numerical driver in W298 is the NDT scan-matching RHS, with
its largest net contribution coming from 2–5 m returns under the coupled solve.**
W298 begins at raw timestamp 107.2746488 s (log-relative 106.92 s). W297's final
node164 correction is only -0.0420 degrees. W298 does not jump 18 degrees in one
step: repeated accepted updates accumulate the bend.

| Iteration (zero-based) | Node164 Euler-z step | Accumulated from actual pre-LM pose |
|---|---:|---:|
| 0 | -0.855775° | -0.855775° |
| 1 | -1.109296° | -1.965071° |
| 2 | -1.103977° | -3.069047° |
| 99 | -0.000005° | -17.917741° |

At iteration 0, shared damped-H responses at node164 are:

| RHS family | Euler-z contribution |
|---|---:|
| All NDT | -0.855777° |
| NDT 2–5 m | -1.064728° |
| NDT below 2 m | +0.208951° |
| NDT beyond 5 m | 0° |
| Accelerometer | approximately +0.000002° |
| Motion/gyro and carried priors | approximately 0° |

These are additive `H_damped^-1 b_family` responses under the same complete
Hessian, not independent optimizations or factor-removal experiments. Near
returns also exert angular influence: their angular-only RHS response is
-1.46050°, offset by translation coupling; therefore do not say only the 2–5 m
points exert rotational force. At this first step, there are 503 fine-grid and 839
coarse-grid matches in 2–5 m; these are factor occurrences, potentially sharing
source returns, not 1342 unique points. All beyond 5 m matches remain zero across
both grids and all 100 W298 iterations. The 203 newly visible distant returns do
not cause this local update. This does not resolve every outdoor failure.

The projection matrix is identity throughout W298 (maximum element deviation
1.34e-15). Degeneracy projection is therefore not the initiating computation in
this window. All updates are accepted. Max reconstruction errors across all 400
iterations: total 1.29e-14, range 7.18e-15, motion-row split 5.05e-15 (state-vector
norm, mixed units; numerical reconstruction checks, not physical accuracy).

### Why a simple decrease check is insufficient

| Iteration | NDT before | After, same matches | After rematching |
|---|---:|---:|---:|
| 0 | 216540.44 | 202230.24 | 216416.25 |
| 1 | 216416.25 | 212457.46 | 221065.94 |
| 2 | 221065.94 | 217444.45 | 228519.67 |

These are robust NDT model-relative costs, not calibrated chi-square values.
The fixed-association NDT cost decreases in all 100 steps, while the rematched
cost increases in 40. A guard checking only the fixed-match NDT decrease would
accept the initial bad rotation. The joint objective also contains inertial terms;
no complete nonlinear-objective monotonicity claim is made. At iteration 0, fine
grid matching gains 234, loses 139 and changes cells for 865 retained returns;
coarse grid gains 81, loses 49 and changes cells for 263 retained returns. That
shows correspondence turnover, not proof that every changing association is wrong.

### Exact code boundary and remaining uncertainty

Executed source is scratch commit `f400479`,
`C:/Repo/dMapper/out/ahrs-gain-worktree/src/domain/scan_to_map.cpp`:

- Cell selection: fine lookup line 518, coarse lookup line 672.
- Point weighting and accumulated NDT RHS: lines 597–607 and 739–749.
- Projected NDT RHS enters the joint system at lines 803–809.
- Joint damped solve line 1330; translation-only acceptance from line 1342.

This isolates the local numerical driver to matching/residual assembly feeding
the joint solve. It does not establish a sign/indexing defect or identify which
specific cell/surface is physically wrong. Prior map distortion, local shape,
correspondence changes, legacy cell statistics and relative factor weights remain
possible upstream explanations. The misconfigured earlier comparison prevents
attributing this event to the AHRS gain correction alone.

The existing trace reaches the exact original state through the sequential prefix;
no standalone odometry restart or reusable full-map checkpoint was created.
Continue with the reviewed interface/isolation decision only after preserving
these distinctions. A numerical fix or another comparison requires separate scope.

### Run diagnostics (reproduction, not quality improvement)

| Measure | Original failing candidate | Diagnostic replay |
|---|---:|---:|
| Windows/iterations | 346 /100 each | 346 /100 each |
| Maximum logged yaw correction | 16.959° | 16.959° |
| Corrections >3° / >10° | 4 /1 | 4 /1 |
| Maximum end velocity | 1.2602 m/s | 1.2602 m/s |
| Maximum loop correction | 4.83 m | 4.83 m |

Replay coverage: bend/speed/bias 346 each, eligible chi-square 111, missing required
rows none. Chi-square median 242.5/p95 4260.5 are investigation triggers under the
agreed contract. The prefix is not a known return; no physical closure or improved
map quality is claimed. Odometry equality establishes this as the same failure,
not a new cloud for visual acceptance.

### Evidence

External bundle `livox4-ahrs-gain-trace/` contains `run.json`, `manifest.json`,
`run-health.json`, `rotation-trace.log`, `factor-analysis.json`, `findings.json`,
`diagnostic.patch`, frozen binary/settings and archived Python tools.

- Native log SHA256: `f5d4a61f10a757ffd69919d2eb6968378ca791798769cb9c703fd16c4d424e1b`.
- Extracted trace SHA256: `36ee8c7a2d563b7670cbbaedb7e28881ffcf74243d453aa26aff0f7ea4c6db89`.
- Analysis SHA256: `1763d35a6c51d1ea9f3c3588a18a73ebf5d425f0028fae1e864e2d142eec656a`.

Keep high effort for interpreting or changing this coupled estimator; routine
artifact inspection/monitoring can use medium. No higher model is needed now.

Task1.3 is complete through the sequential-prefix fallback and identity gate;
Charlie is now3/19. Task1.4 (reviewed concrete interface contract) remains pending.
