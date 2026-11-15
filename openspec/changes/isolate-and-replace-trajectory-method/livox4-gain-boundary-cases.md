# Gain-fixed later heading cases

2026-11-15. Autonomous diagnosis/fixes/replays authorized by the user; no
per-run approval stop. No default adoption or full-route accuracy claim.

## Decision

The source-covariance prefix failed its map gate. See
`livox4-source-surface-check.md`; do not extend it to the full recording.
The cheaper gain-fixed control remains the numerical reference, still with a
wrong marked wall and a10.242deg correction atW313. Original W298/W299 cases
predate its corrected AHRS gain and cannot identify its later matches.

The offline origin-anchor check reproduced native initial solves within7.35e-12.
Removing only the first-node incremental regularizer reduced relative shape
deformation but left last-node tilt slightly worse: W298 .313->.317deg,
W299 .672->.693deg. Last Euler yaw changed negligibly. This does not justify
a free-first-node recording candidate. Outputs remain in the original trace
bundle's `solver-replay-W298/origin-anchor-probe/` and W299 counterpart.
The rigid-tangent diagnostic is a different motion model, not a validated fix;
free base motion also loses the implicit boundary anchor.

## Bounded capture

One existing original-start125s prefix, same input/cache as the matched gain
control, source covariance off, alpha/beta0.1, Huber3. Disable loop closure,
consistency and merged export for this diagnostic only. They do not affect
odometry. No recut and no cold start at the event.

New diagnostic-only environment mode `DMAPPER_TRACE_LIVOX4_GAIN_CASES=1`:
W298,301,313,314; iterations0,1,10,99 plus existing input/stage/carry records.
Capture incoming offset covariance and previous post-tail poses explicitly so
the helper does not reconstruct missing state from rounded log values.
No changes to old trace modes, numerical settings, solver calls or schedule.

Root runner: `out/livox4_gravity_replay.py prepare|run --gain-boundary-trace`.
Fresh external folder: experiments/phase0b-2026-11-12/
`livox4-gain-boundary-trace-125s/`. Freeze executable/settings/source and input
hashes, exclude other DMAPPER environment overrides, check cache ownership.
Expected processing approximately6-10minutes plus preparation, based on the
gain control;16 traced calls instead of the earlier500. No automatic retry.

## Required checks and use

- Focused selection tests, Release build and independent review before launch.
- Successful complete346-window odometry, exact trace window/iteration set,
  stable hashes, correct flags and all346 snapshot payloads/trailer byte-identical
  to gain control. Only the header's settings hash may differ: export_merged_laz
  belongs to a hashed category despite not affecting odometry. Compare magic,
  version and every byte outside that8-byte hash; explicitly check the three
  parsed setting differences before freezing. Independent review caught this
  preflight issue, avoiding a false failure after the run.
- Reconstruct each native case from exact incoming state and the prior map;
  verify source/target geometry and matches against the captured trace.
- Run all100 local native calls with original damping, comparing the four
  captured calls and final state. Do not claim unobserved iterations were
  individually reference-checked.
- Attribute the later correction using complete coupled factor response and
  actual supporting geometry before selecting another numerical candidate.

Worker gravity_candidate owns implementation/helper/build in the external
worktree. Root owns runner/artifacts. Independent reviewer
seed_damping_crosscheck checks capture sufficiency and numerical isolation.
No new map or fix is claimed by this capture.

## Capture launched after user resumed

User paused work at approximately05:29CET; the build was stopped and no new
recording started during the pause. On "Start again", resumed and completed
Release validation:6 focused cases/2,166 assertions passed. Independent review
confirmed the trace-only change, boundary-state sufficiency and snapshot-header
exception. Trace source commit08d0e4c5e5d6f4c8d61254f31a2a85715c7b735e.
Frozen executable SHA256:
`9450ecb31de47412870ea7349c6b275225301eb6344470d5a384a0957d498973`.

Launched2026-11-15 05:35:30 local, nativePID18928, runner22384, in the folder
above. Build/test evidence is under `validation/`. No other replay is active.
The existing rotation analysis accepts explicit expected window/iteration sets;
its sparse output matches the corresponding original100-call fixture rows
exactly and rejects missing, extra and duplicate checkpoints. Sparse first/peak
summaries explicitly mean observed calls only. Evidence:
`validation/sparse-parser-check/validation.json`. Completion/equivalence and
native reconstructed cases were pending at launch; completed results follow.

## Capture and native cases completed

Native processing finished in446.36s (wrapper459.71s), exit0. All346 snapshot
payloads and trailer are byte-identical to the gain-fixed reference; only the
declared header settings hash differs. Input/binary/settings hashes are stable.
All16 requested checkpoints and incoming boundary fields are present. This is
a diagnostic capture, with no new merged cloud.

Native helper commit8f6f8fcd36bae6e1c85e74809038bb170bdbe864 reproduced all100
continuous calls for each of W298,W301,W313,W314. At the four recorded iterations
each case matches900/900 records, including both grids and geometry/cost.
Native tail processing, final poses and carry match the saved post-tail state.
The16 checkpoints total3600 exact records; intervening calls were executed but
were not individually captured for comparison. Evidence is under
`livox4-gain-boundary-trace-125s/native-cases/validation-summary.json` and each
case's `validation.json` plus `control-exact.json`. Do not use export existence
alone as a validity check.

## Coupled factor evidence and next bounded check

`factor-analysis.json` reconstructs all16 sampled updates without errors.
Selected node164 yaw responses in degrees, using the full shared damped matrix:

| Checkpoint | Actual step | NDT RHS | Motion RHS | Acceleration RHS | Carried velocity RHS |
|---|---:|---:|---:|---:|---:|
| W298 i1 | -0.259115 | -0.225231 | -0.009217 | -0.020633 | -0.003982 |
| W301 i10 | -0.087014 | +0.046082 | -0.048685 | -0.011339 | -0.073082 |
| W313 i10 | +0.197014 | +0.044619 | +0.020845 | +0.026698 | +0.105198 |
| W314 i0 | -0.114247 | -0.114246 | ~0 | -0.000001 | ~0 |

These are shared-system RHS contributions, not independent factor-removal
experiments. They demonstrate inertial/carry influence on the later step but
do not prove a tilt-to-velocity-to-yaw causal chain. W313 also restores much of
the preceding heading disagreement with AHRS; AHRS is not ground truth.

Independent review found an untested interaction: the earlier gravity-compensated
candidate used the old Fusion gain adapter. The corrected-gain candidates all
had compensation disabled. Do not treat that old failure as a test of their
combination.

Next, use the exact gain-fixed W301/W313 cases, without a recording replay, for
paired original, scale-only and compensated solves. Recompute exact native
startup gravity G0 from raw IMU and pre-optimization orientation, not overwritten
snapshot model poses. Freeze s0=9.80665/|G0|. Preserve incoming physical velocity
and covariance; map incoming legacy gravity G to compensated world offset
(G-G0)*s0 and retain offset covariance. Compare at the same initial state and,
where available, the captured baseline i10 state. Include the legacy-at-s0
control so scale and compensation effects are separated. Check finite residuals
and full coupled responses before considering a sequential prefix experiment.

The imported prior/map still come from the uncompensated run. A local response
change is therefore directional evidence, not a compensated-route result.
Worker gravity_candidate implements only the helper; seed_damping_crosscheck
independently reviews mapping, seed and covariance consistency. No production
algorithm change or new recording is authorized by a successful local metric
alone; existing user authorization permits a subsequent justified experiment.

### Paired gravity result: no prefix justified

Helper-only commit1ce0b21727309b698ad030db2813448909a9bed3, five focused tests /
26,470 assertions passed. All12 paired updates reproduce independently; NDT
matches, projected H and RHS stay exact. Native startup G0 is
(0.001732556,-0.0000709553,1.001141859), s0=9.795450269; offset mapping error
is below3e-15m/s2. The approximate quaternion-only startup estimate is not used.

At i10, maximum tilt sensitivity to a carried-velocity prior change decreases
only0.60419->0.57385deg/(m/s) in W301 and1.13924->1.13477 in W313. Actual
joint tilt steps instead increase0.07378->0.12134deg and0.09977->0.12100deg;
maximum translations increase0.718->24.60mm and2.503->29.46mm. Scale-only changes
are negligible. Evidence: `native-cases/gravity-gain-probe-summary.json` and
`native-cases/gravity-gain-tools/` with source, binary, tests and provenance.
Independent review confirms that this is not positive evidence for a recording
candidate. No gain-plus-gravity prefix launched; runner mode exists but its
experiment folder has not been created. This fixed-state result does not prove
how a fresh compensated route would behave.

Two bounded existing-data follow-ups address remaining alternatives: (1) test
the exact gain W298 first step against raw historical inserted surface support,
not just its containing-cell Gaussian means; (2) quantify post-tail end-factor
inconsistency and its fixed-H carried-mean response using the already exported
matrices. No extra trajectory capture or numerical setting sweep is selected.

### Raw-surface and tail checks completed

Actual gain W298 i0 uses10,277,225 historical inserted points (seed and W1-297,
no rebuild),12,524 unique matched returns and18,788 grid occurrences. The native
positive step improves equal-source fixed-cell Huber cost by0.03864 and mean
nearest-historical-point distance by0.487mm; reversing it worsens those by0.08085
and0.910mm. Both near and farther range strata improve on average.20.85% improve
NDT while worsening nearest distance. The centered directional derivative agrees
with the assembled RHS within1.03e-7. This does not support an aggregate
wrong-direction association at this checkpoint. The historical map can already
be warped and nearest-point distance is sampling-dependent, so it is not truth.
Artifacts: `native-cases/W298/geometry-probe/` and `geometry-tools/`, helper
commit2e647280562154f9e5485489291b99a0979dcd4f. Native control and tail gates pass.

The tail inconsistency is real: exact saved W301/W313/W314 end-factor residuals
grow from numerical zero to58.34/38.24/58.47mm/s after tail replacement. Existing
W301/W313 full-H forks using inferred previous post-tail velocities change i10
node164 yaw from-0.0870 to-0.0669deg and+0.1970 to+0.1863deg. Old-scale rounding
uncertainty changes those responses by at most~1e-6deg. This is a modest response,
not evidence that the tail mismatch explains the map. Preserve the physical
mean/covariance question rather than silently treating implied velocity as true.
Reproducible script, input hashes and results: `native-cases/tail-carry-check/`.

Research also recovered the rejected normal-only NDT trial in
`openspec/changes/archive/2026-10-11-point-to-plane-ndt/`: it ran away on stairs.
Do not repeat it based merely on translation-RHS-to-rotation coupling, which
does not distinguish normal from tangential source forces.

### Next bounded model-correlation test

Tracker C34 remains active: predicted relative XYZ edges use information1e7
per row (sigma1e-4m, beta0.1), alongside accelerometer and carried-velocity
factors derived from related inputs. Test actual gain W298 i0 with only those
XYZ row weights zero in the helper. Preserve gyro rows, priors, matches, poses
and damping. Require exact control, expected removed H/RHS including angular
cross-blocks, unchanged unrelated factors, and rank assessment after diagonal
equilibration plus structural-zero-row checks. Inspect raw-surface fit and
physical updates. This is a sensitivity test, not a production beta0 proposal:
`../ndt-information-probe/next-step-design.md` already warns that removing XYZ
rows can leave unmatched intermediate translations unconstrained. No recording
or production change yet. Independent reviewer checks the paired result.

### XYZ-row and correction-basis results

The isolated zero-XYZ solve reproduced the native control and exact unrelated
factors. The removed matrix matches the expected XYZ contribution including
angular cross-blocks. Peak tilt falls0.22228->0.08728deg, but eight nodes lose
all translation constraints. A supported node moves0.364m; adjacent-node speed
reaches198.7m/s. Better raw-map distance therefore comes with nonphysical deskew.
Do not run this as a production candidate or repair it only at unmatched nodes.
Artifacts: `W298/xyz-motion-probe/` and `xyz-motion-tools/`, helperf4babf49.

An offline correction basis using the32 actual IMU translation anchors restores
undamped rank (648 unknowns). Intermediate translation increments interpolate
in time; the original nonlinear prediction stays unchanged. Final increment
uses the last two anchors171/177, extrapolated to179 with coefficient0.286859.
All rotations/nuisance variables and lifted pose-damping metric are preserved.
Peak adjacent-node acceleration falls80.74->2.04m/s2, but tilt rises to0.24159deg.
Keeping the original XYZ rows with the same basis gives essentially the same
smoothness (2.014m/s2) and0.22638deg tilt. Thus the basis improves short-scale
smoothness, not the measured initiating tilt. Independent review checked both
matrix/trajectory conventions. See `W298/anchor-translation-probe/`.

End this diagnostic family without promoting a new solver policy. The next
independent method feasibility experiment is `livox4-independent-icp-baseline.md`.
