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
