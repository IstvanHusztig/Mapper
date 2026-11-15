# Livox4 map and runtime postmortem

2026-11-15. Existing artifacts only. The user rejects the full restricted-recovery
cloud as unusable. Completing 1,016 windows validated solver progress, while the
map failed the user's visual check. This candidate has no adoption evidence.

## What the two full runs show

| Measurement | Earlier Huber3 | Latest restricted recovery |
|---|---:|---:|
| Logged processing time | 1,747.35 s | 9,904.02 s |
| Sum of 1,016 window optimization timers | 879.14 s | 9,098.94 s |
| Saved odometry endpoint separation | 36.54 m | 43.57 m |
| Final endpoint separation | 36.72 m | 43.57 m |
| Final endpoint Z difference | −1.72 m | −8.19 m |

Sources: `livox4-indoor-outdoor/huber3-baseline` and
`livox4-w214-restricted-full` in the external experiments root. Timing comes
from native `Processing time` and `finished at iteration ... optimizing window`
rows. Endpoint separation uses 182,880 saved poses per stage, with final poses
transformed from chunk coordinates. The user reports a same-table return.
These are measurements of two configurations, not a single-factor comparison.
The older Huber3 map also fails the return check.

Total time grew by 8,156.67 s, while summed optimization timers grew by
8,219.80 s. The latest log records 101,600 base and 137,969 trial merit
evaluations. The checked solver copies trial state and calls the same assembly
routine to evaluate merit. That routine still builds normal equations before
returning objective and RHS. Guarded matches also refresh each iteration.
Rare recovery attempts are only one part of the cost. Logging within the
window timer contributes an unmeasured share. The percentage attributable to
each operation is unavailable because detailed phase timing was disabled.

## Decisions that went wrong

1. We labeled an AHRS experiment gain-only although the reference used
   alpha/beta 0.1 and the candidate was built with 1.0. The attribution was
   corrected in `livox4-rotation-factor-trace.md`, but we continued developing
   on the alpha/beta 1 branch. The trace established the local NDT force on a
   bad rotation for that configuration. It did not establish the natural
   flyaway's origin in the earlier full Huber3 configuration.
2. Guarding bad matches suppressed one bend and introduced translation drift.
   Refresh improved the short guard result, but that was local. We then added
   acceptance, damping and recovery layers to clear failures introduced by the
   checked solver. Their tests certify bounded updates and descent under the
   selected matches. A pose can still drift through many individually valid
   steps when the model is biased or motion is poorly constrained.
3. The saved tail-factor test measured about 0.026 m/s added residual after
   replacing tail poses. That proved a state inconsistency. It did not explain
   a 10 m wall displacement. Removing replacement for every window produced
   1.15 m of disagreement with the control by 35.64 s, long before the W214
   stall. W214 recovery fixed that stall exactly. It could not undo the earlier
   drift. We should have paused the retained-tail candidate at that point and
   evaluated its geometry before investing in completion.
4. The completed short run placed the marked wall about 10.24 m from its
   expected line in saved odometry and 9.38 m away after postprocessing.
   The full run was authorized to test the cut hypothesis, but I should have
   proposed checking the uncut prefix through the affected interval first.
   During full processing the logged window metrics already matched through
   W346. The final saved snapshots confirmed an exact match of all 62,280 XYZ
   positions and timestamps. The full cloud answered the separate question of
   what a complete run would export, at a cost disproportionate to the cut test.

## Proposed decision gate

Retain the artifacts as evidence. Do not promote this variant as a map fix.
Use existing runs to locate the first sustained pose disagreement in the
original alpha/beta 0.1 full Huber3 run and in the experimental branch. Use
common source timestamps and physical landmarks, excluding the marked trees.
Keep the natural flyaway diagnosis separate from a solver recovery regression.
Before proposing any new full replay, identify one mechanism and one changed
variable. Require a short matched-return geometry check and a measured runtime
on that comparison. Solver completion alone is insufficient.

Independent Codex review checked timing, common-prefix evidence and endpoint
measurements. No code change or replay was performed for this audit.
