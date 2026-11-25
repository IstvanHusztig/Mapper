# Proposal

## Why

On the Livox 4 recording dMapper draws a wall twice, 54 cm apart, although the odometry alone has it within 2 mm. The
Python prototype (`tools/loop_graph.py`, change `livox4-double-walls-cause`, tasks 3a) found two causes in the loop closure:
every odometry step of the pose graph counts as equally certain, so the 5.8 m lost in 9 blind seconds of a corridor is
spread over the whole walk; and one false match (segments 24 -> 63) is kept, because the pruning looks at an edge's residual
after the solve, which an influential edge keeps small by bending the graph. With both removed in the prototype, the user
judged the map "a lot better" and its overlay "close to perfect". The user needs that map from dMapper itself by 2026-11-26.

## What Changes

- The loop closure pose graph takes the uncertainty of each odometry step from the odometry's own readings: a run of
  windows in which the LiDAR did not hold the travel (information ratio below a limit, a common weak axis, not at rest)
  gets a larger translation uncertainty along that axis, sized from the run's duration and from the placed speed against
  the pace before and after it. The rule is the one frozen in
  `experiments/consistency/livox4_lever/odo_cov/RULE.md` (per-step form). Off by default.
- The readings the rule needs (information ratio, weak axis, at rest) are kept per window and written to the odometry
  snapshot and the loop closure capture (new file versions; older files still load, without the readings).
- A match check on the graph that knows the blind stretches: an edge is tested by what adding it costs the graph when the
  edges that reuse its scans are held out (its squared innovation in sigmas), instead of by its residual after the solve.
  The check is frozen in `experiments/consistency/livox4_lever/match_check/RULE.md` before it is built. Off by default.
- Each run logs its weak runs (time, duration, axis, sigma) and the edges the match check removed, with their test values.
- With both switches off every result stays bit-identical.

## Capabilities

### New Capabilities

### Modified Capabilities
- `loop-closure`: the odometry steps' uncertainty may come from the odometry's readings; false loop edges may be found by
  a held-out innovation test; the capture carries the readings.

## Impact

- `src/application/loop_closure.{h,cpp}` (graph weights, pruning), the capture and snapshot files and their readers
  (`src/infrastructure/loop_closure_capture_file.*`, `odometry_snapshot_file.*`), `src/domain/window.h` and the window loop
  (one small record per finished window), `OdometryConfig` / `TomlIO` (new `[loop_closure]` settings, outside the odometry
  settings hash), `tools/loop_graph.py` (`read_capture`), tests, the back-end region's knowledge file and the guide.
- `settings/livox-vertical.toml` changes only after the user's decision on the maps; recording 44's baselines only if the
  switches are turned on for the L2 as well (not part of this change).
