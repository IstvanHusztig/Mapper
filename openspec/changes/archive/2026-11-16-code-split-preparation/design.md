# Design

## Context

Stage 0 of `docs/code-split-and-region-agents-plan.md`. The user's decisions of 2026-11-16:
1. remove the three switches that are off, keep `lm_step_check`;
2. the per-term log comes after the term split;
3. eight regions;
4. remove robust mode and `Consistency2`.

**Before the change**, a Sonnet reviewer checked the facts of the plan against the code:
- the four switches account for about 90 lines of `optimize_lidar_odometry`, about 90 lines of cost and step-check code,
  and four fields in every map cell;
- robust mode is reached only through `use_robust_and_accurate_lidar_odometry`;
- `Consistency2` only through `use_mutliple_gaussian`.

All three are off in `settings/l2-vertical.toml` and `settings/livox-vertical.toml`.

## Goals / Non-Goals

**Goals:**
- Remove the code the shipped settings never reach, bit-identical for every result file but the params TOML.
- Keep older settings files loadable.
- Make every result traceable to its code and build.

**Non-Goals:**
- No change of numbers.
- No split of functions yet (stage 2).

## Decisions

1. **Remove, not keep behind switches.** No dead code (the project's rule). The removed code stays in git history.
2. **The unmatched-point count stays.** Only the setting that gave an unmatched point a cost goes. The count is logged
   and tested, and a later cost may use it.
3. **Retired settings: ignored at the behaviour that remains, refused otherwise.**
   - Without this, the strict loader would refuse every older params file, including those in result folders.
   - A retired switch set to `true` asks for behaviour that no longer exists, so loading it silently would run other
     settings than the file says.
   - Parameters that only tuned a removed feature (the robust bucket sizes) are accepted at any value.
4. **Snapshots before the change are refused.**
   - The robust settings were always part of the odometry settings hash, and `l2-vertical.toml` had a non-default
     `max_distance_lidar_rigid_sf = 25`.
   - Keeping the hash would need their old values hard-coded. A snapshot is quick to make again.
5. **Build identity from a script run at every build.**
   - A value computed at configure time goes stale after a commit, so the identity comes from a CMake script that runs
     at every build.
   - The script rewrites `build_info.h` only when its content changes, so an unchanged build recompiles nothing.
   - The compare tool ignores the commit and the uncommitted-changes flag. The flags and definitions are compared,
     because a change there is a change of the build.
6. **`.asc` files are not compared.**
   - dMapper writes none into a result folder; `map_quality.py` exports them there.
   - The 44 manifest of `f241eb7` had hashed such exports, which made a correct run look different.

## Risks / Trade-offs

- **An older file with a removed switch set to true is refused.** The message names the setting and the value that is
  accepted.
- **Removing a feature someone might want back.** It is in git history, and the plan names where to rebuild it (stage
  8: honest degeneracy measure and carried covariance on the split code).
