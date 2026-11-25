---
name: verify-map
description: Judge the point cloud of a dMapper result - wall thickness, agreement between visits of the same wall, position difference from a reference result, the three map-quality measures on recording 44, plan overlays - and prepare the before/after files for the user's own look. Use this whenever a change may alter the map (odometry, settings, loop closure, consistency), whenever two results must be compared, and always before a map-changing change is pushed, even if the user only asks "is the cloud sharper?" or "did that help?".
---

# Verify a map

## Agreed evaluation rules (2026-11-12)

Read `openspec/changes/archive/2026-11-25-sensor-neutral-yardstick/criteria-proposal.md` before any
verdict. The user agreed this contract; it supersedes legacy tool defaults.
Report missing evidence as NOT MEASURED. Diagnostic triggers are not map-accuracy
verdicts. Overlapping empirical envelopes are inconclusive, not proof of equality
or non-regression. Preserve every perturbation member and check odometry and
final results separately. The user's review of readings and clouds remains required.


The map is the product, so a change is only as good as the cloud it gives. Numbers pick the candidates; the user's
own look at the clouds decides a ship. Run `verify-run` first; diagnostic triggers require investigation, not automatic map rejection.

## Steps

1. **Wall precision** (any recording; needs no plan). Pick a height H in world z about 1 to 2 m above the floor of the
   level of interest (z is 0 at the start pose).

   ```
   python tools/wall_precision.py "<result folder>" --height <H>
   ```

   Read the summary lines `thickness_m` and `pair_diff_m` (p50 and p90). Thickness is the spread across a wall;
   `pair_diff_m` is how far two visits of the same wall disagree, which is what draws a doubled wall.

2. **Against a reference result** of the same recording (the shipped settings, or the run before the change):

   ```
   python tools/trajectory_seams.py "<reference folder>" "<result folder>"
   ```

   It prints the largest position difference and when it first exceeds 5, 20 and 50 cm.

3. **Recording 44 only** (it has a building plan and fixed revisit pairs): the three measures of equal weight, on the
   three inputs s0, s1 and s2 of the same code, and the overlay.

   ```
   python tools/map_quality.py --floors 44_s0=<folder> 44_s1=<folder> 44_s2=<folder>
   python tools/plan_fit.py "<result folder>"
   python tools/plan_section.py "<result folder>" --height <2 m above the floor>
   ```

   "Overlay" means `plan_fit.py` followed by the `plan_section.py` lines, cut 2 m above the floor.

4. **Runs made with a hold-out** (`holdout_block_s` > 0): `python tools/holdout_sharpness.py <reference> <result>`
   measures sharpness on points no stage fitted. Use it when a change could simply fit its own points better.

5. **For the user's look:** give the full paths of `scan_lio_merged.laz` of the reference and of the result, and any
   overlay images written to `<result>/map_quality_cache/`.

## Limits and coverage

Building planar-wall robust spread targets are p50/p90 20/60 mm across pieces;
this is 1.4826 MAD, not full wall width. No absolute cave-wall limit is agreed.
Freeze physical regions and extraction settings; show missing surfaces, range,
incidence and coverage. The 0.2 m extraction band can hide distant ghost walls.
Half-splits are a diagnostic, not independent truth. Geometry, revisits and
sharpness remain separate; no aggregate score can hide a regression.
Require max(candidate) < min(reference) for resolved improvement in a predefined
lower-is-better metric, with no wider perturbation range. Overlap is inconclusive.
Require equal timestamps/counts before trajectory_seams; never trust truncated
or mismatched index comparisons. Full 44 s0/s1/s2 and the visual gate remain.

## Report

One table: check, reference, result, limit. Then the paths for the user's look. Give no verdict on which is better
before the user has seen the numbers, and none on a ship before the user has seen the clouds.
