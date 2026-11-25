# Design

## Context

See proposal.md (Why). `plan_section.py` cuts the merged cloud at world height H (±5 cm), smooths a 2 cm density grid,
and keeps the density ridge across each wall (Hessian direction, one cell wide, `--threshold` share of the 99th
percentile, pieces spanning ≥ 0.5 m). With the plan it places the lines with the whole-floor fit from `plan_fit.json`.
`scan_lio_merged.laz` carries each point's time (`gps_time`, ns), and `map_quality_lib.trajectory()` gives each node's
time, rotation and position, so for every point the sensor pose that saw it is known.

## Goals / Non-Goals

**Goals:** find where the map of 44 loses precision and which of the causes below each place shows; the diagnostics
must also work on a cave (no plan, no building constants).

**Non-Goals:** the fix; a general-purpose map quality tool; changes to dMapper.

## Decisions

- **Faintness by threshold sweep.** A wall that disappears at a higher threshold had few points, so it was seen
  briefly or from far away. One combined image (colour = highest surviving threshold) shows this at a glance, next to
  one image per threshold.
- **Pieces of ~0.5 m along the section line** are the unit of the diagnosis: short enough to locate a bend, long enough
  for a line fit. The slice points within ±0.2 m across the line belong to the piece.
- **Measures per piece, map and trajectory only:**
  - thickness = robust spread (MAD) of the offsets across the line;
  - visits = the point times split at gaps of a few seconds; each visit's own line offset, and the largest
    difference between visits;
  - observation = seen time, points, range and incidence angle (sensor position interpolated at the point's time);
  - odometry windows = the windows covering the visit times, with their diagnostics from the run log or results JSON.
- **The plan is an optional layer** (distance of the piece to the plan's drawn lines, as `plan_fit.residuals`), used
  only to check the map-only measures on 44.
- **Cause signatures:**
  - (a) sensor-side: thick but centred, few points, long range or grazing incidence;
  - (b) pose error within one visit: a straight or bent line that is offset or turned;
  - (c) drift between visits: visits disagree (doubled wall); compared with an odometry-only result to see whether
    loop closure and consistency make it better or worse;
  - (d) noise: unstable across s0/s1/s2.

## Risks / Trade-offs

- [Furniture or window openings at the cut height look like a bad wall] → sections at two or three heights; a leak must
  show at more than one.
- [Visits split by time gap can cut one long visit] → the gap is a parameter, checked on the lift shaft (seen often).
- [The comparison results don't exist yet] → odometry-only and s0/s1/s2 are made by the user; the steps that need them
  wait for them.
