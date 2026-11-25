# Proposal

## Why

The goal is a more precise point cloud. The section lines of `44_baseline` (`tools/plan_section.py`, 1 px lines at height H
through the densest part of each wall) show walls that are not straight, or that don't sit on the house plan. The
hypothesis: walls the sensor saw only briefly are imprecise. This change finds where the map loses precision and why,
so that the fix can be aimed at the cause. The target is caves, so every diagnostic works from the map and the
trajectory alone; the house plan of 44 is only an outside check.

## What Changes

- **Locate the leaks:** `plan_section.py --threshold` takes several values. Sections of `44_baseline` at 0.01 / 0.03 /
  0.1 / 0.3, plus one image with each line coloured by the highest threshold it survives, and at two or three heights
  (e.g. −0.6, 0.0, +0.8) so that furniture and windows don't fake a leak. The output is a list of suspect walls in map
  coordinates.
- **Diagnose each suspect wall:** a small generic script, `tools/wall_precision.py`. For each ~0.5 m piece of section
  line it measures the wall's thickness, how its visits (passes) agree, the observation (seen time, points, range,
  incidence) and the odometry windows that saw it. On 44 it also gives the distance to the plan.
- **Separate the causes:** sensor-side, a pose error within one visit, drift between visits, or noise. This uses an
  odometry-only result and the s0/s1/s2 results, which the user makes. A research subagent does the correlation if the
  data calls for it. The conclusions are reviewed, and the user is asked before any verdict.
- **The fix is not part of this change:** it follows the user's decision on the findings, as its own change.

## Capabilities

### New Capabilities

### Modified Capabilities

None: diagnostic tools and findings only, no dMapper behaviour changes (`skip_specs`).

## Impact

- `tools/plan_section.py` (several thresholds, coloured combined image), new `tools/wall_precision.py`, `tools/README.md`.
- Reuses `tools/map_quality_lib.py` (`trajectory`, `cache_dir`) and `plan_fit.json` from `tools/plan_fit.py`.
- No change to dMapper's code, settings or baselines.
