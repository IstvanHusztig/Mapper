# Design

## Context

`run_loop_closure` (`src/application/loop_closure.cpp`) cuts the trajectory into segments of 12 windows (≈ 4.8 s), registers each segment against earlier ones that qualify as revisits, and solves a pose graph over the segment anchors. Today a same-floor pair qualifies when the earlier segment ended at least `loop_closure_min_time_gap_s` (30 s) before the later one began, and their centres are within `loop_closure_search_radius_m` (8 m) horizontally and 1.5 m vertically. Measurements and the survey of other systems: `docs/loop-closure-research.html`.

## Goals / Non-Goals

**Goals:** out-and-back revisits (dead ends, the gallery of 44) become loops; neighbouring segments, standing still and places merely seen from afar do not; the rule works for slow cave surveys as for walking through a building.

**Non-goals in G1:** edge weights, edge choice, outlier handling, anchors (G2–G5).

## Decisions (G1)

1. **Path walked instead of time.** The time gap is a proxy for "left the place and came back"; the path measures it directly. Standing still adds no path, so it never qualifies (a 30 s gap admits it); a fast out-and-back qualifies (a 30 s gap rejects it). Established systems gate by path or index (hdl_graph_slam, SC-PGO, LAMP); LIO-SAM's time gap is the exception.
   - The path is summed over the first node of each window (0.4 s apart): summing every node would add the pose jitter of a person standing still (≈ 27,000 nodes in 30 s).
2. **The allowed distance grows with the path:** centre distance ≤ min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × path), as LAMP 2.0 scales its search radius with the path (drift grows with it). This one rule also excludes neighbouring segments, whose centre distance is about the path between them (ratio ≈ 1), and places seen from afar.
   - Data on 44 s0 (distance / path of same-floor pairs): the gallery's true revisits 10→13 = 0.04 and 9→14 = 0.06, others 22→25 = 0.13, 15→18 = 0.45; pairs two segments apart 0.6–2.1 (mostly ≈ 1); 9→12 = 0.79 (centres 6.8 m apart: the gallery seen from its other end, accepted by the 8 s experiment only because the whole gallery is visible from anywhere in it). `loop_closure_radius_per_path` = 0.5 separates them with margin on both sides.
3. **A minimum path, `loop_closure_min_path_m` = 8 m:** below it the odometry is good enough and a revisit adds little; it also keeps two segments apart from qualifying on a slow walk (4–5 m between them on 44).
4. **The time gap goes** (`loop_closure_min_time_gap_s` removed from code, TOML table, settings files and docs; no dead code). The cross-floor pass keeps its own time gap (`loop_closure_cross_floor_min_time_gap_s`, 5 s): the shaft is seen from the floor above and below within seconds, and the vertical limit already separates the passes.
5. **Settings with today's names and units**, added to the `[loop_closure]` table; golden TOML files regenerated in their own commit.

Alternatives considered: only lowering the time gap (8 s in the experiment) — admits standing still and neighbours, and the value depends on walking speed; an excursion test ("the path between went at least X m away from the place") — equivalent in most cases, but needs a second threshold and is more complex than the ratio.

## Acceptance rule (written before any G1 run)

Measured from the C10 odometry snapshots (`raw data/44_c10_sX/odometry_snapshot.bin`, current master's odometry) on s0–s5 with `tools/map_quality.py --floors`, against the C10 final-map table (current master, `44_c10_sX`):

- **Fragility (pass/fail):** façade passes when |out| ≤ 1.5° and |back| ≤ 1.5°. C10: 2 of 6 (s0, s1). G1 must not pass fewer runs. No `FAIL` entry (C10: none).
- **Consistency:** revisit worst |yaw| and median offset, median over the six runs, not worse than C10's beyond the six-input spread; the gallery grid image (`topview_grid.py`, x −1.5…2.5, y 6…18) shows the out and back walls of the gallery on top of each other in at least as many runs as C10 (visual check by the user).
- **Sharpness:** railing NN and stairwell offset, median over the runs, not worse beyond the spread; jitter unchanged.
- **At least one measure better beyond the spread** (expected: the revisit measures, or |façade out − façade back|, the gallery's out/back consistency: C10 0.29/0.31/1.84/0.30/0.35/0.47°).
- If measures conflict, the user decides with the numbers. Runtime of loop closure reported (more candidates are registered).

## Risks

- **More loops, more false ones:** the 8 s experiment raised accepted loops from 31–39 to 47–54 per run; the outlier handling (drop the worst edge) stays as it is until G4. → Watch the dropped-edge count and the plausibility check; a false gallery loop would show as a residual above 0.15 m / 2°.
- **The gallery's heading still comes from the junction:** G1 makes out and back consistent, not the gallery parallel to the start (*inference*, see the research notes). The façade angle may stay where it is.
- **Fixed loop-edge sigmas (5 cm) on a gallery revisit** are overconfident along the gallery axis (C23, fixed in G3).
