# Design

## Context

`run_loop_closure` (`src/application/loop_closure.cpp`) cuts the trajectory into segments of 12 windows (≈ 4.8 s), registers each segment against earlier ones that qualify as revisits, and solves a pose graph over the segment anchors. Today a same-floor pair qualifies when the earlier segment ended at least `loop_closure_min_time_gap_s` (30 s) before the later one began, and their centres are within `loop_closure_search_radius_m` (8 m) horizontally and 1.5 m vertically. Measurements and the survey of other systems: `docs/loop-closure-research.html`.

## Goals / Non-Goals

**Goals:** out-and-back revisits (dead ends, the gallery of 44) become loops; neighbouring segments, standing still and places merely seen from afar do not; the rule works for slow cave surveys as for walking through a building.

**Non-goals in G1:** edge weights, edge choice, outlier handling, anchors (G2–G5).

## Decisions (G1)

1. **Path walked instead of time.** The time gap is a proxy for "left the place and came back"; the path measures it directly. Standing still adds no path, so it never qualifies (a 30 s gap admits it); a fast out-and-back qualifies (a 30 s gap rejects it). Established systems gate by path or index (hdl_graph_slam, SC-PGO, LAMP); LIO-SAM's time gap is the exception.
   - The path is summed over the first node of each window (0.4 s apart), counting a step only once the position has moved at least 0.1 m from the last counted one: summing every node, or every window without that threshold, would add the pose jitter of a person standing still (Sonnet review: over minutes it could reach the 8 m minimum).
2. **The allowed distance grows with the path:** centre distance ≤ min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × path), as LAMP 2.0 scales its search radius with the path (drift grows with it). This one rule also excludes neighbouring segments, whose centre distance is about the path between them (ratio ≈ 1), and places seen from afar.
   - Data on 44 s0 (distance / path of same-floor pairs): the gallery's true revisits 10→13 = 0.04 and 9→14 = 0.06, others 22→25 = 0.13, 15→18 = 0.45; pairs two segments apart 0.6–2.1 (mostly ≈ 1); 9→12 = 0.79 (centres 6.8 m apart: the gallery seen from its other end, accepted by the 8 s experiment only because the whole gallery is visible from anywhere in it). `loop_closure_radius_per_path` = 0.5 separates them with margin on both sides.
3. **A minimum path, `loop_closure_min_path_m` = 8 m:** below it the odometry is good enough and a revisit adds little; it also keeps two segments apart from qualifying on a slow walk (4–5 m between them on 44).
4. **The time gap goes** (`loop_closure_min_time_gap_s` removed from code, TOML table, settings files and docs; no dead code). The cross-floor pass keeps its own time gap (`loop_closure_cross_floor_min_time_gap_s`, 5 s): the shaft is seen from the floor above and below within seconds, and the vertical limit already separates the passes.
5. **Longest path first (no slot displacement).** Today the candidates are tried nearest first and the first accepted one is the segment's only edge. With short paths admitted, a near local partner would take the slot of a far one: the 8 s experiment replaced four loops of the global closure on s0 (start ↔ return, 5→50, 5→51, 6→52, 6→53, ≈ 180 m of path) by local ones (47→50, 47→51, 49→52, 50→53). G1 therefore tries the qualifying candidates by decreasing path: a segment with a long-range partner keeps it, a segment whose only partners are local (the gallery's way back) gets those. Still one edge per segment (redundancy is G2).
6. **Logged per candidate:** path and distance/path ratio, so that the accepted loops can be compared with C10's (which pairs were added, which lost).
7. **Settings with today's names and units**, added to the `[loop_closure]` table; golden TOML files regenerated in their own commit. Loading reads only known keys, so an old TOML that still has `loop_closure_min_time_gap_s` loads and the key is ignored; `docs/index.html` and `docs/hu/index.html` say so.

Alternatives considered: only lowering the time gap (8 s in the experiment) — admits standing still and neighbours, and the value depends on walking speed; an excursion test ("the path between went at least X m away from the place") — equivalent in most cases, but needs a second threshold and is more complex than the ratio.

## Acceptance rule (written before any G1 run)

Measured from the C10 odometry snapshots (`raw data/44_c10_sX/odometry_snapshot.bin`, current master's odometry) on s0–s5 with `tools/map_quality.py --floors`, against the C10 final-map table (current master, `44_c10_sX`):

- **Fragility (pass/fail):** façade passes when |out| ≤ 1.5° and |back| ≤ 1.5°. C10: 2 of 6 (s0, s1). G1 must not pass fewer runs. No `FAIL` entry (C10: none).
- **Consistency:** revisit worst |yaw| and median offset, median over the six runs, not worse than C10's beyond the six-input spread; the gallery grid image (`topview_grid.py`, x −1.5…2.5, y 6…18) shows the out and back walls of the gallery on top of each other in at least as many runs as C10 (visual check by the user).
- **Sharpness:** railing NN and stairwell offset, median over the runs, not worse beyond the spread; jitter unchanged.
- **At least one measure better beyond the spread** (expected: the revisit measures, or |façade out − façade back|, the gallery's out/back consistency: C10 0.29/0.31/1.84/0.30/0.35/0.47°).
- If measures conflict, the user decides with the numbers. Runtime of loop closure reported (more candidates are registered).
- **Added 2026-10-14, after the 8 s experiment and before any G1 run** (Sonnet review): a continuous companion to the pass count, the median over the runs of |façade out| and |façade back| (C10: 2.2° / 2.1°; 8 s gap: 3.7° / 3.7°) and their spread, because 2 of 6 against 0 of 6 is not significant (Fisher, two-sided p ≈ 0.45); if the result is marginal, more inputs (s6–s11) are run. Also reported: where the loop correction turns the trajectory (yaw of the result minus the odometry's, over time), and the accepted loops against C10's (added, lost). The pass count stays in the rule: the façade is measured against the plan, so C10's two passes are real accuracy even if their mechanism was chance.

## Risks

- **What the 8 s experiment does and doesn't predict:** it also admitted start-room pairs (1→4, 2→5, 3→6, corrections ≤ 0.4°) that G1 excludes (except 0→6 and 2→6), and it lost four global-closure loops to slot displacement, which G1 avoids (decision 5). Its façade result therefore doesn't predict G1's.
- **More loops, more false ones:** the 8 s experiment raised accepted loops from 31–39 to 47–54 per run; the outlier handling (drop the worst edge) stays as it is until G4. → Watch the dropped-edge count and the plausibility check; a false gallery loop would show as a residual above 0.15 m / 2°.
- **Where C10's façade came from:** the yaw correction of C10's loop closure turns the trajectory by about −3° at 9–16 s, in the start room (Sonnet review, yaw of the result minus the odometry's), although the start room's own revisits (accepted with the 8 s gap) show it consistent within 0.4°. If so, C10's better façade angle in two runs rests on a correction placed where the start room shows no drift. Loop closure alone can make places consistent; the gallery's direction against the start needs less odometry drift or a constraint between them (*inference*).
- **Fixed loop-edge sigmas (5 cm) on a gallery revisit** are overconfident along the gallery axis (C23, fixed in G3).

## G3a — odometry edges weighted by what was walked and turned (C23, part 1)

### Context (measured 2026-10-14)

- Loop closure is where the fragility comes from: the odometry alone gives a façade angle of −3.9…−4.7° on all six inputs (spread 0.8°); C10's loop closure widens it to 3.7°, the 8 s gap to 2.4°, G1 to 5.0°.
- With every odometry edge at the same 0.1 m / 0.5°, the pose graph can't tell where the odometry drifted. It puts the loops' disagreements into whatever edges are cheapest, and both C10 and G1 turn the map in the start room (C10 −3° at 9–16 s; G1 −1.5° at 0–9 s, inside the façade's start reference), where the start room's own revisits show it consistent within 0.4°. The horizon investigation located the odometry's turn between ≈ 30 s and the gallery (corridor), not in the start room.
- The accepted loops' corrections don't grow steadily with time or path (angle rms 1.0° at < 20 m, 0.5–0.8° at 20–120 m, 2.0° at > 120 m): the drift comes in episodes, so no time or path model can say where it happened. What a model can do is stop the graph from putting it where little was walked or turned.

### Review (Sonnet, 2026-10-14) and what it changes

- Path-based odometry variances only **dilute**: the start room's share of a correction falls to its share of the path, the rest is spread by path over everything else. They don't move a turn to where the drift happened (path carries no information about episodes). Locators exist (the LiDAR-vs-motion-model yaw disagreement per window of `log_heading_diagnostics`, the per-window gyro bias), but loop closure can't read them today (not in `Window` or the snapshot): a later step.
- **Series model:** a loop over ≈ 20 segments forces ≈ 98 % of its disagreement into the odometry with 0.3° loop sigmas, ≈ 83 % with 1°. So the six-input spread is mostly the loop registrations' scatter passing straight through; **per-edge loop information** (the registration's shape, weak directions free) matters more than any uniform inflation, and the robust kernel is off (`loop_closure_kernel_scale` 0).
- Hence G3a and G3b are measured together as a 2 × 2 on the snapshots (G1 is the base cell), with κ = 0 and the floors × 3 / ÷ 3 as controls for G3a.

### Decisions (G3a, revised)

1. **Effective length** of the odometry edge k → k + 1: L_k = d_k + κ · ψ_k, d_k the path walked across segment k (G1's path), ψ_k the **rotation about the world vertical** turned across it [deg], summed as |angle · axis.z| of R_{w+1} R_wᵀ between the windows' first nodes (as `is_turn_window`; not the sensor-frame Tait–Bryan yaw: the sensor is mounted vertically). κ = `loop_closure_odometry_turn_m_per_deg` = 0.1 m/deg (no data behind it: κ = 0 is a control run).
2. **Translation and yaw variances scale with L normalised to the run's mean:** σ_t,k² = σ_t² · max(L_k / mean L, f) and σ_r,k² = σ_r² · max(L_k / mean L, f), with σ_t = `loop_closure_odometry_sigma_m` (0.1 m) and σ_r = `loop_closure_odometry_sigma_deg` (0.5°) as today; equal L reproduces today's weights exactly, so the odometry's overall stiffness against the loops is unchanged. Floor f = 0.1 (a segment of standing still is ≈ 3× stiffer in sigma than an average one, not rigid; × 3 / ÷ 3 as controls).
3. **Roll and pitch keep today's constant sigma** (gravity bounds them).
4. **The correction onto the nodes is interpolated by the path** (by L) between anchors, not by time, so standing still inside a segment takes no share of its turn.
5. `optimize_loop_pose_graph` takes per-edge odometry weights (tests build equal ones with a helper), one code path.

## G3b — same-floor loop edges from the registration's information (C23, part 2)

1. Same-floor loop edges get the registration's information like the cross-floor ones (`loop_edge_information`, height kept): its shape from the NDT refinement's normal matrix, scaled so that the best-determined direction has today's 5 cm; weakly determined directions get correspondingly larger sigmas instead of a fixed 5 cm / 0.3° in all directions.
2. The drop rule then uses the weighted residual for them, as for the cross-floor edges today.
3. Calibrating the overall scale against repeated registrations is left for later; the shape is what the series model says matters.

## Acceptance rule for G3 (written before its runs, 2026-10-14; replaces the G3a rule above)

Configurations from the C10 snapshots on s0–s5: G1 (base), G1 + G3a, G1 + G3b, G1 + G3a + G3b; controls for G3a: κ = 0, floors × 3 and ÷ 3 (on the best G3a cell).
- **Primary measure: the façade's spread over the six inputs** (fragility; C10 3.7°, G1 5.0°, odometry alone 0.8°) together with its median |out| / |back| (C10 2.2 / 2.1°). A configuration qualifies when its spread is ≤ C10's and its median not worse than C10's.
- **Hard criterion:** the start room is not turned: the yaw correction in 0–9 s within ±0.3° in every run.
- **Guards** (not worse than G1 beyond the six-input spread): gallery out vs back, revisit worst, railing and stairwell medians, no `FAIL`.
- **Reported:** the yaw correction per time band against the independent evidence (the odometry's turn lies between ≈ 30 s and the gallery), dropped loops, the largest per-segment correction / L, runtime; the controls must not move the result.
- The user decides between the qualifying configurations, or on conflict.

### First draft (superseded by the revised decisions above)

1. **Effective length of an odometry edge** (segment k → k + 1): L_k = d_k + `loop_closure_odometry_turn_m_per_deg` × ψ_k, with d_k the path walked across segment k (the G1 path, standing still adds nothing) and ψ_k the yaw turned across it [deg], summed as |Δyaw| between the windows' first nodes (a turn and the turn back both count). κ = 0.1 m/deg: a 90° turn counts like 9 m walked, about one segment on 44.
2. **Variances grow with it:** σ_t,k² = σ_t0² + σ_t² · L_k / L_ref and σ_r,k² = σ_r0² + σ_r² · L_k / L_ref, with σ_t = `loop_closure_odometry_sigma_m` (0.1 m) and σ_r = `loop_closure_odometry_sigma_deg` (0.5°) now meaning the sigmas of a reference length L_ref = `loop_closure_odometry_reference_length_m` = 8 m (the mean per segment on 44: 456 m in 58 segments ≈ 7.9 m). So the odometry's overall stiffness against the loops stays as it is; only its distribution changes, which keeps the comparison with G1 about one thing.
3. **Floors** σ_t0 = 1 cm and σ_r0 = 0.05° (named constants): a segment of standing still is stiff but not rigid.
4. **All three rotation components scale alike** (roll and pitch too): simpler, and roll/pitch drift is small on 44; to revisit with G3b.
5. `optimize_loop_pose_graph` takes the per-edge weights (the tests build equal ones with a helper, giving today's behaviour), so there is one code path.

Alternative considered: time-scaled variances (q·Δt, the gyro's random walk) — a standing segment would weigh like a walking one, the artifact this step is meant to remove.

### Acceptance rule of the first draft (superseded by the G3 rule above)

From the C10 snapshots on s0–s5 against C10 (and reported against G1 for attribution), as for G1:
- the façade pass count (|out|, |back| ≤ 1.5°) not below C10's 2 of 6, with the continuous companion (median |out| / |back|, C10 2.2 / 2.1°, G1 3.2 / 3.4°) and the spread (C10 3.7°); no `FAIL`;
- consistency: gallery out vs back (C10 0.29–1.84°, G1 0.07–0.24°), revisit worst / offset, the gallery grid image;
- sharpness: railing and stairwell medians not worse beyond the spread;
- at least one measure better beyond the spread; on conflict the user decides;
- reported, not a criterion: the yaw correction in the start room (0–9 s, expected within ±0.3°: the artifact gone) and per band, the accepted loops against G1's, runtime.
