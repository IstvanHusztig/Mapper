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

## G2 — candidate choice, redundancy and sibling consistency (C24, part 1)

### Context (the G3 2 × 2, 2026-10-14)

- One edge per segment, chosen by order (nearest first in C10, longest path first in G1), makes the outcome depend on discrete choices: longest path first always picks segment 0, whose loops are poor (0→57 says 8.5°, its siblings 1→44…1→56 0.4–2.2°), and G1 bent the start room with it.
- After the solve, the greedy drop rule can't tell a right edge from a wrong one near its threshold: in G3b s2 the gallery loop 9→13 (twin 8→13 agrees within 3 cm / 0.4°) and the outlier 0→57 were 2 % and 0.8 % over the limit.
- What separates them is agreement with neighbouring edges, which needs more than one edge per region.

### Review (Sonnet, 2026-10-14) and what it changes

- "Two nearest + two longest path" mostly tries near-duplicates (neighbouring segments share 2/3 of the i−1…i+1 target); overlap after the refinement measures fit, not correctness, and prefers near views; two near twins in both slots re-create the slot displacement and count the same information twice.
- The sibling check fails on exactly the two cases: 9→13 has one sibling (no agreeing pair), and 0→57's siblings 0→56 / 0→58 share the poor segment 0 (common mode) and can outvote 1→56. A consistency check needs distinct sources and thresholds scaled by the length between the ends (PCM): moved to G4.
- The drop order is the cheapest lever: worst ratio first dropped 9→13 (2 % over the limit) before 0→57 (0.8 % over).

### Decisions (revised)

1. **Leave-one-out drop order (G2a):** while edges exceed the residual limit, each of them is removed in turn and the graph solved without it; the edge whose removal lowers the total cost the most is dropped (a false edge carries cost its removal frees; a right edge's removal leaves the others strained). It applies to same-floor and cross-floor edges alike, so it is also the first look at the cross-floor drops behind the stairwell regression. Cost: one extra solve per edge over the limit per round (the graph has ≈ 58 nodes, milliseconds).
2. **Diverse tries (G2b):** of the qualifying segments, the nearest one, then the nearest at least 3 segments away from it; the longest-path one, then the longest-path one at least 3 segments away from it; at most `loop_closure_candidates` = 4 distinct.
3. **Two slots by class (G2b):** up to one accepted "near" edge (the tried set's two nearest) and one "far" edge (the two longest-path), each the better of its class by overlap after the refinement; a slot is left empty when its class registered nothing. Near and far sources are at least 3 segments apart when both are filled, so they never share a target.
4. Settings: `loop_closure_candidates` 3 → 4 (settings files) and `loop_closure_edges_per_segment` (1 or 2: one slot reproduces G1's single edge, with the near class tried first); golden TOML files in their own commit.
5. Measured as G1 + G2a first (drop order alone), then G1 + G2a + G2b, then with G3b: one thing at a time.

### Acceptance rule for G2 and G2 + G3 (addendum written 2026-10-14, before their runs)

The G3 rule's median criterion compared with C10's chance value, which an honest loop closure can't meet. From now on:
- **Primary:** the façade spread over the six inputs ≤ C10's (3.7°), and the median |out| / |back| not worse than the odometry alone's (4.0 / 4.4°) beyond its spread (0.8°): loop closure must not make the angle worse than the odometry, and must not widen the scatter beyond C10's.
- **Hard:** the start room not turned (0–9 s within ±0.3° in every run).
- **Guards against G1:** gallery out vs back ≤ 0.3° in every run; revisit worst median not worse beyond the spread; railing median not worse beyond the spread; the stairwell not worse than C10's in 5 or 6 of the six runs (a consistent regression, as G3b's); no `FAIL`.
- **Reported:** the yaw correction per band, edges removed by the sibling check and dropped after the solve, the accepted loops against G1's, runtime.
- Configurations: G1 + G2a, G1 + G2a + G2b, and with G3b (and G3a + G3b) on the best of them. The user decides between those that pass.
- **Added after the review (2026-10-14, before the runs):** a positive consistency criterion against the odometry alone (revisit worst median not worse than its 1.7°, revisit offset median well below its ≈ 1.2 m); paired per-run comparison of the façade against G1 (worse in at most 3 of 6); G2-specific checks: in s2 the gallery loop 9→13 is kept and 0→57 is dropped, and the set of accepted loops is similar across s0–s5 (most loops present in at least 4 of the 6 runs).

## Segments of 2.4 s and the ablation (2026-10-15)

### What was measured before this rule was written

- G3b + G2b at 4.8 s twisted the gallery in s2 (out − back 2.4°); the cross-floor edges are not the cause (off: twist unchanged, stairwell 23–84 cm, railing 16–39 cm, so they are essential). Information-weighted edges have yaw sigmas 0.85–1.7° (fixed edges 0.3°, odometry edges 0.5° per segment).
- Halving the segments (`loop_closure_segment_windows` 12 → 6, a setting) removed the twist. Candidate C* = G1 + G2a + G2b + G3b at 2.4 s: façade out −2.84 … −3.72 (spread 0.88, median |out| / |back| 3.2 / 3.2), gallery out − back ≤ 0.11° in all six, start room +0.05 … +0.08°, revisit worst median 1.77°, railing median 4.4 cm, stair median 9.5 cm (worse than C10 in 6 of 6 by 0.2–3.3 cm), loop closure 517–572 s per run (259 s at 4.8 s).
- By the G2 rule C* fails the stair guard and misses the positive consistency criterion (1.77 > 1.7°); the halving was chosen after the 4.8 s configurations failed, on the same inputs, and the s2 checks had to be remapped (start loop 0 → 36 at 2.4 s). The user decided (2026-10-15) to run the ablation below before deciding.

### Ablation rule (written 2026-10-15, before the runs)

- Runs at 2.4 s from the C10 snapshots, s0–s5, one at a time: (A) G1 + G2a with fixed sigmas (one edge per segment, G1's tried order; the build before G2b), (B) G1 + G2a + G3b (the build before G2b), (C) C* + G3a.
- **Paired test:** configuration X is worse than Y on a measure when it is worse in at least 5 of the 6 inputs; measures: |façade out|, gallery out − back, revisit worst, railing, stair max; plus the façade spread (worse by more than 0.5°) and the hard criteria (start room within ±0.3°, gallery out − back ≤ 0.3° in every input).
- **G3b stays** only if A is worse than B on at least one measure or fails a hard criterion; **G2b stays** only if B is worse than C* on at least one measure or fails a hard criterion; **G3a is added** only if C is better than C* on at least one measure and worse on none. Otherwise the simpler configuration is chosen and the unused code is removed after the user's OK.
- G2a is not ablated: it changed nothing on 44 and is kept as a correctness fix with its unit test, unless the user decides otherwise.
- **Also measured:** the stairwell of the odometry alone (floors check on the C10 odometry-only results); false registrations: the gross drops (residual > 1 m) and the largest residuals among the kept edges, for the chosen configuration; recording 45f with the chosen configuration if the recording is available.
- 45f (restored by the user on 2026-10-15, which ends the one-time exception granted the same day): master's build and settings run it with a snapshot; the chosen configuration runs from that snapshot; compared by the log (loops accepted and dropped, corrections), map sharpness (`eval_odometry.py` occupied cells) and a top view.

### Ablation result (2026-10-15) and decision

| configuration (2.4 s) | façade out | spread | start room 0–9 s | gallery out − back | revisit worst median | railing median | stair median |
|---|---|---|---|---|---|---|---|
| A: G1 + G2a, fixed sigmas | −0.25 … −2.61 | 2.65 | −0.26 … −0.73 (5 of 6 outside ±0.3) | ≤ 0.16 | 1.5 | 6.4 | 7.5 |
| B: G1 + G2a + G3b | −2.07 … −3.92 | 1.85 | −0.25 … +0.12 | ≤ 0.15 | 1.5 | 6.8 | 7.8 |
| C*: G1 + G2a + G2b + G3b | −2.84 … −3.72 | 0.88 | +0.05 … +0.08 | ≤ 0.11 | 1.77 | 4.4 | 9.5 |
| C: C* + G3a | −2.86 … −3.73 | 0.87 | +0.05 … +0.09 | ≤ 0.15 | 1.9 | 4.3 | 9.2 |
| odometry alone | −3.88 … −4.67 | 0.79 | – | ≤ 0.34 | 1.7 | 18 | 142 |

- By the rule: A fails the hard criterion, so G3b stays; B is worse than C* on the railing (5 of 6), the façade spread (+0.97°) and the gallery (5 of 6, by 0.02–0.1° only), so G2b stays; C is better than C* on no measure, so G3a is not added.
- Reported: A puts a −3° turn at 9 s; in s0 its whole façade lies on the plan (start, gallery and the way back at 71–85 s within 1°), in s2 its way back is 7° off. The odometry's façade error lies in the gallery stretch (−4 … −5.7° there, −0.8° on the way back at 71–85 s in s2); the candidate leaves the gallery about 2° off the same wall seen on the way back. Loop closure cannot locate this error (way out and way back pass the same place); it is the odometry's (change F/E).
- False registrations at 2.4 s: 2–4 gross ones per run (1.2–5.9 m, up to 11°), all dropped; three start ↔ end edges (3 → 113/114, 4 → 112) are kept with 2–6° rotation residual under the weighted limit (0.08–0.14 of 0.15): G4 (consistency between edges) is the next fix for caves.
- Decision (user, 2026-10-15): ship C*; G3a's code removed; G3b permanent (switch removed, fixed sigmas only as the fallback); defaults `loop_closure_segment_windows` 6, `loop_closure_candidates` 4, `loop_closure_edges_per_segment` 2. The shipped build reproduces C* on 44_s2 exactly (max difference 0.000 m, same loops).

## G4 — conflicting loop edges (design, 2026-10-15)

### Diagnosis (temporary logs and an exclusion hook, not committed; 44 from the C10 snapshots, shipped settings)

- **Residuals:** of the 411 kept same-floor edges over s0–s5 the weighted residual is 0.02 at the median and 0.07 at the 95th percentile (limit 0.15, i.e. 3 σ). The edges at 0.08–0.14 are almost only the start ↔ end edges to segments 112–114, in all six inputs; no threshold separates them cleanly (s5: 0.08; a gallery edge in s4: 0.13).
- **Which edges are wrong:** 14–15 edges from the start room to segments 97–111 agree within ±1.5° in yaw. Segment 112's registrations (4 → 112, 7 → 112) correct yaw by +1.4 … +4.3°, those of 113 and 114 by −5 … −8°, from three different sources (0, 3, 4). Removing 3 → 113 / 114 (cut A) brings 4 → 114 and 0 → 114 in with the same −7° and residuals of 6–7°; removing 4 → 112 (cut B) brings 7 → 112 with 2.4°. The recording ends with a fast spin on the spot (+51, +25, −39, −28° per 2 s, 0–0.2 m moved); a 9° odometry error between segments 112 and 114 would need a 20–35 % gyro error, so the registrations of the spinning end segments are biased, all in the same way: **an inconsistent block, probably wrong** (consistent among three sources, 9° from segment 112 and the 97–111 consensus; the gyro argument rules out an IMU error, not a scan-to-map error during the spin, so 112 could in principle be the wrong one), which agreement among its own edges cannot reveal.
- **Cross-floor tilt:** kept cross-floor edges correct roll / pitch by 0.3–3.7° with three at 4.6–6.9°; same-floor edges mostly ≤ 1°, tail to 2.2°. No gap a gate could use on this data; deferred.

### Second review (Sonnet) and the offline vote table

- Votes must count independent evidence, not edges: per distinct target segment j', and only from sources at least 3 segments away (the registration target is i − 1 … i + 1, so closer sources share data). A fixed 6-dof χ² on information edges with free directions and an arbitrary scale is not sound; a 1-dof yaw test is what the diagnosed failure needs (yaw sigma per edge from the registration information, finite where yaw is determined).
- **Offline table** (job tmp `g4_votes.py` on the diagnostic logs; yaw of the corrections, neighbours 1–5 targets away, sources 3–5 away, votes per target, threshold 3°): neighbour pairs' |Δyaw| p50 0.5–0.8°, p90 2.6–2.8°, the end block 4.5–8.5°. The vote removes 3 → 113 in s0 and s2 and nothing else among 70 / 65 edges; 3 → 114 has one independent neighbour (left to the drop rule), 4 → 112 ties (kept).
- **Consequence:** on 44 G4a would remove one edge per input in the last seconds of the recording; its measurable effect is small, and its value lies in caves (self-similar passages, more spins), for which there is no data yet.

### Decisions (after the Sonnet review)

1. **G4a, local consistency of the correction field (first):** a loop edge's correction (a world transform) and the corrections of edges into neighbouring target segments differ only by the odometry's error over the few segments between them. For each same-floor edge e (i → j), its independent neighbours are the edges e' (i' → j') with 1 ≤ |j' − j| ≤ `loop_closure_consistency_segments` (5) and 1 ≤ |i' − i| ≤ the same: a different target and a different source, both close (a long odometry chain makes the test too loose to count as support). The cycle error of e and e' (e's measurement, the odometry from j to j', e''s measurement reversed, the odometry from i' to i) is tested with χ² (6 dof, 99 %: 16.8) against the sum of both edges' covariances and the odometry sigmas of the segments between. An edge that conflicts with more of its independent neighbours than agree with it, with at least two independent neighbours, is removed before the solve and logged; an edge with fewer neighbours is left to the drop rule. Limit: a biased block of adjacent targets larger than its good neighbourhood outvotes it (here the block is 113–114 against 108–112). Settings: `loop_closure_consistency_segments` (0 = off). Unit tests: a target segment with a biased registration from three sources among consistent neighbours is removed; a consistent false pair sharing a target is removed by its independent neighbours; isolated edges stay.
2. **G4b, robust kernel (after G4a, only if conflicts remain):** `loop_closure_kernel_scale` = √χ²₆(95 %) = 3.55 (fixed in advance), with `loop_pose_graph_cost` scoring the loop edges with the same Cauchy cost the solver minimizes.
3. **Tilt gate or 4-DoF loop edges:** deferred until stairwell data shows a gap; noted for the cave retune.
4. Thresholds stay settings; the cave recording retunes them (change I). Agreement on 44 (one building, six near-identical inputs) is about one or two independent trials, not evidence for caves.

### Acceptance rule (written 2026-10-15, before the runs)

- Reference: master (21306a3) on the same snapshots (`44_g3b2bs6_*`; 45f_cand); G4b against master + G4a if G4a is kept.
- **Effect sizes:** a run counts as better or worse only beyond: façade |out| 0.3°, gallery out − back 0.1°, revisit worst 0.3°, start-room revisits (17.5 / 270 and 30 / 265) 0.3° in yaw, railing 0.5 cm, stair max 1 cm.
- **Target:** the start-room revisits better in at least 4 of 6 inputs; **no harm:** no measure worse in 5 or more of 6, façade spread not wider by more than 0.5°, hard criteria (start room 0–9 s within ±0.3°, gallery out − back ≤ 0.3° in every input).
- Reported, not scored: edges removed by the check (which, in how many inputs), the count of kept edges with a rotation residual above 2°, runtime.
- **45f:** no loss of sharpness (`eval_odometry.py` jitter, occupied cells, a top view); edges removed there listed.
- A step that fails is removed after the user's OK.
