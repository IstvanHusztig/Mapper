# Design

## Context

`run_loop_closure` (`src/application/loop_closure.cpp`) groups the windows into segments of 12 windows (about 4.85 s on 44), picks revisit candidates by the horizontal distance of the segment centres (time gap ≥ 30 s, within 8 m horizontally, within 1.5 m vertically), registers each (coarse yaw/translation voxel search, then NDT refinement) and accepts with an overlap ≥ 0.5. `optimize_loop_pose_graph` solves the segment poses by Levenberg–Marquardt with a cost check (odometry σ 0.1 m / 0.5°, loop σ 0.05 m / 0.3°). Then the gating drops the worst loop edge while it disagrees by more than 0.15 m / 2°.

**What 44 looks like** (measured 2026-10-10, perturbed inputs `44_s0/s1/s2`):
- **Three floors** (sensor at +0.15 / −2.75 / −5.63 m). The path between floors takes a straight staircase about 8 m from the round shaft. Floor −2 is visited once.
- **The floor offsets** (`tools/stairwell_check.py`, railing circle per floor and pass): shipped 18–47 cm between floors 0 and −1, and 23–70 cm between floors −1 and −2, always in −x. The fits have 1–3 cm residuals and full arcs.
- **Before the shaft comes back into view,** the way down and the first ~40 s on floor −1 see nothing that was mapped before (no overlap after 97 s), so the odometry runs open-loop there.
- **The way up is registered against the map of the way down,** so the same-floor loops agree with each other and nothing in the graph measures the offset.

## Goals / Non-Goals

**Goals:**
- Floors tied through the stairwell to within the railing's few-cm noise on all perturbed inputs, without bending a floor or losing loops.

**Non-Goals:**
- The odometry's heading drift on stairs (the cause of the offsets): its own investigation.
- Better registration in general (multi-resolution, global descriptors), and place recognition for candidates.

## Decisions

1. **Cross-floor edges are full 6-DoF loop edges.**
   - Measured: the true corrections between floors include rotations of 2–6°, consistent between independent registrations (six pairs on `44_s2` at 5.9–6.4°).
   - Alternatives tried and rejected:
     - **Translation-only edges with a weak rotation (5°, ≤ 1.5°):** inconsistent. On `44_s1` none were accepted. On `44_s0` floor 0/−1 was 12 cm against 4 cm with 6-DoF. A registration's translation is tied to its rotation (the coarse search turns about the source's centroid), so dropping the rotation leaves a translation that is off.
     - **A 3° limit:** it rejected exactly the large true corrections (floor −2 on `44_s2` stayed 68 cm off).
   - The edge uses the loop-edge sigmas, so no rotation setting of its own.
   - Acceptance: overlap ≥ `loop_closure_cross_floor_min_fitness` (0.3), no worse than before registration, translation ≤ 1 m, rotation ≤ `loop_closure_cross_floor_max_angle_deg` (10°).
2. **The drop judges a cross-floor edge by its translation only.**
   - This is the rule all cross-floor measurements were made with (X3).
   - The rotation of a registration in a round shaft is its least certain part. A 6-DoF edge still constrains the rotation, but a rotation residual alone doesn't remove it.
3. **Cross-floor pairs get their own time gap,** `loop_closure_cross_floor_min_time_gap_s` (5 s).
   - The 30 s gap keeps a same-floor loop from pairing a segment with its own odometry neighbourhood. Between floors there is always a staircase in between, so even a short gap spans the drift.
   - Measured: floor −2's only usable partner is floor −1's view of the shaft 10–20 s before. With the 30 s gap no variant tied floor −2 (23–70 cm); with 5 s it is tied to 1–7 cm.
4. **Cross-floor registration only on the footprint both segments share,** with the coarse search limited to `loop_closure_cross_floor_search_m` (1 m).
   - Both clouds are cropped to the horizontal 0.5 m cells (one cell of margin) the other segment also covers.
   - Without the crop, most of each cloud is ground the other floor never saw. The overlap was then ≤ 0.23, and the ±6 m search found corrections of 3–7 m and 6–60°.
5. **Cross-floor is a second pass per segment, one more edge.** Same-floor loops keep their order and their one edge per segment. With `loop_closure_cross_floor_max_vertical_m` = 0 the result is unchanged bit for bit.
6. **The robust kernel (Cauchy) on the loop edges, in two stages,** off by default.
   - Each step computes, per loop edge, the whitened squared residual s², the weight w = 1 / (1 + s² / c²) with c = `loop_closure_kernel_scale`, and the cost ρ = c² · log(1 + s² / c²). The reweighted quadratic bounds ρ from above, so the LM step check on ρ stays valid.
   - **A plain solve first, then the kernel from that solution.** From the odometry's poses every true loop looks like an outlier: it is off by the whole drift (1–2 m on 44, 20–40σ). Single-stage, with all loops right, the largest error on the test square rose from 1.1 to 4.9 cm.
   - **Scale:** after the plain solve, the residuals are well under 1σ. On two test laps with one edge 15 cm off, 0.5σ halves the largest error (6.4 → 3.2 cm, the all-correct value) and moves consistent edges by 0.5 mm; 3σ does nothing measurable.
   - **Not measured on real data yet:** it is switched on only if it improves the evaluation set, and removed otherwise (no unused code).
   - Odometry edges stay quadratic.
7. **The angle wrap in the solve.** The cost wrapped angle residuals into ±180°; the right-hand side of the solve didn't. After the first step, an edge with a relative yaw near ±180° (a revisit walked in the opposite direction) saw a residual of about 360°, so every further step was damped away. Fixed by shifting the measurement's angles by whole turns in `add_relative_pose_edge`; bit-identical on the shipped 44.
8. **Evaluation set and metric:**
   - inputs: the three perturbed 44 inputs, `45f_p`, a `/fp:fast` build, and the unperturbed 44 for the baseline;
   - main metric `tools/stairwell_check.py` (the offset between floors); `tools/spot_check.py` at the railing spot (−3.01, −5.42, −0.12);
   - also: loops, largest correction, floor 0 start against end, jump warnings;
   - `ghost.py` (the job folder's revisit check) is not used for decisions: it gave a 130 cm offset where the stairwell check showed an improvement.

## Risks / Trade-offs

- **[A wrong cross-floor match on stacked, similar floors]** → the shared-footprint crop, the 1 m translation limit, the 10° rotation limit, the translation drop, and (if kept) the kernel. Measured: 0–2 edges dropped per run, no jump warnings.
- **[A larger largest correction]** → on `44_s2` 2.39 m against 1.92 m shipped. Checked in task 4.2; it is the correction moving floor −2 back, which is expected when a 70 cm offset is removed.
- **[More registrations cost time]** → about 100 cross-floor tries on 44. Measured and reported (precision before speed).
- **[44 is a building; caves have no floors]** → cross-floor candidates also cover vertical shafts and steep passages, the cave case. Checked later on the user's cave recording.
- **[The heading drift on stairs remains]** → the loop closure repairs its effect after the fact. Within a stretch that no loop covers, the drift stays.

## Migration Plan

Settings off in code (`loop_closure_cross_floor_max_vertical_m` = 0, `loop_closure_kernel_scale` = 0). `settings/lidar-vertical.toml` switches cross-floor edges on with the measured values. New baselines in their own commit, the old folders renamed with a date suffix. Rollback: set the cross-floor height to 0.
