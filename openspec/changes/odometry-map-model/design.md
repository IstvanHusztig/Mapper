## Context

- Test bed: `raw data/44c95b_s0..s5` (44_sX cut to −10 … 95 s with `cut_recording`), odometry only (`use_loop_closure`, `use_consistency` false, window diagnostics on). The cut reproduces the full run's trajectory exactly (0.000 m, checked on s0 with and without a code change), so an effect measured on it is the full run's effect up to 95 s. About 7.5 min per input instead of 20.
- Reference: master (33d7abd build) on the same cuts, `44c95bo_master_sX`.
- Metric: the façade line fitted per time window (`map_quality_lib` façade fit, job tool `facade_time.py`): start (0.9–5.9 s), gallery way out (48.9–59.9 s), way back (59.9–70.9 s); the angle gallery − start is the global-geometry measure. Also the start's split (0.9–5.9 vs 5.9–10.9 s), the onset in 40–49 s, and the runs' sanity (largest window time, big-jump rejections, route).

## Phase 1: screening variants (settings only)

| variant | setting | tests |
|---|---|---|
| V1 coarse grid off | `min_distance_lidar_outdoor_grid` = 1000 (no point reaches the 0.4 m grid) | C18 |
| V2 projection off | `degenerate_projection_ratio` = `degenerate_projection_ratio_full` = 0 | C16 |
| V3 exact statistics | `use_exact_ndt_bucket_statistics` = true | C20 on top of C10 |
| V4 short initial map | `threshold_initial_points` = 10 000 (≈ 0.16 s instead of ≈ 0.94 s) | C21 |
| V5 finer cells (positive control) | indoor / outdoor resolution 0.10 / 0.27 m | known ≈ 25 % less drift; validates the test bed |

Plus **V4b** `threshold_initial_points` = 120 000 (dose-response for V4) and **V6**, V1 + V2 + V3 + V4 combined (do small effects add up?). Run order: master s0–s5 (reference; s0 checked against the known odometry values), V5 (stop if it fails), then V1, V2, V3, V4, V4b, V6, each on s0–s2, one at a time, with one pinned build.

### Screening rule (written 2026-10-16, before the runs; revised after the Sonnet review the same day)

- **Readings per run** (façade line, `map_quality_lib`, room frame, absolute): start A (0.9–5.9 s), start B (5.9–10.9 s), gallery (mean of way out 48.9–59.9 s and way back 59.9–70.9 s, one sample: the same wall in one map), the angle gallery − start A, and a heading profile (40–49 s, 71–85 s) for the onset. Sanity: largest window time, big-jump rejections, `trajectory_seams.py` against master, jitter.
- **Noise floor:** master on all six cuts s0–s5: mean M and range [min, max] of each reading.
- **Moves:** a variant's mean over s0–s2 lies outside master's s0–s5 range **and** at least 0.5° from M, with all three inputs on the same side of M. **Inconclusive:** 0.25–0.5° from M or inside the range. **Gain** counts only if the gallery reading moves towards the start's (the angle towards 0) without start A and start B moving apart by more than master's range, or if start A − start B moves towards 0 (the truth is one straight wall).
- **Worse:** a variant that moves the angle away from 0 marks a protective feature, not a drift source.
- **Positive control:** V5 must pass the same rule (gain). If it does not, the test bed or the metric is in doubt and the screening stops.
- **V1:** dropping the coarse grid also halves the NDT information against the IMU and motion priors, so a V1 effect reads "C18 or the NDT / prior balance" until phase 2 re-runs it with the priors rescaled. **V4:** a shorter initial map also moves the window grid (windows start where the initial map ends) and leaves a sparser first map; V4 and V4b must move in opposite directions (monotone) to count, and their start A / start B are read first.
- **V6:** if the combination doesn't move the angle, the map-model thesis is closed **for 44's angle** (not for caves); if it moves while the single ones don't, they add up.
- **Unusable as is:** a window over 10 s, more than 100 big-jump rejections, or a position difference to master of more than 1 m before 40 s.
- Screening decides only what phase 2 works on; nothing is shipped from phase 1. The machine is kept free of other heavy jobs (the 10 s window limit equals `real_time_threshold_seconds`, so load can change results).

### Screening results (2026-10-16; verdict reviewed by Sonnet)

gal−A = mean(way out, way back) − start A, in degrees; master s0–s5: M = −4.31, range [−4.71, −4.02], A−B 0.89–1.00. ΔA / Δgallery against master s0–s2. Every run: 0 big-jump nodes, skipped steps, rebuilds or dropped windows; every window 100 iterations (none hit the 10 s limit); jitter p50/p99 to 45 s 0.36–0.42 / 1.15–1.32° (master 0.40 / 1.21°). Routes against master's 76 m, positions time-matched (window grids differ for V4/V4b/V6).

| variant | gal−A s0/s1/s2 | mean − M | A−B | ΔA | Δgallery | route [m] | outcome |
|---|---|---|---|---|---|---|---|
| V5 finer cells | −3.10 −3.15 −3.23 | +1.15 | 0.84–0.89 | +0.07 | +1.26 | 76 | **gain** (control passes; runtime on the cut not higher) |
| V1 coarse grid off | −3.22 −3.41 n/a | (+1.00) | 0.79–0.82 | +0.07 | (+1.10) | 102 / 116 / 147 | letter: moves (2 inputs); **unusable** (runaway from 49–62 s); way out only, before the divergence: +0.7 / +0.5 |
| V2 projection off | −4.17 −4.14 −4.83 | −0.07 | 0.89–0.96 | 0.00 | −0.03 | 76 | **no move**: within 5 cm of master until 60 s (the projection doesn't act before), so not the 30–49 s drift |
| V3 exact statistics | −3.90 −4.99 −4.23 | −0.06 | 0.99–1.01 | +0.07 | +0.06 | 83 / 75 / 119 | letter: no move; s0, s2 run away from ~45 s (similar to the earlier rejection), s1 usable (−0.82 vs master s1) |
| V4 10 000 initial pts | −3.83 −3.67 −3.98 | +0.48 | 1.08–1.28 | −0.49 | +0.04 | 75 | **does not count**: inconclusive band, not monotone with V4b, start split wider |
| V4b 120 000 initial pts | −4.14 −3.89 −4.13 | +0.26 | 0.87–1.02 | −0.08 | +0.22 | 75 | no move (mean inside the range) |
| V6 V1+V2+V3+V4 | −3.13 −2.99 −3.19 | +1.21 | 0.45–0.70 | −0.98 | +0.27 | 76 | **gain** by the A−B clause; no runaway; 0.55 m off master at 40 s |

- **Deviation from the written rule (added after the data):** a run whose route exceeds master's by more than 10 % or which ends more than 5 m off master is treated as unusable, although the written criteria (window > 10 s, > 100 big jumps, > 1 m before 40 s) did not foresee a runaway after 40 s. Both runaways were silent (0 big-jump rejections).
- **V6:** about 80 % of its change is the start reading turning (A −0.98°, B −0.6°), its gallery change (+0.27°) is inside master's gallery spread: a one-off offset of the start, not a lower drift rate. The singles don't add up to it (ΔA +0.07, +0.07, −0.49). Its stability could come from V2 or V4 (V2's ratios thresholds were tuned on the legacy statistics; under V3 the projection may act earlier).
- **Only V5 moves the gallery itself:** 1.15° of the 4.3°, start split unchanged; the remaining ~3° are not explained by C16, C18 (as far as readable), C20 or C21.
- V1's start split (0.79–0.82, below master's minimum) and the room-frame caveat (A is uncertain by about A−B ≈ 1°; only differences between variants carry meaning) are noted for phase 2.

## Phase 2 (designed after the screening)

The user's decision (2026-10-16): a finer-cell fix (2a), and a second screening round for the remaining ~3° on the research candidates (2b). Rules written 2026-10-16, before the runs, revised after a Sonnet review.

### 2a Finer cells

- **Dose-response on the test bed** (settings only; `44c95b_s0..s2`, odometry only, like phase 1), paired against master on the same input:
  - both grids: F1 0.12 / 0.32 m (indoor / outdoor, × 0.8), V5 0.10 / 0.27 (done), F3 0.08 / 0.21 (× 0.53);
  - one grid: F4 indoor 0.10, outdoor 0.40; F5 indoor 0.15, outdoor 0.27.
  - Readings and sanity as phase 1, plus runtime and peak working set per run. Unusable also: route more than 10 % above master's, or more than 5 m off master at the end (the criterion added in phase 1). Decimation stays 0.03 m, so a planar 0.08 m cell holds at most ≈ 7 points (0.10 m ≈ 11) with the legacy first-point mean: a confound for F3, noted.
- **Gain** of a variant = mean over s0–s2 of (variant − master, same input) of the gallery reading (mean of way out and way back). It counts only if gal−A also passes phase 1's "moves" condition, |ΔA| ≤ 0.25°, A−B and jitter p99 stay within master's range ± 0.1°: a gain must turn the gallery, not offset the start (V6).
- **Choice:** G* = the best usable gain. Among the usable variants whose gain is within 0.4° of G* (≈ 2 SE of a difference of two 3-input means), the one that changes the fewest grids, then the coarsest. If a finer variant than the chosen one runs away, that is a cliff next to it: the chosen setting must then run on all six cuts without a runaway, else the next coarser candidate is taken. The chosen setting runs on s3–s5 too (all six cuts usable).
- **Validation** of the chosen setting in `settings/lidar-vertical.toml`. The consistency pass uses the indoor resolution (`consistency.cpp`), loop closure doesn't (its refinement grids are fixed at 0.5 / 0.25 m) and changes only through the trajectory. Full 44 s0–s5 with loop closure and consistency, `tools/map_quality.py --floors`, paired per input against master's `44_g3b2bs6_s0..s5`:
  - global geometry: façade way out / back better by more than master's s0–s5 spread (0.88 / 0.91°) in the mean, or the spread itself smaller by more than half;
  - consistency (revisit |yaw| median and worst, pairs) and sharpness (railing NN, stair max, jitter p50 / p99): each mean worse by less than half master's s0–s5 spread; no spread wider by more than half;
  - 45f against master's `45f_cand`: loops found ≥ 90 % of master's, no runaway (route within 10 %), the position difference reported;
  - runtime of every stage and peak working set reported; every window under the 10 s limit (100 iterations).
  - If consistency or sharpness is what fails, the consistency pass gets its own resolution setting (code), run at 0.15 m with the new odometry cells, with its own rule.
- Then the plan overlay and before/after views for the user's visual check, new baselines in their own commit, push with the user's go.

#### 2a dose-response results (2026-10-16)

Gain = gallery reading paired against master on the same input (s0–s2); master's gal−A M = −4.31, range [−4.71, −4.02], A−B 0.89–1.00 (allowed 0.79–1.10), jitter p99 over the 95 s 1.30–1.36°. Every run usable: routes 75–77 m, at most 1.5 m off master at the end, 0 big jumps, every window 100 iterations; peak working set 0.66 (master) / 0.66–0.73 GB; runtime 7–8.5 min per cut for all, within the run-to-run spread (master s0 twice: 400 / 478 s window time; variants 388–438 s), largest window 3.4 s.

| variant | cells in / out [m] | gain s0/s1/s2 | mean | gal−A mean (vs M) | moves | ΔA | A−B | jitter p99 Δ | counts |
|---|---|---|---|---|---|---|---|---|---|
| F1 | 0.12 / 0.32 | +0.39 +0.06 +0.52 | +0.32 | −4.04 (+0.27) | inconclusive | +0.01 | 0.88–0.95 | −0.09…0.00 | no |
| V5 | 0.10 / 0.27 | +1.17 +1.05 +1.57 | **+1.26** | −3.16 (+1.15) | yes | +0.07 | 0.84–0.89 | −0.06…−0.02 | **yes** |
| F3 | 0.08 / 0.21 | +0.43 +1.05 +1.18 | +0.89 | −3.48 (+0.83) | yes | +0.02 | 0.72–0.79 | −0.08…−0.02 | letter: no (A−B below 0.79, i.e. straighter start) |
| F4 | 0.10 / 0.40 | +0.04 +0.67 +1.00 | +0.57 | −3.78 (+0.53) | yes | −0.01 | 0.85–0.98 | −0.06…−0.01 | yes |
| F5 | 0.15 / 0.27 | +0.50 +0.76 +0.95 | +0.74 | −3.65 (+0.66) | yes | +0.04 | 0.84–0.89 | −0.05…−0.01 | yes |

- **Choice: V5 (0.10 / 0.27 m).** G* = +1.26 (V5). Within 0.4°: only V5 (F3 at 0.37 would be too if its A−B clause is read as a gain, as phase 1's rule does for A−B towards 0; then V5 still wins as the coarser of two two-grid variants). F5 / F4 (one grid) are 0.52 / 0.69 below G*. No finer variant runs away (no cliff). The choice doesn't depend on the F3 reading.
- Not monotone: F1 +0.32 < V5 +1.26 > F3 +0.89. The F1 → V5 step (+0.94, same sign on all three inputs) is real and threshold-like, so V5 may sit near an edge that other (cave) geometry can move; V5 over F3 (0.37 ± 0.21) is not significant, and A−B falls monotonically with cell size (F1 0.91, V5 0.86, F3 0.76), so a saturating dose-response fits too. F3 has ≤ ≈ 7 points per planar cell with the legacy first-point mean, a noted confound. F4 + F5 = +1.31 ≈ V5's +1.26 may be coincidence (F4's gains spread by 0.49). The onset reading (40–49 s) doesn't follow the gain and master's own spread there is 3.4°: no corroboration either way.
- V5 on s3–s5: all usable (routes 75–76 m, 0 big jumps, 100 iterations); gal−A −2.89 / −3.25 / −3.07, gallery gains +1.21 / +1.03 / +1.57 (s0–s2: +1.17 / +1.05 / +1.57). V5 on s0–s5, the 2b reference: gal−A mean −3.12, range [−3.25, −2.89], A−B 0.84–0.90.
- Probe build (incidence drop, off by default through an environment variable): `44_short` IDENTICAL with the probe off.

#### 2a validation results (2026-10-16): V5 on full 44 s0–s5 and 45f

`settings/lidar-vertical.toml` with 0.10 / 0.27 m (job tmp `lv_v5.toml`, snapshot and stage timing on), build `bin_final_g` (master); results `raw data/44_v5_s0..s5`, `45f_v5`; `tools/map_quality.py --floors` (job tmp `mq_v5.txt`) paired against `44_g3b2bs6_s0..s5` (`mq_g3b2bs6.txt`).

| measure | V5 mean | master mean | paired diff s0…s5 | mean diff | master spread (half) | V5 spread | rule |
|---|---|---|---|---|---|---|---|
| façade out [°] | −3.35 | −3.23 | −0.50 −0.68 −0.04 +0.29 +0.03 +0.23 | −0.11 | 0.88 (0.44) | 1.04 | needs better by > 0.88: **fails** |
| façade back [°] | −3.28 | −3.29 | −0.39 −0.58 +0.02 +0.41 +0.15 +0.40 | 0.00 | 0.91 (0.46) | 1.07 | **fails** |
| revisit \|yaw\| median [°] | 0.52 | 0.49 | | +0.03 worse | 0.36 (0.18) | 0.19 | ok |
| revisit worst [°] | 2.07 | 1.85 | +0.92 −0.49 −0.10 +0.49 +0.53 −0.02 | +0.22 worse | 0.85 (0.42) | 0.59 | ok |
| pairs | 62 / 66 | 60 / 66 | | +2 | | | ok |
| jitter p50 / p99 [°] | 0.82 / 3.40 | 0.92 / 4.44 | p99 −1.18 −0.88 −0.94 −1.32 −0.96 −0.93 | −0.10 / −1.03 better | 0.02 / 0.29 | 0.00 / 0.33 | better beyond the spread |
| railing NN [cm] | 5.27 | 4.72 | +3.0 +0.8 −0.7 −0.8 +1.2 −0.2 | +0.55 worse | 3.60 (1.80) | 2.70 | ok |
| stair max [cm] | 8.10 | 10.85 | −5.5 −0.4 −1.3 +0.8 −9.2 −0.9 | −2.75 better | 8.40 (4.20) | 2.50 | ok (not beyond the spread) |

- **45f:** 62 loops against master's 67 (93 % ≥ 90 %); positions at most 1.67 m apart (after 185 s), no runaway. The raw route is 11 % shorter (262 / 295 m; the letter "within 10 %" fails), but on a 1 s grid the routes are 80.5 / 79.2 m: the difference is sub-second pose jitter (p50 / p99 0.81 / 2.86° against 0.91 / 3.76°).
- **Runtime / memory:** V5 s0 1583 s (odometry 1092, loop closure 424, consistency 52, save 18 s), 24–28 min per input, peak 4.35 GB (45f 3.18 GB); master's full run `44_finalg` took 2086 s (odometry 1375 s; V5's odometry 974–1169 s, loads differ): no extra cost. Every window ran 100 iterations (69 000 LM calls for 690 windows), none hit the time limit.
- **Verdict by the 2a rule: fails.** Global geometry, the goal of 2a, is not better on the full map (−0.11 / 0.00°), (paired −0.11 ± 0.16 / 0.00 ± 0.17° SE). Consistency and sharpness pass. Not reasons against V5 (Sonnet review): the wider façade range (1.04 / 1.07) is driven by one input (s3), the SDs are equal (0.39 / 0.38); the railing's +0.55 ± 0.59 cm is noise (master's s0 2.9 cm is its best of six).
- **What the numbers say about loop closure** (revised after the review): the cut and the full-map façade metric are the same fit on the same windows (master odometry-only on the full map, `44_c10o`, −3.88…−4.67, matches its cut). Finer cells (+1.26°) and loop closure on master (+1.05°) remove about the same 1.1–1.2° component; loop closure on V5 is neutral (−0.20 ± 0.10°), and a residual of ≈ −3.2° stays that neither sees. The odometry's scatter between inputs survives loop closure (cut vs full r = 0.67 master, 0.83 V5), so loop closure does not pin the angle. The end-spin edges of G4 change about one edge per input: a weak candidate.
- **For caves:** V5's odometry needs less from loop closure: largest correction 0.76–1.21 m against 1.06–3.00 m, accepted loops 101 against 92 on average. That matters where loops are few, and the rule (beat the loop-closed master) could not show it.
- **Jitter:** made in post-processing (master s0 odometry-only 0.39 / 1.29°, final 0.93 / 4.64°; V5 final 0.82 / 3.46°, on the cut V5's p99 is only 2–5 % lower). V5 also changes the consistency pass (indoor resolution 0.15 → 0.10 m), so the drop may be consistency's, and whether it is sharpness or smoothing needs a map-level measure (wall thickness, occupied cells) and a control with consistency at 0.15 m.
- **The user's look** (s0, not blind): s0 is V5's worst input on façade (−0.50), railing (+3.0 cm) and revisit worst (+0.92°); a blind look at s3 / s5 is still to come.
- **45f route:** a 1 s grid (or anything from 0.25 s) belongs in the rule's runaway criterion; the raw route is per-node noise (0.1 s grid: 93.9 / 93.1 m).

#### 2b pre-check: cell thickness (2026-10-16, job tool `fe_thick.py`, master's cut s0, 0.15 m cells, one 1 s slice per cell)

- Cells with ≥ 6 points and λ₁ ≥ 10 λ₀ (the cells G1 / G2 act on, 80 157): median thickness 5.5 mm, flat in incidence (0–90°) and range (0.5–20 m), 3.8–6.7 mm in every bin; the fit with d · sin α lowers the residual by ≈ 12 % (4.0 → 3.5 mm², σ_ω 0.05°), with d · tan α not at all.
- Looser cells (λ₁ ≥ 2 λ₀): thickness grows with range at α ≥ 30° (6 → 11 mm at 6–20 m), fitted d · sin α with σ_ω ≈ 0.09°; these are mostly the less planar cells G2 would not touch, and even there σ is ≈ 3 cm at 20 m / 80°.
- Limits (Sonnet review): the planarity filter removes the thick cells (λ₁ ≥ 10 λ₀ keeps 75 % of the cells near, 29–40 % at 6–20 m and α ≥ 30°); with every 2nd point, "≥ 6 points" means ≈ 12 real ones; in a 3 m gallery d ≥ 6 m means α ≥ 75°, and those bins hold only 33 cells (none beyond 10 m). The looser run (added after seeing the strict one) shows growth with σ_ω ≈ 0.09°, an upper bound on random bearing noise (it includes odometry error within 1 s).
- **G2 dropped** by the rule, for a random-noise model: the cells it acts on don't thicken with α · d, and even the loose bound gives σ ≈ 3 cm at 20 m / 80°. G3 (G1 + G2) falls away with it. The test cannot see a systematic bearing or range offset with incidence (which bends a wall without thickening it), nor inconsistency between passes; it doesn't exclude a bearing error. G1 is the probe that remains, and it too acts only on planar cells: a G1 null needs its logged coverage of the far grazing points.

### 2b Second screening (probe code on a scratch build, removed or made a real setting afterwards)

- **Order:** starts from 2a's dose-response choice (not after its full validation). Reference: that choice on `44c95b_s0..s5` (new noise floor, six runs); G1 and any variant that counts also once on master's cells, comparable with the earlier −0.4…−0.9°. With every probe off, the build must be bit-identical to the reference (`44_short` manifest).
- **Incidence angle**, per grid, for a matched cell with at least 6 points and a planar covariance (λ₁ ≥ 10 λ₀; others unaffected): α = acos |n · b̂|, n the cell's normal, b̂ the unit beam from the node position to the point in world coordinates. The legacy normal is fixed from the cell's first three points, so the normal of the current covariance (smallest eigenvector) is used.
- **G1 / G1b:** drop points with α > 70° / > 80° before every use of the point (normal equations, `normal_information`, the degenerate projection, `ndt_window_cost`). Logged: fraction of points dropped, and the yaw information retained.
- **G2 (only if a measurement supports it):** first, offline on master's cut map s0, the cell thickness (√λ₀) against α and range d. If it grows with d · sin α (bearing error) or d · tan α, σ_ω is fitted from it and G2 adds σ² = (d · σ_ω · f(α))² along n to the cell covariance (Sherman–Morrison: λ₀ → λ₀ + σ²), with f the fitted form, at the fitted σ_ω and at 2 × (dose), weighted identically in every use (as G1). If thickness doesn't grow with α · d, G2 is dropped and reported.
- **G3:** the strongest counting dose of G1 and of G2 combined (small effects that add up).
- **Rule:** phase 1's rule against the new reference, plus the 2a gain condition (gallery turns, start not offset) and the added runaway criterion. A variant whose stronger dose moves less than its weaker one (|G1| < |G1b|, |G2 at 2 ×| < |G2|) does not count. Inconclusive: run s3–s5 once, then classify with all six. A variant that counts becomes a real setting with its own validation like 2a's.

#### 2b results (2026-10-16): G1 incidence drop

Probe build `bin_probe` (environment variable, off = `44_short` IDENTICAL). Reference V5 s0–s5: M = −3.115, range [−3.25, −2.89]. Every run usable (routes 75.7–77.7 m, at most 1.5 m off master at the end, 0 big jumps, every window 100 iterations, peak 0.66–0.71 GB, 6–7 min).

| variant | dropped (points with a normal) | yaw information kept | gal−A s0/s1/s2 | mean − ref | gallery gain (paired) | ΔA | A−B | outcome |
|---|---|---|---|---|---|---|---|---|
| G1 V5 cells, α > 70° | 14.1–15.7 % | 70.5–71.0 % | −2.80 −3.07 −3.58 | −0.03 | +0.26 +0.06 −0.33 (0.00) | −0.04…+0.02 | 0.81–0.88 | no move (six inputs) |
| G1b V5 cells, α > 80° | 3.8 % | 87.0–87.3 % | −2.94 −3.20 −2.99 | +0.07 | +0.12 −0.04 +0.26 (+0.11) | −0.04…+0.02 | 0.85–0.88 | no move (six inputs) |
| G1m master cells, α > 70° (comparison) | 14.4–14.8 % | 70.2–71.1 % | −3.88 −3.55 −4.11 | +0.47 vs master | +0.32 +0.69 +0.62 (+0.54) | +0.01…+0.07 | 0.93–1.06 | inconclusive band; reproduces the earlier −0.4…−0.9° |

- **Reading of "inconclusive"** (Sonnet review): the rule says "0.25–0.5° from M **or inside the range**". Phase 1 read |Δ| < 0.25° inside the range as no move without writing it down (V2, V4b at +0.26 is inconclusive by the letter; it doesn't change V4's verdict). By the letter G1, G1b and G1m are inconclusive, so all three run on s3–s5 (after the 2a validation) and are classified with six inputs; the verdict below is provisional until then.
- **Provisional:** G1 on 0.10 m cells doesn't move (−0.03, +0.07 from the reference; paired gallery 0.00 / +0.11); the drop removes 14–16 % of the points whose cell has a planar normal and 29 % of their yaw information (the matched-point share and the far-grazing coverage were not logged). On 0.15 m cells the drop gives +0.54° in the gallery (3 inputs, inconclusive; the contrast with V5 cells is ≈ 2.7 SE, suggestive only). Consistent with grazing points and cell size acting on the same part of the drift (not additive); the mechanism is not identified (the 0.15 m pre-check showed no thickening with α). It excludes a range bias concentrated at α > 70° in planar cells larger than about 0.4°; not tested: a bias at 30–70°, non-planar cells, the map (it still holds the grazing points). Whether the earlier hook's −0.4…−0.9° used the same cell selection can't be checked (code removed).
- **G2:** the pre-check rule had no numeric growth criterion. With λ₁ ≥ 10 λ₀ the d · sin α fit lowers the residual by 12 % (σ_ω 0.05°, σ ≈ 1.7 cm at 20 m / 80°); with λ₁ ≥ 2 λ₀ σ_ω ≈ 0.09° (≈ 3 cm, several times the 5.5 mm thickness). G2 was not run: an incidence-dependent covariance in the registration remains untested. G1, the harder version of it above 70°, doesn't move on 0.10 m cells; at mid incidence nothing is known. Left as a candidate for the cave retune (change I).
- **With six inputs (s3–s5 run 2026-10-16 evening; two batches overlapped by mistake on different inputs, every window still ran 100 iterations, so the load changed nothing; G1 s5 re-run alone):**
  - G1 (V5 cells, 70°): gal−A mean −3.02 against V5's −3.12 (+0.10, inside the range), paired gallery +0.08 ± 0.11: **no move**.
  - G1b (V5 cells, 80°): −3.13 (−0.01), paired gallery −0.02 ± 0.08: **no move**.
  - G1m (master cells, 70°): −3.88 against master's −4.31 (+0.44, outside the range, all six on the same side), paired gallery +0.46 ± 0.12: inconclusive band, consistently positive.
  - **2b closed: no setting.** The grazing drop helps on 0.15 m cells (+0.46 ± 0.12) and not on 0.10 m cells (+0.08 ± 0.11): the two act on the same part of the drift (not additive).
- The remaining ≈ 3° on the cut (V5 gal−A −3.12) are not explained by C16, C18 (as far as readable), C20, C21, cell size or grazing points above 70°.
- The probe code is removed from the tree (kept only as the job-tmp build `bin_probe` for the s3–s5 runs); the job-tmp settings and `bin_probe` are deleted with the result folders.

For each mechanism that moves the angle: a fix in the code where the setting was only a probe (for example a down-weighted or hierarchical coarse grid instead of none, a consistent degenerate projection, motion-compensated initial points), its own acceptance rule written before its runs, full runs on s0–s5 with loop closure and consistency, `map_quality.py --floors` against master's results (global geometry, consistency, sharpness, equal weight), 45f, the user's visual check, baselines, push with the user's go.

## After phase 2: odometry-only full maps and the brainstorm checks (2026-10-16 evening)

- **V5 odometry only, full 44 from the snapshots** (`44_v5o_sX`, job tmp `mq_v5o.txt`, against master's `44_c10o`, `mq_c10o.txt`): façade out / back −3.01 / −3.22° against −4.18 / −4.45°, spread 0.34 / 0.39 against 0.79 / 0.59, mean of the per-input revisit offset medians 0.65 against 1.18 m, pairs 62 against 53 of 66, revisit yaw worse (mean of medians 0.60 against 0.47°), jitter identical (0.40 / 1.27°). The geometry gain is solid; the narrower spread is not significant (F ≈ 3–6 at 5 / 5 df). The final maps' jitter difference is made in post-processing.
- **Brainstorm** (research agent): the error is a regional swing, not a growing drift. The wall direction at 14–30 s equals the start's; the map turns during 30–49 s; the gallery is straight but rotated; after 71 s the reading returns to ≈ 0. Ranked causes: (1) a beam-angle calibration error of the L2 (SDK parameters `alpha_angle_bias`, `beta`, `xi`, `theta_angle_bias`, `range_scale`, `range_bias`) bending far points ahead; (2) rotation–translation coupling in the weakly constrained 30–49 s stretch; (3) the start reading itself off by ≈ 0.9°; (4) the gyro bias soaking up LiDAR drift (C26); then NDT cell bias, incidence range bias, LiDAR–IMU misalignment / tilt, the plan; de-skew and timing set aside.
- **Check a, heading against the gyro** (job tool `yaw_resid.py`, cut s0, constant body bias +0.612 / +0.464 / −0.716 °/s from master's final map; positive = the trajectory turns more than the gyro, counter-clockwise, matching gal−A ≈ −3°): per 1 s the residual alternates ±1–2 °/s (lag-1 autocorrelation −0.54: pose noise of ≈ 0.75° per node, not events). Regression on the running sum: 0.081 / 0.123 / −0.133 °/s in 0–30 / 30–71 / 71–84 s (V5 0.070 / 0.089 / −0.077); 0–10 s alone 0.25 ± 0.11. The sensor z stays within ≈ 6° of world z, so an error of the reference bias only adds a constant slope: the slope changes at 30 s and 71 s are robust to it. The extra turn in 30–71 s is 1.7° against the 0–30 s slope (3.5° against 10–30 s); after 71 s the sum falls ≈ 1.7° (≈ 3° below the trend). Single-sample master−V5 differences are at the noise level. A gyro scale error up to ≈ 1 % is not excluded (slope 0.07 ± 0.5 %); a timing offset wouldn't correlate with the rate anyway.
- **Check b, gyro-bias estimate:** steady through 30–49 s (z −0.43…−0.50 °/s, master and V5 alike), so it doesn't soak up the swing; it is 30–50 % below the final map's bias on every axis (consistent with C6).
- **Check d, start window refit:** A's halves (0.9–2.2 / 3.2–5.9 s, around the 0.40 s IMU gap; the first inside Fusion's 3 s initialisation, the second after it) agree (+0.58…+0.73°), A and B on the same wall stretch differ as before (+0.77 / −0.22°): the start turns ≈ 0.95° between 0.9–5.9 and 5.9–10.9 s; not the IMU gap, not the fit, and not the initialisation's heading reset alone (which weakens D1 as its cause). gal−B: master −3.19 / −3.16 / −3.82°, V5 −2.20 / −2.32 / −2.37°.
- **Check e, static recording 43** (user's OK; LiDAR points only, sensor frame, no IMU; job tool `static_planes.py`): a small room (1–4 m), some "planes" are furniture. Planes 0, 2 and 5 share a normal at different offsets with residual sd 14–26 mm: clutter. Only plane 4 is clean (3.4–4.4 m, θ 0–45°): flat to ±2 mm, near / far normals 0.14°. A ±12 mm bend at 3–6 m is ≈ 0.2°, so a 0.4° bend is not excluded; the test can't reach 10–20 m at grazing incidence, where 44's bend appears.
- **Check c, floor and ceiling near / far in the gallery** (job tool `fcb2.py`): far points sparse (1–11 of 23 slices), the far band may hold other structures, and values differ strongly between near-identical inputs (far ceiling −5.4 / −6.3 / −13.4° on master s0–s2, −0.7° on V5): inconclusive. The far floor rises on both sides of the sensor in all four runs (a bowl centred on the sensor, which a building slope can't make; the four runs share the same points): unexplained, inconclusive.

- **Review (Sonnet):** none of the checks a–e supports the beam-angle hypothesis yet (it comes from the research agent's reasoning). A static calibration scan is one-sided (a null excludes, a positive doesn't prove): it needs grazing geometry (sensor 1.5–3 m from a long flat wall, pointing along it, 10–25 m), turns about the sensor's own axis, and a flatness reference; first check whether the recorder applies the unit's SDK calibration parameters. On 44 itself: a sensor-fixed against world-fixed bend from the out / back passes. Change D as correctness fixes is sound, but not D1 first: D3 (C6, bias 30–50 % low) before or with D4 (C5, prior π× weak), since a stronger prior with a low bias would hurt; C15 needs a red test first; add a 3-axis hand-eye fit (gyro bias, scale, misalignment) on the existing data; don't expect D to remove the 3°, and a stronger gyro weighting can mask a calibration error.

- **dLidar's answer** (prompt `dlidar-calibration-prompt.md`, 2026-10-16): the recorded points are already computed with the unit's 8 factory calibration parameters from every packet, by Unitree's SDK formula unchanged (matched the vendor SDK within 4 µm); vertical-mount remap x = orig_z, y = orig_y, z = −orig_x; no lever arm, rotation or deskew; LAZ scale 0.1 mm; dropped at capture: no return, < 0.1 m, > 100 m, intensity < 5. So "calibration not applied" is excluded; what remains of the hypothesis is an error of the factory values (or of the model) themselves. Recording format 2 (none recorded yet) stores per point the raw range, sample index, packet and the two angles, and per packet all calibration values: a static scan in format 2 allows recomputing the points with adjusted parameters (self-calibration from flatness). 44 is format 1 (no extras). dMapper must be checked to read format-2 LAZ (extra bytes) on the first real recording.

## Research (agent, 2026-10-16; sources in the session report, summarised)

- Incidence-angle range bias bends straight tunnels and causes drift; correcting it reduced the drift (Laconte et al., ICRA 2019, up to 20 cm at high incidence). Ouster self-calibration from map consistency improved corridor orientation by ≈ 0.15° (Agishev et al.). Remedies in use: drop near-grazing points (Loam_livox), an incidence term σ = d·σ_ω·tan α in each point's covariance (LOG-LIO2).
- In a corridor, yaw information grows with the square of the distance along it: yaw is pinned by far wall points at grazing angles, where 44's façade bends 0.5–0.7° inside single scans. A bearing error of ≈ 0.4° there would suffice; the L2 manual gives no calibration data (inference).
- NDT practice: a cell's distribution from more than 5 points, near-singular covariances inflated (Magnusson; PCL ≥ 6 points, eigenvalue floor); no source uses the first point as the mean. Multi-resolution runs coarse to fine in sequence, not both grids at equal weight at once. Discretisation biases rotation (Gupta et al., ECMR 2023).
- Degeneracy remapping depends on thresholds and frames and can misalign rotation (X-ICP, Tuna et al. 2024).
- No source on a systematic first-frame yaw bias.
- Phase-2 candidates added by the research: an incidence-dependent point covariance, and a sensor self-calibration from map consistency (plane thickness).

