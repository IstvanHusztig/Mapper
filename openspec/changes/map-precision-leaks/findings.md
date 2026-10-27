# Findings: where the map of 44 loses precision (2026-10-18)

Measured on recording 44 at the top floor's section (world z = 0) and on 44_short. Every number below was reproduced by a
second run of its script and the conclusions were reviewed (Sonnet); wording follows the review. Limits that apply to
all of it: one recording and one odometry snapshot; only two independent repeat runs (44_s0 equals the baseline, so the
noise floor is s1/s2); several statements rest on tens of visit pairs without an uncertainty.

## Measures

- `tools/plan_section.py`: the line through the densest part of each wall at height H, thresholds sweepable.
- `tools/wall_precision.py`: per 0.5 m piece of that line, thickness, visits (split by sensor side), `pair_diff_m`
  (median difference between two same-side visits; does not grow with the visit count, unlike max-minus-min) and the
  noise floor `half_split_m` (~1.1 cm). The first version's max-minus-min "visit spread" overstated the disagreement
  (5.7 cm, growing to 8.5 cm with 4–5 visits); it is replaced.
- Repeat-run spread of `pair_diff_m` p50: 2.2 (s1) – 3.0 cm (s0 = baseline, s2 2.9).
- Correction (post-push review, consistency-gyro-term 1.1): `half_split_m` comes from a piece's longest visit, so it is
  not the floor of a typical pair. The pair-matched noise (`pair_excess_m`: difference minus 1.25 √(s₁²/n₁ + s₂²/n₂))
  is only 0.4–0.6 cm on 44_baseline, and the excess over it is 2.1–2.5 cm from 10 s apart on: the disagreement in F1 and
  F6 is not the medians' noise. Where along a piece two visits looked explains little of it (Spearman +0.06).

## F1. Loop closure brings repeat visits together; consistency adds little to that

Ablation from one snapshot (44_snap_*), `pair_diff_m` p50 / p90: odometry only 4.5 / 12.9 cm, loop closure only
2.9 / 6.4, consistency only 4.7 / 13.2, both (shipped) 3.0 / 6.1. On the 133 pieces common to odometry only and loop
closure only: 4.3 → 2.9 cm. Consistency's effect is weak and only on pairs under 10 s apart (2.2 → 1.3 cm, ~45 pairs, no
interval; the repeat runs are as small there). The code explains why it can't do more: each consistency chunk (1e6
points) is registered only against its own map (`consistency.cpp:190-216`), so visits in different chunks never
constrain each other. Section lines differ between variants (416 / 285 / 437 / 295 pieces): a rough comparison.

## F2. Consistency raises node-to-node rotation about 4x, and on 44_short it generalises worse

- Rotation increment per IMU sample p50: 0.041° (odometry only), 0.041° (loop closure only), 0.167° (consistency only
  and shipped); gyro 0.024°. The jitter is not white (lag-45 rotation 2.16° vs gyro 1.06°).
- Hold-out on 44_short (odd 10 ms blocks left out of odometry and consistency, at half point density): on the points it
  never fitted, consistency makes walls 1.85 mm thicker than odometry only; on the points it fitted, 2.5 mm thinner. So
  sharpness measured on all points rewards it. (Interval from spatial resampling only; run-to-run variance not
  included, a ~10x wider interval is more honest.)
- A rotation term between consecutive nodes from the gyro (sigma 2–5x the odometry's ARW rule; flat optimum, best of ~7
  tried on the same metric) made the never-fitted points 0.87 mm thinner than odometry only, at ~10 % more consistency
  time. A hypothesis for the cause and a candidate fix, not validated: one 28 s recording, half density, global geometry
  untested. Experiment code: `research_experiments.diff` (environment switches, not in the repo).

## F3. The gyro values at 253–259.7 s are inconsistent with gravity and with the LiDAR

- Over 255.86–259.93 s the raw gyro turns 89.6° (83.7° about gravity, 32° across) and predicts the gravity direction
  29.6° from the measured one; the result turns 5.3° and is 2.4° off (reference stretches: both 4–6°).
- Raw LiDAR slices registered without the odometry (point-to-plane ICP, three starts) give 0.3–2.9° per window in
  windows 630–639 against the gyro's 8–11°; on 624–628 they recover the 9–15° the gyro and result show.
- Gyro z runs at 20–30 °/s from 253 s and drops to −3 °/s within 0.25 s at 259.7 s with the accelerometer still. The
  restamping of 2026-10-09 only shifted timestamps.
- The yaw part rests on the ICP check alone (the gravity test sees only the part across gravity); a degeneracy check of
  that ICP is still missing. Cause (sensor, firmware, other) unknown. The odometry followed the LiDAR there, but the
  gyro-bias estimate swung (−0.86 → +0.48 °/s).

## F4. The damping dependence is chain amplification, not per-window non-convergence

On 44_short, solving each window twice from the identical state with λ0 = 5 and 0.5 differs by p50 0.0006 mm (max
0.026 mm); the chained runs differ at their windows' first node by p50 0.65 mm (windows 1–35) → 4.2 mm (36–70). The
shipped code's only stop rule (delta < 1e-12) never fires, so every window runs all 100 iterations; a stop rule in
physical units, tested in the experiment, doesn't remove the dependence (and would cut LM iterations ~45 %). Which carried state amplifies it is untested; the map effect was not measured in numbers.

## F5. Code findings (verified in the code; effects not measured)

- With `imu_always_on` the motion-model translation term ties each node edge to a guess integrated from the
  start-velocity prior with sigma 1e-4 m (`run_odometry.cpp:1090,1105`; loosened only on the gated path, `:182-194`;
  `l2-vertical.toml:167-169`): about 5 mm/s on a window's velocity (derived, not measured), whatever its covariance.
- Consistency's chunk-local maps (F1).
- LM damping is absolute (λ·I) on the poses only and grows ×10 per 10 iterations; the comment calls ~1e10 negligible
  against ~1e8 (`scan_to_map.cpp:997-1009`). Extends tracker C13.
- Consistency2 (off): `index_end_exclusive = i` instead of `i + 1` (`consistency.cpp:581,585`), dropping each cell's
  last point.
- Already tracked: C1 (consistency jitter), C13 (100 iterations), C4 (map rebuild at window 586/587).

## F6. The suspect walls (weakest part: few pieces, few pairs)

| wall | what the numbers show | reading |
|---|---|---|
| B right end of bottom wall | one piece, thin, on the plan | only faint (seen briefly) |
| A far gallery line | visits agree (0.9–2.6 cm), on the plan; thickness 1.7–9.4 cm across runs | not a pose error; scene geometry is a hypothesis |
| D upper flat bottom wall | ~2 cm, within the repeat-run spread | noise level; single off visits differ between runs |
| C lower-left flat wall | the 4 s and 278 s visits 3.6–5.4 cm apart, 6 cm off the plan (one visit pair) | start/end disagreement; a start artefact not excluded |
| F small rooms | 11 cm odometry only, 1.3–3.9 cm with loop closure | drift between start and end, removed by loop closure |
| E corridor wall | 2 pieces, 4.7–11 cm across runs, off visits around 258–263 s | unresolved; link to F3 is a timing coincidence so far |
| lift shaft | ~3–4 cm between visits whether close in time or far apart | not drift; per-visit errors or the scene (moving car, doors, metal) |

## Open questions

1. Does the gyro term in consistency (F2) hold on the full 44 and s1/s2 with `map_quality.py` (global geometry,
   revisits, sharpness on held-out points)?
2. Does the 5 mm/s velocity anchor (F5) matter: 44_short with the motion-model sigmas loosened under `imu_always_on`?
3. ICP degeneracy check for F3, and whether the gyro prior contaminates the result after 259.7 s (E).
4. Which carried state amplifies the chain differences (F4): a perturbation-growth test.
