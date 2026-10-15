# Proposal

## Why

Change G of the code-review fix plan (`docs/code-review-tracker.md`, findings C22–C25), pulled forward because the map's main weakness on recording 44 is fragility: six near-identical inputs (s0–s5) give maps from good to broken, and the global façade angle passes in only 2 of 6 runs after C10. Diagnosis and the survey of established LiDAR SLAM systems: `docs/loop-closure-research.html`.

The first finding is concrete: **no loop ever touches the gallery.** It is a dead end walked out and back within 10–25 s, under the 30 s minimum time gap between two segments, so its out and back passes are never registered against each other. A settings-only test with an 8 s gap gave the gallery its loops in all six runs; their corrections (0.6–1.75 m, up to 2.7°) are the doubled walls the odometry leaves there.

## What Changes

In steps, each measured from the odometry snapshots of s0–s5 against the C10 noise floor before the next one (the acceptance rule is in `design.md`):

- **G1 — revisits by path, not time (C22):** a same-floor candidate needs at least `loop_closure_min_path_m` (8 m) walked between the two segments and a horizontal centre distance of at most min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × that path) (0.5). Out-and-back revisits of dead ends qualify; standing still, neighbouring segments and places merely seen from afar do not. The time gap `loop_closure_min_time_gap_s` goes (the cross-floor gap stays).
- **G2 — candidate choice and redundancy (C24):** the best passing registration among the candidates instead of the first, and up to `loop_closure_edges_per_segment` edges per segment.
- **G3 — weights from what was measured (C23):** odometry edges by distance, time and turn, inflated along the windows' weak directions; same-floor loop edges by the registration's information, inflated and floored.
- **G4 — outliers (C24):** consistency checks (odometry cycle, pairwise) and graduated non-convexity instead of "drop the worst edge"; needs G3.
- **G5 — anchors (C25):** segment anchor at its middle node; the correction onto the nodes by path length.

G2–G5 are designed in detail after G1's result; this proposal fixes their order and purpose.

## Capabilities

### Modified Capabilities

- `loop-closure`: which segment pairs count as revisits (G1); later the edge choice, weights and outlier handling.

## Impact

- **Code:** `src/application/loop_closure.cpp`; settings in `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`, `settings/lidar-vertical.toml`, `settings/default_settings.toml` (golden TOML files regenerated in their own commit).
- **Tests:** `tests/loop_closure_test.cpp` (the candidate rule on synthetic segments: out-and-back accepted, neighbours and standing still excluded).
- **Docs:** `docs/index.html` (settings table, revisit description), `docs/loop-closure-research.html` (results).
- **Results:** every step changes numbers; measured from snapshots, new baselines in their own commits after acceptance.
