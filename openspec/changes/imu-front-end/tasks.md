## 1. Preparation

- [x] 1.1 Rule reviewed by Sonnet (done 2026-10-16, corrections folded in); the user's go for the implementation (medium effort).
- [x] 1.2 Red tests: D3 (≥ 3 chained windows with a carried bias ≠ 0: the estimate settles near half), D5 (chained windows lose 1/N of a constant yaw rate; a `compute_step_1` fixture for the points between windows).
  - D3 (`scan_to_map_test.cpp`): six chained windows, 2.0 °/s injected: 0.98–0.99 before, 1.96 after.
  - D1 (`orientation_test.cpp`, Fusion adapter): 3.5 s at 10 °/s from the reset: yaw 5.0° before (start-up zeroed), 35.0° after.
  - D5 (`window_cut_test.cpp`, the first test of `compute_step_1`): 6 chained windows at 1 rad/s lose 0.005 rad before, 1e-16 after; 500 points between windows in no window before, all kept after (the last window also keeps the points up to the next IMU sample).
- [x] 1.3 D2 pre-measurement: fraction of IMU samples the ±0.1 g gate sets to gain 0 on 44 and 45f.
  - 44: 44.5 % of the samples (walking; |acc| p1/p99 0.77/1.31 g), 3407 gated runs, median 4.5 ms, p90 114 ms, max 289 ms.
  - 45f: 11.7 %, median run 3.0 ms, p90 43 ms, max 301 ms.
  - So variant a would switch the tilt correction off for almost half of 44, in short bursts.
  - Latent today (`d2_ramp.py`): the ramp ends at the first sample with `rampedGain < settings.gain`, and after it the gain never changes again. With `ahrs_gain` 0.05 the ramp passes 0.05 at 3.14 s; a gated stretch of ≥ 16 ms at that moment would end it with gain 0, i.e. a gyro-only AHRS for the rest of the run. 44 and 45f are at rest then (0 % gated in 3.0–3.3 s), so they are not hit; a recording that starts walking could be. With variant a the gain follows the settings again after the ramp.

## 2. Fixes (one commit each, test bed s0–s2 per fix)

- [x] 2.1 D4: Cauchy factor removed, rotation sigmas × √π in settings, defaults, docs (index.html, hu/index.html), the motion-model spec (delta spec) and test fixtures; golden TOML regenerated (own commit); ≈ identical to master. Not identical (the (1 + x²) factor is amplified at the gallery entry); accepted narrowly on s0–s2, rule amendment 2026-10-17 (design.md): D4 also on s3–s5, the later fixes on s0–s5.
- [x] 2.2 D3: bias measurement subtracts only the update; test green; readings by the rule. Accepted on s0–s5 (design.md "D3 results"): bias estimate now −0.81 (z), gallery unchanged.
- [ ] 2.3 D1: no heading reset during Fusion's initialisation; unit test; readings (differences only; positions after a start-yaw alignment).
- [ ] 2.4 D2: variant a (dynamic gain through Fusion's settings API, own settings copy, unit test, readings + tilt reading); variant b (removal, bit-identical) only with the user's decision.
- [ ] 2.5 D5: the pinned first node advanced by the IMU's motion between the two samples, the points between windows kept; tests green; Release + Debug ctest; loop closure and consistency on `44_short`; readings.
- [ ] 2.6 After each accepted numeric fix: `44_short` new baseline (own commit).

## 3. Cumulative and probes

- [ ] 3.1 All accepted fixes on the cuts s0–s5; table and verdict (high effort, Sonnet review).
- [ ] 3.2 P1 prior strength (×1 / ×π / ×π²) on s0–s2; P2 gyro scale fit on 44 s0–s5 and 45f.

## 4. Validation and shipping

- [ ] 4.1 Full 44 s0–s5 + 45f, `map_quality.py --floors`; verdict by the rule (high effort, Sonnet review).
- [ ] 4.2 The user's visual check (plan overlays, folders); full-44 baseline and manifest (old folders dated); `docs/PIPELINE.md`; tracker C2, C3, C5, C6, C15; master-reviewer; push with the user's go.

## Parked (2026-10-17, user's decision: the corridor bend first, change corridor-heading)

- D1 measured on s0–s5 (`fe_read_d1.txt`, gal−A mean −4.30, no verdict written); D2 variant a coded and tested (red 12.6° → green 0°), not committed: job-tmp patch `d2a_final.patch`, its runs stopped after s1. D5 committed, Debug ctest green, not measured on its own (it is part of the corridor-heading reference build, HEAD b97e2e1).
- Open: D1 verdict, D2a runs and decision, D5 reading, P1 / P2, the cumulative and full validation (3.1–4.2).
