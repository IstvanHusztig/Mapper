## 1. Preparation

- [ ] 1.1 Rule reviewed by Sonnet (done 2026-10-16, corrections folded in); the user's go for the implementation (medium effort).
- [ ] 1.2 Red tests: D3 (≥ 3 chained windows with a carried bias ≠ 0: the estimate settles near half), D5 (chained windows lose 1/N of a constant yaw rate; a `compute_step_1` fixture for the points between windows).
- [ ] 1.3 D2 pre-measurement: fraction of IMU samples the ±0.1 g gate sets to gain 0 on 44 and 45f.

## 2. Fixes (one commit each, test bed s0–s2 per fix)

- [ ] 2.1 D4: Cauchy factor removed, rotation sigmas × √π in settings, defaults, docs (index.html, hu/index.html), the motion-model spec (delta spec) and test fixtures; golden TOML regenerated (own commit); ≈ identical to master.
- [ ] 2.2 D3: bias measurement subtracts only the update; test green; readings by the rule.
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
