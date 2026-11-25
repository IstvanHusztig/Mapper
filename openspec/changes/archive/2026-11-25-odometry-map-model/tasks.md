## 1. Phase 1: screening

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Parked 44 diagnosis (V5 parked without a verdict; 3.3 done as imu-front-end); the findings here are the record.**

- [x] 1.1 Master on the cuts s0–s5 (reference, odometry only).
- [x] 1.2 V5 (positive control), then V1–V4 on s0–s2, odometry only, one at a time; per run: façade gallery − start (out / back), start split, onset, sanity.
- [x] 1.3 Table and verdict by the screening rule (high effort, Sonnet review); the user decides what phase 2 works on. (Table and verdict in design.md. The user's decision, 2026-10-16: phase 2 = a finer-cell fix, plus a second screening round on the research candidates for the remaining ~3°: dropping grazing points, an incidence-angle point covariance.)

## 2. Phase 2

- [x] 2.1 Designed after 1.3 (2a finer cells, 2b grazing points / incidence covariance; rules and results in design.md).

## 3. After phase 2 (the user's decision, 2026-10-16: "go with option 1")

- [x] 3.1 2a validation of V5 on full 44 s0–s5 + 45f: fails the 2a rule on global geometry, passes consistency and sharpness (design.md "2a validation results"). **V5 parked** without a verdict.
- [x] 3.2 V5 odometry-only full-map readings from the snapshots (master's: `44_c10o`): geometry +1.2° solid (design.md "After phase 2").
- [ ] 3.3 Change D (IMU front end) with its own rule, judged on odometry-only cuts first, then loop-closed.
- [ ] 3.4 Cell size again on top of D: decomposition (odometry only / loop closure / + consistency, V5 with consistency at 0.15 m), a map-level sharpness measure, the rule written first; a blind look at s3 / s5.
- [x] 3.5 G1 on s3–s5: with six inputs G1 and G1b no move on V5 cells, G1m +0.46 on master cells; 2b closed, no setting.
- [x] 3.6 Research agent brainstorm of the remaining ≈ 3°, checks a–e on existing data (design.md "After phase 2"): the swing is regional (30–71 s, back after 71 s); the top hypothesis (L2 beam-angle calibration) is not yet supported by data; needs a static calibration scan.
