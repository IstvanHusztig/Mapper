## 1. Implementation

- [ ] 1.1 `std::stable_sort` in `load_data` (points by timestamp), `decimate` (by cell index) and the consistency pass ((cell, point) pairs by cell).
- [ ] 1.2 Unit test for `decimate`: same representatives with extra points elsewhere, the first point in input order kept. Release `ctest`; Debug not needed (no memory, threading or build change).

## 2. Measurement (rule in design.md)

- [ ] 2.1 Exact cut: `44c95_s0` vs full `44_s0`, odometry only, ≤ 1 mm up to 90 s.
- [ ] 2.2 Full s0–s5 with `settings/lidar-vertical.toml`, `map_quality.py --floors`, against `44_g3b2bs6_*`.
- [ ] 2.3 45f: jitter, top view.

## 3. Ship

- [ ] 3.1 New 44 and 44_short baselines (own commit); tracker C14; the user's visual check; master-reviewer; push with the user's go; archive.
