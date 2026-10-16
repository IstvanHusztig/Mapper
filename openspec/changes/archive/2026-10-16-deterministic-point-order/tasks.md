## 1. Implementation

- [x] 1.1 `std::stable_sort` in `load_data` (points by timestamp), `decimate` (by cell index) and the consistency pass ((cell, point) pairs by cell).
- [x] 1.2 Unit test for `decimate`: same representatives with extra points elsewhere, the first point in input order kept. Release `ctest`; Debug not needed (no memory, threading or build change).

## 2. Measurement (rule in design.md)

- [x] 2.1 Exact cut: `44c95_s0` vs full `44_s0`, odometry only, ≤ 1 mm up to 90 s.
- [x] 2.2 Full s0–s5 with `settings/lidar-vertical.toml`, `map_quality.py --floors`, against `44_g3b2bs6_*`.
- [x] 2.3 45f: jitter, top view.

## 3. Ship

- [x] 3.1 New 44 and 44_short baselines (own commit); tracker C14; the user's visual check; master-reviewer; push with the user's go; archive.

Rejected 2026-10-16 (design.md, Result): 2.1 failed until the cut was made from −10 s, and then passed without the change too; 2.2–3.1 not run; code reverted (cc6481f).
