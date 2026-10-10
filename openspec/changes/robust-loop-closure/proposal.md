# Proposal

## Why

On recording 44 the round railings of the stairwell are drawn twice on each floor. Measured with `tools/stairwell_check.py`, which fits the railing circle per floor and per pass (the shaft is plumb, so the circles must share a centre):
- **Each floor is consistent with itself, and the floors are shifted against each other.** With the shipped settings, floor −1 lies 18–47 cm off floor 0 and floor −2 23–70 cm off floor −1 (perturbed inputs `44_s0/s1/s2`), always in −x, the walking direction on the stairs.
- **The odometry turns 2–6° on the straight stairs.** Six independent registrations between floors −1 and −2 on `44_s2` agree on 6°. It is not the accelerometer term (switching it off made the offsets worse) and not a degenerate stretch (the information ratio stays at 0.25–0.89). Its cause in the odometry is open and not part of this change.
- **The loop closure never sees the shift:**
  - it pairs segments only within 1.5 m in height;
  - the one view that could tie floor −2, floor −1's view of the shaft just before going down, is excluded by the 30 s time gap;
  - whole-segment registration between floors scores low, because most of each segment is ground the other floor never saw.

## What Changes

- **Cross-floor loop edges.** Each segment also tries earlier segments between `loop_closure_max_vertical_offset_m` and `loop_closure_cross_floor_max_vertical_m` above or below it, as one more edge per segment:
  - full 6-DoF, with the loop-edge sigmas;
  - accepted with an overlap of at least `loop_closure_cross_floor_min_fitness`, a translation of at most 1 m and a rotation of at most `loop_closure_cross_floor_max_angle_deg` (default 10°: the true corrections on 44 reach 5–7°);
  - judged by its translation only when false edges are dropped.
- **For cross-floor pairs only:**
  - a shorter time gap, `loop_closure_cross_floor_min_time_gap_s` (default 5 s; there is always a staircase in between);
  - registration on the horizontal footprint both segments share (in practice the shaft), searched within `loop_closure_cross_floor_search_m` (default 1 m).
- **A robust kernel (Cauchy) on the loop edges of the pose graph,** `loop_closure_kernel_scale`, in two stages: plain least squares first, then the kernel from that solution. It is measured on real data in this change and switched on, or removed if it doesn't help.
- **A bug fix in the pose-graph solve.** Angle residuals were not wrapped in the right-hand side, so an edge with a relative yaw near ±180° stopped every step after the first. This is bit-identical on the shipped 44.
- **Two tools:** `tools/spot_check.py` (per-pass offsets around one point) and `tools/stairwell_check.py` (drift between floors on a round stairwell), documented in `tools/README.md`.
- **Shipping:** `settings/lidar-vertical.toml` switches cross-floor edges on (6 m), with new baselines.

## Capabilities

### New Capabilities

### Modified Capabilities
- `loop-closure`:
  - cross-floor revisits become loop edges, with their own time gap, shared-footprint registration and acceptance test;
  - loop edges may be weighted by a robust kernel;
  - the false-edge drop judges cross-floor edges by translation;
  - the pose-graph solve wraps angle residuals.

## Impact

- `src/application/loop_closure.{h,cpp}`, `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h` (+ golden TOML files), `settings/lidar-vertical.toml`.
- `tests/loop_closure_test.cpp`: pose-graph cases (kernel, cross-floor edges, drop ratio) and the shared-footprint crop.
- The loop-closure stage's results change with the new settings, so new 44 and 44_short baselines. The odometry is unchanged. Cross-floor registrations (about 100 tries on 44) add time, measured and reported.
- Docs (EN/HU) on loop closure; `tools/README.md`.
- Independent of synchronized-retune's candidate 2 (branch `candidate-2-lm-step-check`).
