# Design

## Context

`repropagate_window_tail(window, tail_nodes m, velocity_nodes s, bias_update)` keeps the nodes up to a = n-1-m,
estimates `v = (p[a] - p[a-s]) / (t[a] - t[a-s])` and sets `p[k] = p[k-1] + v dt` for the tail. For a quadratic motion
that `v` is the velocity at the span's middle, a·(span time)/2 below the velocity at node a, so the tail lags by about
a·span·tail/2 terms; the lag grows with the tail and the span (-0.75 mm at tail 15 / span 10, -6 mm at tail 30 / span 20
at 2 m/s², measured). The rotation of the tail comes from the IMU orientation relative to node a and is not part of this
change (C32 is the rotation half).

## Goals / Non-Goals

**Goals:**
- The tail's last node and the next first pose equal the truth under constant acceleration (K4, 0.05 mm).
- Unchanged at constant velocity and at rest; unchanged fallback where the window is short.

**Non-Goals:**
- No change to the rotation tail, the tail length, the first-pose design, or what the next window does with the pose.

## Decisions

1. **Estimator (open):** (a) a quadratic fit of position through nodes a-2s … a (needs a ≥ 2s; the shipped 20-sample span
   needs 40 kept nodes, to be checked against the shortest windows); (b) velocity at a by correcting the finite difference
   with the acceleration, the acceleration from the window's accelerometer samples (world frame, offset removed) or from
   the solved end velocity `window.velocity.end` and the velocity at the start of the tail; (c) integrate the tail from the
   accelerometer alone. Recommendation: (b) with the solved end velocity as the anchor, because the window's solve already
   estimates the velocity at the end node with the accelerometer and the LiDAR together. To be checked in the first
   replay: whether `window.velocity.end` refers to the last node or the last kept node. A fit on noisy optimized positions
   (a) amplifies noise in the second difference; measure it before choosing.
2. **Fallback:** the constant-velocity line when the needed samples or velocity are missing (non-IMU windows, short
   windows), so the non-accelerometer path stays identical.
3. **Order of work:** red marker first (exists); implementation; the order and replays as in `start-velocity-over-gap`
   but measured on top of it, with the tail effect isolated by a switch-free comparison of the two commits.

## Risks / Trade-offs

- The tail moves the map points of the last nodes, so map sharpness can change either way; measure with
  `tools/map_quality.py` on 44's s0/s1/s2 and look at the maps before any baseline change.
- The correction is a few millimetres per window; it matters through its accumulation and through the next window's fixed
  first pose (the audit's main design flaw, step 6), which this does not remove.
- Using the solved end velocity ties the tail to the solver's estimate; where that estimate is wrong (the turn events),
  the tail inherits it. The effect should be read at the windows 369 to 378 of Livox 4.
