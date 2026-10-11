# Findings

Each finding: the measurement (with how to reproduce it), the interpretation, and the confidence (measured / likely / guessed).

## 1. Why cross-floor edges move the gallery (task 4)

### 1.1 X3 moves floor 0's gallery and compresses the storeys (measured, 2026-10-10)

Per loop-closure segment, the world position at the segment's start time, X3 minus shipped (same odometry, so the difference is the loop closure). Reproduce: `C:/Users/User/.claude/jobs/84c55dda/tmp/t41_segments.py` (output `t41_segments.txt`). The segment poses are taken from the results' world trajectories rather than from a new debug log, which gives the same values without new code.

| segments | where | `44_s0` | `44_s1` | `44_s2` |
|---|---|---|---|---|
| 0–4 | floor 0, start | ≤ 3 cm | ≤ 5 cm | ≤ 1 cm |
| 7–14 | floor 0, gallery (34–69 s) | 0.16–0.40 m, dz −0.07…−0.15 | 0.07–0.21 m, dz −0.07…−0.18 | 0.36–0.83 m, dx +0.26…+0.77 |
| 20–32 | floor −1 | 0.07–0.39 m, dz up to +0.31 | 0.14–0.53 m, dz up to +0.31 | 0.38–0.55 m, dz up to +0.27 |
| 33–39 | floor −2 | 0.32–0.76 m, **dz +0.29…+0.70** | 0.41–0.76 m, **dz +0.09…+0.65** | 0.57–1.09 m, **dz +0.22…+0.72** |
| 55–57 | floor 0, end | ≤ 6 cm | ≤ 8 cm | ≤ 6 cm |

Storey heights (difference of the sensor's level on the floors, from `tools/stairwell_check.py`'s first line):

| | floor 0 → −1 | floor −1 → −2 |
|---|---|---|
| shipped `44_s0` / `s1` / `s2` | 2.90 / 2.94 / 2.87 m | 2.88 / 2.67 / 3.15 m |
| X3 `44_s0` / `s1` / `s2` | 2.73 / 2.80 / 2.51 m | 2.48 / 2.36 / 2.74 m |

- **Measured:** X3 lifts the lower floors by 0.2–0.7 m and compresses the storeys by 0.1–0.5 m. On `44_s2` it also shifts floor 0's gallery by up to 0.77 m in x. The start and the end of the recording stay put (segment 0 is fixed; the end is tied to the start by same-floor loops).
- **Likely (mechanism, not yet tested):**
  - the shared region of a cross-floor pair is the round shaft, a vertical cylinder, which leaves the translation along its axis (the height) unconstrained; the floor slabs of the two floors lie at different heights and don't overlap;
  - the registration's z (searched ±1 m) slips, and the 6-DoF edge pulls the floors vertically;
  - the graph partly balances that through floor 0, which moves the gallery.
  - The information-ratio check of X4 missed it because it measured the whole cropped target (slabs included), not the region both clouds share.
- **Also measured:** the shipped storeys scatter by ±0.25 m between the perturbed inputs (2.67–3.15 m), so the odometry has some vertical drift of its own.
- **Test (task 4.3):** cross-floor edges keep the odometry's height (`loop_closure_cross_floor_keep_height`: the vertical part of an accepted correction, measured at the shared region's centre, is removed; a per-axis world-z sigma would need the pose-graph solve rewritten, because its edges have one translation weight in the segment's body frame). Prediction: the storeys go back to about 2.9 m, the floors stay tied horizontally (`stairwell_check`), and the gallery (`facade_check`) is no longer pulled.

## 2. The gallery drift (task 5)

### 2.1 When and where the gallery's façade is off (measured, 2026-10-10)

`facade_check` logic per 5 s and 2 s time slice, out and back separately, on shipped `44_lc_s0/s1/s2`. Done by a subagent and checked against the tool's own output on `44_s0`. Full tables: `C:/Users/User/.claude/jobs/84c55dda/tmp/t51_facade_by_time.md`.
- **The pass:** in all three, the gallery is entered at 48–50 s, the far end (v ≈ 15.5 m) is reached at 59.5 s, and the operator is back at v < 7 m at 69 s.
- **`44_s0` (+29 cm) and `44_s2` (+39 cm):**
  - the offset is the same from the first sight of the façade (44–50 s) to the last slice;
  - out and back agree within 2–4 cm;
  - the façade is straight in every slice, with a small tilt of −0.3…−0.9°;
  - the offset was acquired before the gallery came into view, or the reference of 2.15 m (the façade seen through the Wohnen windows at the start) is not the same plane as the gallery's façade. These data can't tell the two apart.
- **`44_s1` (−52 cm):** the odometry drifts during the pass.
  - Out: −36 cm at v = 7 m to −100 cm at v = 20 m, a tilt of −3…−5 cm/m.
  - Back: straight but displaced (−90…−100 cm).
  - The two passes agree only at the far end.
  - Against `44_s0` the trajectory moves sideways at about −4…−5 cm/s from 34 s on, snapping back by 1.16 m at 69–73 s.
- **Consequence for the metric:** `facade_check`'s absolute deviation is reliable as a **difference between variants** on the same input (X3 moves the gallery by +20 cm on `44_s0` and +64 cm on `44_s2` against shipped). The absolute offsets of `44_s0` and `44_s2` may partly be the reference.
  - The default band (1.5–5.5 m) cuts off `44_s1`'s back-pass façade. Its −52 cm comes from one slice; with 0.6–5.5 m the pairs span −36…−100 cm.

## 3. The heading on the stairs (task 6)

### 3.1 No turn on the stairs; slow drift where nothing known is in view (measured / likely, 2026-10-10)

Per second over 0–230 s: the result's yaw change (world trajectory with the chunk poses) against the integrated gyro (raw minus the odometry's per-window bias from the results JSON, rotated into the world with the trajectory's orientation). Reproduce: `C:/Users/User/.claude/jobs/84c55dda/tmp/t61_heading.py <input> <result> 0 230`.
- **Measured:** the two agree turn by turn (for example +53.6° against +52.1°, −82.1° against −85.9°), so the frames match.
  - The cumulative difference grows at a constant +0.30 / +0.34 / +0.36 °/s on `44_s1/s0/s2`, the same on floors and stairs: a residual gyro bias that the outputs don't let me model exactly. Adding the first-2-s stationary bias flips the sign to −0.55 °/s.
  - After removing the trend fitted on floor 0 (30–88 s), the heading stays within ±3° of the gyro everywhere:

    | | after stairs 1 (100 s) | shaft from floor −1 (139 s) | after stairs 2 (166 s) |
    |---|---|---|---|
    | `44_s0` | −0.7° | −2.8° | −0.3° |
    | `44_s1` | +0.8° | −0.4° | +2.8° |
    | `44_s2` | +0.4° | −1.2° | +0.1° |

    That is the per-second noise level (±2–4°).
- **Interpretation (likely):** the 2–6° rotation between floors, measured by the cross-floor registrations, is **not a sudden turn on the stairs**. It accumulates slowly (under about 0.1 °/s) over the stretch where nothing mapped before is in view (from 97 s until the shaft comes back into view at about 139 s). There the heading rests on the gyro and on the LiDAR's own new map. This method can't separate such slow drift from the bias trend.
- **Not the cause** (earlier, measured): the accelerometer term (off: worse), a degenerate stretch (information ratio 0.25–0.89), the IMU timing (−5…−11 ms residual).
- **Next (task 6.2):** the single-change runs should target slow heading drift in unmapped stretches: the gyro-bias prior (`gyro_bias_rw_deg_s_sqrt_s`, the bias seen by the windows) and the rotation sigma. A −6 ms IMU shift is the cheapest control.

### 3.2 The gyro-bias estimate is healthy; the floors' rotation is the expected drift of an unmapped stretch (measured / likely, 2026-10-10)

- **Measured** (`gyro_bias_per_window` in the results JSON, `44_lc_s0` / `44_lc_s2`):
  - the bias estimate is stable from 40 s on: x +0.39…+0.46, y +0.16…+0.27, z −0.38…−0.43 °/s;
  - the two inputs agree to a few hundredths;
  - it is far from the 20 °/s clamp;
  - its sigma about the vertical is 0.045–0.057 °/s (`gyro_bias_rw_deg_s_sqrt_s` 0.005).
- So the constant offset in 3.1 comes from the comparison (how the AHRS input and its stationary bias combine with the odometry's estimate), not from a bias the odometry fails to track.
- **Likely:** over the ~40 s on floor −1 where nothing mapped before is in view (97–139 s), a bias known to ±0.05 °/s gives ±2° (1σ) of heading. The 2–6° rotation between floors is 1–3σ of that.
  - This is the expected drift of open-loop odometry over an unmapped stretch, not a defect.
  - It can only be removed afterwards, by a constraint from outside the stretch: the cross-floor edges, without their vertical slip (1.1).
- **Not tested:** the single-change runs of task 6.2 (bias prior, rotation sigma, −6 ms IMU shift). They can only reduce this drift a little; the decisive fix is on the loop-closure side.

### 1.2 Keep-height (X5): the vertical slip is confirmed and fixed, but it is not what pulls the gallery (measured, 2026-10-11)

X3 + `loop_closure_cross_floor_keep_height`, reprocessed from the shipped odometry snapshots (`raw data/44_x5_s0..s2`; tables `C:/Users/User/.claude/jobs/84c55dda/tmp/eval_x5.txt`, `eval_x3.txt`):

| | `44_s0` | `44_s1` | `44_s2` |
|---|---|---|---|
| storeys X5 (shipped; X3) | 2.89 / 2.89 m (2.90 / 2.88; 2.73 / 2.48) | 2.96 / 2.63 (2.94 / 2.67; 2.80 / 2.36) | 2.83 / 3.03 (2.87 / 3.15; 2.51 / 2.74) |
| floor −1 vs 0 / floor −2 vs −1 | 5.8 / 1.8 cm | 11.3 / 0.8 cm | 14.7 / 7.1 cm |
| gallery façade X5 (shipped; X3) | +51.0 cm (+29.1; +48.5) | −64.0 (−52.0; −64.0) | +103.5 (+38.6; +102.5) |
| loops / largest correction | 42 / 1.92 m | 37 / 1.95 m | 37 / 2.49 m |

- **Measured:** the removed vertical corrections are 0.4–0.5 m on most shaft edges (up to 0.86 m), and near 0 on the others. With them removed, the storeys equal shipped and the floors stay tied horizontally.
- **Measured:** the gallery's façade is unchanged from X3 on all three inputs. **The vertical slip is not what moves the gallery.**
- **Next:** task 4.2, leaving out cross-floor edge groups to find the edges that pull the gallery.

### 1.3 Which edges pull the gallery: those that impose the shaft's yaw (measured, 2026-10-11)

Reprocessed from the snapshots, all with X5 (keep-height), cross-floor edge groups left out (`loop_closure_cross_floor_exclude_segments`, `loop_closure_cross_floor_only_near`). Gallery façade mean, cm (shipped +29.1 / −52.0 / +38.6):

| variant | `44_s0` | `44_s1` | `44_s2` |
|---|---|---|---|
| X5, all edges | +51.0 | −64.0 | +103.5 |
| X5a, without pairs touching floor-0 gallery segments 8–15 | +49.4 | −63.3 | +97.4 |
| X5b, without pairs touching end-of-recording segments 45–57 | +38.7 | −64.0 | +47.7 |
| X5d, only edges whose shared region is within 2.5 m of the shaft | +44.6 | −50.0 | +84.9 |

- **The shared-region centres** (logged per edge): on `44_s0`, 7 edges share the round shaft, 10 the area of the straight staircase, 1 a far region. None shares the gallery.
- **No single false edge:** in X5's final graph every residual is at most 9 cm, but several cross-floor edges keep **rotation residuals of 1–3°** (`19=>40` 3.2°, `31=>55` 1.9°, `19=>39` 1.8°, `31=>45` 1.5°).
- **Interpretation (confirmed by 1.4):** a round shaft fixes neither the height (its axis, 1.1) nor the **rotation about its axis**. The 6-DoF edges imposed their registration's yaw with the full loop sigma (0.3°). Where those yaws disagree with each other and with floor 0's loops, the graph turns the floors, and a turn about an axis metres away swings the gallery.
- `44_s1` doesn't respond to any variant: its gallery error is odometry drift during the pass (2.1).

### 1.4 Point edges (X6): the floors tied without imposing yaw or height; the gallery back near shipped (measured, 2026-10-11)

`loop_closure_cross_floor_point_edges`: an accepted cross-floor registration becomes a 2-D constraint. The shared region's centre, as segment j sees it and where the registration puts it in segment i, must coincide horizontally. There is no rotation or height constraint; the Jacobian is numeric. Unit test: `loop closure pose graph: a point edge ties a point horizontally, not the rotation or the height` (the gap left equals the weighted-least-squares prediction).

| X6 (shipped) | `44_s0` | `44_s1` | `44_s2` |
|---|---|---|---|
| floor −1 vs 0, stack / below | 4.5 / 8.0 cm (29.6 / 34.4) | 15.4 / 12.4 (46.7 / 46.2) | 4.1 / 7.1 (18.2 / 23.6) |
| floor −2 vs −1 (X5) | 5.8 (31.0; X5 1.8) | 6.7 (37.1; 0.8) | **24.3** (69.1; 7.1) |
| storeys | 2.90 / 2.89 m (2.90 / 2.88) | 2.95 / 2.64 (2.94 / 2.67) | 2.88 / 3.15 (2.87 / 3.15) |
| gallery façade | +36.0 (+29.1) | −59.5 (−52.0) | +46.3 (+38.6) |
| loops / largest correction | 42 / 1.72 m (23 / 1.72) | 37 / 1.85 (21 / 1.86) | 37 / 2.05 (21 / 1.92) |

- **Measured:**
  - the storeys equal shipped;
  - the floors are tied (floor −2 on `44_s2` only to 24 cm);
  - the gallery is within 7–8 cm of shipped, about the façade check's spread (6–16 cm);
  - the largest corrections equal shipped, there are no jump warnings, and every point-edge residual is ≤ 4 cm, so the graph has no tension left.
- **Remaining weakness:** floor −2 on `44_s2` is turned about 6° (1.1). Points at the shaft alone can't correct a turn, and the other shared points (around the staircase) are too few.
  - Refinement (not done): weight each cross-floor edge by its registration's information matrix (what the shared geometry actually determines), so a well-conditioned registration also ties the rotation.
