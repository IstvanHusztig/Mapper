# Results (2026-10-18, recording 44, Release)

Term on = `settings/l2-vertical.toml` + `consistency_gyro_rotation_term = true` (scale 3, gate 3°). Off = the
shipped results (44_baseline, 44_s1_diag, 44_s2_diag; s0 = the baseline). On: 44_g4_s0_on (from the baseline's
snapshot), 44_g4_s1_on, 44_g4_s2_on. Every run: all windows 100/100 iterations. Script `group4.sh` (scratchpad).

## 4.1 Gate

- Gated windows: 15 / 14 / 24 of 690 (s0 / s1 / s2).
- The gyro-fault windows 629–639 (C38) differ from the odometry by 6.7–12.0° per window: above the gate.
- Rotation over 255.8–259.9 s: 5.43° off, 5.42° on — the term does not drag the fault stretch.
- In hold-out mode the gate removes far more windows on s0 and s2 (250 and 220 of 690; s1 19): with half the points
  the odometry departs from the IMU more often. The hold-out numbers below therefore show the term on fewer windows.

## 4.3 Full density, map_quality.py (s0 / s1 / s2; spread = max − min over the three)

| measure | off | on | spread off → on |
|---|---|---|---|
| façade out [°] | −3.63 / −2.81 / −4.49 | −3.65 / −2.76 / −4.44 | 1.68 → 1.68 |
| façade back [°] | −3.63 / −2.86 / −4.48 | −3.64 / −2.81 / −4.46 | 1.61 → 1.65 |
| revisit \|yaw\| med [°] | 0.42 / 0.52 / 0.67 | 0.42 / 0.66 / 0.53 | 0.25 → 0.24 |
| revisit worst [°] | 1.64 / 3.03 / 2.53 | 1.64 / 1.87 / 2.56 | 1.39 → 0.92 |
| revisit offset med [m] | 0.15 / 0.05 / 0.05 | 0.13 / 0.05 / 0.05 | 0.10 → 0.08 |
| jitter p50 [°] | 0.91 / 0.91 / 0.93 | 0.49 / 0.49 / 0.50 | 0.02 → 0.01 |
| jitter p99 [°] | 4.44 / 4.23 / 4.57 | 1.40 / 1.45 / 1.51 | 0.34 → 0.11 |
| railing NN [cm] | 3.5 / 5.2 / 6.5 | 3.3 / 3.7 / 7.4 | 3.0 → 4.1 |
| stair max [cm] | 9.7 / 12.3 / 16.4 | 9.8 / 12.3 / 16.3 | 6.7 → 6.5 |

`wall_precision.py` (H = 0): thickness 2.5 / 2.0 / 2.2 → 2.6 / 2.1 / 2.2 cm; pair_diff 3.0 / 2.2 / 2.9 → 2.8 / 2.1 /
3.0 cm; pair_excess 2.5 / 1.7 / 2.3 → 2.3 / 1.6 / 2.3 cm. Rotation per IMU sample p50 0.167 / 0.166 / 0.170 →
0.058 / 0.058 / 0.059° (gyro 0.024°, odometry only 0.041°).

## 4.2 Held-out points (holdout_block_s 0.01; s0 / s1 / s2; against consistency off = odometry + loop closure)

| | all-time thickness, paired [mm] | slice [mm] |
|---|---|---|
| consistency without the term | +1.32 / +1.66 / +0.95, mean +1.31 (interval +0.89 .. +1.72) | +0.03 |
| consistency with the term | −1.19 / −1.50 / −1.49, mean −1.40 (interval −1.57 .. −1.12) | +0.01 |

## Cost

Consistency with the term about +10–20 % (from-snapshot runs, including loop closure and saving: 243 → 278 s,
370 → 408 s, 173 → 215 s).
