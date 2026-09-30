# Baseline (pre-change code)

Binary: `build/bin/Release/lidar_odometry_step_1.exe` built 2026-09-29 11:44, before any code of this change. A copy is kept for the regression checks.
IMU timestamps of both datasets were repaired with the anchored re-stamp.
Metrics: `python tools/eval_odometry.py <dataset> <results...>`, Z section [-0.3, 0.7] m.

| dataset | run | rotation sigma om/fi/ka [deg] | jitter median / p99 [deg] | win diff mean (>3 deg) | end spikes median (>3 deg) | final yaw LiDAR / gyro [deg] | cells 3/5/10 cm | time [s] |
|---|---|---|---|---|---|---|---|---|
| 43 | `lio_result_fixed_ts` | 0.1 / 0.1 / 0.15 | 0.343 / 4.13 | 2.06 (8) | 1.00 (9) | -140.3 / -155.2 | 39307 / 20223 / 7307 | 107 |
| 43 | `lio_result_fixed_ts_sigma002` | 0.02 | 0.039 / 0.24 | 1.18 (3) | 0.73 (0) | -128.4 / -155.2 | 15945 / 7785 / 2881 | 108 |
| 42 | `baseline_sigma_old` | 0.1 / 0.1 / 0.15 | 0.279 / 2.11 | 2.61 (20) | 0.85 (3) | -139.0 / -248.1 | 7297 / 3395 / 1200 | 111 |
| 42 | `baseline_sigma002` | 0.02 | 0.034 / 0.19 | 2.45 (15) | 0.93 (0) | -140.9 / -248.1 | 6872 / 3207 / 1143 | 113 |

Settings: `settings/lidar-vertical.toml` as of 2026-09-29, before the new keys were added. The sigma 0.02 runs change only `lidar_odometry_motion_model_{om,fi,ka}_1_sigma_deg`.

Observations:
- Tight rotation sigma removes jitter and end-of-window spikes on both datasets.
- The steady LiDAR-vs-gyro gap is about -1.4 deg/s on 43 (27 deg over 19 s) and about -6.3 deg/s on 42 (107 deg over 17 s). Gyro z reads more negative than the LiDAR, so the expected z-bias estimate is negative on both.
- On 42, the per-window difference stays high (15 windows > 3 deg) even with tight sigma. At about 6 deg/s × 0.36 s ≈ 2.3 deg per window, the uncorrected bias alone reaches that level. This is the main case for bias estimation.
- Gyro noise estimated from dataset 43 sample differences (MAD): x 0.54, y 1.22, z 1.53 deg/s per sample at 893 Hz, giving ARW of 0.018 / 0.041 / 0.051 deg/sqrt(s).

## Results after the change

All runs use the new binary: single-threaded, NDT normal fix, tail re-propagation `m = 30` unless noted.

### Reproducibility (task 2.3)
- Two single-threaded runs of the new binary with all new switches off (`det_off_A`, `det_off_B`, dataset 43): `trajectory_lio_0.csv` and `scan_lio_0.laz` byte-identical.
- Pre-change binary run twice (multithreaded): trajectories differ by 42 mm median, up to 159 mm and 12°. Single-threaded, it still differs from window 1 on (uninitialised `NDT::Bucket::normal_vector`, design D7).
- All-off new binary vs pre-change spread (43): jitter 0.038 vs 0.038–0.039°, windows > 3° 2 vs 3–4, 3 cm cells 14,896 vs 15,223–15,945. Within the spread.

### Tuning sweep (task 3.2): q = bias random walk [°/s/√s], k = sigma inflation

| run | k | q | window | bias est. | 43 jitter med/p99 | 43 win > 3° | 43 cells 3 cm | 42 jitter med/p99 | 42 win > 3° | 42 cells 3 cm |
|---|---|---|---|---|---|---|---|---|---|---|
| baseline sigma 0.02 (pre-change) | – | – | 360 | – | 0.039 / 0.24 | 3 | 15945 | 0.034 / 0.19 | 15 | 6872 |
| v3 | 12 | 0.05 | 360 | on | 0.035 / 0.23 | 5 | 15564 | 0.029 / 0.21 | 14 | 6829 |
| A | 12 | 0.5 | 360 | on | 0.035 / 0.25 | 4 | 16001 | 0.029 / 0.20 | 8 | 6803 |
| B | 4 | 0.5 | 360 | on | 0.021 / 0.09 | 3 | 15173 | 0.016 / 0.08 | 2 | 6689 |
| C | 2 | 0.5 | 360 | on | 0.017 / 0.07 | 3 | 15799 | 0.012 / 0.06 | 1 | 7002 |
| D | 1 | 0.5 | 360 | on | 0.012 / 0.06 | 2 | 17060 | 0.010 / 0.05 | 0 | 7441 |
| E | 4 | 0.5 | 720 | on | 0.022 / 0.09 | 3* | 14683 | 0.016 / 0.08 | 9* | 6626 |
| **F = default** | 4 | 0.5 | 360 | **off** | 0.022 / 0.09 | 2 | **14823** | 0.016 / 0.08 | 16** | **6685** |

\* counted per 720-node window. \** measured against the uncorrected gyro, so the bias remains in this count.
No run had NaN, a bias clamp or a "big jump". End-of-window spikes > 3°: 0 in every run.
Chosen defaults: k = 4, q = 0.5, bias estimation off (F). F meets the map-quality scenario on both datasets: jitter ≤ 0.03°, 0 spikes, cells below the sigma-0.02 baseline.

### Bias estimate (task 4.5), z-axis mean over windows after the first 5 s [°/s]

| run | 42 | 43 | 43 window-to-window change (median) |
|---|---|---|---|
| v3 (q = 0.05) | -2.16 | -0.52 | 0.09 |
| A | -3.45 | -0.64 | 0.41 |
| B (defaults + bias on) | -3.85 | -0.66 | 1.47 |
| C | -4.13 | -0.65 | 1.90 |
| D | -4.31 | -0.68 | 1.93 |
| E | -4.09 | -0.67 | 2.08 |
| whole-run LiDAR-vs-gyro offset | about -6.3 | about -1.4 | |

### Final regression (tasks 5.1, 6.1, 6.2), final binary

| check | result |
|---|---|
| all switches off (`final_off`) vs `det_off_A` (built before tail, SO(3) and damping changes) | trajectory + LAZ byte-identical |
| default run 43, twice | byte-identical |
| default run 43 vs `tune_F` | byte-identical |
| default run 42 vs `tune_F` | byte-identical |

Top views of the Z section [-0.3, 0.7] m, left to right: old sigma 0.1/0.1/0.15, constant sigma 0.02, new default. Red is before 10.5 s, cyan after.
- `img/topview_43_oldsigma_sigma002_default.png`
- `img/topview_42_oldsigma_sigma002_default.png`

### Static recording `raw data/43/static` (tasks 7.1, 7.2)

17.2 s at rest, 15,160 samples, 882 Hz. Timestamps strictly increasing, but within each batch the samples are only about 0.02 ms apart (see design, Open Questions).

| axis | gyro mean [°/s] | 1 s block means std | sample noise | ARW [°/√s] |
|---|---|---|---|---|
| x | +0.033 | 0.059 | 0.84 | 0.028 |
| y | -0.008 | 0.049 | 1.24 | 0.042 |
| z | -0.612 | 0.081 | 1.70 | 0.057 |

Bias estimate, mean after 5 s (z) and window-to-window change:

| run | static | 43 | 42 |
|---|---|---|---|
| q = 0.5 (B), as recorded | -0.35 / 1.53 | -0.66 / 1.47 | -3.85 / 0.54 |
| q = 0.5, evenly re-stamped IMU | -0.36 / 1.75 | – | – |
| q = 0.05 (G) | -0.32 / 0.16 | -0.60 / 0.15 | -2.81 / 0.24 |
| **final defaults** (q = 0.05, bias on, ARW above) | **-0.33 / 0.15** (x +0.03, y -0.03) | **-0.56 / 0.13** | **-2.70 / 0.21** |

Final defaults vs the same settings with bias off (3 cm cells): 43 15191 vs 15159 (+0.2 %), 42 6628 vs 6640 (-0.2 %). Jitter 0.022 / 0.017°, 0 end spikes, no clamp, no big jump. Two default runs on 43 byte-identical.
Spec check: static bias within ±0.5 °/s on all axes (z off by 0.28). 43 within ±0.5 °/s of the static bias (off by 0.05). 42 negative. Stability 0.15 ≤ 0.3. Map ≤ +5 %. Map at least as sharp as the sigma-0.02 baseline (15191 ≤ 15945, 6628 ≤ 6872).

B vs F (bias on vs off, otherwise default), 3 cm cells: 42 6689 vs 6685 (+0.1 %), 43 15173 vs 14823 (+2.4 %). Both within the 5 % limit. Sign correct, magnitude ≥ 1/3 of the offset on both. Bias vs static recording: pending.
