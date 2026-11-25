# Design

## Context

- **Pipeline** (`OdometryPipeline`): `load` (read chunks, AHRS, cut windows; window points to the `PointStore` in `<input>/cache`) → `run_odometry` (`optimize_windows`, about 20–30 min on 44) → `post_process` (loop closure, consistency) → `save`.
- **Determinism:** for one recording and the same odometry settings, the windows after `run_odometry` are bit-identical between runs. Today every loop-closure experiment repeats them anyway.
- **What the later stages read:** `windows` (poses, timestamps, gyro bias, …), the `PointStore`'s window points (written by `load`), and parts of `state` (paths; trajectory length and the like for the results JSON).
- **Measurements this change starts from** (2026-10-10, `robust-loop-closure` tasks 4.1–4.2, synchronized-retune 4.8):

  | | `44_s0` | `44_s1` | `44_s2` |
  |---|---|---|---|
  | Floor −1 vs 0 / floor −2 vs −1, shipped | 29.6 / 31.0 cm | 46.7 / 37.1 cm | 18.2 / 69.1 cm |
  | Same, X3 | 5.4 / 1.2 cm | 11.4 / 1.4 cm | 13.7 / 7.2 cm |
  | Gallery façade (`facade_check`, reference 2.15 m, spacing 1.135 m), shipped | +29 cm | −52 cm | +39 cm |
  | Same, X3 | +49 cm | −64 cm | +103 cm |

  - X3's corrections on `44_s2` move floor 0 by +0.45 m at 40 s, **+0.94 m at 60 s** (the gallery's far end) and +0.37 m at 80 s.
  - Floor 0's gallery loops (`7->15`, `7->16`) end with residuals of 6–8 cm.
  - The cross-floor edges onto gallery segments (`13=>24`, `15=>19`, `16=>27`, `16=>28`) have information ratios of 0.27–0.43.
  - The stairs: info ratio 0.25–0.89 per window. The accelerometer off makes the offsets worse. The IMU lags by −5 to −11 ms.
  - 45f has a single floor: no cross-floor candidates, a result identical to shipped.

## Goals / Non-Goals

**Goals:**
- Loop-closure experiments in minutes, with results identical to full runs.
- One evaluation suite that sees the stairwell, the gallery and the rest at once, on all inputs.
- Measured answers to the three open questions, each with its confidence, before any fix is chosen.

**Non-Goals:**
- Shipping a fix. Each fix that comes out of a finding is its own change, judged by the suite.
- Changing the shipped results.

## Decisions

1. **The snapshot is a file in the output folder:** `odometry_snapshot.bin`, binary, little-endian, with a version header.
   - It holds the window fields listed in the spec, the run state the later stages read, the window count and timestamps, and a hash of the odometry settings: every `OdometryConfig` member except the post-processing ones (loop closure, consistency, export), listed explicitly.
   - Restore checks the count, the timestamps and the hash, and stops on any difference.
   - Alternatives:
     - The `PointStore` cache as the snapshot: it holds points, not poses.
     - JSON: the doubles must round-trip bit for bit, so writing them as decimal text would need care, and the files would be large (690 windows × ~120 nodes × several matrices).
2. **The reprocessing entry point is a CLI flag:** `dMapper <input> <params.toml> <output> --from-snapshot <file>`.
   - It reuses `run_cli`'s sink and settings handling, so the result path is the same code as a normal run, which is what makes bit-identity likely.
   - Alternative: a separate executable like `tools/cut_recording`. That would duplicate the composition root.
3. **`load` runs in full on reprocessing.** The window points must be in the `PointStore`, and they come from `load` (about 1–2 min).
   - Alternative: keep the cache between runs. Two jobs on one input share `cache/`, which is fragile; rejected.
4. **The evaluation suite** is a Python script, `tools/evaluate_floors.py`:
   - it takes a set of result folders, one per input, plus the reference set (shipped `44_lc_s*`, `45f_lc`);
   - it runs the four checks;
   - it prints one table: floor −1 / −2 offsets, the gallery façade mean and spread, the worst railing pass, loops, largest correction, floor 0 start against end.
   - The parameters are fixed in the script (shaft centre −4.7 −4.9, façade 2.15 / 1.135 m, room angle 10.55°, railing spot) and documented, so every variant is judged the same way.
5. **Diagnosis 1 (the graph) works on the pose graph, not on images:**
   - log the solved segment poses (a debug setting, read-only) for shipped and X3;
   - compute each segment's correction difference;
   - with snapshots, rerun X3 with cross-floor edge groups removed by segment range (a debug list of excluded pairs): shaft pairs (segments 26–39 with a shaft view), gallery pairs (floor-0 segments 8–18), end-of-recording pairs (45–57);
   - the group whose removal restores the gallery while keeping the floors tied is the answer;
   - candidates to test after that, each measured by the suite: weighting cross-floor edges by their registration's covariance instead of the fixed loop sigma, and freeing the rotation part of cross-floor edges that span a long, weakly anchored stretch.
6. **Diagnosis 2 (the gallery):**
   - `facade_check` per 5 s time slice (out and back separately), to see when and where the façade bends;
   - the odometry's `log_window_diagnostics` along the gallery (information ratio, weak direction, result against guess);
   - the loop corrections of `7->15`, `7->16`, `6->17` (1.7–1.9 m), set against where the drift arises.
7. **Diagnosis 3 (the stairs):**
   - the heading per window over 88–100 s and 155–166 s: the LiDAR result's yaw increments against the integrated gyro (`log_window_diagnostics` plus the IMU CSV);
   - find the windows where the 2–6° appear;
   - then three single-change runs, each judged by `stairwell_check` on all inputs: IMU shifted −6 ms; `use_gyro_bias_estimation` handling during the descent (the bias prior); the rotation sigma in turns (`rotation_sigma_inflation_k`).
8. **The findings file:** `findings.md` in this change. For each question: the measurement (with the command that reproduces it), the interpretation, and the confidence (measured / likely / guessed). Nothing is called a cause without a measurement that would have come out differently otherwise.

## Risks / Trade-offs

- **[The snapshot misses a field a later stage reads, and the reprocessed result differs]** → the bit-identity scenario is the test: a full run and a reprocessed one compared with `compare_baseline.py` on 44_short and the full 44.
- **[The odometry hash misses a setting that changes the odometry]** → the hash covers every member except an explicit list of post-processing ones, so a new setting counts as odometry unless listed.
- **[The evaluation suite's fixed parameters fit only 44]** → it is a judge for 44 and 45f in this change, named and documented as such, not a general tool.
- **[Diagnoses find correlations, not causes]** → each finding states its confidence, and a cause needs an intervention (a run with one change) that moves the metric as predicted.

## Migration Plan

Nothing ships. The snapshot setting defaults to false. The `--from-snapshot` flag is additive, and `run_cli` without it is unchanged. Rollback: remove the flag and the setting.
