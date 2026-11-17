# tools

## eval_odometry.py

Quality metrics for dMapper results. Pure Python 3, with no extra packages.

### 1. Export the point cloud to ASCII (optional, needed for the cell metrics and PNG)

With the CloudCompare command line, from Git Bash:

```
"/c/Program Files/CloudCompare/CloudCompare.exe" -SILENT -AUTO_SAVE OFF \
  -O "<result_folder>/scan_lio_0.laz" \
  -C_EXPORT_FMT ASC -SEP SPACE -PREC 4 \
  -SAVE_CLOUDS FILE "\"<result_folder>/scan_lio_0.asc\""
```

The escaped inner quotes are required: `-SAVE_CLOUDS FILE` splits its argument on spaces, so a path such as `raw data/43/...` would otherwise be read as two file names and nothing is saved. The export has the columns `x y z intensity gps_time`.

### 2. Run the metrics

```
python tools/eval_odometry.py "<dataset_folder>" "<result_folder_A>" "<result_folder_B>" --png compare.png
```

- `<dataset_folder>` is the input folder of the run (the one containing `imu*.csv`).
- Pass any number of result folders. The PNG shows their top views side by side: red is before `--tsplit` seconds (default 10.5), cyan is after.
- `--zmin/--zmax` select the Z section used for the cell metrics (default -0.3..0.7 m).
- `--window` sets the nodes per optimization window (`threshold_nr_poses`, default 360).
- `--json` prints the metrics as JSON.

| metric | meaning | better |
|---|---|---|
| jitter | median / p99 yaw change between consecutive trajectory nodes [deg] | lower |
| win diff | mean per-window LiDAR-vs-gyro yaw difference [deg], and windows > 3 deg. The gyro is bias-corrected when the results JSON holds `gyro_bias_per_window` | lower |
| end spikes | yaw of each window's last node vs the median of nodes 30..60 before it [deg], and count > 3 deg | lower |
| final yaw | total LiDAR yaw vs total (bias-corrected) gyro yaw [deg] | closer |
| cells 3/5/10cm | occupied top-view cells of the Z section, for the same scene | lower = sharper |

The IMU timestamps must be strictly increasing. Recordings from dLidar builds before the timestamp fix must be repaired first.

## convergence_summary.py

How many Levenberg–Marquardt iterations the windows of a run needed. Pure Python 3. Run dMapper with `log_iteration_convergence = true` in `[lidar_odometry_debug_info]` (a copy of the settings file is enough), and keep the console output:

```
dMapper "<input>" "<params_with_the_setting.toml>" "<output>" > run.log
python tools/convergence_summary.py run.log [--top 10] [--bin 5]
```

It prints:
- the median, p90 and max of two counts per window: **settled after**, the iteration after which the poses stay within 0.1 mm / 0.001° of the window's result, and **delta < 1e-2 after**, where `convergence_delta_threshold = 1e-2` (fast mode) would stop the window;
- a histogram of each;
- the windows with the largest pose error at that fast-mode stop.

The run's results are the same with the setting on or off; only the parameter file in the output records it.

## fix_imu_timestamps.py

Rebuilds a steady sample clock for an `imu*.csv` whose timestamps are arrival times (bursts of samples ~0.02 ms apart followed by gaps, or clock corrections applied as jumps). Pure Python 3.

```
python tools/fix_imu_timestamps.py "<input_folder>/imu.csv" -o "<new_folder>/imu.csv"
```

- Every sample is acquired no later than its own arrival and no later than any later arrival minus the periods in between. The new time of each sample is the tightest such bound over the next `--window` seconds (default 0.5). Dropped samples only make the bound looser, so they need no special handling.
- The sample period is fitted from the lower edge of the arrival times over the whole file.
- `timestamp` gets the new time, `timestampUnix` is shifted by the same amount; all other columns are copied verbatim. A partial last line (recording cut off) is dropped. The original file is never modified.
- It prints the step statistics before and after and how much later the samples arrived than their new time.
- Write the output into a **new** input folder together with copies of the LAZ files: every `.csv` in an input folder is read as IMU data.

## imu_time_offset.py

Measures how far the IMU timestamps of a recording are off against its LiDAR timestamps, and how well the accelerometer agrees with a result's trajectory. Pure Python 3.

```
python tools/imu_time_offset.py "<raw data>/44" "<raw data>/44_baseline" [--shift-ms S] [--from T0 --to T1] [--no-prediction]
```

- The trajectory files are chunk-relative; the tool places them with `session_poses.mrp`. The accelerometer is rotated into the world with the trajectory's rotations (one node per IMU sample).
- **Prediction:** from p(t0), v(t0), the error of predicting the trajectory h = 0.1 to 5 s ahead, by constant velocity and by integrating the accelerometer.
- **Coherence:** per frequency band and axis, the coherence and gain between the trajectory's acceleration and the accelerometer. A good trajectory follows the walk at 0.3 to 2 Hz (vertical coherence about 0.9).
- **Delay:** from the cross-spectrum phase slope at 0.5 to 4 Hz. Negative means the IMU stamps are late against the LiDAR's.
- `--shift-ms` tests an offset (the prediction error is smallest at the true one); `--from/--to` restricts the analysis to a time range, so running it per segment shows a drift.
- **Found with it on 2026-10-09:** the IMU of recording 44 was 79 ms behind the LiDAR at the start and 52 ms at the end. A per-window yaw check with `eval_odometry.py` against shifted gyro data agreed (minimum at 80 to 100 ms). The recording was corrected with `restamp_imu.py`.

## restamp_imu.py

Corrects the IMU timestamps of a recording by a measured lag d(t) = A + B·t against the LiDAR (from `imu_time_offset.py`). Pure Python 3.

```
python tools/restamp_imu.py "<original>/imu0001.csv" "<raw data>/44/imu0001.csv" --offset-ms 79.4 --drift-ms-per-s -0.0983
```

- `timestamp` and `timestampUnix` [ns] are reduced by d(t), t in seconds from the first sample. Samples that would fall before zero are dropped; the timestamps must keep increasing. All other columns are copied unchanged.
- The output must be a new file. Keep the original **outside** `raw data`, so a wrongly timed file can't be processed by accident. The originals of all unsynchronized recordings are in `dLidar/unsynced_archive` (35, 42, 43 and 44's original IMU file; see its README).
- `fix_imu_timestamps.py` is a different repair: it removes bursts by rebuilding a steady clock from the arrival times. It doesn't align the IMU with the LiDAR, and its fitted period can drift.

## merge_chunks.sh

Merges the chunk clouds of a result folder into one LAZ in world coordinates, using the CloudCompare command line. Each `scan_lio_N.laz` is stored relative to its chunk's first pose; `session_poses.mrp` holds those poses.

```
tools/merge_chunks.sh "<result_folder>" ["<output.laz>"]
```

The default output is `<result_folder>/scan_lio_merged.laz`. Set `CLOUDCOMPARE` if CloudCompare is not installed in `C:\Program Files\CloudCompare`.

## trajectory_seams.py

Shows where two results of the same recording disagree in position, which `eval_odometry.py` (yaw only) can't: a stretch of the path that lags or leaps by decimetres draws a ghost wall where the path passes the same place again. Pure Python.

```
python tools/trajectory_seams.py "<raw data>/44/lio_result_2" "<raw data>/44/lio_result_3"
```

- The first folder is the reference; each further folder is compared pose by pose (same recording and settings, same number of poses).
- Per result it prints the route length, the largest position difference with its time and the reference position, the first time the difference exceeds each threshold (`--thresholds`, default 5, 20 and 50 cm), and the longest stretch above the first threshold.
- Used for `odometry-stall-fix`: a `/fp:fast` and a `/fp:precise` build of the same code should stay within a few centimetres over the whole 44.
- Positions are world positions: each chunk's rows are placed with its pose from `session_poses.mrp`. Before 2026-10-10 the tool compared the chunk-relative rows, so it missed what loop closure moves (the chunk poses) and mixed frames in the route length. Earlier comparisons of loop-closure variants with it understate the differences.

## spot_check.py

Checks one place of a result for a structure drawn twice (a doubled railing or wall) and tells which passes draw which copy. `eval_odometry.py` and revisit checks that fit walls and floors don't see thin structures. Pure Python, plus CloudCompare for the export.

```
python tools/spot_check.py "<raw data>/44_lc_s0" -3.0121 -5.4233 -0.1179 0.8
```

- The arguments are a point picked in `scan_lio_merged.laz` (CloudCompare point picking) and a radius (default 0.8 m).
- Each chunk LAZ is exported once to `scan_lio_<n>.asc` in the result folder and placed with the chunk poses of `session_poses.mrp`, so the coordinates are those of the merged cloud.
- The points within the radius are split into passes at time gaps over 2 s. Each pass is compared with the largest one: median distance to its nearest neighbour, the share farther than 5 cm, and the median offset vector.
- Passes within a few centimetres draw the same surface (the L2's noise); a pass 10+ cm off draws a second copy, and its time range points to the stretch of the trajectory that is off.
- Found the doubled stairwell railings of 44 (`robust-loop-closure`): seen from floor 0 every pass agrees within 2–5 cm, seen from floor −1 at 139–149 s the railing is 24 cm off.

## stairwell_check.py

Measures the drift between floors on a round stairwell. The shaft is plumb, so its railing circle has the same centre on every floor; an offset between the circles is drift of the trajectory between the floors (the doubled railings of 44). Pure Python, plus CloudCompare for the export (as `spot_check.py`).

```
python tools/stairwell_check.py "<raw data>/44_lc_s0" -4.7 -4.9
```

- The arguments are a rough guess of the shaft's centre in the coordinates of `scan_lio_merged.laz` (within half a metre is enough). Options: `--radius` (points within this of the guess, default 3.5 m), `--ring` (starting railing radius, 1.5 m), `--band` (railing band relative to the sensor's level on a floor, default −0.55..−0.05 m), `--min-storey` (1.8 m).
- Floors are the heights where the trajectory dwells; each point belongs to the pass it was seen from (a stretch with the sensor on one floor). In each floor's railing band, a circle is fitted per pass.
- Per band it prints every pass's centre, radius, median residual and arc, and the offset from the floor's own first pass. The summary gives, for neighbouring floors, the stack (each floor's railing seen from its own floor) and the upper railing seen from the floor below. A few cm is the noise; the railing of 44 fits with 1–3 cm residuals.
- On 44 with the shipped settings (input `44_s0`): floor −1 is 30 cm off floor 0 (stack), floor −2 31 cm off floor −1 (seen from below; floor −2 has no railing of its own).

## facade_check.py

Checks that a long façade is straight. A straight façade with a parapet at a known distance from the building plan must appear at one position along its whole length. Pure Python, plus CloudCompare for the export (as `spot_check.py`).

```
python tools/facade_check.py "<raw data>/44_lc_s0" --facade 2.15 --spacing 1.135 --window 35 95
```

- The points seen in the time window (the pass along the gallery) are turned into the room frame (`--angle`, the direction of the building's walls in the map, 10.55° on 44).
- In each 1 m slice along the gallery, the façade and the parapet are found as the pair of histogram peaks across it whose spacing matches the plan (`--spacing`, ± `--tolerance`).
- The façade's position in each slice is compared with `--facade`, here the façade's outer face seen through the Wohnen windows at the start.
- On 44 (the gallery in front of flat 56, building plan Haus 47 LG): shipped +29 / −52 / +39 cm mean deviation on `44_s0/s1/s2`. Few slices find the pair (1–7), so the numbers are rough.

## floor_shape_check.py

Checks the shape of the floors of a multi-storey result: the tilt of the slabs, the lift shaft's walls on each floor, and a 2-D ICP between floors. Pure Python, plus CloudCompare for the export (as `spot_check.py`). The defaults are the floors of recording 44.

```
python tools/floor_shape_check.py slabs "<raw data>/44_x6_s2" --levels -1.1 -4.0 -6.8
python tools/floor_shape_check.py lift "<raw data>/44_x6_s2"
python tools/floor_shape_check.py floors-icp "<raw data>/44_x6_s2" --box -7.35 -2.15 -1.2 4.0 --band 0.3 2.3
```

- Floors are `--floor=SLAB_Z:T0-T1[,T0-T1]` (slab height and the pass times in seconds on it, repeated; write the `=` for a negative height). Default `-1.1:1-92,220-280`, `-4.0:99-158,201-212`, `-6.8:165-194`. Each floor's points come only from its own pass times.
- `slabs` fits a plane to the points near each slab height (trimmed least squares) and prints the tilt about y and x. A level slab gives 0; on 44 floor −1 tilts about +1 degree.
- `lift` fits a line to each wall box (`--wall name:x0,x1,y0,y1`, repeated; default the lift shaft of 44: `bottom`, `right`, `long`) in the wall band above each slab (`--band`, default 0.3..2.3 m), and prints the direction and the offset against the first floor. `long@shaft` is the `long` wall over the seconds in which `bottom` was seen.
- `floors-icp` moves each floor's wall points (`--band`, default 0.8..1.8 m) onto its neighbour's over the 0.5 m cells both cover, from starts at −6/0/+6 degrees, and prints rotation, shift, rms and matched share. `--box x0 x1 y0 y1` or `--radius R --centre x y` (from `--radius-min`) restricts it to a region. A rotation that every start finds is the floors turned against each other; starts that disagree mean no reliable fit.
- Each run takes a few minutes (it reads every point).
- On `44_x6_s2`: slab tilt about y −0.11 / +1.03 / +1.06 degrees; the bottom wall of floor −2 is 12.9 cm and 2.19 degrees off floor 0; whole-floor ICP from +0: floor −1 onto 0 +0.02, −2 onto −1 +0.39, −2 onto 0 +0.65 degrees.

## evaluate_floors.py

The evaluation suite of `floor-alignment-diagnosis`: one table over the perturbed 44 inputs (and 45f), against reference results. It runs `stairwell_check.py`, `facade_check.py` and `spot_check.py` one after the other, and reads the loop summary from the run logs.

```
python tools/evaluate_floors.py --results 44_s0=<folder> 44_s1=<folder> 44_s2=<folder> [45f=<folder>]     --reference 44_s0=<raw data>/44_lc_s0 44_s1=<raw data>/44_lc_s1 44_s2=<raw data>/44_lc_s2 --logs <folder with <result>.log>
```

- **Columns:**
  - floor −1 against 0 (stack and seen from below), floor −2 against −1 (seen from below), floor 0's start against its end;
  - the **storey heights** (the sensor's levels on neighbouring floors; the loop closure must not squeeze or stretch them);
  - the gallery façade (mean and spread), the worst railing pass at the railing spot;
  - loops, largest correction, cross-floor edges accepted, edges dropped, jump warnings.
- **Fixed parameters** (constants at the top), specific to 44: shaft centre (−4.7, −4.9); façade 2.15 m, spacing 1.135 m, window 35–95 s; railing spot (−3.0121, −5.4233, −0.1179), radius 0.8 m. 45f gets only the log columns.
- About 2–3 minutes per result folder.

## map_quality.py

Judges results of recording 44 on the three map-quality measures of equal weight (change `map-quality-harness`): global geometry, consistency, sharpness. Needs numpy (and pymupdf for `--overlay`), and CloudCompare for the one-off ASCII export of the chunk LAZ files (as `spot_check.py`).

```
python tools/map_quality.py [--floors] [--overlay] [label=]result_folder ...
```

- One row per result, plus a `spread` row (max − min over the results): give s0, s1 and s2 of the same code to see how fragile it is.
- **Global geometry:** `facade out` / `facade back`: the gallery façade's line angle along the gallery (way out / way back) minus its angle seen through the windows at the start [deg]. The building plan makes them parallel, so 0 is right.
- **Consistency:** `revisit |yaw| med` / `worst` / `offset med`: heading [deg] and offset [m] between two passes of the same place (2D registration of 2.5 s slices, 11 fixed pairs on 44, pairs with quality ≥ 1.08 counted in `pairs`). 0 means no doubled wall.
- **Sharpness:** `jitter p50` / `p99`: rotation over 10 trajectory nodes (11 ms) [deg]; with `--floors` also the railing NN and the largest stairwell offset of `evaluate_floors.py` [cm] (labels must then be `44_s0` / `44_s1` / `44_s2`).
- `--overlay` writes `<result>/map_quality_cache/plan_overlay.png`: the first 100 s in red over the building plan (`haus47_LG_plan.pdf`), anchored at the stairwell centre.
- A measure that recording 44 always allows but that cannot be computed (the façade or the stairwell not found) is shown as `FAIL`: the map is broken there, it is not missing data. A revisit heading at the ±8° search limit is shown as `>=8.00` (saturated: the true error is at least that). `pairs` is registered / all pairs; the noise floor registers 4–6 of 11 on s0–s2, so fewer means more places that can't be matched.
- The placed points are cached in `<result>/map_quality_cache/points.npy`. The constants of recording 44 (room angle, façade bands, revisit pairs, plan anchor) are at the top of `map_quality_lib.py`.
- Reference (2026-10-13): shipped final `44_wd_s0` façade −0.40 / −0.73°, revisit 1.33° / 2.29° / 0.11 m (4 pairs), jitter 0.94 / 4.49°; odometry only `44_odo_h0_s0` −3.87 / −4.19°, 0.18° / 1.52° / 1.00 m (10 pairs), 0.40 / 1.29°.

## plan_fit.py

Fits the top floor of a result of recording 44 onto the building plan (`haus47_LG_plan.pdf`, scale 1:50 fixed) and draws the map over it. Needs numpy, scipy, laspy (with lazrs) and pymupdf; reads `scan_lio_merged.laz` directly (no CloudCompare export).

```
python tools/plan_fit.py <result_folder> [out_prefix] [--start ROT_DEG X Y] [--z-band LO HI] [--no-parts]
```

- A recording with another world frame than 44 (another start room or sensor) needs `--start`: the map → plan rotation [deg] and the stairwell centre in its map [m], read off the map; the search around them is the same. `--z-band` is the world z of the storey's walls (default −0.5 1.5, the top floor of 44; about 0.6 to 2.6 m above the floor), and `--no-parts` leaves out the start room and gallery fits, whose times are 44's. Livox 3 (start storey, floor at −0.70 m): `--start 195.84 0.96 -6.46 --z-band -0.1 1.65 --no-parts`, then `plan_section.py --height 1.3`.

- The walls (world z −0.5…1.5 m, 5 cm cells with ≥ 8 points) are placed by the 2D rotation and shift that bring them closest to the plan's drawn lines; the start is `map_quality_lib`'s plan anchor.
- Prints the whole-floor fit (rotation, mean distance to the plan, share within 10 / 25 cm) and the start room (0–15 s) and the gallery (49–71 s) fitted on their own, with their rotation against the whole fit.
- Writes `<out_prefix>_whole_fit.png`, `_start_fit.png` (placed by the start room's own fit), `_full_sheet.png`, `_zoom_gallery.png`, `_zoom_core.png`; default prefix `<result>/map_quality_cache/plan_fit`. Red the whole floor, green the start room, blue the gallery.
- Reference (2026-10-18): `44_baseline` (after D4, D3, D1, D5) whole floor 256.34°, 9.9 cm, 71 % / 89 %; start room +0.93°, gallery +0.47° against it (gallery − start −0.46°). The previous `44_baseline_2026-10-15`: 256.78°, 6.9 cm, 80 % / 94 %; start room +0.55°, gallery +0.50° (−0.04°). The first version of the tool (cost in whole pixels, turned about the map origin) gave that map's start room −2.18°, a wrong local fit.

## plan_section.py

A horizontal cross-section of a result of 44 at world height H, drawn as lines over the building plan: the points within ±`--half-thickness` (default 5 cm) of H are counted in 2 cm cells, smoothed, and a line is drawn along the ridge of highest density across each wall. The lines are placed with the whole-floor fit that `plan_fit.py` saves to `<result>/map_quality_cache/plan_fit.json` (run it first).

```
python tools/plan_section.py <result_folder> [--height 0.0] [--threshold 0.03] [--half-thickness 0.05]
```

- `--height` is the world z of the section [m]; the top floor's slab of 44 is at about −1.1 m, so the default 0 cuts it about 1.1 m above the floor.
- `--threshold` is the lowest density drawn, as a share of the 99th percentile of the density (default 0.03; at 0.15 the walls seen only from far away disappear).
- One 1-pixel line per wall (smoothing `--sigma` 3 cells merges a wall's faces); pieces spanning less than `--min-length` (0.5 m) are dropped.
- Writes `<result>/map_quality_cache/plan_section_<H>_t<T>.png` (top floor) and `_sheet.png` (whole plan sheet).
- `--threshold` takes up to 5 values (e.g. `--threshold 0.01 0.03 0.1 0.3`): one image per value, plus `plan_section_<H>_sweep.png` (and `_sheet`) with each line in the colour of the highest threshold it survives: red (only the lowest: a faint wall, seen briefly or from far away), orange, green, blue, purple.

## wall_precision.py

How precise each wall of a result is at height H, from the map and the trajectory alone (no plan needed; caves). The section line (`section_lib.py`, shared with `plan_section.py`) is cut into ~0.5 m pieces, and for each piece the slice points near its line are measured.

```
python tools/wall_precision.py <result_folder> [--height 0.0] [--threshold 0.03] [--plan]
```

- Columns of `<result>/map_quality_cache/wall_precision_<H>.csv`, one row per piece:
  - `sides`, `faces_m`: the side of the line the sensor was on. A wall seen from both sides shows two faces (`faces_m` apart), which is not an error, so each side is centred on its own median and visits are compared only within a side.
  - `thickness_m`: the wall's spread across the line (1.4826 MAD); a precise wall is about the sensor noise.
  - `visits`, `visit_times_s`, `visit_offsets_m`, `visit_sides`: per side, the point times split at gaps over `--gap` (2 s), each visit's median offset.
  - `pair_diff_m`: the median, over the pairs of visits from the same side, of the difference of their offsets: how far two visits typically disagree. Use it rather than `visit_spread_m` (largest minus smallest offset per side), which grows with the number of visits.
  - `half_split_m`: the longest visit's two halves against each other, the noise floor of the visit offsets.
  - `pair_excess_m`: the median over the pairs of their difference minus the noise their two medians alone would give (1.25 √(s₁²/n₁ + s₂²/n₂)); what is left is the pose (or scene) part.
  - `incidence_deg` uses the 3D ray; `length_m` is the line's extent (a zigzag or diagonal ridge has a cell count that doesn't match its length).
  - The summary also prints the visit pairs of all pieces by the time between the two visits (with the pairs' own noise and the excess over it): drift between visits shows as a difference that grows with that time; the correlation of a pair's difference with how far apart along the piece its two visits lie (the piece's shape); and `plan_dist_m` by the number of sides.
- `--cell` / `--sigma` set the section line's grid (as `plan_section.py`). Without gyro-bias estimation the results JSON has no window times and the `windows` column stays empty.
  - `seen_s`, `points`, `range_m`, `incidence_deg` (0 head-on, 90 grazing), `windows` (odometry windows from the results JSON), and with `--plan` `plan_dist_m`.
- Images `_thickness.png` and `_visits.png` (`pair_diff_m`): each piece's line in 1 px, blue / green / orange / red by the class limits printed (grey = no two visits from the same side); on the plan with `--plan` (after `plan_fit.py`), else on a map canvas with a 1 m grid.
- Prints quantiles and Spearman correlations of thickness, pair difference and plan distance against seen time, points, visits, range and incidence.
- Reference (2026-10-18, `44_baseline`, H = 0): 295 pieces; thickness p50 2.6 cm; pair difference p50 3.0 cm against a noise floor (`half_split_m`) of 1.1 cm; by time between the visits 1.3 cm (< 10 s), 2.7 cm (10–30 s), 2.9 cm (30–100 s), 3.0 cm (> 100 s); by same-side visit count 2.1 / 4.0 / 4.1 / 3.1 / 2.4 cm for 2–6 visits (the max-minus-min spread grows 2.1 → 8.5 cm).

## holdout_sharpness.py

Map sharpness on the points no stage ever fitted. Results made with the hold-out mode (`[evaluation] holdout_block_s` > 0) leave the points of odd time blocks out of the odometry's, the loop closure's and the consistency pass's maps and point terms, but save them; this tool measures only those. Sharpness on all points rewards a stage that fits its own points without placing the walls better (overfitting); on held-out points that shows as worse.

```
python tools/holdout_sharpness.py [--block 0.01] <reference> <result> [<result> ...]
python tools/holdout_sharpness.py [--block 0.01] --sets <ref_s0>,<res_s0> <ref_s1>,<res_s1> <ref_s2>,<res_s2>
```

- Planar spots (PCA within `--r` 0.12 m, at least 12 neighbours) of the held-out points, chosen on the reference and kept for all results; thickness `sqrt(smallest eigenvalue)` all-time (the whole map at the spot) and within ±0.05 s (slice: node-to-node jitter), paired spot by spot [mm], with a bootstrap interval over the spots.
- `--sets`: one pair per repeat run; prints each set and the mean, with an interval from the lowest to the highest set's bound (the spot bootstrap alone misses the run-to-run spread). `--fitted` measures the even blocks instead.
- All results of a set must hold the same points (same recording and hold-out), else it stops.
- Reference (2026-10-18, 44_short at `holdout_block_s` 0.01, research runs): consistency +1.85 mm all-time against odometry only, the scratch gyro term (s = 3) −0.87 mm.

## Regression baseline (compare_baseline.py, cut_recording)

Refactors must reproduce `44_short` and the full 44 bit for bit (see `docs/ARCHITECTURE.md`). Two reference datasets live next to the raw data, outside git:

| dataset | what | run time (Release) | manifest |
|---|---|---|---|
| `raw data/44_short` | first 30 s of 44, made with `cut_recording` | about 1.5 min | `tools/baseline/44_short.manifest.json` |
| `raw data/44` | the full recording (loops on every floor) | about 10 min | `tools/baseline/44.manifest.json` |

Workflow: check **every intermediate commit on `44_short`**, and run the **full 44 once at the end of each change** (only the full recording exercises loop closure). Always run with the Release build and `settings/l2-vertical.toml`, into a new output folder outside the input folder. Never run two jobs on the same input folder at the same time: they share its `cache/`. A stale `cache/` from an older build is harmless (every run rewrites it).

```
out/build/x64-Release/bin/dMapper.exe "<raw data>/44_short" settings/l2-vertical.toml "<raw data>/44_short_check"
python tools/compare_baseline.py "<raw data>/44_short_check" tools/baseline/44_short.manifest.json
```

**Since 2026-10-09 the datasets are the corrected recording 44:**
- its IMU was re-stamped by `restamp_imu.py` (79.4 ms − 0.0983 ms/s · t), because it lagged the LiDAR;
- `44_short` was cut again from it (`cut_recording`, 0 to 30 s);
- the baselines were made from it by the code of that day.

All earlier baselines came from the unsynchronized data and were deleted, together with the other recordings' results.

The folders `raw data/44_baseline` and `raw data/44_short_baseline` hold the output the manifests were written from. Keep them: a change that deliberately alters numbers compares its output with them to measure the effect, with `--tolerance`, before it writes new baselines:

```
python tools/compare_baseline.py --tolerance 1e-6 "<raw data>/44_check" "<raw data>/44_baseline"
```

To set new baselines after such a change: rename the two folders with a date suffix, move the new output folders in their place, and write the manifests from them with `compare_baseline.py --write-manifest <out> <manifest> --dataset <44|44_short> --params settings/l2-vertical.toml`.

There is one build and one floating-point mode (`/fp:precise`), so an exact refactor is bit-identical on both datasets; no second "precise" build is needed.

### compare_baseline.py

Compares an output folder with a baseline output folder or a manifest. Pure Python 3.

```
python tools/compare_baseline.py <new_out> <baseline_dir | manifest.json> [--verbose]
python tools/compare_baseline.py --write-manifest <out_dir> <manifest.json> --dataset 44 --params settings/l2-vertical.toml
```

- Every file is compared by SHA-256. The only things normalized are the values that change on every run:
  - the date-time in the `dMapper_params_*`/`dMapper_results_*` file names;
  - `elapsed time seconds`, `elapsed_time_seconds`, `processing_date` and `build_date`, plus the `# Created on` line of the params TOML;
  - the results JSON's `git_commit` and `git_uncommitted_changes` (`build_info`: the build's flags and definitions are compared);
  - absolute paths of the output and input folders, which become `<OUT>` and `<IN>`.

  Everything else, including every LAZ and trajectory byte, must match.
- It prints one line per `different`, `missing` or `extra` file. Exit code: 0 identical, 1 different, 2 usage or input error.
- `--tolerance <abs>` (needs a baseline folder) re-checks each differing text file (trajectories, session, poses, JSON, TOML) number by number. Text outside the numbers must match, integers must match exactly, and other numbers may differ by at most `<abs>`. It prints `tolerance <file> (max |difference| …)` for each accepted file. A differing LAZ can't be read here; it is listed as `derived` (its points are the raw points placed with the trajectory) and passes only if every text file passes. Exit code 0 then means identical or within tolerance.
- `--write-manifest` records the dataset, parameter file, commit and `HD_CPU_OPTIMIZATION` (`--cpu-optimization`, default `AUTO`). Only write a new manifest for a change that deliberately alters the numbers, and do it in that change's own commit.

### cut_recording

Cuts a time range of a recording into a new input folder. It is built with the app (CMake target `cut_recording`, LASzip only).

```
out/build/x64-Release/bin/cut_recording.exe "<raw data>/44" "<raw data>/44_short" 0 30
```

- Arguments: `<in_folder> <out_folder> <start_s> <duration_s>`, measured from the first IMU sample. It cuts every `.laz`/`.las` and `.csv` of the input folder and keeps their names.
- Kept points are unchanged: the input header (point format, scale, offset) is reused, so the stored coordinates and every attribute are identical. After writing, every kept point is re-read and compared. IMU lines and the header line are copied byte for byte.
- It refuses an output folder that already holds `.laz`, `.las` or `.csv` files. The input folder is never modified.
- The range starts at the first IMU sample, and LiDAR points recorded before it are dropped (about 3 900 on 44). The full run keeps them, so a cut from 0 s builds a different initial map and shifts every window. For a cut that reproduces the full run's windows exactly, start before the first sample: `cut_recording "<raw data>/44_s0" "<raw data>/44c95b_s0" -10 105` (checked 2026-10-16: identical trajectory up to the cut's end).

## rename_identifiers.py

Renames C++ identifiers in code only, never in comments, string or character literals. Used for the refactors' mechanical renames, so the compiler and the baseline check are the only things left to verify. Pure Python 3.

```
python tools/rename_identifiers.py app -r worker_data=windows -r WorkerData=Window --dry-run
python tools/rename_identifiers.py app -m intermediate_trajectory=poses -m gyro_bias=gyro_bias.mean
```

- `-r OLD=NEW` renames every whole identifier `OLD`: variables, types, functions and parameters, but not member accesses (`obj.OLD`, `p->OLD`; use `-m` for those). `NEW` may be dotted (`-r intermediate_trajectory=window.poses`).
- `-m OLD=NEW` renames `OLD` only as a member access, right after `.` or `->`, so a parameter or local with the same name is left alone. `NEW` may contain a dot (`gyro_bias.mean`).
- `--lines A-B` limits the renames to lines A..B of a single file, for example one function's body, when a name like `timestamps` is used elsewhere too.
- `--dry-run` lists every hit as `file:line: old -> new | code`, without changing anything; review it before a real run.
- **Paths:** files or folders (`.cpp,.cc,.c,.h,.hpp`; `--ext` changes the list, `--recursive` descends into sub-folders).
- **Output:** line endings are kept. A summary per file and in total is printed.
- **Exit codes:** 0 done, 2 usage error.
- **Known limit:** raw string literals (`R"(...)"`) are treated as code.

## function_sizes.py

Lists the functions of a source tree longer than a limit, largest first: the "huge functions" of
`docs/code-split-and-region-agents-plan.md` (stage 6), as a number that can be tracked.

```
python tools/function_sizes.py [src] [--min 80]
```

- A function runs from its first line at column 0 to the next line starting with `}` (dMapper's .cpp style); functions
  inside an anonymous namespace are indented and not counted. The length includes blank lines and comments, so it is
  approximate, and a class body defined at column 0 can show up under the name of its first member.

## window_terms.py

Prints the per-term window tables that a run or a replay writes with `log_window_terms = true`
(`[lidar_odometry_debug_info]`; stage 3 of `docs/code-split-and-region-agents-plan.md`). Pure Python 3.

```
python tools/window_terms.py <window_terms.csv> --window N [--state result|guess]
python tools/window_terms.py <window_terms.csv> [--from S] [--to S] [--state result|guess]
```

- The files: a CLI run writes `window_terms.csv` and `window_features.csv` to its output folder; `dMapper --replay
  <capture> <params.toml>` writes `<params name>_window_terms.csv` and `_window_features.csv` next to the capture.
- Per window, state (`guess`: before the first iteration, `result`: after the last) and term (NDT indoor and outdoor as
  solved, NDT before the degeneracy projection, edge positions and rotations, gyro-bias prior, velocity continuity,
  accelerometer, velocity prior, zero velocity, weak end-velocity tie, offset prior, origin fix, total, damping): the cost,
  the residuals, and for four motions (shift, stretch, curve, turn) along travel, sideways and vertical the weight
  `m^T H m` and the pull `m^T b / weight`. Each motion has amplitude 1 at the window's last node; turn is in degrees.
- `--window N`: the table, with the hold (1 / sqrt(weight), a sigma) and the pull per motion and direction.
- Without `--window`: per term over the range, for stretch and curve along travel, curve vertical and turn about
  vertical, how often the term holds the window most, its median hold, its median and largest pull, and the remaining
  step (the total's pull: near zero when the terms balance, large when the damping froze the window).
- Read the pull with its limits: it is one linear step with everything else held, on the cells the points match now
  (NDT reaches about 85 % of a 1 cm offset in the unit test, less beyond). Cost per residual is about 1 only for terms
  whose weight is a noise model (gyro rows, accelerometer, the carried priors); compare NDT, the scaled edge positions and
  the weak tie with their own usual values.
- `window_features.csv`: one row per window (time, iterations, damping, moves, velocities, gyro bias, acceleration
  offset, information ratio and weak axis, matched points, and the window's change from guess to result in the twelve
  motion numbers): the per-window features for a future learned tuner. Since stage 4 also the window verdict: the four
  flags (`verdict_bent`, `verdict_bent_severely`, `verdict_fast`, `verdict_bias_high`: CLEAR / TRIGGER / NOT MEASURED),
  the bends, speed, bias, velocity chi², why the iterations stopped, the solver's events, the remaining step and the
  LiDAR's weight share (empty where NOT MEASURED).
