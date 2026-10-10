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

## Regression baseline (compare_baseline.py, cut_recording)

Refactors must reproduce `44_short` and the full 44 bit for bit (see `docs/ARCHITECTURE.md`). Two reference datasets live next to the raw data, outside git:

| dataset | what | run time (Release) | manifest |
|---|---|---|---|
| `raw data/44_short` | first 30 s of 44, made with `cut_recording` | about 1.5 min | `tools/baseline/44_short.manifest.json` |
| `raw data/44` | the full recording (loops on every floor) | about 10 min | `tools/baseline/44.manifest.json` |

Workflow: check **every intermediate commit on `44_short`**, and run the **full 44 once at the end of each change** (only the full recording exercises loop closure). Always run with the Release build and `settings/lidar-vertical.toml`, into a new output folder outside the input folder. Never run two jobs on the same input folder at the same time: they share its `cache/`. A stale `cache/` from an older build is harmless (every run rewrites it).

```
out/build/x64-Release/bin/dMapper.exe "<raw data>/44_short" settings/lidar-vertical.toml "<raw data>/44_short_check"
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

To set new baselines after such a change: rename the two folders with a date suffix, move the new output folders in their place, and write the manifests from them with `compare_baseline.py --write-manifest <out> <manifest> --dataset <44|44_short> --params settings/lidar-vertical.toml`.

There is one build and one floating-point mode (`/fp:precise`), so an exact refactor is bit-identical on both datasets; no second "precise" build is needed.

### compare_baseline.py

Compares an output folder with a baseline output folder or a manifest. Pure Python 3.

```
python tools/compare_baseline.py <new_out> <baseline_dir | manifest.json> [--verbose]
python tools/compare_baseline.py --write-manifest <out_dir> <manifest.json> --dataset 44 --params settings/lidar-vertical.toml
```

- Every file is compared by SHA-256. The only things normalized are the values that change on every run:
  - the date-time in the `dMapper_params_*`/`dMapper_results_*` file names;
  - `elapsed time seconds`, `elapsed_time_seconds`, `processing_date` and `build_date`, plus the `# Created on` line of the params TOML;
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
