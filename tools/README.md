# tools

## eval_odometry.py

Quality metrics for `lidar_odometry_step_1` results. Pure Python 3, with no extra packages.

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

## merge_chunks.sh

Merges the chunk clouds of a result folder into one LAZ in world coordinates, using the CloudCompare command line. Each `scan_lio_N.laz` is stored relative to its chunk's first pose; `session_poses.mrp` holds those poses.

```
tools/merge_chunks.sh "<result_folder>" ["<output.laz>"]
```

The default output is `<result_folder>/scan_lio_merged.laz`. Set `CLOUDCOMPARE` if CloudCompare is not installed in `C:\Program Files\CloudCompare`.

## Regression baseline (compare_baseline.py, cut_recording)

Refactors must reproduce the output of the reference dataset bit for bit (see `docs/ARCHITECTURE.md`). Two reference datasets live next to the raw data, outside git:

| dataset | what | run time (Release) | manifest |
|---|---|---|---|
| `raw data/44_short` | first 30 s of 44, made with `cut_recording` | about 3.5 min | `tools/baseline/44_short.manifest.json` |
| `raw data/44` | the full recording (loops on every floor) | about 30 min | `tools/baseline/44.manifest.json` |

Workflow: check **every intermediate commit on `44_short`**, and run the **full 44 once at the end of each change** (only the full recording exercises loop closure). Always run with the Release build and `settings/lidar-vertical.toml`, into a new output folder outside the input folder. Never run two jobs on the same input folder at the same time: they share its `cache/`. A stale `cache/` from an older build is harmless (every run rewrites it).

```
out/build/x64-Release/bin/lidar_odometry_step_1.exe "<raw data>/44_short" settings/lidar-vertical.toml "<raw data>/44_short_check"
python tools/compare_baseline.py "<raw data>/44_short_check" tools/baseline/44_short.manifest.json
```

### compare_baseline.py

Compares an output folder with a baseline output folder or a manifest. Pure Python 3.

```
python tools/compare_baseline.py <new_out> <baseline_dir | manifest.json> [--verbose]
python tools/compare_baseline.py --write-manifest <out_dir> <manifest.json> --dataset 44 --params settings/lidar-vertical.toml
```

- Every file is compared by SHA-256. The only things normalized are the values that change on every run:
  - the date-time in the `HDMapping_params_*`/`HDMapping_results_*` file names;
  - `elapsed time seconds`, `elapsed_time_seconds`, `processing_date` and `build_date`, plus the `# Created on` line of the params TOML;
  - absolute paths of the output and input folders, which become `<OUT>` and `<IN>`.

  Everything else, including every LAZ and trajectory byte, must match.
- It prints one line per `different`, `missing` or `extra` file. Exit code: 0 identical, 1 different, 2 usage or input error.
- `--write-manifest` records the dataset, parameter file, commit and `HD_CPU_OPTIMIZATION` (`--cpu-optimization`, default `AUTO`). Only write a new manifest for a change that deliberately alters the numbers, and do it in that change's own commit.

### cut_recording

Cuts a time range of a recording into a new input folder. It is built with the app (CMake target `cut_recording`, LASzip only).

```
out/build/x64-Release/bin/cut_recording.exe "<raw data>/44" "<raw data>/44_short" 0 30
```

- Arguments: `<in_folder> <out_folder> <start_s> <duration_s>`, measured from the first IMU sample. It cuts every `.laz`/`.las` and `.csv` of the input folder and keeps their names.
- Kept points are unchanged: the input header (point format, scale, offset) is reused, so the stored coordinates and every attribute are identical. After writing, every kept point is re-read and compared. IMU lines and the header line are copied byte for byte.
- It refuses an output folder that already holds `.laz`, `.las` or `.csv` files. The input folder is never modified.
