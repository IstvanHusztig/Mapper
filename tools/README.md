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

## merge_chunks.sh

Merges the chunk clouds of a result folder into one LAZ in world coordinates, using the CloudCompare command line. Each `scan_lio_N.laz` is stored relative to its chunk's first pose; `session_poses.mrp` holds those poses.

```
tools/merge_chunks.sh "<result_folder>" ["<output.laz>"]
```

The default output is `<result_folder>/scan_lio_merged.laz`. Set `CLOUDCOMPARE` if CloudCompare is not installed in `C:\Program Files\CloudCompare`.

## Regression baseline (compare_baseline.py, cut_recording)

Refactors must reproduce `44_short` bit for bit, and the full 44 within a tolerance of 1e-6 (see `docs/ARCHITECTURE.md`). Two reference datasets live next to the raw data, outside git:

| dataset | what | run time (Release) | manifest |
|---|---|---|---|
| `raw data/44_short` | first 30 s of 44, made with `cut_recording` | about 2.5 min | `tools/baseline/44_short.manifest.json` |
| `raw data/44` | the full recording (loops on every floor) | about 25 min | `tools/baseline/44.manifest.json` |

Workflow: check **every intermediate commit on `44_short`**, and run the **full 44 once at the end of each change** (only the full recording exercises loop closure). Always run with the Release build and `settings/lidar-vertical.toml`, into a new output folder outside the input folder. Never run two jobs on the same input folder at the same time: they share its `cache/`. A stale `cache/` from an older build is harmless (every run rewrites it).

```
out/build/x64-Release/bin/dMapper.exe "<raw data>/44_short" settings/lidar-vertical.toml "<raw data>/44_short_check"
python tools/compare_baseline.py "<raw data>/44_short_check" tools/baseline/44_short.manifest.json
```

The 44 reference was refreshed on 2026-10-03 from the output of the `domain-types` code (step 5a). Every 44 run since step 3 had differed from the 2026-10-01 reference by the same `/fp:fast` last-bit pattern, at most 1e-9 m. The old reference is kept as `raw data/44_baseline_2026-10-01`.

Both references were refreshed again on 2026-10-03 by `chain-next-window-only`, which deliberately changes the last bits: the window loop places only the next window instead of all later ones (`docs/window-chaining.html`). Its results were within `--tolerance 1e-6` of the old references (44 at most 1e-9, 44_short at most 1e-11), and the manifests and the folders `raw data/44_baseline` and `raw data/44_short_baseline` now hold its output. The previous references are kept as `raw data/44_baseline_2026-10-03b` and `raw data/44_short_baseline_2026-10-03b`.

The full 44 is checked against the manifest first. If that reports differences, check it again with `--tolerance 1e-6` against the baseline output folder `raw data/44_baseline`, which must be kept for this:

```
python tools/compare_baseline.py --tolerance 1e-6 "<raw data>/44_check" "<raw data>/44_baseline"
```

Why the tolerance: the Release build with `HD_CPU_OPTIMIZATION=AUTO` on an AMD CPU uses `/fp:fast` with whole-program optimization. The compiler may then evaluate floating-point expressions in a different order when unrelated code changes, which changes the last bits of a long run. In step 3 (`logger-progress-ports`), changing only console output moved the 44 trajectories by at most 1e-9, while `44_short` stayed identical.

### Precise build for exact refactor checks

A refactor that reshapes numeric code can change the last bits of the normal (`/fp:fast`) build although it is exact (see `docs/ARCHITECTURE.md`, Decisions). Such a change is checked with a second Release build without `/fp:fast`, built outside the repo's `out/` tree (it is not a preset):

```
cmake -S . -B <scratch>/bp -G Ninja -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=GENERIC -DCMAKE_C_COMPILER=cl.exe -DCMAKE_CXX_COMPILER=cl.exe
cmake --build <scratch>/bp --target dMapper
```

1. Build master this way, run it on `44_short`, and write a manifest of its output: `compare_baseline.py --write-manifest <out> <scratch>/44_short_precise.manifest.json --dataset 44_short --params settings/lidar-vertical.toml --cpu-optimization GENERIC`.
2. On every commit of the change, rebuild `<scratch>/bp`, run `44_short` again and compare with that manifest: it must be IDENTICAL.
3. The normal build's `44_short` must then be within `--tolerance 1e-6` of `raw data/44_short_baseline`.

The precise and the normal build differ from each other by millimetres over a run, so never compare one with the other's reference. In `window-loop-steps` (2026-10-03) every commit was precise-identical, while the normal build moved by at most 1e-11.

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
