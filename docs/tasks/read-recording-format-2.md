# Task: read dLidar recording format 2 in dMapper (no correction yet)

Branch: `claude/epic-meitner-plfjh8`. Run locally on Windows, where CMake 4, the `x64-Release`/`x64-Debug` presets, the datasets and the dLidar format samples are.

## Why
dLidar now writes recording format 2 (`docs/dlidar-recording-format-2.md`). It adds per-point LAS extra bytes (corrected range, raw range, sample index, packet seq, scan angles alpha/theta) and a per-packet `packetstateNNNN.tsv` (calibration and angle parameters, APD voltage and temperature, laser voltage, `dirty_index`). The standard LAZ fields and the IMU CSV are unchanged.

dMapper should read and carry this data as the groundwork for the intensity normalization planned in `reports/Unitree L2 reflectivity correction.md`. That plan is a range and incidence-angle correction with 255 treated as censored, run after loop closure and consistency. This task applies **no correction**.

## Steps
1. **Verify compatibility first.** Run the current dMapper (Release, `settings/lidar-vertical.toml`) on `/home/vs2022/dLidar_format_samples/format1_old/` and `.../format2_new/`. Compare the two outputs with `tools/compare_baseline.py`; they must be bit-identical. Report the result before changing code. The samples are synthetic (a stationary sensor in a box room), so use them for parsing and geometry only, never for fitting.
2. **Read format 2** in `load_point_cloud` (`src/infrastructure/recording_folder_source.cpp`, around lines 145–235):
   - Detect the format with `generating_software == "dLidar recording format 2"` and `num_extra_bytes == 20`.
   - Read the 20 extra bytes at the offsets in the spec, section 3.
   - Load `packetstateNNNN.tsv` (paired by the same NNNN) and match each point to its packet row as described in the spec, section 6. Walk forward and handle the seq wrap at 1024. Check `gps_time == stamp_ns/1e9 + sample_index*(double)time_increment`.
   - From the row's β, ξ, a, b and the point's α, θ, compute the emission point P0 and the unit beam direction D (spec section 4), in the LAZ frame.
   - Take these from the **raw** xyz, before the lever-arm shift that `load_point_cloud` applies.
3. **Format-1 fallback:** range = |raw xyz|, P0 = 0, D = raw xyz / |raw xyz|, no packet state. Mark the point as approximate.
4. **Carry the data.**
   - Add the new per-point fields to `Point3Di`, `Point3DiDisk`, `convert_to_disk` and `convert_from_disk` together (`src/domain/structures.h`, around lines 65–138), so they survive `DiskPointStore`'s `.cache` files. The fields: range, P0, D (or alpha/theta plus a packet-row index), a packet-state index, a format flag.
   - Keep the per-packet state in a table referenced by index, not copied into every point.
   - Initialize `index_point`, which is currently left uninitialized.
   - Check how much memory the larger points cost on the full 44.
5. **Optional output, off by default.** Add a new setting, e.g. `[radiometry] export_beam_geometry = false`, in `TomlIO::POINTERS` and `CATEGORIES`, and regenerate the golden TOML files with `LO_UPDATE_GOLDEN=1` in their own commit.
   - When on, `exportLaz` (`src/infrastructure/laz_export.cpp`) and `MergedLazWriter` (`src/infrastructure/folder_result_sink.cpp`) add LAS extra bytes with `laszip_add_attribute`: range, the sensor position and beam direction in the output frame, and the packet-state index.
   - Transform these where `points_to_vector` (`src/domain/point_placement.h`) applies the per-point pose: the pose for positions, the rotation only for directions.
   - When off, the output files are byte-identical with today's.
   - Extend `ResultSink`/`PreviewSink` (`src/application/output_ports.h`, `input_ports.h`) as needed, keeping the layer rules.
6. **Tests** (doctest, no recorded data):
   - Extend `tests/disk_point_store_test.cpp` to round-trip the new fields.
   - New reader test: write a small LAZ with 20 extra bytes and a TSV through laszip inside the test, then check the parsed range/P0/D against the spec formulas, the row matching across a seq wrap and a lost packet, and the format-1 fallback.

## Pass criteria
- `44_short` bit-identical with `tools/baseline/44_short.manifest.json` on every intermediate commit, and the full 44 with `tools/baseline/44.manifest.json` once, both with the new setting off.
- `format1_old` and `format2_new` give bit-identical dMapper results.
- The unit tests pass in x64-Release. Also build x64-Debug and run `ctest`, because the change touches struct layout, uninitialized memory and new containers.
- With the setting on: laspy reads the new attributes by name, and the range on `format2_new` differs from |raw xyz| by at most hypot(a, b) (about 8 mm). Do not rely on a CloudCompare ASCII export here, because it may not carry the extra bytes in a usable form.

## Python libraries (tools/)
The Python tools so far use numpy only and read LAZ through a CloudCompare ASCII export. The radiometry and rendering tools read LAZ directly. The libraries are chosen once for all of them:
- **`laspy[lazrs]`**: read and write LAZ with the extra bytes by name. Used by the checks of this task, `tools/intensity_probe.py` and `tools/render_cave.py`.
- **`numpy`, `scipy`**: arrays, `cKDTree` for surface cells and neighbours, `scipy.optimize` / `scipy.sparse` for the range and angle fit.
- **`open3d`**: only in `tools/render_cave.py` (Poisson mesh, offscreen render, PLY). PyMeshLab is the fallback if its meshes need better cleaning.
- **Not used:** PDAL (conda-only on Windows; reconsider for COPC or web viewers), PyVista, MeshLib, Pyntcloud.

Install: `python -m pip install --user "laspy[lazrs]" numpy scipy open3d`. Each tool checks its imports at startup and prints this command when one is missing. Moving `map_quality_lib.py` and `spot_check.py` from the CloudCompare export to laspy is a separate change.

## Not in this task
The probe tool (intensity against range, angle and APD voltage per surface cell, 255 censored) and the self-calibrated range/angle correction. Each is a later task, which needs the first real format-2 recording.
