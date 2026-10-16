# Task: render a realistic cave from a dMapper result (Open3D)

Branch: `claude/epic-meitner-plfjh8`. Run locally on Windows (GPU, the datasets, CloudCompare for checks).

## Builds on
This task comes after the radiometry chain on the research branch `claude/busy-ride-bsd230`:
1. `docs/tasks/read-recording-format-2.md`: dMapper reads dLidar format 2 and, with `[radiometry] export_beam_geometry = true`, writes range, sensor position P0 and beam direction D per output point as LAS extra bytes.
2. The probe tool and the self-calibrated range/angle correction (`reports/Unitree L2 reflectivity correction.md`, steps 1–5): per-point ρ_corr, per-cell ρ_p with MAD, n_obs, the fitted angular exponent n and a validity flag, again as extra bytes.

Rendering does not wait for both. It reads whatever the result has and falls back step by step (see step 2). A render from raw intensity is labelled as such. Only a render from ρ_p is called "reflectance".

## Why
Cave surveys are shown to people who don't read point clouds: cavers, landowners, the public. A render that lights the measured walls with a headlamp reads as a cave. Its brightness comes from the measured reflectance, not from a model's invention (unlike AI video tools). The same prepared geometry also feeds Blender Cycles or Unreal (Vagon Streams) later.

## Decision: dependencies
`tools/` stays pure Python as a rule (CLAUDE.md). This tool is the deliberate exception: it needs `open3d`, `laspy[lazrs]`, `numpy` and `scipy` (`python -m pip install --user open3d "laspy[lazrs]" numpy scipy`). Check for them at startup and print that command when one is missing. No other tool may import these because of this one.

## Steps
1. **`tools/render_cave.py <result_folder> <out_folder> [options]`**. Input: `scan_lio_merged.laz` and the trajectory of the result folder. Recentre on the trajectory's first pose and thin with a voxel grid (`--voxel`, default 0.01 m). Read the extra bytes by name and report which were found.
2. **Albedo, best available first:**
   1. cell ρ_p where the validity flag is set;
   2. per-point ρ_corr;
   3. raw intensity: percentile stretch (1–99 %), 255 drawn as clipped (full brightness, never averaged).

   Record the source in the output (`render_info.json` and the frame caption). Multiply by a rock tint (`--tint`, default a pale limestone) and say in the caption that the tint is chosen.
3. **Normals:**
   - Use PCA on 5–15 cm cells of the final map, the same method as the correction (reuse its normals if it exports them).
   - Orient each normal toward the point's own sensor position P0 when `export_beam_geometry` data is present. Otherwise use the nearest trajectory pose (KD-tree). The sensor is always inside the passage, so this is reliable where generic orientation fails.
4. **Materials:**
   - Open3D's `defaultLit` has one roughness per geometry. Split the cloud into 3–4 geometries by the fitted angular exponent n (glossy flowstone through granular rock) when n exists; otherwise use one rough material.
   - An optional class attribute (a later classification task) maps to a material table in the same way. It is optional, not required.
5. **Two geometry modes:**
   - `--mode points` (default): lit points, faithful; gaps show up close.
   - `--mode mesh`: Poisson reconstruction in overlapping chunks along the trajectory (depth 10–12), with vertices below a density quantile removed and vertex colours from the nearest points. Write the mesh as `.ply` too, for Blender or Unreal.

   Mesh mode invents surface where there are no points. Colour vertices farther than `--max-gap` (default 0.05 m) from any measured point in a neutral grey, or drop them (`--drop-gaps`), so invented surface is visible as such.
6. **Light and camera:**
   - `OffscreenRenderer`, no sun and no image-based light, ambient occlusion on.
   - A spot light that moves with the camera: warm white, 20°/40° cones, shadows on.
   - Camera path: the trajectory smoothed (`--smooth`, default 1 m window), looking 2 m ahead, height offset `--eye-height`. `--start`/`--end` take trajectory times.
   - Write PNG frames and `render_info.json` (inputs, albedo source, settings, version of the result), and print the `ffmpeg` command for the video.
7. **Tests** (pure functions, no GPU, no recorded data): albedo fallback order and the clipped-255 handling; normal orientation toward P0 and toward the trajectory on a synthetic tube; camera path smoothing and look-ahead on a synthetic polyline; gap masking on a mesh with a known hole. Put them next to the tool, runnable with `python -m pytest tools` or plain `python`, whichever the repo's existing tools use.
8. **Document** the tool in `tools/README.md`.

## Pass criteria
- Runs on recording 44 s0's result in both modes. Report the time and peak memory for the whole recording. Precision first: report the cost rather than cutting density to avoid it.
- Points and mesh renders of the same frame line up (same camera, same walls).
- With ρ_p input: a wall seen by two passes shows no brightness seam where the raw-intensity render of the same frame does. Put both frames side by side in the report.
- Mesh mode: every surface farther than `--max-gap` from a measured point is grey or absent.
- The `.ply` opens in CloudCompare and Blender with its vertex colours.
- The tests pass. dMapper itself is unchanged: no C++ or settings change, so no baseline run is needed.

## Not in this task
Rock-type classification (needs ρ_p first; see the report's last section), photoreal path tracing (export to Blender Cycles), the Unreal / Vagon Streams build, and any change to dMapper's processing.
