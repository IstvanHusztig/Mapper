# Feature: cave rendering (Render menu in dMapper, offline tool for meshes)

Branch: `claude/epic-meitner-plfjh8`. Run locally on Windows (GPU, the datasets, CloudCompare for checks). This is a feature in three phases. When it is picked up, turn it into an OpenSpec change (`/opsx:propose`) with this file as the input.

## Builds on
This feature comes after the radiometry chain (both in this branch: `docs/tasks/read-recording-format-2.md`, `reports/`, `research_notes/`):
1. `docs/tasks/read-recording-format-2.md`: dMapper reads dLidar format 2 and, with `[radiometry] export_beam_geometry = true`, writes range, sensor position P0 and beam direction D per output point as LAS extra bytes.
2. The probe tool and the self-calibrated range/angle correction (`reports/Unitree L2 reflectivity correction.md`, steps 1–5): per-point ρ_corr, per-cell ρ_p with MAD, n_obs, the fitted angular exponent n and a validity flag, kept in the run and written as extra bytes.

Rendering does not wait for both. It uses whatever the run or result has and falls back step by step (Phase 1, albedo). A render from raw intensity is labelled as such. Only a render from ρ_p is called "reflectance".

## Why
Cave surveys are shown to people who don't read point clouds: cavers, landowners, the public. A view that lights the measured walls with a headlamp reads as a cave. Its brightness comes from the measured reflectance, not from a model's invention (unlike AI video tools). Having it in the app means a surveyor sees it right after processing, without another program.

## Rules for the whole feature
- Rendering never changes processing or results, like the live preview. It runs only on final data: after a run has ended, or on a result folder opened from disk.
- No new dependency in the C++ build. The viewer is legacy OpenGL without a function loader (GLEW left with `core/`), so the in-app renderer uses only what OpenGL 1.1 offers: client vertex arrays, fixed-function lighting with a spot light, `glReadPixels`. Shaders, shadows and screen-space effects are out of reach without a loader. Adding one is a separate decision, not part of this feature.
- The maths is in the domain and tested; the GUI only draws.

## Phase 1: render scene in the layers (no GUI yet)
1. **Domain** (`src/domain/render_scene.{h,cpp}`, Eigen only): a `RenderScene` with positions, normals, albedo, material group per point, and the albedo source. Functions:
   - **Voxel thinning:** default 0.01 m. Keep the point nearest each voxel's centroid, not an average, so the geometry stays measured.
   - **Normals:** PCA on 5–15 cm cells of the final map, the same method as the radiometric correction (reuse its normals when the run has them). Orient each normal toward the point's sensor position P0 when present, otherwise toward the nearest trajectory pose. The sensor is always inside the passage, so this is reliable where generic orientation fails.
   - **Albedo, best available first:**
     1. cell ρ_p where the validity flag is set;
     2. per-point ρ_corr;
     3. raw intensity: percentile stretch (1–99 %), 255 drawn as clipped (full brightness, never averaged).

     Multiply by a rock tint (a pale limestone by default; the tint is chosen and says so).
   - **Baked ambient occlusion:** an openness term per point from its neighbourhood (the share of a hemisphere around the normal that is free of points within 0.5 m), multiplied into the albedo once. This stands in for the screen-space AO that fixed-function GL can't do.
   - **Material groups:** 3–4 groups by the fitted angular exponent n (glossy flowstone through granular rock), each with its own specular and shininess. Without n, one rough group.
   - **Camera path:** the trajectory smoothed (1 m window), looking 2 m ahead, eye height offset.
2. **Application:** `prepare_render_scene` takes the final points (from the `PointStore` through the final trajectory, as the preview rebuild does, but at full density), the trajectory and the radiometry results if present.
3. **Infrastructure:** read a result folder (`scan_lio_merged.laz` with its extra bytes by name, and the trajectory) into the same inputs, so past results render without reprocessing.
4. **Tests** (doctest, no data, no display):
   - albedo fallback order and clipped 255;
   - normal orientation toward P0 and toward the trajectory on a synthetic tube;
   - AO darker in a corner than on a flat wall;
   - thinning keeps only input points;
   - camera path smoothing and look-ahead on a synthetic polyline.

## Phase 2: Render menu in the GUI
A new **Render** menu next to View and Camera (`lidar_odometry_gui.cpp`, state and actions in `GuiViewModel`):
- **Render cave view.** Enabled when a run has ended or a result is open. It prepares the `RenderScene` on a worker thread with progress, like Load. It then switches the viewer from the height-coloured preview to lit points:
  - the headlamp is `GL_LIGHT0` as a spot light set in eye coordinates, so it moves with the camera: warm white, about 25° cutoff, distance attenuation;
  - the per-point albedo goes in through `glColorMaterial`;
  - the material groups are drawn as separate arrays;
  - background black, the point size adjustable.
- **Open result…** Loads a result folder (Phase 1, step 3) for rendering.
- **Fly along trajectory** (play/pause, speed) follows the camera path, and **Free camera** goes back to the normal controls.
- **Lamp and look:** a small panel for lamp cone, intensity and range, tint, point size, and the albedo source shown read-only. These are view state, not processing settings, so they stay out of the TOML.
- **Export frames…**: renders the fly-through at a chosen resolution to numbered image files (BMP or PPM written directly, or a vendored `stb_image_write.h` if PNG is wanted) and shows the `ffmpeg` command for a video.
- **Large clouds:** while the camera moves, draw a coarser level (every k-th point); draw the full one when it stops or during export. Report memory per point and the total for recording 44.
- **Tests:** `tests/gui_view_model_test.cpp` covers the menu's enabling rules (no render during a run, render after it ends or from an opened result) and that rendering leaves the pipeline's data unchanged.

## Phase 3: offline mesh and photoreal export (`tools/render_cave.py`)
For what the app does not do: Poisson meshes and input for Blender Cycles or Unreal.
- Poisson reconstruction in overlapping chunks along the trajectory (depth 10–12), with vertices below a density quantile removed and vertex colours from the albedo.
- Vertices farther than `--max-gap` (default 0.05 m) from any measured point are coloured neutral grey or dropped, so invented surface is visible as such.
- Writes `.ply` with vertex colours and a headlamp-lit Open3D preview.
- Takes its albedo and normals from a scene file that the app exports (Render → Export scene…), so the Python tool does not reimplement Phase 1.
- **Libraries** (the shared choice for the Python tools, in `docs/tasks/read-recording-format-2.md`, section "Python libraries"):
  - `open3d` for Poisson, normals, the offscreen render and PLY;
  - `laspy[lazrs]` to read the LAZ with its extra bytes;
  - `numpy`, `scipy` (`cKDTree` for the gap distances);
  - PyMeshLab only if Open3D's meshes need better cleaning.

  Install with `python -m pip install --user "laspy[lazrs]" numpy scipy open3d`. The tool checks its imports at startup and prints this command when one is missing. Open3D's Windows wheels can lag the newest Python, so check the supported versions first.

## Pass criteria
- Phase 1–2 change no processing: `44_short` stays bit-identical with its manifest, and the unit tests pass in x64-Release. Also run x64-Debug with `ctest`, because of the new containers and the worker thread.
- On recording 44 s0, both from the finished run and from the opened result folder:
  - Render cave view and a full fly-through export work;
  - report the preparation time, frame rate while moving and when still, and memory. Precision first: report the cost rather than cutting density to avoid it.
- With ρ_p input: a wall seen by two passes shows no brightness seam where the raw-intensity render of the same frame does. Show both frames side by side.
- Phase 3: the `.ply` opens in CloudCompare and Blender with vertex colours, and every surface farther than `--max-gap` from a measured point is grey or absent.

## Not in this feature
- rock-type classification (needs ρ_p first; see the report's last section);
- shaders, shadows or eye-dome lighting in the app (needs a GL loader, a separate decision);
- photoreal path tracing (done in Blender from the Phase 3 export);
- the Unreal / Vagon Streams build.
