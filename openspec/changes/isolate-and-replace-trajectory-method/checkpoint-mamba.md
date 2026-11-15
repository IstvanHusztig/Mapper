# Checkpoint mamba

2026-11-15. User named this checkpoint and instructed continued work after saving.
Usage balance is not visible to the agent. Keep progress durable; no push or
default adoption. Charlie remains3/19. Astra High last confirmed.

## Confirmed physical reference

The long displaced strip is **a wall at apartment56's level**. It must fit the
wall the user drew on the house plan. The user has resolved the identity/level
question; do not ask it again or explain displacement as another floor/railing.
The initial plan alignment remains preliminary, not surveyed ground truth.

## Latest preserved result

Base: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/`.
Cloud: `livox4-kiss-gyro-deskew-125s/result/cloud.laz`.
SHA256: `4506b2c0a4fbe87f435135f4cd35874370071523a6c2c99226f1b59399f78d73`.
All20,272,168 raw source IDs/attributes and1,250 frame timings verified;118.8s.
User feedback: "Thats a good result". Preserve the candidate; no fixed-map claim.
This is an external KISS-derived gyro-deskew experiment, not integrated dMapper.
`settings/livox-vertical.toml` still selects the matched-gain dMapper control.

Quality folder contains fixed plan overlay, overview, marked-cohort comparison
and frozen indoor revisit comparison. Marked median/p90 offset3.343/3.535m,
line angle-2.828deg, height median0.525m under the unchanged alignment. Indoor
results mixed; no full gyro-deskew replay selected. Raw files are unchanged.
Latest plot helpers copied to the run's `checkpoint-tools/` directory.

## Evidence to reuse; do not repeat blindly

- Stock full KISS: `livox4-kiss-icp-fine-full/`; return-scene registration supports
  approximately2.02m/51.2deg error. Exact prefix correspondence was verified.
- `livox4-kiss-return-graph/`: one endpoint edge closes the end but warps early
  rooms; rejected for further export.
- `livox4-kiss-gyro-source-35s/`: controlled deskew/prediction2x2. Gyro deskew
  removes the effective timing lag; prediction alone does little. Geometry mixed.
- Its `fixed-map-case/`: F271 raw map/source captured, faithful six-DOF replay
  matches native to roundoff. Fixed-gyro rotation worsens central geometric fit.
- `livox4-kiss-map-scale-35s/`: wider map recovers matches but not map quality.
- `livox4-kiss-nonendpoint-revisits/`: four40-55s frames do not provide a reliable
  initial-room loop. This does not rule out other revisits.
- MapClosures2.2 feasibility researched only; no installation/retrieval. Default
  image border/resolution is unsuitable for small indoor maps. Do not install or
  launch blindly; dependency isolation and 3D verification would be required.

Full evidence, caveats and research links: `livox4-independent-icp-baseline.md`.
Source/experiments/failed configurations remain preserved. No accepted general fix.

## Resume action

Use the newly confirmed same-level wall constraint. First audit the provenance
and geometry of the fixed initial plan alignment, and the already recorded route
leading to the wall. Distinguish a global alignment error from deformation between
the starting rooms and that wall; do not fit the wall independently or move it
to the annotation. Then select a bounded correction test from actual evidence.
No more timestamp-guessing loop queries or unexplained tuning sweeps.

All prior agents completed; reactivate with followup_task. Root owns docs/quality;
gravity_candidate owns the external worktree/tools; seed_damping_crosscheck is
the independent reviewer; ndt_failure_next_step is the Terra research agent.
Preserve unrelated dirty files. No processing was active when checkpoint saved.

## Latest progress saved after wall confirmation

The plan overlay was audited against its archived source. Initial fit used first
15s and eight traced Wohnen segments, with no surveyed position/heading anchor
and55% weighted support within10cm. Alignment is ambiguous, but no identified
metre-scale unit/frame/time error: measured anchor mismatch12.5mm/0.079deg and
whole-sheet plan scale sanity~0.22%.

Two independently reviewed checks are saved in the candidate's `checkpoint-tools/`
and `quality/`:

- `relative_wall_check.py` / `relative-wall-check.json`: median3.508m separation
  from the frozen initial south-region plane;3.502m using the same candidate early
  support. A common rigid transform cannot change this internal separation.
  Early windows/insets remain a physical-attribution caveat.
- `wall_start_constraint.py` / `wall-start-constraint.json`: matching the confirmed
  east wall's direction/location at plan y10.9 forces start y8.700, outside Wohnen
  (south bound11.07). Reversing orientation puts the wall west of the room.
  Independently trimmed/time-balanced checks give the same incompatibility.
  One common planar alignment cannot satisfy both constraints under this approximate
  correspondence. This does not locate the error or isolate translation/rotation.

One bounded existing-data axis audit also completed, with synthetic SVD test:
`checkpoint-tools/check_axis_consistency.py`, `quality/axis-consistency/comparison.json`.
Fit a single proper gyro-to-LiDAR rotation on25-60s, test60-100s and100-125s,
separately at lag0/-50ms. Fitted angles1.806/1.269deg; training/first-holdout RMS
changes negligible, late holdout worsens1.930->1.942deg /2.391->2.419deg. Excitation
is highly directional. Worker report only: independent review still pending;
do not promote it to a calibration or sensor-fault verdict. No correction applied.

All assigned computations are finished and saved. No new replay, installation,
production modification or default change. Next resume step is review the saved
axis audit and localize deformation along the existing route to the confirmed
wall, using the reviewed rejection of a simple global alignment explanation.
Preserve the positively received candidate and all failed branches. No new
experiment is selected by this checkpoint.

## Axis audit reviewed (2026-11-15, Claude; independent Sonnet review agrees)

`checkpoint-tools/axis_observability.py` (read-only, on the same inputs) splits the
fitted rotation and its uncertainty. Lag 0: tilt 0.50 deg, about vertical -1.73 deg;
lag -50 ms: 0.70 / -1.06 deg. Gyro excitation is almost only yaw (eigenvalues
0.17 / 0.48 / 12.9 rad^2, dominant axis z), so rotation about z is barely observable.
1-sigma (x, y, z): 0.48 / 0.48 / 2.15 deg for independent samples, about 0.77 / 0.77 /
3.4 deg with the overlapping windows. Residual 1.74 deg per component per 0.5 s,
dominated by LiDAR-odometry noise. Every fitted component is within 1 sigma.

Result: no misalignment detected. The audit only bounds it to roughly 1 deg tilt and
a few degrees about z; it is a gross check, not a calibration, with two lags only.
In this KISS run the gyro only deskews within a frame (`run_kiss.py`: per-return
rotation in deskew; prediction native `last_delta`) and heading comes from ICP, so an
axis error of this size cannot explain the 3.5 m / -2.8 deg wall offset here. It says
nothing about dMapper, where the gyro is a rotation prior. Axis audit closed.
Next: localize the deformation along the existing route to the confirmed wall.

## Route localization (2026-11-15, Claude; independent Sonnet reviews)

Scripts in `checkpoint-tools/` (route_block_plan_fit, plan_walls, step_vs_map,
pdr_check), outputs in `quality/route-localization/`.

- Per-block plan fit (5 s blocks onto plan pixels): failed its positive control
  (does not recover the known 3.3 m at the wall; ambiguous in the start room). Plan
  draws walls as double thin lines, so a thickness filter keeps text, not walls.
  Only 75-95 s is usable: map within ~0.6 m of the plan, slow dy creep to ~0.6 m.
- IMU steps vs map: cadence steady 1.6/s through 95-110 s; map path 105-110 s is
  3.13 m against 4.77 m expected (ratio 0.65; other walking blocks 0.85-1.12).
  100-112 s: 8.05 m against 10.14 m (step length 0.597 m from 60-95 s, +-10%).
- Yaw map minus gyro-propagated (anchors 95/100 s): within 0.7 deg to 105 s, -3.8
  at 107.5 s, -7.1 at 110 s, plateau ~-7 to 125 s. Not a lag (lag errors return to
  zero). Map under-rotates the south-to-east turn, or gyro scale ~8 % off; unresolved.
  This yaw part REDUCES the north offset (and the -2.8 deg line angle points the same
  way), so it does not explain the wall.
- PDR from 100 s (fixed body walking direction, 15-20 deg off the passage axis):
  3.1-3.5 m south of the map at 112.5-115 s; mixes travel deficit and direction error.

Reviewed reading: a likely under-travel of about 2-4 m along the narrow passage
between the buildings (plan x ~27-29) near 105-112 s, plus a ~7 deg heading mismatch
against the gyro of unresolved cause. Neither is proven. Only one 5 s block is clearly
low. Next decision is the user's.

**User confirmation (2026-11-15):** the user walked through the passage and marked on
`quality/route-localization/passage-overlay-e.png` where the sensor really was:
110 s at about plan (27.7, 10.0), 125 s at about (39.7, 10.0). Map (gyro-deskew KISS):
110 s (27.10, 13.79), 125 s (38.14, 13.51). So the full offset (~3.8 m south) exists
by 110 s: in the passage 100-110 s the map moved ~6.2 m against ~10 m walked, almost
standing still at 105-107.5 s (0.45 m) while the IMU shows steady steps. Along the
covered gallery 110-125 s the map is also ~1 m short (11.0 m against ~12 m). dMapper's
matched-gain control is ~3.3 m north there too (route-audit five-second-blocks), with
`imu_always_on = true` in settings/livox-vertical.toml.

## dMapper in the passage (2026-11-15, Claude; independent Sonnet review)

Control run `livox4-matched-gain-fix-125s/run.log`, table in
`quality/route-localization/dmapper-windows-99-114.txt` (`checkpoint-tools/window_table.py`).
- Windows 287-302 (103.0-108.4 s): weak axis (0.72, 0.68, -0.08) along the passage;
  info ratio 0.15-0.27 (288-290 at 0.19/0.15/0.19, so the projection ramp 0.1-0.2 is
  partly active), 300-302 at 0.37-0.42.
- End speed 1.15 -> 1.01 -> 0.99 -> 0.84 -> 0.60 -> 0.65 -> 0.37 -> 0.25 -> 0.13 m/s
  (windows 286-294) while the user walks steadily; 0.2-0.5 m/s to 108.4 s; 304-310 still
  0.5-0.8 m/s (results 0.14-0.28 m per window). Not recovered before ~111 s.
- Start-velocity jump against the carried velocity: -0.05 to -0.10 m/s in 288-295
  (chi2 92-423, prior sigma along the axis 0.006-0.007 m/s), ~0 in 296-299, positive in
  300-302. Most of the collapse is in the carried guess itself (guess 0.395 -> 0.325 ->
  0.30 m per window), each window ending 0.04-0.05 m below its guess: a compounding decay.
  Implied deceleration 0.4-0.7 m/s^2, which the accelerometer term does not resist.
- Sum over 287-302 of (0.395 - result move) = 3.7 m; including 304-310 gives ~5 m against
  the measured 3.3-3.8 m (user may have slowed/turned near 110 s). Order of magnitude fits.
- NDT profile along the weak axis: lower cost at +-2 m in several windows, but
  `ndt_window_cost` skips points in no map cell, so shifting points off the map lowers the
  cost without a better fit (cause 2 of synchronized-retune 4.8; candidate 2's
  `ndt_unmatched_point_cost` addresses exactly this). Not proof of a wrong pull by itself.
Next (proposed): code diagnosis of what lowers each window below its guess (motion model
tied to the slower guess, NDT with escaping points, accelerometer weight); no fix chosen.

## Code diagnosis and three one-setting variants (2026-11-15, Claude Fable; Sonnet review)

Code (`scan_to_map.cpp`, `run_odometry.cpp`, `ndt_map.cpp`): only the NDT term can move a
window off its guess. The motion model (sigma 1e-4 m) is centred on the guess; the
accelerometer triples see only acceleration (floor 0.1 m/s^2, weak against a false
0.5 m/s^2); the start-velocity prior is overruled (chi2 100-400). The end-velocity
covariance handed to the next window comes from the undamped normal matrix INCLUDING the
NDT terms, so the slowed speed is carried with ~5 mm/s confidence (self-reinforcing).
Default cells: mean = first point, normal fixed from the first 3 points (never updated);
the degeneracy projection uses exactly those normals (sum n n^T).

Variants: the control's frozen binary and TOML with one setting changed, loop closure
and consistency off, folders `livox4-passage-<name>/`, table in
`quality/route-localization/passage-variants-summary.txt` (one run each, no noise floor):

| run | 100->110 s (truth ~10 m) | 110->124 s (truth ~11-12 m) | passage info ratio |
|---|---|---|---|
| control | 5.30 | 11.05 | 0.15-0.27 |
| use_exact_ndt_bucket_statistics | 7.34 | 8.80 | 0.001-0.016 |
| projection 0.3 / 0.5 | 8.93 | 4.84 | as control |
| imu_accel_floor_sigma 0.02 | 5.31 | 30.3 (runaway) | as control |

- No setting fixes passage and gallery together. Exact statistics read the passage as
  ~0.005 against 0.15-0.27 by default: the detector's reading depends on the cell normals
  (likely the noisy first-3-point normal; real oblique surfaces not excluded; direct test:
  the information matrix with exact normals on the same cells).
- With the NDT off along the axis the speed still drifts about -0.1 to -0.25 m/s per
  second (IMU-only carry), later the other way (2.6-2.9 m/s at 124 s, proj and exactstats).
- Raw-IMU test (`imu_only_passage.py`: gyro + slow gravity correction, v0 and yaw fitted
  on 96-103 or 93-102 s): 100->110 s from 0.9 to 29 m depending on gain and fit window.
  This simple model cannot carry the passage; it fits no accelerometer bias or tilt, so it
  does not show the IMU cannot.
- History: point-to-plane NDT (archive 2026-10-11) removed the in-plane pull; 44 ran away
  on the stairwell, its gallery stall shrank. Same trade-off: the in-plane pull prevents
  runaway where the LiDAR has no real information along an axis and causes under-travel in
  a passage.
Direction (no candidate proven): (a) honest degeneracy measure (reliable normals);
(b) along a detected weak axis keep only real information (surfaces facing along it), no
in-plane pull; (c) estimate accelerometer bias and tilt over a span long enough to cover a
passage, so the IMU carry holds; (d) do not hand on a velocity confidence that comes from
the NDT's in-plane terms. The user decides the next step.
