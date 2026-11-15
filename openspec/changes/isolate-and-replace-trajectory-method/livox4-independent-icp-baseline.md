# Independent Livox4 ICP baseline

2026-11-15. User resumed autonomous work with "Start again". Existing authorization
covers required experiments without per-run approval stops. No default adoption,
push, replacement of the legacy method, or claim of a fixed map is made here.

## Why change approach

The bounded checks in `livox4-gain-boundary-cases.md` have not produced a geometry
fix. The actual gain W298 step improves agreement with the historical map, which
may already contain drift. Removing XYZ motion constraints creates nonphysical
motion. A reduced translation-correction basis restores smoothness but does not
improve the measured tilt. Do not turn these diagnostic findings into another
unproven production guard stack or continue arbitrary weight sweeps.

Charlie remains the approved replacement-method roadmap. This external baseline
is a feasibility experiment for its research/design decision, before implementing
an alternative behind the interface. It does not complete legacy extraction,
equivalence, method selection or adoption tasks. Preserve all existing controls.

## Selected reference implementation

Freeze stock KISS-ICP1.3.0, an MIT-licensed LiDAR-only point-to-point ICP method.
Use its standalone Python/C++ implementation and constant-motion deskew, without
dMapper trajectory estimates, NDT map, bias state or covariance carry. This is
an independent odometry baseline, not an isolated numerical modification to LIO.

Primary references reviewed by the research agent:

- https://github.com/PRBonn/kiss-icp
- https://github.com/PRBonn/kiss-icp/blob/main/python/kiss_icp/kiss_icp.py
- https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Preprocessing.cpp
- https://pypi.org/project/kiss-icp/
- https://www.ipb.uni-bonn.de/pdfs/vizzo2023ral.pdf

Published Windows wheels cover CPython through3.13, whereas this machine's active
Python is3.14. Use the existing uv executable to provision an isolated compatible
interpreter/environment under external experiment tooling. Do not replace the
user's Python or change PATH. Record exact package versions, executable/source
identities and dependency artifacts. Preparation is separate from registration.

## Input and output contract

Run folder: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/`
`livox4-kiss-icp-125s/`. Every setting, log, input identity, result and analysis
belongs there. Reuse the verified original-start125s prefix; no event-time cold
start and no recut. Raw recordings remain untouched.

- Read sensor-local XYZ in metres and actual per-return timestamps directly
  from the verified prefix LAZ. Include the initial seed returns; window caches
  alone omit them. Verify units, ordering, count and complete temporal coverage.
- Define fixed-duration Livox frames, initially0.1s for feasibility review.
  These are temporal bins, not rotating-scanner revolutions. Handle empty and
  single-timestamp frames explicitly; stock normalization must not divide by0.
- Before the run freeze range/voxel/frame choices from the existing sensor
  profile and documented method assumptions. No after-the-fact parameter sweep.
- Preserve source-return identity and intensity through filtering/deskew.
  Verify API preprocessing order before exporting. The full cloud uses returned
  deskewed points and end-of-frame poses, not the sparse registration voxel map.
- Save timestamps, transforms and numerical failures. Report processing and
  export time separately. A successful ICP call is not an accuracy verdict.

Worker gravity_candidate owns the isolated environment and runner. Root owns
OpenSpec and comparison decisions. Reviewer seed_damping_crosscheck independently
checks point/time/pose conventions, provenance, filtering identity and comparison
scope before registration. No registration launched at this planning checkpoint.

## Evaluation and next gate

Use one declared rigid transform into the reference coordinate frame, with no
scale or local warping. Prefer the original common sensor pose/gravity alignment;
never fit away the displaced wall. Compare the same marked physical returns and
report any lost coverage. Inspect plan, side and top views plus trajectory and
raw gyro-relative orientation, with stock-method diagnostic limitations explicit.

Apply the agreed quality contract: geometry, revisits, sharpness and coverage
stay separate; odometry-only output is compared with reference odometry. No
dMapper loop closure or consistency is silently applied to this baseline.
Full-route, perturbation, indoor-regression and integration evidence remain
NOT MEASURED. If the prefix is credible, extend the same frozen method/settings
to the full recording before recommending integration or acceptance. If it
fails, preserve the evidence and do not call it a solution.

## Frozen preparation

The isolated environment imports KISS-ICP1.3.0 under CPython3.13.15. The official
Windows wheel SHA256 is
`eff7f8c84555e7e487349b2ef5e41091756f2f42afab0c25c1b0554fffa3a061`.
Dependency versions and wheel hashes are under the run's `tools/` directory.
Raw prefix SHA256:
`49abf5131e010e7fd54ca71fb94d2664ef2b8d62188b25374bf71aee7ced05a9`.

All20,272,168 raw points are finite and have nonzero XYZ. Keep all raw returns,
including491 before the first IMU timestamp and the single time-zero return.
Use1250 equal bins over[0,125.00246995)s, width0.10000197596s; this preregistered
partition avoids a tiny partial final scan. Every bin is populated:6,714-19,254
points, observed durations94.725-100ms, inter-frame gaps5us-4.755ms.

Configuration is max range40m (existing profile's outer limit), stock min0m,
voxel0.4m from max-range/100, eight threads, deskew on; all other KISS defaults
retained. The legacy inner filter is horizontal XY, unlike KISS radial filtering,
so no preprocessing parity is claimed. No IMU deskew or lever arm is applied
(the archived lever arm is zero). Export all deskewed raw returns and an explicit
registration-range mask, preserving raw attributes and source row IDs.

Prepared output contract:

- `result/cloud.laz`: `source_row` uint64, original GPS seconds/intensity,
  `icp_frame` and `icp_range_kept`, coordinates in KISS world metres.
- `result/frame-poses.npz`: `gps_time_s` and `sensor_to_world`, poses at each
  frame's maximum observed timestamp; `frames.csv` retains timing diagnostics.

Native deskew and compressed-LAZ attribute round-trip fixtures passed. The root
comparison independently joined all27,299 marked cached returns to unique raw
LAZ row IDs with exact integer XYZ, timestamp, intensity and user-data keys.
There were no ambiguous duplicates. Cache `index_point` is not used, because
the reader does not initialize it. Evidence: `quality/reference.json` and NPZ.
The comparison aligns at the first common reference timestamp, never at the
later displaced wall. Runtime remains unmeasured; registration has not started
at this preparation checkpoint.

## Coarse baseline completed; one resolution check

Independent runner review cleared source/deskew identity, actual end timestamps
and output verification. Launched2026-11-15 06:30CET. Completed all1250 frames /
20,272,168 points in44.979s including export/readback; last frame clock28.11s.
No warnings; compressed-LAZ source IDs and original attributes verified.

The0.4m voxel result fails the geometry comparison. Under one initial rigid
alignment and the unchanged plan fit, the same marked late wall has median/p90
offset16.106/17.675m and line angle-43.179deg, versus gain-fixed odometry
3.543/4.207m and+12.295deg. All27,299 marked points passed the range filter;
voxel/correspondence membership is NOT MEASURED. The first report's field
`marked_used_for_registration` means only range-filter survival; later reports
use the explicit name. Root inspected the overlay: indoor geometry already
warps, so no full run of this configuration.

Source support provides a concrete resolution concern. Stock0.4m uses0.6m
registration sampling; actual median318 source points, minimum38. Heading
disagreement persists above1deg from29.3s and5deg from29.8s, before the outdoor
event. Around107s median58 registration points remain from~13,920 range-valid
returns. The dMapper profile uses0.03m source decimation and0.15m fine NDT cells.

Independent review supports ONE0.10m voxel comparison, giving0.15m registration
sampling; this matches the fine-cell spacing, not the finer source decimation.
All other settings, timing bins, input, binary and export conventions stay
identical. Dry support estimates are~10times higher; this is not proof of cause.
Evidence: coarse run `tools/voxel-support-and-departure.{json,csv}`.

The fresh `livox4-kiss-icp-fine-125s/` run launched once at~06:35CET, PID11904.
Config SHA256 `d69eff51bc8ada82df6d270a570bb2339b4f0e2372a2ea9352100ad7813d076d`;
only voxel_size changes0.4->0.1. Root comparison reuses the byte-identical frozen
marked-return reference. No other tuning or full run selected. Accuracy pending.

## Fine prefix result and full-route discriminator

The0.10m prefix completed128.630s including verification; last frame111.754s.
Source support median2,552.5 points per frame, minimum456. All20,272,168 raw
IDs/attributes verified. The identical27,299 marked returns survive;23,442 are
in the predefined late interval. Their line angle is-0.794deg, median/p90 offset
3.642/3.758m and height median0.966m. Gain-fixed odometry was+12.295deg,
3.543/4.207m and0.481m. The wall becomes nearly parallel but is still displaced;
some indoor deformation remains. No accepted fix is claimed.

Independent review verified exact cohort identity, unchanged plan fit and sole
initial SE3 alignment at0.35440164s. Similar offsets are compatible with shared
estimation error, plan-frame error or a different physical surface; they do not
prove the user's annotation is wrong or that reflections caused the issue.
The user was asked to identify the marked surface, without recalling timestamps.

One full, uncut0.10m KISS route is selected to test the confirmed return and
revisit geometry before integration. Folder `livox4-kiss-icp-fine-full/`, source
original `raw data/livox/4/lidar0001.laz`. Frozen preflight verifies60,037,017
returns, no invalid XYZ, and all attributes of the first20,272,168 returns
byte-identical to the prefix. All first1251 frame edges are bit-identical;
subsequent edges extend the same period, not a new full-span linspace.
All3,663 bins are retained, including the explicit65.175ms final partial frame
with11,396 points. Same ICP parameters/environment. Prefix numerical agreement
will be measured because parallel reductions need not be bit-identical.

Independent source/preflight review cleared execution; available memory/disk
covered the estimated5-6GiB working set. Launched2026-11-15 06:44CET, PID24504.
Expected end-to-end duration approximately7-10minutes after measured progress.
Raw IMU checks confirm502Hz, no timestamp regressions/gaps/false offsets and
23s quiet at startup. No later qualifying rest block is detected, so the final
time block must not be labeled stationary solely from the same-table report.
Evaluate both raw-supported return geometry and trajectory displacement; no
endpoint-fitting correction is applied to the output. Full result pending.

## Full run completed

All3,663 frames and60,037,017 source rows/attributes verified; exit0, empty
stderr. Last frame processing/export clock521.389s; end-to-end572.063s.
Cloud: `livox4-kiss-icp-fine-full/result/cloud.laz`, SHA256
`c079f11d04d7b83492827f6f8ba37fe159bb3f5151ada486cfff611611b036d1`.
The first1,250 frame times and input/range/voxel counts equal the prefix exactly.
Maximum pose translation difference3.935e-14m and rotation2.316e-13deg: roundoff,
not bitwise identity. Evidence: `tools/shared-prefix-comparison.json`.

Full path length289.643m; first-to-last sensor displacement1.990m, maximum
frame-interval speed4.177m/s. These are trajectory measurements, not proof of
accuracy or a fixed map. The marked prefix wall measurements remain unchanged.
Root inspected full top/side views and the fixed prefix overlay. Full-run
`quality/comparison.json`, `plan-comparison.png`, `overview.png` preserve evidence.
The user can inspect the full cloud; no dMapper production method has changed.

Next existing-data check compares first2-10s room geometry with complete ending
frames in their local sensor coordinates. The user reports a return to the same
table, but the final IMU block is not independently classified stationary. Keep
estimated map placement separate from a diagnostic local registration; retain
source identity, overlap, geometric support and held-out checks, and do not apply
an endpoint correction to the cloud. This determines whether a geometrically
supported loop constraint is available. Questions about the marked physical
surface and outdoor floor level remain pending; no annotation/reflection claim.

## Return geometry and bounded graph test

Existing raw-ID-preserving return samples independently support one room loop:
identity and bias-corrected raw-gyro seeds converge to local relative translation
28–32mm and rotation9.26–9.29deg. Against the accumulated KISS placement, this
requires approximately2.02m and51.2deg correction (total rotation, not pure yaw).
Disjoint-target/source holdout median improves1.29–1.32m to0.171–0.172m;
whole sampled-source median improves~1.18m to~0.0205m. Target coverage differs,
so neither is surveyed accuracy. Adjacent ending frames count as ONE constraint.
KISS-orientation initialization remains in a worse basin. Evidence and scripts:
`livox4-kiss-icp-fine-full/return-check/seeded-return-comparison.json`.
Independent review checked conventions, heldout selection and seed sensitivity.

One offline native graph test used actual KISS keyposes every22frames (~2.2s),
fixed F19 gauge and exact F3660 ending pose, and that single measured edge.
World correction C=Ti*Tfit*inverse(Tj), native measurement=inverse(Ti)*C*Tj.
Archived graph weights were unchanged and explicitly uncalibrated for KISS.
Native no-loop control differs by9.33e-15; known-rigid direction test passes;
marked-point retract/place identity max7.11e-15m. Independent reviewer cleared
the bounded solve with fixed early-room and marked-cohort gates before export.

The trial FAILS those gates. Loop residual2.019m/51.192deg shrinks to
0.00545m/0.11478deg, but maximum pose shift9.492m fails native plausibility.
First60s shifts up to1.271m/8.786deg. Early room2–10s versus15–23s nearest
p50/p90 worsens9.57/25.83mm to43.68/118.12mm; within10cm falls99.87% to83.27%.
Marked wall median offset improves3.642 to1.906m while line angle worsens
-0.794 to-18.179deg. No full corrected cloud was exported or original modified.
Artifacts: `livox4-kiss-return-graph/{native-result.json,quality-gate.json,
trial-frame-poses.npz,sampled-quality-geometry.npz,before-after-preview.png}`.
Closing the endpoint by distributing the large error is insufficient.

## Raw gyro disagreement timeline

Existing-data Python `return-check/trace_rotation_drift.py` integrates body rates
by right SO3 multiplication, with timestamp interpolation at every stored frame
endpoint and no extrapolation beyond the final IMU. Bias uses quiet0–23s;
two disjoint startup halves are sensitivity checks, not confidence intervals.
Gyro is an independent diagnostic, not ground truth. Total KISS/gyro rotation
disagreement exceeds5deg at27.200535s and10deg at33.700665s, before108s.
Final covered disagreement44.569deg; half-bias variants43.904 and45.242deg
with similar first crossings. Several later intervals add or reduce disagreement,
so one isolated108s rotation jump does not explain this independent baseline.
The return-scene correction51.2deg and gyro discrepancy44.6deg are different
references and must not be conflated. Results/CSV/PNG in
`livox4-kiss-icp-fine-full/return-check/rotation-timeline/`.
Independent review verified units, body conventions, recorder shared time epoch
and endpoint pairing. The earliest sharp case F271 at27.200535s has6.094deg
gyro rotation versus9.252deg KISS rotation, with7.645deg relative disagreement
(different rotation axes, so magnitude subtraction is not the discrepancy).
The graph result and preview were also independently crosschecked: no export.

Next bounded comparison: original-start0–35s, fork25s, four branches (control,
gyro prediction rotation only, gyro deskew rotation only, both), external folder
`livox4-kiss-gyro-source-35s/`. Every branch must reproduce archived poses and
counts through the fork, and control through35s. Preserve stock configuration,
translation prediction, adaptive threshold machinery, sampling and map update.
For the deskew branch use per-return Rgyro(end)^-1*Rgyro(t), preserving the exact
stock per-time translation curve obtained from native preprocessing of zeros.
This separates measured rotation from the SE3 translation interpolation; merely
replacing delta's endpoint rotation would alter both. Prediction rotation uses
gyro between successive observed frame ends, retaining original delta translation.
Stock ICP still freely estimates pose; this is no hard gyro constraint.
Compare identical heldout raw IDs against fixed stationary-room geometry as well
as gyro disagreement and source support. Changed map/threshold after the fork are
downstream effects, not separately isolated factors. Script and synthetic checks
require independent review before execution. No full replay or integration chosen.
