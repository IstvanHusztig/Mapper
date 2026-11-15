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

### Bounded rotation-source comparison completed

Independent review cleared the exact translation-preserving deskew implementation;
native synthetic reconstruction error8.88e-16m. All350 stock poses reproduce
archive within2.67e-15, with identical registration/range counts. Each candidate
reproduces all249 pre-fork poses/counts. All four score the same89,505 raw rows.
Sequential processing costs10.01/9.90/7.65/6.80s (control/prediction/deskew/both),
excluding shared loading/preparation. No full cloud export or production change.

At F271 on the exact same pre-update map and threshold, gyro disagreement is
7.645/7.659/3.424/3.268deg. At35s, fork-relative disagreement is
15.267/15.306/11.077/9.738deg. Prediction alone changes little; measured rotational
deskew has the principal local effect. This is not a map fix: whole25–35s
heldout-room median distances are0.4305/0.4300/0.4366/0.4384m, p90
2.537/2.531/2.492/2.480m. Leaving the reference room affects coverage, so mixed
nearest-distance statistics cannot rank full-route quality. Fixed-section plots
also do not show a decisive geometry improvement. Evidence: run `result/summary.json`,
`paired-audit.json`, `geometry-comparison.png`; review checked timing/conventions.

The exact F271 state is now persisted once in `fixed-map-case/frozen-F271.npz`,
SHA25612aaf1f8af390d7a64d7d0fe3034259bf3546b7a78450bb167515fb2b7378a6f.
Preroll271 frames matches archived poses within6.37e-16 and counts exactly;
map162,356 points, raw frame18,920 returns, stock/gyro sources3,861/3,521;
sigma1.5566417021. It includes poses, source IDs, gyro rotations and preprocessing
arrays. No F271 registration or state mutation occurred during this capture.

### Check search support before another solver implementation

Terra's primary-source review identified a coupling in the resolution test:
[KISS1.3 map search](https://raw.githubusercontent.com/PRBonn/kiss-icp/v1.3.0/cpp/kiss_icp/core/VoxelHashMap.cpp)
uses only containing+26 adjacent cells, while
[pipeline voxelization](https://raw.githubusercontent.com/PRBonn/kiss-icp/v1.3.0/python/kiss_icp/kiss_icp.py)
derives source sampling from the same setting. Refining0.4 to0.1m increased
source support but also reduced map search reach. With0.1m cells the cube cannot
contain neighbors farther than~0.346m, even though current adaptive cutoff is
~4.670m; it is not a radial0.1m cutoff. This is a hypothesis, not a proven cause.

Before building translation-only ICP, perform a read-only exact frozen-map audit
of closest points inside the27 cells versus global nearest points, preserving
strict cutoff and separating no-candidate from worse-candidate cases. If material,
one API-only bounded35s comparison can use0.4m map cells while keeping source
voxelization0.15m/frame downsample0.05m and other behavior unchanged. This would
also change capacity, representative spacing and pruning, not only search reach;
judge independent geometry and support. No such candidate replay launched yet.

Frozen support audit found498/3,861 stock-guess points without a local candidate,
versus52/3,521 for gyro deskew+prediction; at fitted poses100/3,861 and44/3,521.
Global nearest points all pass the same4.670m cutoff; no locally found point
fails that cutoff. Missing stock-guess nearest distances have median0.274m and
p900.452m: larger search admits farther, not necessarily correct, correspondences.
Map bucket membership/order was checked by reinserting into the actual1.3 wheel;
no1e-12 ties. Per-query exact positions/indices and distributions are preserved
in `fixed-map-case/search-support/`. Independent review reproduced counts and
virtual0.4m rebucketing recovers3,858/3,861 stock-guess candidates on identical
points. This does not explain all gyro-assisted residual rotation.

Reviewer supports one practical35s map-scale candidate before implementing a
new ICP solver. Separate folder `livox4-kiss-map-scale-35s/`: native empty map
voxel0.4m from frame zero, configuration voxel0.1m retained for fine source
sampling; stock deskew and prediction. Preserve all other settings and timestamp
bins. Compare the same89,505 source IDs, near-room geometry/coverage, angular
behavior and motion, recording actual map occupancy/capacity effects. Thin runner
requires review before launch; full-route continuation requires credible geometry
and angular evidence together. This is an experimental API override, not stock
configuration or dMapper integration. Current raw files/control are unchanged.

### Wider map and fixed-rotation results: neither promoted

The reviewed0.4m-map35s candidate completed. F271 initial support rises from
3,363/3,861 to3,870/3,873, all3,873 at fitted pose; map points fall162,356 to11,208.
Angular disagreement7.645 to7.252deg locally and15.267 to10.579deg at35s does
not have a corresponding geometry gain. Identical89,505 rows: heldout median
0.4305 to0.4315m, p902.537 to2.603m. Near-room25–27s essentially unchanged;
27–30s heldout p900.53959 to0.54318m. Reviewer confirms joint gate fails; no
full replay/export. Search support alone was insufficient. No map-size sweep.

One already reviewed offline F271 test then fixed rotation to the measured gyro
increment and optimized only translation on the same gyro-deskewed source/map.
Rehydrated official native endpoint agrees with archive to3.3e-16m/3.8e-15deg;
Python six-DOF reproduces native1.9e-15m/1.1e-14deg and exact selected rows.
Same27-cell search, strict cutoff, GM IRLS and left-SE3 updates; fixed rotation
is enforced during solving, not clamped afterward. Fixed versus free result:
support3,468 versus3,477/3,521; all-source median/p9034.6/85.9mm versus16.8/38.3mm;
speed0.962 versus0.907m/s. On15,399 withheld raw nonrepresentatives, heldout-room
median0.2016 versus0.1998m; p900.6109 versus0.6347m; within10cm29.13% versus32.24%.
The mixed tail does not outweigh worse central fit/support. No hard gyro lock
selected. Case evidence: `fixed-map-case/rotation-constraint/comparison.json`.

### Lag sensitivity separates constant-motion deskew from clock hypotheses

Root `return-check/check_rotation_timing.py` compares stored LiDAR angular
increments with the original raw gyro over shifts[-200,+200]ms. Full moving
intervals favor-50 to-70ms across all five temporal blocks; aggregate median
disagreement1.499 to1.120deg at-60ms. This is estimate-based sensitivity, NOT
sensor clock calibration. Positive lag means IMU evaluated later than LiDAR.

Crucially, identical25–35s intervals from the four controlled branches favor
-50ms/-60ms for stock/prediction-only, but ZERO for measured rotational deskew
and both. Zero-lag median per-frame disagreement1.412/1.352/0.211/0.199deg.
Independent review confirms conventions and supports deskew-dependent effective
lag rather than a recorder offset claim. Since gyro enters those corrected
estimates, this remains a mechanism check, not independent ground truth.
Preserve original timestamps; no retiming or bias adjustment. Evidence:
`rotation-timeline/timing-sensitivity.json`, `timing-branches.json`.

### Gyro-deskew-only125s geometry coverage test

Independent review supports ONE expanded prefix to obtain geometry coverage
beyond the35s room departure, not to declare that the prior joint gate passed.
Run folder `livox4-kiss-gyro-deskew-125s/`, .1m map and stock prediction,
registration and adaptation. Only per-return rotational deskew changes.
F0 retains stock identity deskew because points begin before first IMU .00247s;
F1 first gyro query .100005s is covered. Original full IMU supplies the final
bracket [125.0005489,125.00272627] for last point125.00166s; its prefix bytes and
startup0–23s bias match the verified input. No extrapolation, retiming or dropped
leading points. All20,272,168 raw rows/attributes must survive compressed export.

Before candidate launch, root froze `livox4-kiss-icp-fine-125s/quality/
frozen-indoor-revisits.{npz,json}` with control-only plan boxes, same-side camera
location and source-ID modulo19. South region has2,243 static2–10s and3,816
later25–60s points. Static-only plane support62.2% within30mm; all broad-box
points remain in scoring. Mixed window/surface attribution prevents surveyed-wall
accuracy claims. Partition/north lack static coverage and are NOT MEASURED.
The existing27,299 marked IDs and exact old rigid/plan alignment remain frozen.
Reviewer accepts these plus the overview as a bounded diagnostic, not whole
indoor preservation or adoption. Candidate positions must not reselect cohorts.

Runner and freeze independently reviewed; launched once PID32420. Estimated
2–4min including export/readback, no full route. Root evaluates fixed marked
direction/offset/height/coverage and frozen-room distributions after completion.
This remains an external experimental KISS-derived method; no dMapper algorithm
or user settings switch. Wider-map and hard-rotation branches stay unpromoted.

The125s run completed118.763s including export/readback; all1,250 frame boundaries
and20,272,168 raw IDs/attributes verified. F0 pose bitwise equals stock. Cloud
SHA2564506b2c0a4fbe87f435135f4cd35874370071523a6c2c99226f1b59399f78d73,
`livox4-kiss-gyro-deskew-125s/result/cloud.laz`. Native range total20,250,275;
per-frame range membership can change with deskew and was verified explicitly.

Same marked27,299 returns: late offset p50/p903.343/3.535m versus stock fine
3.642/3.758m; line angle-2.828 versus-0.794deg; median height0.525 versus0.966m.
The frozen mixed south cohort25–60s abs median/p9044.5/150.4mm versus43.4/148.6mm,
spread62.3 versus70.1mm, within10cm67.95% versus70.07%. For40–60s coverage falls
76.90% to72.33%; early25–30s median improves28.3 to22.9mm. All15,369 frozen indoor
IDs retained; north/partition static comparison remains NOT MEASURED. Root viewed
overlay; reviewer crosschecked trade-offs. No joint whole-map pass or full-route
gyro-deskew replay selected. Figure caption corrected to identify the gyro-deskew
experiment, not stock KISS; initial report retained as `comparison.initial-label.json`.

User feedback after these results: **"Thats a good result"**. Preserve this
candidate and positive feedback; it does not constitute a default-adoption or
fixed-map claim. Root requested the physical identity/level of the marked strip
again because that reference remains uncertain. Existing-data work continues:
four predefined complete raw frames F400/F450/F500/F550 against the static room,
with gyro/stock rotation seeds, held-out coverage and all outcomes preserved,
to seek a second non-endpoint geometry constraint. No new odometry is needed.

All four predefined queries failed to support an initial-room loop. Best10cm
whole/heldout overlap: F40025.1%/9.5%, F45017.1%/7.3%, F500zero with both seeds,
F5509.6%/3.0%. All eight outcomes and raw-source identities preserved in
`livox4-kiss-nonendpoint-revisits/`. No reverse test triggered and no edge added.
Independent review confirms this rejects these specific queries, not existence
of all non-endpoint revisits. Further discovery needs independent scene retrieval
or a physically identified repeated region, not additional ad hoc timestamps.
Terra assessed MapClosures2.2: pinned WindowsCP313 wheel exists but dependencies
conflict with the frozen KISS environment, requiring a separate environment.
Default0.5m BEV pixels and31-pixel ORB border leave images under~31m with no
usable interior; do not blindly process small indoor blocks. Internal ground
alignment, source-to-query transform direction, 2D-only inliers and multifloor
aliases require explicit pilot/3D verification. No install/retrieval selected.
