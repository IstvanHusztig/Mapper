# Full Livox2 Huber3 replay — 2026-11-13

The fresh full replay finished successfully, but this is not yet an accepted
full-route correction. The former large downward excursion is absent in the
candidate odometry; a later speed escalation and substantial postprocessing
changes remain. The user has the new cloud for inspection. No settings were
shipped and no additional mapper replay was started during analysis.

## Execution and evidence

The frozen conditions and build/test provenance are in
`full-livox2-huber-replay.md`. Native processing time was 4076.10 s. Both stages
contain 273,060 poses across 1,517 windows. All 82,688,088 exported returns have
the same ordered GPS-time and intensity fields as the retained control.
Candidate XYZ changes are expected. All 116 focused windows, 11,600 iterations
and 23,200 assembly checks passed; no assembly mismatches. Trace SHA256:
`33aaa21d15feba3fe9662741fc706d2786ca6b901a57fcaf3d178af917def604`.

The original runner stopped its collection gate because window 798 (286.92 s)
prints information ratio 0.450 without a conditional chi-square row. The native
cutoff uses the unrounded ratio <0.45. This window is outside the focused trace,
so exact eligibility is unresolved. This is not evidence of a mapper failure,
nor permission to call the uncertainty measured. The strict health exit remains
2 and `run.json` remains `needs_review`, unchanged.

An independently reviewed offline collector preserved that record and completed
raw/build/settings/control identities, snapshot/log correspondence, exact
within-stage timestamp comparisons, return-sequence checks and output hashes.
Its separate `reviewed-collection.json` has `artifact_integrity_complete=true`
and `comparison_complete=true`, but `evidence_complete=false` and
`diagnostics_complete=false`. It reuses the successful trace validation only
after verifying the pinned record and exact trace bytes/hash. No threshold was
relaxed and the mapper was not rerun. Observational analyses explicitly retain
the gap. Missing evidence is not a pass.

## Descriptive full-run comparison

| Measure | Retained control | Full Huber3 |
|---|---:|---:|
| Windows with logged endpoint speed >2.5 m/s | 93 | 12 |
| Largest logged endpoint speed | 7.647 m/s | 5.154 m/s |
| Yaw correction >3 degrees | 84 | 22 |
| Yaw correction >10 degrees | 14 | 1 |
| Odometry minimum trajectory Z | -49.289 m | -7.872 m |
| Final minimum trajectory Z | -49.076 m | -3.615 m |
| Final maximum trajectory Z | 0.826 m | 5.686 m |
| Largest logged loop-closure correction | 2.56 m | 10.46 m |
| Conditional uncertainty rows | 831/1517 | 810/1517; W798 unresolved |

These are diagnostics and estimated trajectories, not surveyed position/height
errors. Fewer triggers do not establish map accuracy or robustness. Candidate
raw conditional chi-square median/p95 are 89/1121; maximum residual gyro-bias
estimates are 0.29/0.34/1.19 deg/s. The candidate still triggers these checks.

## Later escalation and evidence boundary

The candidate crosses 2.5 m/s at W904 (325.08 s window-start time), after speed
has already risen in earlier windows. W915 peaks at approximately 5.154 m/s.
The map was rebuilt at W914. W916 discards the previous pose-derived velocity
(5.13281 m/s) through the existing speed guard and uses a wider velocity prior;
its subsequent lower speed cannot be called spontaneous stabilization.
Small prediction-to-result corrections during growth are compatible with
inherited velocity/acceleration propagation, not proof of physically wrong
motion or proof of which factor initiated it.

Existing focused traces end at W885. Their last traced endpoint tilt change is
0.0655 degrees relative to that window's incoming prediction and gravity;
this does not establish absolute attitude accuracy. Snapshot v1 lacks per-window
gravity, acceleration scale and offset, and its model poses are overwritten.
The motion diagnostic therefore reports gravity-dependent calculations after
W885 as NOT MEASURED. It preserves the distinction between final optimizer
velocity states and post-handoff snapshot poses.

Requested Terra 6.1 research was used for this new clue. It recommends checking
weak-direction carry, accumulated attitude/acceleration and independently
observable local geometry rather than assuming a repeat of W754. It specifically
warns that weak-axis ratios are not complete observability, matched-only robust
cost profiles are not the full accepted-step objective, and recovery after a
guard is not independent evidence of stability. No new mapper run was requested
by that research.

## Artifacts and current follow-up

All artifacts are under
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3/`:

- `result/scan_lio_merged.laz`: new complete cloud supplied to the user.
- `views/actual-merged-clouds.png`: actual exported clouds, shared axes, all
  returns represented in bounds/counts; deterministic display thinning.
- `views/trajectory-odometry.png` and `trajectory-final.png`: separate stages.
- `views/first60-frozen-plan-overlay.png`: inherited apartment-56 alignment,
  no candidate fit; nominal plan, supplementary slice only.
- `views/summary.json`, `motion-830-950/motion.json`: verified inputs, coverage,
  numerical diagnostics and explicit missing-evidence limits.
- `reviewed-collection.json`, `reviewed-run-health.txt`, `analysis_scripts/`:
  supplemental verification and preserved analysis source.

## First-minute matched geometry

The user-marked Schlafen mirror/window projections remain distinct from drift;
none was deleted or automatically scored as physical error. The archived patch
functions were reused with a streaming, source-matched wrapper. It selected
9,636,890 first-minute returns, froze 117 reference patches before candidate XYZ
analysis, and retained every corresponding candidate return regardless of its
displacement. There are 221,377 unique patch returns out of 240,261 in the
reference Z=0.50 +/-0.05 m slice: 92.14% of that slice, only about 2.30% of
first-minute returns. This is not full-route coverage.

| Exploratory final-cloud measure, p50/p90 | Control | Huber3 |
|---|---:|---:|
| Side-wise XY-PCA orthogonal MAD across patches | 31.58/83.36 mm | 20.48/80.70 mm |
| Frozen-normal centered MAD across patches | 33.66/95.23 mm | 27.95/95.09 mm |
| 115 same-side visit-pair differences, 66 patches | 31.37/115.87 mm | 21.32/162.02 mm |

Readings are mixed. All source returns remain, including 18 candidate patches
entirely outside their frozen 0.2 m reference band (62 with at least some band
exit). The side-wise PCA fits local XY line orientation; it is not a 3D planar
wall-thickness test and cannot be judged against that yardstick. Reference
patches can mix or omit geometry, and pooled visit pairs are correlated with
unequal patch weights. No perturbation envelope has been measured for Huber3;
these readings do not establish a resolved improvement or regression.

## Separate postprocessing issue

Independent reading of original chunk-local CSV positions reproduces final
trajectory arclength 892.159 m versus odometry 334.276 m. Only 0.091 m comes from
chunk boundaries. The extra variation is principally within windows, not a
chunk-transform/parser mistake or a few large seams. The largest final adjacent
step is 0.03593 m near 265.198 s. This arclength is not physical travel distance.
For a descriptive check, taking every 50th final pose gives 336.820 m, versus
332.171 m for odometry; this is not a proposed smoothing fix or accuracy metric.

Both loop closure and ten consistency passes ran, with no intermediate outputs.
Requested Terra 6.1 source research identified possible consistency mechanisms:
individual-node NDT forces, fixed anisotropic increment smoothness without time
normalization, and repeated full solved updates without objective acceptance.
The optional gyro rotation term is disabled in the retained settings. Loop
closure interpolates corrections between segment anchors. These source facts
make consistency a candidate, not a demonstrated cause; the retained outputs
cannot separate the stages.

Independent Codex evidence review (not Sonnet) verified the motion, export and
patch calculations. It added the W914 map rebuild, confirmed all 112 final
traced velocity vectors against the snapshot, narrowed the coverage claim to
the selected slice, and prevented interpreting the 892 m as physical travel or
the patch measures as a 3D wall-thickness pass.

The bounded postprocessing-only replay has now reproduced all 64 numeric final
files exactly. Intermediate captures attribute the large global/vertical
adjustment to loop closure and progressive fine-scale variation to consistency.
See `full-livox2-postprocess-attribution.md`. Native exports of the saved earlier
stages are next, with no optimization or new full odometry run, so actual cloud
geometry can be compared before choosing a correction.

The user has been asked where the complete recording ended (starting apartment,
elsewhere inside, or outside), without asking for timestamp recollection.
Endpoint truth, full-route geometry and perturbation robustness remain unresolved.
High remains useful for a later correction-design decision; Medium is sufficient
for the current exports/measurements and the user has been notified. The
continuing authorization does not imply candidate acceptance or push.
