# Livox3 house-plan registration audit

2026-11-13. User authorized High-level research and registration analysis, not
odometry/settings changes or another processing run. The original ambiguous
quality-overlay artifacts are preserved. This work reads the completed Livox3
final cloud and the supplied Haus47 LG plan only.

## What was wrong with the first overlay

The original transform used only stationary 0-15 s returns and a partial Wohnen
wall mask. That support does not determine a unique structural alignment: the
near-180-degree alternative had only 4.36% larger truncated cost. Room furniture,
door frames, limited visibility and repeated parallel walls can attract this fit.
The automatic best-ranked transform was therefore not a confirmed house tie.

Broader existing-data correspondence inspection gives a different orientation.
A fixed-scale rigid trial using manually traced inner faces of the western/northern/
eastern apartment envelope and Bad south changes the transform from
`[22.254087845, 22.151655830, 13.683239830]` to
`[19.326900243, 22.246497705, 13.654176396]` (degrees, plan metres XY).
The approximately 2.93-degree rotation difference is enough to create about
0.51 m transverse displacement across a 10 m lever arm. Thus a substantial
systematic mismatch can arise from registration alone; it is not evidence that
those cloud points moved by that amount during mapping.

This new transform is a **preliminary structural candidate**, not an accepted
registration or an absolute accuracy result. It improves several long-wall
directions but does not reconcile every room/stair surface. Some fixed wide
correspondence regions have little reliable wall support and contain furniture,
window/ceiling returns or another face. They must not become survey residuals.

## Bounded method and held-out evidence

- Use only a fixed world-Z slice, strictly +0.45 to +0.55 m. No scale fit,
  reflection, affine warp, point deletion or per-visit alignment.
- Fit top-floor walking observations at 20-120 s; use 25 mm XY voxels and equal
  per-surface leverage. Inner wall faces are manually read from the raster;
  text, dimension glyphs and door arcs are not target geometry. Fixed 0.8 m
  source-selection regions are declared and not tightened to hide errors.
- Robust rigid least squares uses a declared 0.08 m Cauchy scale. This is a
  registration outlier choice, not sensor-noise calibration.
- Southern apartment faces, the internal partition/storage and elevator faces
  are withheld from optimization. The 215-245 s return interval is also withheld.
  Every panel/time uses the same selected transform.
- Broad-region signed median offsets are descriptive. For example, the withheld
  elevator-east first-visit offset changes from -0.378 to -0.088 m and the
  south-Schlafen first-visit offset from +0.259 to +0.079 m. Other regions worsen
  or lack interpretable support. A single registration therefore has not been
  shown to explain all mismatch. No pooled score or global PASS is claimed.

## Coordinate, plan and section limits

The raster mapping uses x right and y up, with image row `height - y*ppm`.
The point transform has determinant +1 and retains the original cloud scale.
No sign/handedness inversion was found in this mapping path. The old transform
was not reused as final truth; it only defines declared broad source regions.
That conditional correspondence choice does not independently resolve all
possible 180-degree registrations. Named asymmetric room/stair topology is more
informative than the original partial-room score, but still requires visual
physical correspondence confirmation.

The nominal 1:50 conversion is 118.110236 pixels/m at 150 dpi. No cloud scale
was optimized. Existing approximate full-sheet dimension sanity evidence is
not a calibrated scan-distortion model; this audit establishes no global scale
fault and does not silently compensate possible paper/as-built differences.

The plan explicitly includes attic/sloped-ceiling geometry (Dachschraege,
Kniestock, ceiling-height notes). A constant world-Z section is not a surveyed
floor-plan section: it can intersect sloping ceilings/windows/furniture at
positions different from floor-outline walls. The actual sensor/floor height
and as-built wall correspondence have not been surveyed. Known Schlafen
mirror/window projections remain; they are not automatically odometry errors.

## Artifacts and stopping point

External directory:
`BASE/livox3-indoor/huber3-baseline/registration-audit/`

- `registration-before-after.png`: original versus broader structural trial;
  green faces fitted, pink dashed faces withheld; common scale and view.
- `registration.json`: transforms, fixed source regions, fit/held-out per-surface
  readings and cloud/plan/script identities.
- `height-slice.npz`: unchanged selected actual merged returns for repeatable
  analysis; not a modified map.
- `livox3_registration_audit.py`: archived analysis source.

The existing result and previous overlays remain untouched. No optimizer,
mapper, processing settings or production numerical code changed.

The useful next decision is physical correspondence confirmation, not further
automatic fitting: confirm two distinct visible structural corners on both the
plan and cloud (for example an apartment entrance jamb and an elevator/shaft
corner), and whether this attic plan depicts their current as-built faces.
That would constrain the rigid tie independently of mixed room-slice support.
Without it, show the candidate and remaining mismatch rather than force fit.
