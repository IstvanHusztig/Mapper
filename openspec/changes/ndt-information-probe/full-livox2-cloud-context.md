# Full Livox2: scene around the flyaway

User said they were probably outside and explicitly requested using Python to
inspect the surrounding cloud, rather than relying on recall of a timestamp.
They also authorized crafting reusable tools for difficult/repeated analysis.
This follows full-livox2-jump-diagnosis.md. No new SLAM run, build, numerical
change or candidate acceptance occurred.

## Method and provenance

New reusable tool: tools/cloud_context.py. It streams raw XYZ range statistics
and exports selected final-cloud intervals using existing trajectory/MRP loaders.
It accepts explicit raw timestamp units, extraction intervals, optional panel
intervals and event markers. It refuses a nonempty output directory. No factory
point calibration is reapplied. Original data and results are unchanged.

External artifact directory:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-motion-prior/cloud-context/`.

All 89,002,885 raw returns were inspected; timestamps had zero reversals. Raw
gps_time is seconds, exported gps_time is nanoseconds. Both were converted to
seconds from the first trajectory node at sensor time 0.38408168s. Raw coordinates
are sensor-local; configured lever arm is zero. Chunk output is already deskewed
and chunk-local; each session_poses.mrp transform was applied exactly once.
The merged cloud was not transformed again. Source semantics independently checked.

Raw range is Euclidean XYZ norm, excluding nonfinite/<=0.05m returns. Per-second
quantiles use 0.25m histogram bins and report their lower edge, not exact values.
Percentages below are weighted by the number of recorded valid returns. They
are not emitted-pulse coverage, scan-matcher inlier counts or visibility of sky.
Visual samples retain every twentieth selected return per read block.

Raw 0.2s snapshots are deliberately not deskewed or placed with an estimated
trajectory: they show the observed local scene without the flyaway transform.
Their sensor axes are not necessarily world vertical. Motion within 0.2s can
blur surfaces. Final-world views inherit the failed trajectory and cannot prove
the undistorted physical shape.

provenance.json binds the raw recording (hash matches frozen run.json), selected
source chunks, all trajectory CSVs, MRP, helper and output artifacts. manifest.json
preserves the initial extraction identities. The archived cloud_context.py is
the executed first version; root subsequently generalized its panel/event
options and added integrated input hashing, without changing extraction maths.
The new CLI was checked with a tiny synthetic one-panel fixture: 100 points,
single known MRP translation, selected output count and input manifest verified.
Four full-density raw frames were reread after writing and matched original
structured point records exactly, including timestamps and extra fields.

## Measured change in surroundings

| Relative interval | Valid raw returns | Within 5m | Beyond 15m | Context |
|---|---:|---:|---:|---|
| 240–250s |1,222,136|80.41%|6.78%|Substantial more-distant structure visible |
| 255–260s |654,148|88.82%|3.90%|Mix of nearby surfaces and farther structure |
| 265–268s |425,709|99.989%|0%|Each second's 90th-percentile range lies in [1.50,1.75)m |
| 268–271s |462,589|98.34%|0%|Predominantly near surfaces, changing visible geometry |
| 271–274s |500,244|96.73%|0.57%|Sustained estimated-speed growth begins in this interval |
| 300–307s |1,085,272|97.57%|0.00028%|Another near-scene episode before the second escalation |
| 307–313s |928,716|98.68%|0.109%|Changing geometry during the second speed escalation |

The raw 266s and 305s snapshots show a roughly rectangular arrangement of nearby
surfaces in the sensor-YZ projection. More extended surfaces appear at 269s and
311s. This pattern exists without applying any SLAM pose, so it is not created
by the estimated flyaway. The final-cloud panels show those nearby structures
being spread along the estimated path during rapid-motion intervals.

Supported interpretation: nearby enclosing or obscuring surfaces and a change
in visible structure deserve investigation. This is stronger support for a
close-scene transition than for simply entering unobstructed open space and
losing all nearby returns. It does NOT identify a particular room, passage,
elevator, doorway, indoor/outdoor location or moving object. Being outside near
walls/cover or with a partly obstructed view remains compatible with the data.
Sensor field of view and the no-return directions are additional limits.

The scene observations narrow the next diagnostic question to association and
geometrical-constraint changes around these transitions. They do not establish
that the scene caused the failure, nor prove that all available returns were
used successfully by NDT. Existing ordinary diagnostics cannot reveal the
per-iteration selected matches, joint factor balance or initial wrong update.
The previously proposed diagnostic replay remains unapproved and was not run.

## Views and CloudCompare extracts

- raw-range-context.png: raw range, nearby/far return shares and recorded counts.
- raw-near-scene-3d.png and raw-near-projections.png: eight/four brief sensor-frame
  views, including the two close-scene episodes; short-view limits stated above.
- local-cloud-panels.png: same-scale, sensor-centred top/side final-cloud views.
- final_context_240_285s.laz: 6,370,830 points, world coordinates, first episode.
- final_context_300_320s.laz: 3,048,487 points, world coordinates, second episode.
- raw_sensor_frame_266_266.2s.laz: 28,510 original sensor-frame points.
- raw_sensor_frame_269_269.2s.laz: 30,845 original sensor-frame points.
- raw_sensor_frame_305_305.2s.laz: 27,749 original sensor-frame points.
- raw_sensor_frame_311_311.2s.laz: 30,707 original sensor-frame points.

Open raw frames separately: their sensor origins coincide by definition and
they are not registered world maps. The final-context extracts are already in
the full result's world frame and can be displayed alongside it. Neither kind
of extract is a newly improved SLAM result.

## Independent review

Codex reviewer (not Sonnet) independently checked coordinate/time handling and
viewed the raw plots. Its interpretation supports nearby enclosing/obscuring
surfaces while rejecting specific location claims. It corrected the count-
weighted 300–307s near-return share to 97.565%, incorporated above. Tool review
also prompted configurable markers/panels, one-panel handling, and supplemental
input provenance. All analysis used retained data; no new acquisition is needed
to inspect these extracts. High remains useful for subsequent causal work.
