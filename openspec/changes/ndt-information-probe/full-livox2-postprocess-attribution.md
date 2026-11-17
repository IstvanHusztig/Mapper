# Separate loop closure from consistency on retained Livox2 odometry

The full Huber3 result removes the previous approximately 49 m downward
odometry excursion, but does not establish a usable full-route map. Independent
analysis also found fine-scale final trajectory variation absent from odometry.
Current artifacts cannot say which postprocessing stage introduced it.

The user's standing instruction is to continue toward the goal until new
information is required. This bounded diagnostic continues that authorization;
it does not authorize a shipped setting, accuracy acceptance or push. The user
has been told that saved odometry will be reused and High remains useful for
interpretation. No further full odometry run is part of this check.

## Exact control first

- Use the original Livox2 input, unchanged effective Huber3-run settings, pinned
  odometry snapshot, source revision and experimental alpha/beta/Huber identity.
- Prepare a standalone diagnostic driver in the isolated scratch worktree.
  Preserve the normal CLI's experimental snapshot refusal. The diagnostic
  driver must explicitly validate settings hash, recording timestamps and the
  externally pinned snapshot/experimental provenance.
- Load/restore, then execute the normal loop-closure and consistency algorithms
  in their original order with unchanged arithmetic and ten consistency passes.
  Capture poses/state after loop closure and each consistency pass using
  observational hooks. No numerical ablation or tuning in this first replay.
- Require exact retained final trajectory and merged-cloud reproduction before
  treating new intermediate states as attribution for the existing full result.
  Preserve mismatch/failure artifacts and investigate rather than rerun silently.
- Build/test appropriate configurations, independently review hooks/runner,
  freeze source/executable/dependencies and inputs before executing. One process
  owns the original recording/cache; no executable rebuild while it runs.

## Analysis and preservation

Keep results under the existing external experiment root in the fresh
`full-livox2-huber3/postprocess-attribution/` directory. Preserve the original
`run.json` and `reviewed-collection.json`, including W798's unresolved conditional
uncertainty. This check cannot fill that odometry diagnostic gap.

Compare identical timestamped stage poses for arclength, adjacent and second
differences, local orientation increments, boundaries and spatial/time coverage.
Distinguish smooth large corrections from fine-scale oscillation. Export useful
stage clouds only as needed to inspect the implicated geometry; do not hide
unmatched/displaced surfaces or align each stage independently. Identify whether
variation first appears after loop closure or within a consistency pass before
proposing a correction. Any subsequent ablation must be explicitly documented
and independently reviewed; never infer physical truth from numerical parity.

## Reviewed preparation and launch

The standalone driver changes only a new diagnostic source and an
`EXCLUDE_FROM_ALL` CMake target. It uses the existing main-thread progress and
logger boundaries to capture after loop closure and each completed consistency
pass. No core numerical hook or normal CLI guard changed. Its fresh point store
and no-op preview sink are installed before loading, preserving the original
recording cache/preview. A newly serialized restored snapshot must equal the
pinned input byte-for-byte before postprocessing begins.

Scratch commit `f18353edb0bd491e985a36ee6f9eb50b49f29192`, parent `cbca36d`.
Release driver SHA256:
`b8bcde8ad91f78320226fc1bbfe5cdbbfbb56804f274596599971be18d2c85fa`.
Runner SHA256:
`f1786e50fd169706c64d8170070449d05670160f8ca3b2086fe8dc1b7cd10479`.
Release and Debug observer selftests passed, including thread ownership,
ordering, disabled capture and capture-failure handling. Twelve runner tests
passed. Independent Codex source/runner review (not Sonnet) cleared execution.
The frozen bundle covers twelve files; runner/tests are separately bound by
`preparation-manifest.json`.

The single unchanged postprocessing replay completed on November 13 in
1127.06 s, exit 0. `postprocess-attribution/run.json` reports
`exact_postprocess_reproduction`: all 64 numeric final files reproduce exactly,
and all twelve stage snapshots preserve settings and exact timestamp/window
coverage. The restored snapshot also matches the original byte-for-byte before
postprocessing. Original inputs/cache/preview and retained outputs are unchanged.
No odometry optimizer ran. The inherited W798 diagnostic gap remains unresolved.

## Result: the two stages have different effects

| Saved stage | Dense trajectory arclength | Stride-50 arclength, all 50 offsets |
|---|---:|---:|
| Restored odometry | 334.276 m | 332.153â€“332.171 m |
| After loop closure | 333.228 m | 331.102â€“331.120 m |
| After consistency pass 1 | 408.553 m | 331.587â€“331.948 m |
| After consistency pass 5 | 674.183 m | 333.601â€“334.562 m |
| After consistency pass 10 | 892.159 m | 335.674â€“337.671 m |

Loop closure causes the large global/vertical adjustment: maximum 3D change
10.4697 m and maximum absolute Z change 5.3631 m. Trajectory Z changes from
[-7.8724, 0.8465] m to [-3.6127, 5.6827] m. It does not cause the subsequent
dense-arclength inflation.

The consistency passes progressively introduce fine-scale variation. Across
all ten passes, the maximum position change from the after-loop-closure state
is 0.3310 m, median 0.0300 m; maximum absolute Z change is 0.1113 m. Dense
arclength rises on every pass, while all coarse offsets remain much closer.
The maximum final adjacent-position derivative reaches 23.09 m/s; this is not
the optimizer velocity state or physical speed. Different coarse offsets omit
different endpoint tails; their envelope is descriptive, not a noise floor.

Independent Codex review (not Sonnet) checked exact execution gates and used
a separate binary snapshot parser to reproduce these readings. This attributes
the observed changes to stages, not to a specific residual/coding defect, and
does not establish whether a large loop correction is physically right or wrong.
The exporter and trajectory parser are not the source of these two effects.

Artifacts: `postprocess-attribution/analysis/analysis.json` and `stages.png`;
the executed analysis source is preserved in its `analysis_scripts/` directory
(SHA256 `08bd7c5ae117d4fba7b59d0e584e7e9d98b242dbe9866120efa3fb7f1a68099c`).
The unused later root-script output-guard refinement is not the executed version.

## Next bounded map check

Export clouds from the retained odometry and after-loop-closure snapshots,
without optimization, alongside the existing final cloud. First export the
saved final snapshot and require exact reproduction of all 64 numeric files;
only then export the earlier stages. Use the original recording/settings and
fresh external cache, and verify source return sequence and coverage. This
supplies actual geometry for the user's inspection before selecting a correction.
No algorithm change or stage-removal acceptance is implied.

Medium is sufficient for this export/measurement step; the user was notified.
Notify before a later correction-design step requires High if they switch down.


## Stage-cloud export launched (2026-11-13)
Reviewed standalone exporter scratch commit `4f8af9aa592d5e90a211cbe851716ac8efe6b0c8`,
parent f18353e; Release SHA256 `732be39070087799bfbf61e04eb46531c38a66ae3912c11efb42513e4c3d1706`.
Runner SHA256 `63ad2011e7de406b0b8dae9a6a3cd0a5694ef3eb35d74cacabf4bb37c708b0fb`.
Release and Debug builds/selftests and nine runner tests passed. Independent Codex
review (not Sonnet) added explicit native writer-error detection and full merged
return-count/ordered GPS-time/intensity plus trajectory coverage checks before
clearing execution. Source/settings/snapshot hashes and original cache are checked
before/after. A fresh twelve-file bundle and separately archived runner/tests are
under `full-livox2-huber3/stage-clouds/`.
One export-only job launched from that frozen runner. It must reproduce all64
final-control numeric files before saving after-loop-closure and odometry clouds.
No odometry or postprocessing optimizer is called. Completion is pending; W798
uncertainty remains unresolved and physical accuracy is not established.


## Terra timing research (2026-11-13)
Terra6.1 inspected the consistency source and retained snapshots; an independent
Codex reviewer (not Sonnet) reproduced the measurements. At273060poses, median
sampling interval is1.9924ms. Adjacent interval relative difference median10.58%
shows that time-independent increment smoothing is a relevant hypothesis.

| Stage | World-position second-difference RMS | Locally lateral component RMS | Signed timing-model correlation |
|---|---:|---:|---:|
| After loop closure |0.326mm|0.126mm|0.976|
| Consistency1 |0.434mm|0.246mm|0.009|
| Consistency10 |1.109mm|0.519mm|0.005|

These discrete position increments are not physical acceleration. The local
constant-velocity timing estimate derives from the same positions, so correlation
is descriptive. It does not explain the added variation, but does not identify
a defective factor. The actual smoothness residual compares consecutive relative
pose translations/rotations, not world XYZ second differences; its anisotropic
weights apply in relative-motion residual coordinates. No time interval enters.
The predecessor lower_bound assignment drops exact first timestamps and permits
points beyond the last node, but preprocessing restricts cached returns and uses
the same convention. Its actual impact is NOT MEASURED, not an established cause.
The exploratory reader has no general format/provenance guards; these results
apply only to the already verified retained snapshots. Its source and output
are archived in stage-clouds/analysis_scripts. Next read-only diagnostic evaluates
the actual relative-pose factor residuals and checks whether assignment edge cases
can activate before selecting any correction. No numerical code change accepted.


## Stage exports and geometry comparison completed
Export driver completed exit0 in368.31s. Collection status is
`stage_cloud_exports_verified`; manifest SHA256
`85ad6404c0f2f422cf8a55a691be045f05ed28b623ba441b193e6f7bba54124d`.
All three clouds contain82,688,088 ordered matching returns and273,060 matching
trajectory timestamps. All64 final-control numeric files reproduce exactly.
Original inputs/cache and saved stages are unchanged. W798 remains unresolved.

Reviewed geometry source SHA256
`dc8b9236284b5ffb27528ffe69c0d1966e4f39aa4d628ca7ccc1338587386e6b`
is archived in stage-clouds/analysis_scripts; it completed successfully with
input hashes rechecked. All returns contribute to exact30s-bin displacement
quantiles. Plots use shared source indices/axes; no spatial clipping or alignment.
Independent Codex review (not Sonnet) reproduced selected sidecar quantiles/hashes.

| Same-return displacement | Loop closure minus odometry | Consistency10 minus loop closure |
|---|---:|---:|
| Maximum, full recording |13.23445m|0.66401m|
|318–330s median |4.35220m|0.029514m|
|318–330s95th percentile |6.79122m|0.104541m|

The focused interval includes2,143,979returns. Cloud-point displacement includes
attitude effects and differs from pose displacement; neither is physical error.
The elongated structure is already present in the odometry cloud. Loop closure
substantially changes placement/orientation; consistency is subtler at overview
scale, which says nothing about wall thickness. Current evidence does not identify
physical travel, reflected returns or erroneous movement as the explanation.
Prioritize large odometry placement before a consistency-weight intervention.

CloudCompare files (relative to full-livox2-huber3):
- stage-clouds/workspace/odometry-restored/scan_lio_merged.laz
- stage-clouds/workspace/after-loop-closure/scan_lio_merged.laz
- result/scan_lio_merged.laz (same as exported final-control)
Figures: stage-clouds/analysis/actual-clouds-overview.png,
actual-clouds-318-330s.png, displacement-by-time.png. All are descriptive.

## Actual consistency-factor research and prioritization
Terra6.1 computed the generated relative-pose residual on LC/C1/C10. Independent
Codex review reproduced the values and checked the XYZ angular branch against
C++: angles stay below0.036rad, away from singularities. This is mathematical
agreement, not bitwise execution of the compiled factor.
Local translation RMS(x/y/z) is[.238/.191/.114] -> [.429/.0498/.0445] ->
[1.099/.126/.0812]mm. The X quadratic weight is100times smaller (10times larger
implied residual standard deviation); these are relative-pose coordinates, not
fixed world axes or necessarily travel direction. Weighted smoothness energy
1.368M -> .563M ->1.912M includes rotation contribution4424 ->390384 ->971705.
This establishes a tradeoff pattern, not cause or total-objective descent.

Upstream point selection deliberately permits some returns after a window's last
pose within its support rules. The predecessor convention therefore needs an
explicit extrapolation contract before being called a defect. Actual affected
point counts remain NOT MEASURED. No time-assignment or weight change is adopted.
A future isolated X-weight single-pass diagnostic could test sensitivity, but
is deferred behind the larger odometry excursion; it is not the current fix.

Loop-closure research locates broad correction growth at270–410s and persistence
to the end. Final graph retains54edges after19drops; logs lack full measurement
transforms/information and correspondence identities. Neither large correction
nor low graph residual proves false/true physical matches. A causal edge ablation
would need exact retained graph instrumentation; no such replay has been started.


## Beginning/end inspection: bounded-view rejection
The first bounded chunk-view attempt (archived sourceSHA
1e6e746d70b9b481dbdeda1f86dbae246c3fa569cda46aa2075bac41b36ad366)
exited1 on its chronological chunk-support check. No output directory/report was
created; no room-shape conclusion is taken. This does not invalidate verified
full merged-stage exports or their completed geometry analysis. Preserve the
failed source. The revised view will use one complete actual merged LAZ stream
with pinned pre/post hash and full count, removing inferred chunk support and
chunk-coordinate reconstruction rather than relaxing the guard.


## Beginning/end view completed from actual merged cloud
The rejected chunk-bound assumption was specifically the final165returns: their
last time is546.12073332s while the last trajectory pose is546.11875436s, an
excess1.97896ms. This was not floating-point tolerance. Native point support
permits these returns. No data was removed from any export.
Reviewed replacement full_livox2_room_shapes_v2.py (archivedSHA
53282371b61c2ff3dbaab3418bd1efea25fafe3440e6537e6666075d44b6be4b)
completed one full actual merged stream,82,688,088returns. It selected9,636,890
returns at0–60s and9,458,025 at486.11875436–546.11875436s. The165laterreturns
are counted separately and excluded only from the explicitly defined last60s
of the exported trajectory. All input hashes rechecked; no reconstruction or
registration. Figure/report in stage-clouds/room-shapes/. Both versions retained.

The endpoint displacement before LC is[-57.374455,-23.884900,-0.159468]m,
norm62.147743m; final relative to its own first pose is
[-53.371723,-31.793888,5.162238]m,norm62.338117m. These are separations, not
physical closure errors. Beginning/end views show separated indoor geometry;
60s intervals can span different scenes and do not identify the physical room.
User route confirmation (apartment56/other indoor/outside at end) remains pending.

Next targeted investigation should separate accumulated heading/tilt changes
against available IMU increments from local translation observability near
318–330s. The retained snapshot does not contain exact per-window gravity,
acceleration offset or scale afterW885; do not invent these states or claim an
exact inertial-factor replay. A correction or graph-edge intervention requires
that distinction, physical-route context and independent evidence review. No
new optimizer execution is started at this boundary; no candidate accepted.

Independent Codex evidence review confirmed the physical route question is justified
for closure interpretation and approved the bounded next diagnostic. The analysis
milestone is complete; the full-cloud improvement goal remains unfinished.
