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
| Restored odometry | 334.276 m | 332.153–332.171 m |
| After loop closure | 333.228 m | 331.102–331.120 m |
| After consistency pass 1 | 408.553 m | 331.587–331.948 m |
| After consistency pass 5 | 674.183 m | 333.601–334.562 m |
| After consistency pass 10 | 892.159 m | 335.674–337.671 m |

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
