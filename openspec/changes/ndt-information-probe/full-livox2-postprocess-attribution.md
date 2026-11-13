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

The single unchanged postprocessing replay launched on November 13, around
04:49 Europe/Berlin. Live state is `postprocess-attribution/run.json`; root
monitoring session 41934. No odometry optimizer runs. All twelve stage snapshots
and numeric final exports must pass identity/coverage gates before attribution.
Keep monitoring through completion; no additional mapper or cache job in parallel.
