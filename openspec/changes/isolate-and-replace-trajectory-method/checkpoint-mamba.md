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
