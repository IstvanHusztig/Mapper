# Original-control NDT admission experiment

2026-11-15. User authorized continued fixes and runs without approval stops.
This is one bounded candidate on the original Huber3 alpha/beta0.1 baseline,
not an adoption or completion of checkpoint Charlie (still3/19).

## Evidence and decision

See `livox4-gravity-compensation.md` for the failed gravity experiment and
the subsequent exact original-control trace. All346 window payloads match;
500 factor decompositions pass. NDT initiates tilt at W298/299; later IMU and
velocity factors amplify it. Earlier alpha1/gain experiments are distinct.

Native inspector b2cbdcb reconstructs W298 iterations0/1/2/10/99 and W299i0
from60001 actual filtered seed returns and preceding snapshot windows. Twelve
grid checks reproduce exact source, association, target geometry and counts;
raw and robust losses match exactly. Range quadratic reconstruction differs by
at most1.08e-14 normalized. All read inputs rehashed unchanged.
Artifacts: external `livox4-original-tilt-trace-125s/cell-inspection/`.

Thin cells admit distant returns merely sharing their voxel. Examples include
a W298 coarse cell with3.23mm minimum width and228mm mean normal residual,
and an11-point floor cell receiving225 new returns with51mm mean normal
residual and172mm p95. Near-floor side views show a main surface plus returns
well above it assigned to floor targets. Their physical identity is not proven.
`near-floor-matches.png` preserves the view; `cell-work-summary.json` reports
RHS work along the recorded step, not per-cell endpoint-angle contributions.

The existing min6/residual<=6 policy retains11637 unique sources at W298 and
9848 at W299. Coarse matched terms change7952->7118 and7497->5937. It removes
substantial work along the observed update, but some high-to-floor assignments
remain. Changing H and RHS means this preview cannot predict the new solution.
Independent reviewer seed_damping_crosscheck supports one bounded test, with
capture-range/support/geometry risks retained. No arbitrary covariance floor,
absolute attitude penalty or new recovery layer is selected.

## Minimal implementation

- Default-off `use_ndt_match_admission` under `[lidar_odometry_control]`, shared
  CLI/GUI and snapshot identity. Original Huber3 and alpha/beta0.1 retained.
- Reuse prior cell checks: occupancy>=6, finite mean/normal, SPD covariance and
  information, usable consistent inverse. Supported zero-normal volumetric cells
  remain allowed. No planarity requirement or changed map statistics.
- Cache cell validity only, refreshed with cell statistics; evaluate raw
  `r^T I r <=36` anew for the current pose at every iteration lookup.
- Share admission between both grids, costs and traces. Fixed-support post-step
  diagnostic costs keep actually admitted targets; do not silently drop them.
- Require six unique source returns before solving; do not double-count grids.
  Unsupported iterations fail with diagnostics, without relaxing the gate.
- Preserve flag-off arithmetic. No old owned/frozen-match framework, gain fix,
  nonlinear acceptance stack or covariance model is brought into this candidate.

## Validation and disposition

- [x] Native tests cover invalid cells, supported volumes, floor/vertical mismatch,
  pose-dependent refreshed admission, both grids, no-support failure and exact
  disabled behavior; shared configuration/snapshot tests.
- [x] Independent source review; matched Release build and focused/full tests.
- [x] Frozen candidate `livox4-original-admission-125s`, existing verified prefix,
  own binaries/config/logs/results; original full raw recording untouched.
- [x] Compare diagnostics, identical marked returns, fixed house-plan overlay,
  early indoor geometry and retained coverage, separately odometry/final.
- [ ] Only if local geometry survives, validate whole Livox4 and indoor Livox3.
  A short successful solve cannot certify the map. Robustness, historical and
  user visual gates remain before general adoption.

Separate incidental audit finding: original first-node angular sigma is used as
radians despite degree naming. Across500 updates node0's total parameter-path
rotation is only0.000225deg; max seed-relative tilt0.000086deg. It is not the
measured onset cause and is deliberately excluded from this comparison.

Implementation committed as8c4fc26 in the isolated original-control worktree.
Independent source review is clear. Matched Release executable SHA256
`954b6d7fc7e0142dc5b8fa6234b086aad3482e44de5c237dad1f1c3ae50a0195`.
External `livox4-original-admission-125s` is frozen/prepared with only admission
added to the verified original-control settings; compensation remains false.
Focused Release and Debug each passed16cases/2283assertions. Full matched Release
completed134passed/1failed/3skipped; only the already reproduced corridor
threshold17.9039mm versus7.5mm failed, with the same three expected consistency
assertions. Independent source review completed before launch.
Flag-off native fixtures/source equivalence are checked; a fresh flag-off whole
recording on this build is NOT MEASURED and must not be confused with the prior
trace build's346/346 identity. General-adoption regression gates remain.

Prepared candidate launched2026-11-15 03:07:43 local, native PID12716,
sequence PID26620. The startup log confirms both Huber3 and the admission policy;
gravity compensation remains off. Automatic continuation collects run-health
and fixed map/marked-return comparisons after native success. Sol6.1-medium
worker monitors. No full replay or map-improvement claim yet.

## Outcome: local geometry still fails; no full replay

Native645.52s, helper661.235s, exit0. All346windows and34600admission rows present;
Huber/admission markers1 each, gravity0; frozen inputs and executable unchanged.
Peak speed1.28m/s (original5.09), but11 bends exceed3deg and2 exceed10deg;
maximum13.612deg atW319. Run-health remains incomplete for conditional
uncertainty atW278; all346heading/speed/bias rows present.

Same27299marked returns,23442late: approximate line offset+5.297m odometry,
+4.284m final, versus original-8.615m. The feature rotates toward vertical rather
than the expected horizontal wall. First60s maximum window-start disagreement
0.291896m at38.88s. Minimum admitted fraction21.38% atW304i0. Lower speed does
not establish map correction. Independent reviewer viewed the overlay and
confirmed no full replay/adoption is justified. Preserve candidate and evidence.

The existing source-surface audit estimates normals from20/40 nearest returns,
requires stable planar support at both scales, and compares sign-invariantly
against target covariance axes. W298/W299 coarse mismatches over60deg number
144/314, with substantial ambiguity outside the eligible subset. This supports
a bounded original solver replay/ablation, not an immediate new production gate.
Native W298 control replay is being reconstructed from existing trace/state.

Priority correction to the experiment sequence: independent provenance review
found the confirmed AHRS adapter gain fix has NEVER been mapped at the original
alpha/beta0.1/0.1. The old gain-only label was confounded by1/1. See
`livox4-matched-gain-fix.md`; test that existing correction before further NDT
modeling. Neither source geometry nor a new covariance model is adopted here.
