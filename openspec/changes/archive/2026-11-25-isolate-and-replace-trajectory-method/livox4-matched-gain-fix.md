# Correctly controlled AHRS gain-fix comparison

2026-11-15. Continued work authorized without approval stops. Charlie remains3/19.

## Comparison gap found

The confirmed adapter correction was previously compared using alpha/beta1/1
against an original0.1/0.1 control. That confound was documented, but a correctly
matched gain-only recording comparison was never collected. Independent reviewer
seed_damping_crosscheck checked saved manifests, trace weights and source hashes:
the newer0.1/0.1 gravity, original trace and admission trials all retained the
old adapter. They cannot reject the gain fix's effect on incoming AHRS/map state.

The old adapter writes `settings.gain` while Fusion applies `rampedGain`.
The public setter synchronizes effective gain after startup and updates the
startup ramp slope. Earlier native regression tests demonstrate the defect;
24.17% of raw Livox4 samples request zero gain,31.1% around106–110s. Neither
the defect nor those counts prove it causes the map error. See
`caller-audit-and-gain-fix.md` and `gain-jump-107s-analysis.md`.

## Bounded test

Port ONLY the established fc73404 adapter/header and its native tests/registration
onto the original-control b2cbdcb lineage. Keep the already verified inactive
gravity path off and existing TOML Huber3 configuration. No match admission,
cell/solver changes, gain parameter adjustment or acceptance stack is included.
The inspection-only helper changes do not alter the processing executable.

Use the identical125s raw prefix and a TOML byte-identical to frozen
`livox4-ahrs-gravity-control-125s/livox-vertical.toml`. Explicitly verify BOTH
compiled alpha and beta equal0.1 before freezing. This corrects the historical
comparison error; it is not a weight sweep. Own run folder:
`livox4-matched-gain-fix-125s`.

- [x] Reuse confirmed adapter regression tests in Release and Debug; verify
  configuration/golden behavior and independent source review.
- [x] Freeze matched build/source/config/input identities and launch prefix.
- [x] Compare original control versus candidate early indoor geometry, W298–325
  trajectory, identical marked wall returns, complete coverage and runtime;
  inspect odometry and final stages separately.
- [ ] Only if local geometry survives, validate full Livox4/indoor Livox3 and
  remaining project regression/robustness/visual gates before general adoption.

Existing native W298100-step reproduction and source-surface evidence continue
without another recording replay; no production GICP/normal gate is selected.

## Implementation and launch

Source fd4c000 (four reviewed files); both Release and Debug focused suites
passed 13 cases / 27,683 assertions. Independent reviewer confirms the active
change is adapter initialization/public gain setter only; this comparison cannot
separate startup-ramp behavior from gain switching after startup.

Frozen Release SHA256:
`c2f159b27ea81928a5c7cbb0b326e2087f2bffce25c901d1a84578087fb5e6ea`.
Control/candidate TOML hashes match exactly; both compiled scales are0.1.
Native PID20144 launched2026-11-15 03:34:33 local, sequence PID8088. Actual log
confirms Huber3 and alpha0.1, no gravity/admission activation. Build and tests
are preserved in the run's `validation/` folder. Map validation remains pending.

The original W298 offline case separately reproduced one continuous100-call
solve and all22,500 trace records exactly, including updates/covariances.
Source e41771c; external `livox4-original-tilt-trace-125s/solver-replay-W298`.
This case is available if further correspondence experiments are needed; no
source-mask ablation or new production covariance model has run.

## Completed prefix (2026-11-15)

Native exit0,346 windows,564.65s processing/582.006s wrapper time. Frozen
artifacts and inputs unchanged; Huber activation1, gravity0, admission0.
Run-health has complete346 speed/bend/bias rows: peak speed1.25m/s (original
5.086), five corrections above3deg and one above10deg (W313,10.242deg).
Residual bias maximum0.26deg/s. Conditional velocity chi-square median/p9581/614
over126 eligible windows remains a diagnostic trigger.

Fixed27,299 marked returns,23,442 late returns, unchanged preliminary alignment:

| Stage | Median absolute offset from approximate expected wall | p90 |
|---|---:|---:|
| Original odometry |8.615m|9.010m|
| Gain-fix odometry |3.543m|4.207m|
| Gain-fix final |3.062m|3.673m|

First60s maximum position disagreement0.070723m,p950.062219m. This is
disagreement with the control, not measured accuracy. Overlay inspection shows
the long wall substantially less warped but still displaced and rotated. The
candidate is not a fixed map; no full-route validation or default adoption.
The wall annotation is approximate, selection may contain mixed surfaces and
the displayed context height slice can lose drifting surfaces. No perturbation
envelope has been measured.

Artifacts: external `livox4-matched-gain-fix-125s/quality/comparison.json`,
`quality/plan-comparison.png`, corresponding `quality-odometry/`, `validation/`,
and `result/scan_lio_merged.laz`. Independent evidence review completed by
seed_damping_crosscheck: provenance and map limitations confirmed.

Saved-orientation check (`validation/orientation-disagreement.json`) shows the
remaining initiating region: gain-fixed map/AHRS gravity-axis disagreement
grows0.567->6.729deg acrossW298–301, compared with0.869->7.792deg in the
original. Relative first-column heading changes0.528->-14.294deg; W313 later
changes-9.833->0.101deg. These are post-tail snapshot disagreements, not true
orientation errors or exact pre-solver inputs. Native RxRyRz convention and
gravity-axis formula independently checked; the heading is not pure twist.

Automatic final-wall diagnostics at z0.50m found281 pieces (119m of line):
thickness p50/p900.014/0.068m; revisit differences0.038/0.146m over164 pieces.
Control uses different extracted surfaces/coverage, so these do not establish
resolved improvement or hide the marked-wall displacement.

Next bounded diagnostic uses the original W298 exact native case: remove the
previously identified reliable source/target normal mismatches and recompute
the complete Hessian/RHS through100 iterations. This is a causal ablation, not
a deployable filter or proof about the gain-fixed W313 state. Preserve original
control identity first. It tests whether those pairs explain the old initiating
error before selecting another numerical change.
