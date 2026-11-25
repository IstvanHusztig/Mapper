# Fresh full Livox2 Huber3 replay

2026-11-13. Explicitly authorized after the reviewed W754 result. The user also
authorized continuing toward the goal until new information is required and
requested Terra research when useful.

Status: mapper completed with exit 0; offline comparison collected with one
explicit conditional-uncertainty eligibility gap at W798. Original `run.json`
remains `needs_review`; supplemental `reviewed-collection.json` distinguishes
verified artifact integrity from incomplete diagnostics. See
`full-livox2-huber-full-results.md`. No accuracy or corrected-cloud acceptance.

## Frozen conditions

- Original recording: `C:/Users/User/Downloads/dLidar/raw data/livox/2/`.
- Fresh output: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3/`.
- Exact archived settings from `full-livox2-motion-prior/settings.toml`;
  alpha0.1/beta0.1 unchanged. No snapshot restart, cut input or additional filter.
- Huber3 applies to odometry scan matching through child-only environment
  `DMAPPER_DIAGNOSTIC_NDT_HUBER3=1`. The strict application hook logs activation;
  unset remains disabled. Postprocessing and shipped settings are unchanged.
- Existing focused trace windows408–416,730–780,830–885, with additional active
  Huber support/cost/information records. Ordinary diagnostics cover every window.

Scratch commit `cbca36d2747cfcd1fc3ceac3d906bd221080d8f8`.
Frozen Release dMapper SHA256:
`101881d74dfcb4369103d070701b003e5a83bdf4b95538aa3cc0a0114f997287`.
Runner SHA256:
`2bbb0fc664381055516bdd2cc67ab9ca67bcfe68202cbc5ec63a8475a5bb2e27`.
Source patch, binary/DLLs, CMake cache and build/test logs are frozen in the new
experiment folder. `bundle.json` covers13 files; preparation-manifest binds the
runner/tests/log, up-to-date build check and bundle metadata.

## Preparation gates

Release and Debug focused tests each passed4 cases/491 assertions. Full Release
reported128 cases,127 passed,1 unexpected failure,3 skipped;52,108 assertions
with four printed failures. Their values exactly match the retained baseline,
including the pre-existing corridor failure. No new observed test failures;
the suite is not fully passing. Final Release targets were verified up to date.

Runner tests:18 passed, including correct startup-marker ordering for the native
finished-window log format, actual early-convergence counts, active Huber trace
coverage/conservation, fresh-output refusal and capture cleanup.
Independent Codex source/test/runner review (not Sonnet) passed before launch.

## Monitoring and completion contract

Quietly monitor `monitor.log` for trajectory/velocity/solver changes, especially
the previous271–313s onset/escalation and any relocation. Preserve failures and
all output; no automatic retry is hidden in the runner.

Completion requires successful mapper exit, original raw/settings/control and
frozen executable/helper integrity, native-window/iteration trace completeness,
finite values and Huber3 coverage, association and support/cost/information checks,
matching stage timestamps and merged-cloud GPS/intensity sequences. Candidate
XYZ and trajectory changes are intended and are compared, not rejected as an
exact-reproduction failure. Raw recording files are immutable; the normal cache
belongs exclusively to this one mapper process during processing.

After collection, apply run/map verification: separate odometry and final-stage
trajectory behavior, matched-region geometry/sharpness/revisit checks, coverage
and a new cloud for the user's inspection. A single full run is not robustness
or surveyed-accuracy acceptance. Original baseline results remain preserved.

Medium is sufficient for processing/monitoring; the user was notified. Tell the
user before deeper causal/design work requires High if they have switched down.
