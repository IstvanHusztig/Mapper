# Experimental implementation status, 2026-11-13

## Latest: tasks2.3/2.4 complete; checkpoint prepared

Scratch5be333f adds information-eligibility/skip counters, explicit rejection of
unsupported alternate robust preprocessing, both-grid cost-gradient checks,
room/corridor/open-plane and wrong-target cases, mixed-axis Huber coverage,
contaminated incoming priors and a four-window production optimize_windows test
exercising bias initialization plus gravity/offset/velocity carry in both floor
modes. Counters' `accepted` means information-eligible before viewpoint filtering,
not the number of final matched contributions. Existing match diagnostics remain
separate. No direct regularizer call was added to loop closure/consistency; their
post-odometry inputs can still change.

Final Release AND Debug focused checks:8cases/219assertions each, all pass.
Release full suite:135/136cases pass,3skipped; same17.9039mm corridor failure.
First Debug build failed with MSVC C1902 program-database-manager access in the
restricted process sandbox. Serial retry outside that sandbox succeeded; original
failure and retry logs are retained. No Debug full-suite run was needed.

Independent review found no control/state-reset/counter-threading blocker. Its
initial carry-coverage concern was addressed by using apply_gyro_bias_prior in
the contaminated-prior test and adding the actual production window-loop test.
It also required command/hash/explicit-boolean approval ties in the runner; these
were added. Six fail-closed runner checks pass without invoking any optimizer.

Frozen bundle: experiments/phase0b-2026-11-12/open-closed-floor6mm-w415/
with bin/, manifest.json and run_residual_floor_checkpoint.py. Preflight verified
426 hashes: executable/runtime files, original settings/snapshot/raw, saved W415
states/control records, map-history cache inputs and runner. Manifest approval is
explicitly false; result/ and run.json do not exist. Native --floor6mm permits
only W415, first requires the exact100-iteration Huber3 control, then restores
all initial state and runs one100-iteration Huber3 candidate with sigma0.006m.
It retains all cells and guards source/build weights. No automatic retry/output
reuse. Planning estimate5–15minutes, not a measured runtime guarantee.

Task4.1 is prepared but remains unchecked until explicit user run approval.
No checkpoint/full replay has been started. This implementation is experimental;
unit-test success is not evidence of a corrected Livox2 cloud.

## Earlier implementation checkpoint (superseded where noted above)

Isolated branch codex/phase0b-ndt-information:
-4358c5f: intentional saved-TOML format change (one disabled key in each golden).
-90dad3c: residual floor/configuration propagation and focused verification.
Production checkout algorithm/default settings are unchanged. No replay launched.

Tasks2.1/2.2 implemented: finite nonnegative metre-valued configuration, positive
squared-value representation checks, omitted-key reset to zero, load/save,
odometry options, zero hash compatibility and positive hash discrimination.
Floor helper preserves cached/uncached legacy eligibility, delegates directly at
zero, rejects invalid/non-SPD covariance and checks reconstructed finite/SPD
information. Stored cell geometry/cache are unchanged. Cost, trace and both-grid
assembly call the helper. Startup identifies positive activation and scope.

Release final focused checks:6cases/49assertions pass, including the3TOML goldens.
Full CTest:133/134cases pass,3skipped; sole unexpected failure remains the archived
corridor across-error17.9039mm versus7.5mm condition. Existing expected-failure
consistency assertions remain reported. This is not a clean full suite.
No Debug run: changes are configuration/pure numerical logic without
configuration-dependent assertions/lifetimes/threading changes.

Independent review found no zero-path/cached-alias blocker and required more
cached/invalid/SPD and old-hash tests. Those were added. The first hash fixture
failed because it omitted the frozen performance timeout3600s; the field IS
hashed. After matching the frozen configuration, zero hash equals retained
snapshot header15425577911643724949; positive floor differs. Initial failed
focused output is preserved; final output passes. Saved goldens retain their
existing Windows CRLF bytes (git diff --check flags new CR as whitespace).

**Not ready for experiment execution.** Tasks2.3/2.4 remain unchecked: dedicated
both-grid cost/gradient and room/corridor/open-plane/coupled prior/carry tests,
identifiable skip-reason diagnostics, explicit handling of unsupported alternate
robust preprocessing, and final executable activation checks. No new mapper
executable has been frozen for a replay. No map improvement or candidate
acceptance is claimed from unit tests. Each replay still requires authorization.

Logs archived in experiments/phase0b-2026-11-12/open-closed-existing-data/:
floor-build*.log, floor-focused.log (initial failed fixture),
floor-focused-final.log and floor-full-suite.log.
