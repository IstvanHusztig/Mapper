# Experimental implementation status, 2026-11-13

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
