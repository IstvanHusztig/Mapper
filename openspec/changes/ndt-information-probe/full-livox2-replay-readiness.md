# Full Livox2 diagnostic replay preparation

The user stopped launch before any new run. Subsequent authorization covers the
offline timeline and runner preparation only. No replay has started. A fresh
explicit go-ahead is required before launch; preparation does not override this.

## Frozen instrumentation

Scratch commit `5eb882e3764d09726bf69a55db2e829e759e0fb2` extends the retained
`16807ac` candidate with opt-in logging, keeping alpha0.1/beta0.1 unchanged.
It traces windows408–416,730–780,830–885:116 windows,100 iterations each.
Records distinguish prediction stages, incoming/outgoing state, IMU samples,
per-factor contributions, projection, actual assembly match identity, range
support, turnover and accepted/rejected steps.

Bundle: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-trace/`.
Executable SHA256:
`d7915fba00a131f032d0a620fec2f11ae99b5f9bda291e2f8b8d213f61db7673`.
The bundle preserves source patch, CMake cache, build/test logs and hashes.
Settings will be copied byte-for-byte from the retained full-run archive at
launch and rechecked afterward. Original raw Livox2 remains the input.

Independent Codex source review (not Sonnet) found no blocking arithmetic or
thread-ownership change. Per-range accumulators follow existing node ownership;
projected range quadratics sum to the actual NDT contribution. Unprojected
range information summaries must not be interpreted as calibrated covariance.

## Existing C++ gates

- Focused Release trace parity/conservation:1 case,1668 assertions passed,
  including serial/parallel, both grids and rejected steps.
- Full Release:124 cases,123 passed,1 failed,3 skipped. The sole unexpected
  corridor failure and all four printed assertion values match the retained
  candidate; three assertions belong to expected-failure coverage. This is not
  a fully passing suite.
- Debug was not rerun for these diagnostic-only read paths.

## Launch boundary and remaining work

The runner now requires the executable and exact binary-directory inventory,
normalizes Windows manifest separators, and checks every frozen file hash.
It validates all required iteration/window records, matrix shapes and finite
payloads, numeric scalar fields, sample column metadata and both grids' actual
assembly identity. A missing identity record cannot pass. It refuses to replace
a different archived runner. Capture errors terminate/wait for its own child,
record failure and preserve evidence without retrying.

Ten synthetic tests passed without launching dMapper: valid rich trace, missing
records, malformed/nonfinite payloads, duplicates, window metadata, inventory
changes, absent/identical/conflicting archives, byte-preserving capture and
capture-failure child cleanup. The real frozen bundle passed read-only
verification for all11 files. These tests do not prove successful full-run capture.
The validator deliberately treats failed/nonfinite solver traces as incomplete
evidence and is specific to the approved active-IMU/hierarchical-NDT configuration.

Runner SHA256: `78e92062fe309346f51ce0ce139865bfe7925dc5a211d2b9cedd519020758b92`.
The runner, synthetic tests and test log are preserved in the external bundle
with a separate preparation manifest; the original binary bundle is unchanged.
Final independent Codex review (not Sonnet) checked this runner hash against the
C++ emitters and found no remaining readiness blocker. Review caught and resolved
Windows separator handling and incomplete scalar/window metadata validation.

Exact retained snapshot, trajectories, poses and
cloud hashes remain required after the eventual replay before trace attribution.
A mismatch must stop causal interpretation and must not trigger an automatic
rerun. Medium is sufficient for preparation; request High before substantive
interpretation of the new trace.

The synchronized retained-run view and its browser-verification limitation are
documented in [flyaway-timeline-delivery.md](flyaway-timeline-delivery.md).
