## Context

See proposal and sensor-neutral-yardstick/readiness.md. Isolated scratch checkout;
main application and shipped TOMLs remain unchanged.

## Goals / Non-Goals

Measure response to NDT information scale, not choose final physical weights or
claim a robust candidate from single runs. No snapshot reuse across multipliers.

## Decisions

Compile-time alpha in1,.3,.1,.03,.01. Scale both indoor/outdoor point Hessian/RHS
after existing weights, before degeneracy projection/assembly. Scale
ndt_window_cost consistently. Alpha1 bypasses extra arithmetic. Never scale the
assembled system or geometric sum(n*n^T); covariance then changes naturally.
Record alpha, diff and executable/input/config hashes. Refuse unsupported robust,
norm or planarity paths in non-control experiment runs. High wall-clock cutoff
uniform across controls/interventions; inspect iteration and skip events.

Run matrix: Livox2 first60s all five alphas; wholeLivox1 alpha1/.1;44_short
alpha1/.1. Historical estimate40minutes plus builds/analysis; latest Livox1
control measured6m34s. Preserve every cloud/snapshot/log. Read velocity jump and
sigma separately, chi-square coverage, bend/speed/bias, geometry/revisits/sharpness.
Use matched physical map regions and show preliminary before/after views.

## Risks / Trade-offs

Projection makes diagnostic cost curvature differ from the projected Hessian.
Fixed LM damping changes relative strength. Existing outdoor norm mismatch is
out of scope; keep norm/planarity off. Numerical sensitivity may defeat exact
control reproduction: report it, do not quietly replace baseline. Full-family
and held-out validation remain required before candidate acceptance.

## Existing-run follow-up (user authorized November12)

Localize Livox1 alpha0.1 versus alpha1 in time using retained snapshots, final
trajectories and logs. Distinguish first relative differences, sustained growth
and later runaway. No new mapper runs, filtering, setting changes or root-cause
acceptance. Notify the user before any step requiring higher effort.

User additionally authorized causal investigation of existing runs/source after
switching to Astra High. No new runs may start without advance notice and a
separate decision; distinguish observed propagation from an identified trigger.


## Authorized instrumentation and replay

User approved logging implementation and two full Livox1 runs (alpha1/.1).
Extend the existing scratch checkout; no correction or shipped-setting changes.
Trace only windows100–115 and143–156, using an explicit opt-in switch. Preserve
solver arithmetic, accumulation order and parallel ownership. Record per-iteration
matched support, projection factors, factor contributions, state/carry, damping
and acceptance. Clearly label partial cost diagnostics; unmatched support is
separate. Unit parity and exact retained-run trajectory comparison are gates for
using traces as causal evidence. Build immutable executable bundles and run
sequentially against the original raw Livox1; no snapshot reuse or mid-walk cut.

Artifact root: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/
livox1-instrumented/. Root OpenSpec remains authoritative. Preserve old artifacts.
Current high effort is useful through trace review and interpretation; medium
suffices for mechanical waiting, but do not silently change user model settings.

## Approved finite translation-prior follow-up
User approved next-step-design.md and its two-run matrix. That document governs the isolated beta experiment and required synthetic gates. Prior completed tasks remain historical; section7 tracks the new work.

## Full Livox2 follow-through
After visually liking the new short Livox2 cloud, the user explicitly selected launching the full run. Use the same frozen alpha0.1/beta0.1 binary and exact archived settings on original raw data/livox/2. Disable only optional detailed trace (verified behavior-preserving); retain ordinary diagnostics and snapshots. New external output full-livox2-motion-prior. No new tuning or defaults. User will consider recording a new Livox1 after inspecting this full result.


## Authorized existing full Livox2 diagnosis
After switching to High, the user authorized investigating the retained full run and documenting it in OpenSpec, with advance notice before new runs. Scope is existing log/snapshot/final trajectories/raw IMU and primary-source research only. See full-livox2-jump-diagnosis.md for reviewed findings and proposed replay. No new mapper runs, builds or fixes are authorized by this investigation.


## Authorized surrounding-cloud inspection
User requested Python analysis of surroundings and authorized reusable diagnostic tools. Inspect existing raw sensor-frame range/short snapshots and transformed final-cloud intervals; do not rerun SLAM. Record time/frame conventions, identity and scene-inference limits. See full-livox2-cloud-context.md. tools/cloud_context.py is a read-only-input offline analysis tool; artifacts stay in the external experiment directory.


## Approved full Livox2 focused replay
User explicitly approved focused logging and one full original Livox2 replay after the close-scene inspection. Scratch instrumentation only: separate DMAPPER_TRACE_LIVOX2 opt-in covers windows408-416,730-780,830-885 (116windows), retaining pre-onset lead-ins and both near-surface episodes. Preserve alpha0.1/beta0.1, exact archived TOML and original raw2; no snapshot restart/cut, automatic extra run, filtering or fix. Extend read-only traces with node times/IMU samples and prediction/handoff poses, per-range match support/geometry/turnover, actual projected NDT step contributions by range and final iteration poses. Existing factor/carry/acceptance traces remain. Release trace parity/conservation and full recorded reproduction are required gates. New external artifact root full-livox2-trace/. Notify when High is no longer required for mechanical execution; causal interpretation still benefits from High.


## User stop before replay launch
User explicitly requested stopping before a new run. No Livox2 replay has started. Scratch logging commit5eb882e is built; parity/conservation test passed1668assertions, fullRelease reproduced the same existing assertion failures. Frozen binary is in full-livox2-trace/bin (SHA256 d7915fba00a131f032d0a620fec2f11ae99b5f9bda291e2f8b8d213f61db7673). Runner out/full_livox2_trace_replay.py has not launched and still needs review fixes for bundle inventory, rich trace completeness, archive overwrite protection and capture-error child handling. Do not launch until the user explicitly resumes. Tasks11.2-11.4 remain unfinished; no automatic retry or processing.


## Preparation complete; replay still stopped
See full-livox2-replay-readiness.md for finalized runner safeguards, ten synthetic tests, final independent review and archived preparation manifest. Tasks11.1/11.2 complete; replay/reproduction/causal analysis remain pending explicit user resume. User subsequently objected to standalone HTML presentation overhead and prefers reusable Python diagnostics suitable for later dMapper integration. Do not extend the HTML viewer; no replacement implementation is implied by that feedback.
