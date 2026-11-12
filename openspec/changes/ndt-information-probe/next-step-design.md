# Next experiment: translation continuity pressure

2026-11-12. User-authorized design following the completed 18/18 tasks.
User approved implementation and both runs with "Yes go" after reviewing this
design. Implementation gates still precede processing; no further runs implied.

## Recommendation and evidence

Test a finite beta=0.1 multiplier on motion-model XYZ observation information,
with NDT alpha fixed at0.1, against retained beta1 controls. Beta is diagnostic,
not calibrated noise: tenfold lower information means sqrt(10) larger sigma.
Question: does restraint toward the integrated predicted path contribute to
Livox1's slowdown and feedback, while retaining Livox2's visual improvement?

At window107 the candidate's Y velocity-ramp NDT/(motion+carry) information
ratio is0.795 versus8.298 for alpha1. At the final iteration NDT pushes against
motion and carry. Later turns are favored by NDT on diverged geometry, sometimes
while matches increase. Neither another global NDT decrease nor a match-count
guard follows as a fix. See instrumented-results.md for evidence and limits.
The motion edges and carry constrain related predictions; statistical double
counting is NOT established. Motion edges also supply interpolation support.

## Exact intervention and preservation

- In src/domain/scan_to_map.cpp multiply the XYZ observation weights by beta
  in BOTH generated AtPA and AtPB calls (currently561-563 and587-589), only when
  the always-on acceleration factor is actually active.
- Preserve rotational observation rows, gyro bias, origin regularization,
  acceleration factors, carry/covariance, prediction, NDT, projection, map
  construction, damping and acceptance. First-window/inactive paths stay intact.
- XYZ residuals depend on attitude: scale their mixed and rotational Hessian
  contributions too. Do not merely scale XYZ parameter blocks. Preserving
  rotational observations does not preserve every rotational matrix entry.
- Scale corresponding cost diagnostics consistently. Default beta1 retains its
  arithmetic path. Record alpha/beta/activation/source/settings/build identity.
  No shipped setting changes and no concurrent time-scaling/covariance change.

Do not use beta0: accelerometer translation factors connect sparse anchors,
not every node (scan_to_map.cpp:798 onward). Without motion XYZ edges, unmatched
intermediate/tail nodes can have three translation null modes each. Damping
can conceal this while undamped covariance fails. Finite beta retains the
connections but still needs conditioning and geometry checks.

## Implementation gates before mapper runs

1. Frozen-state assembly: H/RHS differences equal (beta-1) times isolated XYZ
   residual contributions, including attitude coupling. Other factors identical.
2. Default/inactive numerical parity, rotated-frame finite differences,
   constant-velocity fixture and serial/parallel agreement.
3. Sparse/no-match nonanchor fixture: expose zero-weight null modes; verify
   finite beta retains support and valid finite undamped covariance. Test room
   and blind corridor geometry against truth. Do not relax existing tolerances.
4. Full Release suite and focused trace parity. Debug only for a change with
   configuration-sensitive risk. Failed gates block recorded-data processing;
   no arbitrary damping or covariance floor to force a pass.

## Approved two-run matrix

| Input | Candidate | Retained control |
|---|---|---|
| Full Livox1, original start | alpha0.1/beta0.1 | instrumented alpha0.1/beta1 |
| Livox2 retained0-60s, original start | alpha0.1/beta0.1 | original alpha0.1/beta1 |

Use exact archived per-recording TOMLs, inputs, timing limits and stages.
Verify baseline identity and beta1 parity before reusing controls. If a fresh
control is needed, notify the user before adding that run. No mid-walk cut or
odometry snapshot restart. Sequential jobs, immutable binaries, exclusive input
cache ownership. New artifacts go under the approved external experiment root,
in motion-prior-probe/; preserve previous outputs and raw recordings.

Run verify-run and verify-map on both results. Livox1: windows100-115/143-156,
especially105-108/149-151; onset, speed/heading, velocity changes and claimed
uncertainty separately, full-route geometry and revisits. Differences from
alpha1 are not ground-truth error. Livox2: frozen regions/alignment, median AND
tail wall spread, revisits, before/after clouds; retain mirror/window annotations
without deleting points or automatically calling exterior returns drift.

Early improvement supports a contribution from prior pressure, not a unique
cause. Later failure would leave registration reliability unresolved. Worsening,
relocated failure or diagnostic-only improvement is not a successful correction.
Even improvement in both runs remains provisional: neutral perturbations,
held-out geometry, L2 regression and user inspection are still required under
sensor-neutral-yardstick before acceptance. Two runs cannot establish a fixed
final point cloud. No new perturbation or L2 batch is implicitly authorized.

## Reconcile the existing roadmap

- Keep phase2 time scaling separate. Constant edge variance gives common-velocity
  ramp information T*dt/sigma_ref^2. A continuous velocity-deviation regularizer
  Sigma_edge=Q*dt, Q=diag(sigma_ref^2)/dt_ref (m^2/s), gives
  T*dt_ref/sigma_ref^2. Sigma proportional to dt still depends on node count.
  Validate200/500/1000Hz, irregular subdivisions, smooth-profile convergence,
  rotated coupling and invalid timestamps. This defines a regularizer, not
  measured acceleration covariance. Referencing nominal Livox500Hz leaves its
  uniform-rate strength unchanged; referencing L2 also changes strength. Neither
  choice alone is a demonstrated Livox1 fix; do not bundle with beta.
- D1 is implemented (fusion_orientation_estimator.cpp:35); D5 handoff exists
  (window_steps.cpp:78, window_cut_test.cpp). Historical acceptance tasks remain
  open; do not implement them twice. D2 dynamic gain still writes settings.gain
  directly and remains a separate unresolved behavior/calibration task.
- synchronized-retune task4.8 already records LM/unmatched-cost experiments:
  mixed map results, including runaway with zero unmatched cost. Do not repeat
  them unchanged or assert that objective acceptance guarantees correct mapping.
- Carry already uses full undamped inverse-system marginal covariance plus gap
  growth. It is conditional on map/model assumptions, not registration-basin
  uncertainty. No arbitrary covariance floor is proposed.

## Review and effort

Local research and independent Codex review (not Sonnet) agree on this bounded
experiment. Review changed the initially considered zero-weight ablation to
finite weakening because intermediate translations would otherwise lose support;
it also clarified observation-row versus parameter-block scaling.

Keep High through numerical implementation and assembly/covariance review.
Medium suffices for approved run execution and routine collection afterward.
Notify the user before High is needed again for causal interpretation. No model
switch is performed by the agent. The original18/18 completed tasks remain a
historical record, not approval or completion of this follow-up.
