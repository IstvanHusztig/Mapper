# Independent evidence collection before high-effort analysis

2026-11-14. User requested a Sol 6.1 cross-check and a research agent, then an
explicit stop before the parent's own code analysis. No new run or code change.
The following records agent findings; synthesis and proposed corrections remain
pending the user's effort switch.

## Sol 6.1 independent cross-check

Agent `seed_damping_crosscheck` verified the existing external bundle
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-seed-damping-125s`.

- Exit 1, 300/346 completed windows, empty result directory. W301 at 108 s
  accepted 51 local updates (iterations 0-50), then exhausted iteration 51.
- Bundled executable matches tested scratch ba0268c and its Release executable
  hash; settings match v2 byte for byte, input hashes and three DLL hashes match
  the v2 manifest. Control settings differ only in five acceptance settings.
  Alpha=1 and Huber cutoff=3 are logged; beta=1 is supported by the build cache
  and executable provenance, not a checked-path experiment-weight log row.
- W87's four diagnostic lines are byte-identical across latest, v2 and refresh
  control. This is logged equality, not a map-quality verdict.
- All 30,051 accepted rows satisfy the logged merit tolerance, 1-degree step
  cap and 3-degree prediction cap in the independent scalar audit.
- W1-300 accept all 100 iterations at attempt 0. Retry damping carry is not
  exercised before W301, so it cannot explain earlier differences from v2.
- First precise merit difference against v2 is W69 iteration 0: IMU 3.67988
  versus 16115.74452, with identical NDT 182222.67244. First target-hash
  difference is W70. At W301 the source hash matches but target hashes differ:
  this is not an isolated same-state comparison of the two corrections.
- W301 iterations 5-40 have 299 preceding prediction-rotation-bound rejections.
  Node 152 (recording time 108.65889624 s) limits the angle from iteration 7.
- Iteration 50 accepts at LM 5e18; the periodic multiplier starts iteration 51
  at 5e19. All 48 iteration-51 trials pass the angle bounds but fail the merit
  decrease requirement. Best positive logged decrease is 2.764864e-10, below
  the approximately 9.558e-10 acceptance threshold. Final LM is about 5e24.
  Thus the reason means insufficient required decrease, not necessarily a
  literally increasing objective for every trial.
- Full state/factor/RHS traces are gated to W297-300, excluding W301.
  W301 scalar merit and acceptance rows exist. This limits causal attribution.
- Existing run-health check reports incomplete evidence, velocity jump/sigma
  medians 34/3.7 mm/s, and conditional eligibility gap [97]. Both acceptance
  replays stop at 300/346; the refresh control completes 346.

## Terra 6.1 research questions

Agent `acceptance_stall_research` returned these ranked questions, not a root
cause finding or authorization to implement a different optimizer:

1. **Active constraint versus convergence:** the all-node geodesic rotation
   bound is not a coordinate box. Check feasible/tangential descent of the full
   frozen merit at the limiting node. Exhausted radial backtracking and tiny
   steps do not prove constrained optimality. General first-order constraint
   concepts: [Stanford optimization lecture](https://web.stanford.edu/class/msande211x/lecture12.pdf).
2. **Damping ratchet:** Ceres adapts its trust radius using actual/predicted
   reduction and separates termination criteria. Here damping never decreases
   and the periodic multiplier remains. Compare scaled regularization against
   curvature, block step norms and resolvable predicted/actual reduction.
   This does not prove that lowering damping yields a feasible descent step.
   [Ceres trust-region documentation](https://ceres-solver.readthedocs.io/latest/nnls_solving.html).
3. **Proposal/merit consistency:** verify directional descent and derivatives
   on exactly the same state/support. The projected NDT proposal and full
   unprojected merit need not have interchangeable predicted reductions.
   Factor attribution, robust weighting and Euler-update consistency require
   evidence; no sign error is established by the current logs.
4. **Refresh consistency:** compare common-support costs and added/lost or
   reassigned contributions. Per-iteration decrease does not prove decrease of
   a common objective across refreshes. Classical ICP monotonicity assumptions
   do not automatically apply here. [ICP research](https://arxiv.org/abs/2007.07627).
   [PCL NDT reference implementation](https://pointclouds.org/documentation/ndt_8hpp_source.html)
   uses a different objective and More-Thuente line search; it is a comparison,
   not a drop-in correctness proof.
5. **Numerical resolution:** distinguish explicit tolerance, floating-point
   resolution, gradient convergence and constrained stationarity. Equal printed
   scores do not prove binary equality or physical accuracy. Inspect full
   precision component costs and state deltas before choosing a remedy.

## Stop boundary

Reports collected and preserved. Parent synthesis/code analysis has not begun
in this collection step. Ask the user to switch to high before proceeding.
No new replay, build, algorithm edit, cloud, adoption or push. Any additional
recording run still requires approval. Charlie remains 3/19.
