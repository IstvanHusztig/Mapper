## Why

Livox2 and Livox4 fail on indoor/outdoor routes while indoor Livox3 looks much
better. We need to isolate the responsible computation, then evaluate a
research-grounded replacement without losing the current implementation as a
reproducible control. Greater robustness is an acceptance goal, not a promise.

## What Changes

- Diagnose the first corrupting boundary before changing numerical behavior.
- Extract the smallest coherent affected estimator component behind a typed
  interface with explicit inputs, state, outputs and failure semantics.
- Preserve the existing method as `legacy`; prove extraction equivalence.
- Research primary papers and reference implementations against the demonstrated
  mechanism; select one candidate through a reviewed design before coding it.
- Select a method through saved settings, shared by CLI and GUI. Keep methods
  isolated so a rejected or superseded implementation can be removed cleanly.
- Validate local reproduction, indoor preservation, outdoor transitions and
  perturbation robustness before any default change or removal of legacy.

## Capabilities

### New Capabilities
- `replaceable-trajectory-method`: explicit estimator contract, method selection,
  isolation, provenance, compatibility and evaluation gates.

### Modified Capabilities
None. Existing pipeline and sensor-setting requirements remain in force.

## Impact

Likely domain prediction/optimization/carry and application orchestration;
infrastructure settings/snapshot identities and composition wiring. Exact
boundary and alternative method remain deliberately gated on diagnosis/research.
No whole-pipeline rewrite or new dependency is selected. This plan supersedes
the rejected floor's implementation direction, retains its evidence, and reuses
the active ndt-information-probe findings. Planning authorizes no code changes
or new processing runs.

Subsequent user authorization: implement the local NDT cell-validity and
correspondence-consistency candidate described in `ndt-match-guard-plan.md` as
an isolated experimental detour. This does not authorize new processing runs,
default adoption, or completion of the larger interface work.
