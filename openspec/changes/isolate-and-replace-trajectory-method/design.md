# Design

## Authorized local detour

The W298 factor/cell evidence is now recorded in `livox4-rotation-factor-trace.md`
and `livox4-ndt-cell-audit.md`. The user subsequently authorized a local fix for
cell validity and correspondence consistency. `ndt-match-guard-plan.md` (follow-up: `ndt-iteration-refresh.md`) defines
that reviewed, default-off experimental candidate and its checks. It is separate
from the numerical-equivalence extraction below; Charlie remains 3/19. No new
replay or broader method adoption is authorized by this detour.

## Context

See proposal.md for motivation. The pipeline already separates pure domain math,
application orchestration, infrastructure adapters and GUI/CLI entry points.
`RunServices` supplies orientation and execution ports; `run_odometry.cpp` still
coordinates prediction, optimization and state carry. `Window` carries poses,
gyro bias and velocity; TOML and snapshot infrastructure own persistence.

The reviewed evidence is in `../ndt-information-probe/livox4-code-audit.md`:
Livox4 W301–320 has growing incoming predictions; W397 has a separate large
angular correction. A translation-only acceptance guard explains why neither
must trigger rejection, but does not identify the first erroneous calculation.
Snapshot model arrays are overwritten by final poses, so they cannot recover
the original pre-optimization orientation/gravity state. Exact causal isolation
is still an investigation gate, not a completed result.

## Goals / Non-Goals

**Goals:** an evidence-based component boundary, numerical equivalence for the
extracted current method, and independently selectable replacement with explicit
state and provenance. Preserve the application's layering and existing exports.

**Non-Goals:** an entire SLAM rewrite, runtime plugin loader, automatic method
switching during a recording, sensor-reader changes, or declaring outdoor/cave
robustness from an indoor cloud. No numerical correction is bundled with the
extraction. Research chooses a candidate; this document does not choose one.

## Decisions

### 1. Isolate the first corrupting boundary before extraction

Trace incoming velocity and attitude/gravity integration separately from matching
and outgoing carry. Start with existing Livox2/4 evidence and reused synthetic
tests, not a fresh general diagnostic framework. Identify input state, calculation,
output state and propagation for each suspected mechanism. Concentrate on the
first departure and one later angular event rather than repeatedly processing
whole routes. If missing pre-LM state prevents attribution, specify the smallest
capture and reproducible check; obtain explicit run approval before collecting it.
An isolated guard weakness is not sufficient evidence that its boundary contains
the initiating error.

Prefer a bounded window replay only after its incoming state and map have been
captured and validated against the unmodified sequential run. The current
`--from-snapshot` path bypasses odometry and resumes downstream stages; it is
not an odometry restart. The saved `motion_model_poses` have also been overwritten
by final poses. Cutting raw data at the failure time would reinitialize velocity,
bias, gravity and map, so it cannot be treated as the same causal experiment.
If reaching a valid checkpoint requires a prefix pass, run that prefix once and
reuse the verified checkpoint for candidate comparisons. Compare bounded and
original sequential calculations at the boundary before interpreting changes.

Gate: a reviewed evidence note distinguishes demonstrated transitions, upstream
uncertainty and multiple mechanisms. If attribution remains unresolved, record
that and retain the investigation phase instead of labeling a suspect a cause.

### 2. Extract the smallest coherent stateful component

After that gate, write a concrete contract before moving code. Candidate seams
span prediction, estimation and carry; a single-function wrapper is insufficient
if the demonstrated failure involves shared state. Include only the state needed
by the selected seam, with explicit ownership/lifetime, sensor/body/world frames,
time and units, calibration, gravity convention, biases, velocity and covariance
meaning. Define initialization, reset, accepted/rejected/invalid results, diagnostic
coverage and map updates. Rejection must specify what state remains committed.

Pure calculations remain domain code. Application orchestration owns the selected
method for a run and shared lifecycle; composition wiring instantiates it. External
library adapters, if justified later, remain outside the domain. Reuse existing
point storage, reporting and result formats rather than coupling implementations
to GUI, TOML or filesystem operations.

Alternative rejected: exposing every current mutable object as an untyped context.
That would preserve hidden coupling and make clean removal or independent tests
unreliable. Also reject redesigning unrelated loop closure/export under this seam.

### 3. Preserve legacy before adding another method

Extract in a behavior-only refactor: no weight changes, reordered summation,
different initialization or changed acceptance rules. Freeze and identify the
reference sources, settings, binaries and test inputs first. The Livox experimental
Huber control is frozen source cbca36d with its recorded settings/environment;
scratch pre-floor c52b25f also contains inactive experiments and is not implicitly
the adopted baseline. Preserve shipped-default behavior separately from Huber.

Require exact equivalence for controlled legacy inputs and the existing golden
checks, using approved runs where recordings are required. TOML's new selector
may change serialization bytes; numerical outputs must not. Historical known
test failures remain explicit and do not become passing extraction evidence.

### 4. Select once per run and persist the choice

Add a stable method identifier to saved settings; an absent key means `legacy`.
Use the existing GUI Load parameters path and the same CLI pipeline. A new GUI
settings editor is unnecessary for this requirement. Validate method-specific
settings before processing; unknown or removed identifiers fail clearly.

Record method ID, implementation/configuration identity and diagnostic schema in
results and checkpoints. A restart must match method, configuration, input and
state version. Define a tested compatibility path for existing legacy snapshots;
never reinterpret another method's state or silently fall back. If a method cannot
support a downstream operation, report that limitation before starting the run.

### 5. Research against the demonstrated mechanism

Reuse `../ndt-information-probe/open-closed-terra-research.md`,
`open-closed-improvement-plan.md` and the failed Cauchy/floor/bias experiments.
Use primary papers and original implementations to compare approaches relevant
to the isolated failure: joint inertial estimation, weak-geometry handling,
correspondence/support robustness, uncertainty and failure detection as applicable.
Check handheld nonrepetitive Livox sampling, indoor/outdoor transitions, cave-scale
geometry, observability, initialization, computation, dependencies and licensing.

Produce one reviewed candidate decision with equations/assumptions, fit to the
interface, predicted discriminating outcome, rejection criteria and alternatives.
If research requires a larger boundary, revise the contract/specs before coding.
Do not force a fashionable whole SLAM stack through an unsuitable narrow seam.

### 6. Stage comparison and adoption

Implement the candidate separately and opt-in. First validate its contract and
the isolated mechanism using existing fixtures or explicitly approved captures;
then test indoor preservation and the Livox2/4 transitions, followed by full-route
and perturbation gates. Each new processing/checkpoint run requires approval of
its concrete scope. The separately authorized stronger-gyro Livox4 replay is one
additional experiment, not evidence that either a fix or this interface is accepted.

Use `../sensor-neutral-yardstick/criteria-proposal.md`: geometry, revisits and
sharpness separately; identical physical regions and retained coverage; empirical
perturbation envelopes; odometry and final stages independently. Require full
diagnostic coverage or report NOT MEASURED. Same-table return is useful evidence,
but endpoint closure alone cannot establish whole-map accuracy. Include indoor
Livox3 and historical 44 regression gates without treating the old sensor as a
Livox noise calibration. House-plan alignment remains preliminary and must not
be independently refitted to hide candidate distortion.

Default changes and removal are separate user decisions after numerical evidence
and visual review. Remove a rejected implementation and its exclusive dependencies
without deleting experiment evidence. Selecting a removed ID must produce an
actionable error; never silently map old settings to a different method.

## Risks / Trade-offs

- Wrong seam isolates symptoms -> require causal review and include coupled state.
- Refactor changes floating-point order -> compare frozen outputs before candidate work.
- Stronger gyro weighting masks drift elsewhere -> treat replay as an experiment;
  inspect both translation and rotation and both output stages.
- Attractive local result fails globally -> stage full-route, indoor and perturbation gates.
- Covariance looks precise but is uncalibrated -> record its mathematical meaning;
  do not treat inverse Hessians as calibrated accuracy.
- Larger candidate costs more time -> report runtime/memory; map quality remains first.

## Migration Plan

1. Review isolation evidence and approve the concrete interface contract.
2. Land the legacy extraction and setting, with equivalence evidence.
3. Review research and select one experimental implementation.
4. Add it opt-in, validate and present both clouds and measurements.
5. On explicit acceptance, update existing sensor-neutral HTML documentation and
   settings guidance. A default switch or legacy removal is separately authorized.

Rollback before adoption is selecting `legacy`; rollback of the refactor is its
isolated commit range. Preserve frozen binaries and evidence for either path.
