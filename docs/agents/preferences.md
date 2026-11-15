# Shared preferences

Latest authorization (2026-11-15): continue fixing the map without per-run
approval stops; user explicitly revoked them. Preserve tests, independent review
and map-quality gates. Do not end work merely to answer a status or filename
question. User replay settings belong in `settings/livox-vertical.toml`, paired
with the matching executable; no hidden Huber environment flags.

Applies to Claude Code and Codex. Curated on 2026-11-12 from the current user's
instructions and `C:/Users/User/.claude/projects/C--Repo-dMapper/memory/`.
Historical memories provide context, not new authorization. Current explicit
instructions override them. Keep personal/global configuration outside this repo.

- **Model and effort:** tell the user before starting a step that needs a higher
  model or higher reasoning effort, explaining the reason. Prefer medium for
  mechanical execution; flag when returning to medium is sufficient. Do not
  silently switch models or pretend a setting changed. This extends the Claude
  effort preference with the user's explicit model/effort request on 2026-11-12.
- **Scope and goal:** work on dMapper. Reading recorder code to understand inputs
  is fine; do not plan recorder changes. Precision and map quality beat speed.
  Geometry, consistency and sharpness have equal weight. Tools serve diagnosis
  and application fixes; do not substitute tooling for the mapping goal.
- **Results and decisions:** perform authorized runs yourself. Present readings
  before experiment verdicts and ask how to proceed before accepting/rejecting
  candidates or starting an unapproved fix. Do not remove/archive a failed
  experiment until the user decides. Remove rejected dead code after that decision.
- **Visual gate:** before a map-changing push, supply before/after images and
  full cloud/result paths and wait for the user's look. Documentation-only work
  needs no map run. Overlay means plan_fit.py then plan_section.py at roughly
  two metres above the relevant floor, not automatically world z=0.
- **Recording gate:** check raw IMU/timing and ask how it was mounted/held and
  whether the route closes before declaring a new recording usable. Never
  process unsynced_archive. Do not re-apply factory point calibration.
- **Missing dependencies:** name the package and installation command, ask
  permission, then install and verify it yourself. Do not ask the user to perform
  the installation or use a weaker workaround while waiting. This follows
  feedback-install-missing-tools (2026-10-17), superseding older CLAUDE.md wording.
- **Execution:** Release for processing and ordinary tests; Debug only when the
  change can behave differently there (see CLAUDE.md). Long batches use one
  background job with concise tables; check disk first and coordinate cache and
  executable ownership. Ask before deleting existing experiment outputs.
  Continue authorized work after status/roadmap questions; do not stop merely
  to report progress. Ask only when necessary information or a substantive new
  decision is missing (user reiterated this explicitly on 2026-11-12).
- **Independent review:** before a substantive cause/verdict/recommendation,
  give an independent reviewer the claim, evidence, alternatives and prior
  contradictions. Fold its findings into the answer and disclose what changed.
  Claude retains its Sonnet preference; Codex uses an available review agent and
  must disclose that it is not Sonnet. If delegation is unavailable, say the
  conclusion lacks independent review. Routine status/mechanical work is exempt.
  The primary agent requests one review; reviewers return findings directly,
  without recursively delegating review of their own reports.
- **Research agents:** use a research agent whenever it is useful for the current
  task (user, 2026-11-12). Give it a focused question and evidence scope, then
  check its findings. Use local evidence for repository questions; seek primary
  sources when external research is needed. Do not delegate merely to fill a slot.
- **Learning:** answer explanations directly; do not start unrequested code work.
  Preserve useful per-window features/outcomes for the planned learned tuner
  during authorized diagnostic work, without expanding the current task.
- **Analysis tools:** craft reusable tools when they help difficult or repeated
  jobs (user, 2026-11-12). Prefer inspecting available recordings and clouds over
  asking the user to recall precise timestamps or details from weeks earlier.
  Keep diagnostic artifacts with experiments and preserve original data.

Sources: feedback-effort-level, feedback-review-conclusions,
feedback-ask-before-verdict, feedback-ask-before-removing,
feedback-visual-check-before-push, feedback-install-missing-tools,
feedback-scope-dmapper-only, feedback-goal-precision-not-tools,
feedback-check-data-before-usable, feedback-user-makes-maps,
feedback-batch-runs-background, feedback-clean-test-outputs,
feedback-learning-requests, ml-training-data-logging, precision-before-speed,
dlidar-recording-format. Original Claude memories are preserved untouched.

- **Experiment locations (user, 2026-11-12):** keep experiment data/results outside the repository, under `C:/Users/User/Downloads/dLidar/experiments/`. After active phase0b jobs finish, relocate their data/results to `phase0b-2026-11-12`, verify files and update links. Original recordings stay in `raw data/livox/`; code/OpenSpec stay in the repository. Do not relocate files while a job uses them.

- **Diagnostic tooling (user, 2026-11-12):** prefer reusable Python analysis and structured outputs suitable for later dMapper integration. Do not default to standalone HTML viewers; user objected to the timeline implementation and its extra presentation work. Do not extend it without a clear user request.

- **OpenSpec documentation:** maintain the active change throughout authorized work without waiting for the user to repeat this requirement (reaffirmed2026-11-12).

- **Long-run updates:** monitor quietly; report only completion, failure, or a decision requiring the user. Avoid per-window/progress chatter and unnecessary token use (user, 2026-11-12).

- **Cost discipline (2026-11-13):** user objected to substantial token use without a corrected cloud. Prefer a focused correction test using existing tools over expanding diagnostic tooling or repeated review rounds; keep required evidence checks proportionate.

- **Concise output (2026-11-13):** user explicitly objects to verbose responses and output-token costs. Keep updates to material findings or decisions, with brief final answers; put detailed evidence in OpenSpec.

- **Current model/effort (user, 2026-11-13):** Sol6.1 at medium. Keep this state in mind; explicitly ask the user to switch before work requiring higher effort or another model. Do not silently assume a switch. Routine replay monitoring/collection stays on medium.

- **Accepted-fix documentation (user, 2026-11-13):** when a correction is accepted as a general fix, update existing docs HTML files with what changed, rationale, settings and validation. This complements OpenSpec and does not authorize declaring an experimental correction accepted.

- **Current model/effort (user, 2026-11-13, latest):** Sol6.1 High enabled for open/closed-space research and house-plan registration audit. Propose a plan before algorithm changes or new runs; use Terra research agent. This supersedes the earlier medium state for this phase.

- **Model/effort update (2026-11-13):** user confirmed Astra High for the numerical
  proposal review. Review completed; Sol6.1 High recommended for the next analysis
  and implementation step. Await actual user switch confirmation; no silent switch.

- **Run organization (user, 2026-11-14):** Give each run its own folder under the external experiments directory. Keep its settings, manifest, logs, results and analysis together. Record the run folder and merged cloud path in OpenSpec and handoff. Keep raw recordings in place.
