# Tasks

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Fixes shipped: lever arm (a46bf85), the blind-stretch rule and match check, the fine-time placement, honest-inertial-carry layer 1; consistency off for the Livox. Open hypotheses kept as the record here and in the knowledge files: the odometry map horizon (4.x), the consistency pass (5.x), gaps (6.x), the physical check 3a.2b; the new-recording check (3a.4) moves to the roadmap list.**

## 1. Evidence so far

- [x] 1.1 Locate the doubles on the start storey in six final clouds (`experiments/consistency/doubles-phase1.md`, reviewed)
- [x] 1.2 Independent analysis by Codex (`experiments/consistency/doubles/codex/analysis-codex.md`); compared; the lever-arm axes checked against the recording metadata
- [x] 1.3 Lever arm corrected to the recording's axes (a46bf85)

## 2. New base with the corrected lever arm

- [x] 2.1 Full Livox 4 run with the odometry snapshot (`livox4_lever/recording_frame`, started 2026-11-22 23:03); verify-run; windows 369 to 378 against the reference and the first-vector run
- [x] 2.2 Loop-only cloud from the snapshot (consistency off); the phase 1 wall table and Codex's O / L / F measure on the new base; one large pair checked by hand from the raw returns (all-checks table O / S / variants, `experiments/consistency/livox4_lever/controls/all_checks.csv`)
- [x] 2.3 Leave-one-edge-out on the new base (Codex's graph tool, all-edge control first): the edges that move the measured pairs most, other edges held and with the pruning repeated (the 54 cm double at 41/73 s is made by loop edge 24->63: without it -0.537 -> +0.017 m, reference revisit doubles 124 -> 7; `controls/trace_start_double.csv`, replay `edges_loo/rej_24-63`)
- [x] 2.4 Show the numbers, the plan overlay and the clouds (O, L, F) to the user; no verdict; conclusion reviewer on the reading (odometry-weight prototype rejected by the user's look; user confirmed the 54 cm double and a mirrored wardrobe along the Schlafen north wall, 2026-11-23)

## 3. Loop closure across the family of odometries

- [ ] 3.1 O / L / F and leave-one-out for the reference, the sweep members with snapshots and the new base: per odometry, the doubles made and removed at loop closure and the edges behind them
- [ ] 3.2 Describe the influential edges: correction against the path between the segments, fitness, the spectrum of the registration's normal matrix, directions spanned by the matched surfaces, residual before pruning; read `loop_closure.cpp` for why each was accepted and kept
- [ ] 3.3 Candidate rules by reprocessing from the snapshots (back-end region job on its branch where code is needed): undetermined direction refused or freed, correction bounded by possible drift, verification on held-out geometry, pruning order; per rule the doubles made and removed in every odometry
- [ ] 3.4 Report to the user (numbers, clouds); on their decision the fix becomes its own OpenSpec change

## 3a. Fix roadmap for edge 24->63 (agreed 2026-11-23)

- [x] 3a.0 Edge 24->63 described: the corridor pass (136-139 s) matched 2 m off onto room walls (50-57 s); inconsistent with the rest of the graph and with the room scan's free space; the wardrobe mirror's phantom room is not what it matched (`controls/edge_24_63/overlay.txt`, `mirror_check.txt`, `controls/freespace_edges.csv`)
- [x] 3a.1 What holds the gallery wall (user-marked, 116-124 s), which moves 1.1-1.5 m without 24->63: leave-one-out on the shipped and the without-edge graph (`scripts/gallery_loo.py`, Opus)
- [x] 3a.1a Step A (2026-11-24): the stalled stretch of the lower corridor (173-184 s, 5.84 m missing by the four mouth registrations) freed by hand along the corridor, with and without 24->63 (`scripts/stall_free.py`, `livox4_lever/stall_free/`). Without the edge: 54 cm double 0.00, Wohnen +0.31, facade -0.02 deg, reference doubles 6/0. User's look: "a lot better", overlay with the fit from their marks "close to perfect" (`overlay_shifted_by_marks.png`); one ghost wall left north of the stair
- [ ] 3a.2 Direction changed 2026-11-24 after the loop closure research (`experiments/consistency/research/loop-closure-research.md`, recommendation 1): prototype in Python a per-step odometry uncertainty taken from the odometry's own window readings, so that the stretch is found without being told (design Fable/high, runs Opus). Rule frozen before any solve: `livox4_lever/odo_cov/RULE.md`, code `scripts/odo_step_cov.py`. Variants with and without 24->63, the Step A checks, where the closing correction lands, one map for the user's look
- [ ] 3a.2b Later, own decision: physical verification of each registration (research recommendation 2; the earlier rejection-rule idea: walls where the other pass saw through, two-sided, against the odometry placement, controlled for the target's openness)
- [ ] 3a.3 Freeze the rule and its limit in writing before the new recording is processed
- [ ] 3a.4 New recording (user, 2026-11-24; mirror covered, revisits of the same corridors, optional outdoor stretch): verify-recording, full run with loop capture, the frozen rule applied unchanged, numbers and maps to the user
- [ ] 3a.5 On the user's go: plan of the C++ option for the back-end agent shown first; replay, one reprocessing, recording 44 map-quality gate, Livox 3; its own OpenSpec change

## 4. Map horizon of the odometry

- [ ] 4.1 One run with a longer `sliding_window_trajectory_length_threshold` (time and memory reported); O gaps of pairs across the rebuild against the base
- [ ] 4.2 Report; decision as 3.4

## 5. Consistency pass

- [ ] 5.1 Reprocessing with consistency off and with equal axis weights: wall thickness, within-pass spread, pair gaps, node path length
- [ ] 5.2 Report; decision as 3.4

## 6. The odometry's gaps

- [ ] 6.1 New captures of the stretches where the base's O gaps are made; the blind windows, the carried gravity and velocity at the entry, the first correction at the exit, against the 2026-11-19 tables
- [ ] 6.2 Replay families for the candidates kept (carried state, hand-off); report

## 7. Close

- [ ] 7.1 Final report: where the doubles are made, by stage, with the measurements; what was not determined
- [ ] 7.2 Update `docs/regions/back-end.md`, `map.md`, `inertial.md` and `docs/code-review-tracker.md`; propose the change that reads the lever arm from the recording metadata
