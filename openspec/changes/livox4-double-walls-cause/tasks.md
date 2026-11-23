# Tasks

## 1. Evidence so far

- [x] 1.1 Locate the doubles on the start storey in six final clouds (`experiments/consistency/doubles-phase1.md`, reviewed)
- [x] 1.2 Independent analysis by Codex (`experiments/consistency/doubles/codex/analysis-codex.md`); compared; the lever-arm axes checked against the recording metadata
- [x] 1.3 Lever arm corrected to the recording's axes (a46bf85)

## 2. New base with the corrected lever arm

- [x] 2.1 Full Livox 4 run with the odometry snapshot (`livox4_lever/recording_frame`, started 2026-11-22 23:03); verify-run; windows 369 to 378 against the reference and the first-vector run
- [ ] 2.2 Loop-only cloud from the snapshot (consistency off); the phase 1 wall table and Codex's O / L / F measure on the new base; one large pair checked by hand from the raw returns
- [ ] 2.3 Leave-one-edge-out on the new base (Codex's graph tool, all-edge control first): the edges that move the measured pairs most, other edges held and with the pruning repeated
- [ ] 2.4 Show the numbers, the plan overlay and the clouds (O, L, F) to the user; no verdict; conclusion reviewer on the reading

## 3. Loop closure across the family of odometries

- [ ] 3.1 O / L / F and leave-one-out for the reference, the sweep members with snapshots and the new base: per odometry, the doubles made and removed at loop closure and the edges behind them
- [ ] 3.2 Describe the influential edges: correction against the path between the segments, fitness, the spectrum of the registration's normal matrix, directions spanned by the matched surfaces, residual before pruning; read `loop_closure.cpp` for why each was accepted and kept
- [ ] 3.3 Candidate rules by reprocessing from the snapshots (back-end region job on its branch where code is needed): undetermined direction refused or freed, correction bounded by possible drift, verification on held-out geometry, pruning order; per rule the doubles made and removed in every odometry
- [ ] 3.4 Report to the user (numbers, clouds); on their decision the fix becomes its own OpenSpec change

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
