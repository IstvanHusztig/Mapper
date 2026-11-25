## Context

See `proposal.md` for the why. What the code does today (read 2026-11-19, commit 9ddf0dc):

- **Unknowns of a window:** six per node (one node per IMU sample: 180 on the Livox, 360 on the L2, 0.36 s), the gyro
  bias, the start velocity, the end velocity and the acceleration offset `e` (world frame, zero at the guess).
- **Guess:** in a window that does not rest, `integrate_window_translations` integrates the accelerometer from the
  carried end velocity with the carried gravity vector. It is linear in both. `motion_model_poses` is set to it.
- **Motion edges** (`add_motion_edge_terms`): the relative pose of consecutive nodes against that of the guess; position
  sigma 1e-4 m per edge, information × `imu_position_information_scale` (Livox 0.1, L2 1).
- **Accelerometer terms** (`add_acceleration_terms`): one per 10 ms span, sigma² = density² / τ + floor², each span
  independent; they read `mean acceleration − e`.
- **Priors:** start velocity ~ carried end velocity with its covariance; `e` ~ zero with the carried offset covariance
  grown by the random walk.
- **Carry** (`take_covariance`, `update_acc_term_after_window`): the 3×3 blocks of the solve's inverse for the end
  velocity and for `e`, each alone; `e` is folded whole into the gravity vector.
- **First pose:** fixed (sigma 1e-6). The last 15 nodes (Livox; 30 on the L2) are re-propagated at a constant velocity
  taken from the poses before the next window starts from them.

What was measured (files under `C:/Users/User/Downloads/dLidar/experiments/stage8/`):

| # | Reading | Livox 4 base | Livox 3 | Source |
|---|---|---|---|---|
| E1 | Start-velocity prior sigma, windows the LiDAR sees (ratio > 0.1), p50 / p95 | 5.3 / 6.2 mm/s | 4.9 / 5.7 | `scale/analysis/carry_honesty.txt` |
| E2 | Correction the window makes to that prior there (3-D norm), p50 / p95; along travel p50 | 28 / 64 mm/s (5.5 / 12.6 × the largest sigma); 13 | 25 / 56 (4.9 / 11.2); 11 | same |
| E3 | Prior sigma over a blind stretch (ratio < 0.02), first → last window | 6.0 → 10.7 mm/s over 15 windows (5.4 s) | 9.1 → 10.5 over 6 | same |
| E4 | Largest prior sigma in the lower-corridor replays, edge information × 1 / 0.1 / 0.01 (one replay each) | 5.6 / 10.7 / 20.1 mm/s | | `stalls/prior_cap.txt` |
| E5 | The same with floor 0.2 / 0.3 / 0.5, and with edge information × 0.01 + floor 0.3 | 13.4 / 16.0 / 20.7, 28.3 mm/s | | same |
| E6 | Change of the dead-reckoned speed through the lower corridor's blind windows (4.65 s), 11 runs; part may be real slowing | 250–780 mm/s | | `scale/analysis/README.md` M2 |
| E7 | First window that sees again, upper corridor (one run): start velocity prior → result, end velocity | 0.94 → 0.58, 0.29 m/s; the window ends 158 mm short of its guess | | `scale/analysis/features_upper_base_ep.txt` |
| E8 | First correction after the blind windows of the upper corridor (137.9–138.2 s, ratio 0.05–0.075), floor 0.1 / 0.2 / 0.3 / 0.5 | −32 then −163 mm / −86 / −113 / +45 mm (minus = against travel); with 0.2–0.5 the following windows (ratio > 0.1) correct +47…+191 mm | | `stalls/upper/result/*_window_features.csv`, `s8_calib_compare.py` |
| E9 | Edge information × 0.01 + floor 0.3, one replay per corridor | lower: path 10.14 m (reference 9.09–9.23), up to 2.55 m/s; upper: 8.13 m, up to 1.32 m/s | | `stalls/replay_tables.txt` |

Reading of the table:

- **The believed velocity sigma has a ceiling that the settings set, not what happened** (E3–E5). After 5.4 s without
  the LiDAR it is 10.7 mm/s; in no replay is it above 30 mm/s, while single windows change the speed by 0.3–0.65 m/s
  when the LiDAR sees again (E7, and the 90 floor replays). The ceiling rises when the edges or the accelerometer
  terms are weakened, less than in proportion: the two set it together with the rest of the window. Arithmetic for the
  edges alone: 179 edges, each holding 2 ms of the guess to 0.32 mm, hold the window's mean velocity to 12 mm/s
  around the guess, in every window again.
- **That is a fault in the model's logic, not in a number:** the guess is computed from the carried velocity and
  gravity; an edge that measures the guess as if it were data states the prior a second time, with a weight that has
  nothing to do with the prior's.
- **Where the LiDAR sees, the window corrects the prior by four to five times its sigma** on both recordings (E2; the
  known velocity chi², median 32–40, also on the recording whose map is good). The cause is not separated. Candidates:
  an accelerometer error common to the window; the hand-off (with the first pose fixed, a position mismatch of 7 mm
  between the re-propagated tail and the window becomes 28 mm/s of start velocity); the LiDAR's own confidence;
  the attitude. Most of the correction is not along travel. Replays that remove one candidate at a time are task 1.6.
- **The correction that starts the upper stall comes from the LiDAR and points against travel** (E8), in three of four
  replays, in windows just above the projection ratio. A weaker inertial hold gives way to it more (−86, −113 against
  −32 mm) and recovers because stronger windows follow. Why the LiDAR pulls back there is not known; nothing in this
  change removes that pull.
- **Looser is not automatically better** (E9, one replay): the lower corridor over-ran with both loosened.

Not usable as evidence: the accelerometer's cost per residual at the result (0.114 on Livox 3). The node positions are
nearly free against that term, so the value is far below 1 whatever the noise is.

## Goals / Non-Goals

**Goals:**

- The start-velocity prior's sigma grows through blind windows for as long as they last.
- After a blind corridor the window follows the LiDAR again within two windows, in every member of a family of tiny
  changes, without a new disturbance elsewhere (readings R-recover, R-harm).
- No term states twice what the model knows once, and no link between carried states is dropped. These two are
  corrections of the model's logic (layers 1 and 2): they are judged by unit tests of the logic and by what they do
  on the recordings, not tuned.
- Every further weight comes from a measurement in physical units, so the same model serves the L2 and the Livox with
  their own numbers.

**Non-Goals:**

- Accuracy inside a blind stretch. A consumer IMU whose gravity direction is known to a fraction of a degree drifts by
  decimetres in 5 s; no weighting changes that. This change makes the estimate say so and give way afterwards.
- The LiDAR's pull in weak windows (E8), the degeneracy projection and the NDT weights (lidar-matching).
- The position at re-observation (D6), the gyro-bias carry, the offset's frame (tracker C17), the tail
  re-propagation. Each is read by the new tool; none is changed here.
- New values in `settings/` or new baselines before the user's decision.

## Decisions

### D1 Layer 1: the motion edges follow the window's velocity and offset

In a window whose guess was integrated from the accelerometer, the position measurement of edge k (nodes k−1 → k)
becomes, in the frame the edge already uses,

    m_k(v, e) = m_k(guess) + R_guess,k−1ᵀ · [ (v − v_guess) · dt_k − e · ((t_k − t_0)² − (t_k−1 − t_0)²) / 2 ]

with `v` the start-velocity unknown, `v_guess` the value the guess was integrated from, `e` the offset unknown (signs as
in `integrate_window_translations` and `mean acceleration − e`). The edge gets two more 3×3 Jacobian blocks (columns
of `v` and `e`, which are already dense border columns), in the way the rotation rows couple to the gyro bias. The
rotation rows and every weight stay. A window at rest, or one whose guess was not integrated, keeps today's edges.

- **Contract (unit tests, inertial):** on a window at its guess, change `v` and `e` and move the nodes as the
  re-integration says: the edge term's cost and right-hand side do not change. The existing test of every term
  (right-hand side = minus half the cost's gradient). A window at rest: unchanged.
- **What still holds the two states after it:** their priors, the zero-velocity term at rest, the weak end-velocity
  tie when the end term is dropped. That is intended.
- **Side effect to document:** in the per-term table the edges' hold on the motions "shift" and "stretch" along travel
  falls to nothing; `docs/window-terms.html` says what the rows mean.
- **Alternatives:** a smaller `imu_position_information_scale` (tested at 0.01: corridors 7.57 / 8.80 m, ceiling
  20 mm/s: weaker, not removed); no position rows and an accelerometer term at every node (five times the terms,
  second differences over 2 ms: stiff numerics); leaving the edges and inflating the carried covariance after the
  solve (hides the ceiling inside the window, where the recovery fails).
- **Setting:** `imu_edges_follow_state` (bool, `[imu_translation]`, default false).

### D2 Layer 2: velocity and offset carried with their joint covariance

- The solve hands out the 6×6 block of its inverse for (end velocity, `e`); its rows and columns are in the columns
  the solver already extracts. The block is part of the window problem's saved state, so a rejected step restores it.
- The next window's prior on (start velocity, `e`) is the joint Gaussian with that covariance: propagated over the time
  between the windows with the transition (velocity changes by −`e` × gap), then grown by the noise as today; its
  information is one 6×6 block on the two unknowns. Where the velocity prior falls back (first window, carried speed
  above `max_prior_velocity_mps`) the cross block is zero.
- **Why:** in a blind window the end velocity is the start velocity minus `e` × duration. Its uncertainty from the
  offset must add up coherently over N windows (σ grows with N). With the link dropped at every window it adds up as
  independent pieces (σ grows with √N): too small by √N, 3.9 after the 15 windows of the lower corridor.
- **Contract (unit test, inertial + noise-budget):** a chain of N blind windows with a known offset sigma and the other
  noises small: the carried velocity sigma after N windows equals the closed form (offset part sigma × N × duration,
  plus the white part) within 1 %.
- **Alternative:** carrying the gyro bias in the same block (9×9). Not now: its link to the velocity runs through the
  attitude, and the bias has its own open questions (tracker C26, C38).
- **Setting:** `imu_carry_joint_covariance` (bool, default false). `WindowLoopCarry` gets the 3×3 cross block; see D7.

### D3 Layer 3, only if the measurement asks for it: an acceleration error common to the window

The model would become: white noise per span + an error `w` constant over the window, independent between windows,
sigma `σ_w` + the slow offset `b` (random walk, carried). Within one window `b` and `w` cannot be told apart, so the
window keeps one unknown `e = b + w`:

- prior covariance of `e`: `P_b + σ_w² I` (`P_b` after the random-walk growth);
- after the window, with the solve's `ê`, `P_e'` and `C_ve'` (covariance of end velocity and `e`), and
  `K = P_b (P_b + σ_w² I)⁻¹`: the gravity vector takes `K ê` (today: `ê`); `P_b' = P_b − K (P_b + σ_w² I − P_e') Kᵀ`;
  the carried velocity–offset covariance is `C_ve' Kᵀ`.

This is the exact conditioning of (`b`, `w`) on what the window learned about their sum (checked by the reviewer); no
new unknown. `σ_w = 0` takes today's code path unchanged.

- **Why it is not decided yet:** it rests on reading E2 as an accelerometer error common to the window, and E2 has
  three other candidates. Built only if tasks 1.6 and 5.1 show that such an error exists and how large it is.
- **What it would give:** the velocity's uncertainty grows by `σ_w × duration` per blind window; the gravity vector
  averages over several windows instead of taking each window's offset whole.
- **Alternatives:** raising the white floor (the 90 replays: tiny changes decide the result at 0.2–0.3, and at 0.5
  the guess runs short where the LiDAR sees); a larger random walk of `b` (lets the gravity vector follow each window,
  the opposite of averaging; tested 0.0001 → 0.01 in stage 8); a second unknown per window for `w` (same result, three
  more unknowns).
- **Setting:** `imu_accel_window_sigma_mps2` (double, default 0 = off). Needs layer 2 for the cross term.

### D4 Numbers come from measurements; readings test the model

`tools/carry_honesty.py` (pure Python, from `window_features.csv`; prototype
`experiments/stage8/tools/s8_carry_honesty.py`) prints the readings. They are stated before any run with the layers,
and they test the model: no number is fitted to make a reading come out.

| Reading | Today (Livox 4 / Livox 3) | What an honest model gives |
|---|---|---|
| R-seen: velocity chi² of windows with ratio > 0.1, median / p95 (full runs and the long captures; needs hundreds of windows) | 40 / 188, 32 / 176 | 2.4 / 7.8 if the prior were the only thing wrong; reported with the candidates of E2 separated |
| R-blind: per blind stretch, the prior sigma along travel at its last window and the speed correction of the windows that see again | 10.7 mm/s against 0.3–0.65 m/s | the sigma grows through the stretch; the corrections are reported against it (five stretches per Livox 4 run: a list, not a statistic) |
| R-exit: per blind stretch, the sign and size of the first correction after it, the ratio there, and the number of windows until the speed settles | E8 | reported; this change does not promise to change the first correction |
| R-recover: lower and upper corridor captures, family of seven: speed two windows after the ratio is above 0.02 again; path in the slow stretch; largest speed | 0.2–0.45 m/s; 5.9–6.4 / 5.8–7.5 m; 1.1–1.25 m/s | every member above 0.7 m/s; family spread no wider than today's; no member above 1.5 m/s |
| R-harm: passage, stairs and end-stretch captures: end position against the same code without the layer; flags | families of today's code: task 1.2 | inside the family spread of the code without the layer; no bent or fast flag |

- Layers 1 and 2 have no number. Layer 3's `σ_w` and the offset's random walk come from task 5.1 (the offset a window
  estimates when its prior is loose, where the LiDAR sees), per sensor.
- The readings are investigation readings (contract `sensor-neutral-yardstick`): they say whether the uncertainty is
  honest and whether the corridors recover, not whether the map is good. The map is judged in the full runs and by
  the user.

### D5 Switches while the change runs, none after it

Each layer has its setting, off by default: 44_short, the guards and every capture's own replay stay bit-identical, and
a replay compares a layer against the same code without it. At the end of the change a layer that the user accepts for
both sensors becomes the only behaviour and its switch goes; a rejected layer goes with its code
(`feedback-no-dead-code`). `σ_w`, if built, stays a setting (a sensor number).

### D6 The position when the LiDAR sees again: named, measured, decided after this change

A window's first pose is fixed to the previous window's end. After a blind stretch that end is wrong along the blind
axis by decimetres (stage 8 M5: windows moved 5–27 cm from their guess). The window has no unknown for a position
error at its start; it can only change its velocities and its shape (E7: the window ends 158 mm short, its start
velocity goes from 0.94 to 0.58 and its end velocity to 0.29 m/s). Whether the speed loss at exits is a position
error fitted as a velocity error is a hypothesis; layers 1 and 2 make the next windows free to correct the speed and
do not remove this.

- **Measured in this change:** R-exit.
- **Options, to be designed only if the reading asks for it:** (A) the first pose gets a position prior along the
  directions the previous windows did not see, so the correction is a step in position; (B) the same, with the step
  spread back over the blind stretch's windows so the trajectory stays continuous (the corridor's walls do not change
  under a shift along the axis nobody saw). Both touch the hand-off, the map and loop closure: their own change.

### D7 Captures and snapshots stay usable

- The three settings go into the settings hash's `added_later` list (`odometry_snapshot_file.cpp`), as the gyro scale
  did: at their defaults they do not change the hash, so existing captures still count as "own settings" and existing
  odometry snapshots are still accepted.
- `WindowLoopCarry` gains the cross block (D2); the capture file's format version goes up by one, and the reader
  accepts the previous version with a zero cross block (what the code that wrote it carried).
- Then every existing capture (the five stage 8 stretches, the guards' two) replays bit for bit with its own
  settings, and the layers start from a capture's state as a full run of the old code left it.
- **Consequence for reading replays:** a replay with a layer on starts from a state the old model made. It shows the
  layer's effect from there on; the effect on the entry state, and whether a layer is wrong where the LiDAR sees,
  shows only in full runs. Each layer therefore also gets one full Livox 3 run (about 700 windows, most of them seen)
  before the next layer is judged.

### D8 Who does what

| Job | Region | Worktree | What |
|---|---|---|---|
| J0 | orchestrator | main | three settings (`toml_io.h`, `odometry_config.h`, `added_later`, golden TOML), the options that carry them, `WindowLoopCarry` cross block, capture format (D7); the groundwork replays (tasks 1.2, 1.6) |
| J1 | verification | region-b | `tools/carry_honesty.py` with its test; today's tables for 44, Livox 3, Livox 4 |
| J2 | inertial | region-a | layer 1 (D1) |
| J3 | solver | region-b | the joint block out of `take_covariance` and in the saved state (D2) |
| J4 | noise-budget, then inertial | region-a | joint prior information (D2); the transition and the carry in `prepare_acc_term`, `prepare_velocity_state`, `update_acc_term_after_window` |
| J5 | verification | region-b | replays: five captures × layers in order, families of seven on lower, upper and end; the readings of D4 |
| J6 | noise-budget, inertial | region-a | task 5.1 (is there a window-common error, how large); layer 3 only after the user's yes |
| J7 | orchestrator | main | full runs (Livox 3 per layer; at the end Livox 4 base and the six repeats, Livox 3, 44 s0 / s1 / s2), `verify-run`, `verify-map`, the tables and clouds for the user |

At most two editing agents at once (region-a, region-b). J2 and J3 run together; J4 after both; J1 any time.

## Risks / Trade-offs

- **A weaker inertial hold gives way to a wrong LiDAR pull** (E8; the opposite failure is the runaway: tracker M4, the
  44 stairwell window 213) → every layer is replayed on the stairs and end-stretch captures and on the guards; largest
  speed and the fast flag are in every table; full 44 on s0 / s1 / s2 before any value ships. If the first correction
  against travel grows with the layers and the windows after it do not repair it, the LiDAR side has to be treated
  first (its own change, lidar-matching).
- **An over-run as in E9** → R-recover is read with the largest speed and the path against the reference range, not
  with the path alone; D6 is the answer if it shows.
- **The L2 file was tuned around today's model** (edges at full weight, random walk 0.0001) → the L2 is judged on 44
  on its own; no Livox number is copied.
- **Replays start from the old model's state and hold few seen windows** (D7) → one full Livox 3 run per layer.
- **A family's spread can be as large as an effect** (lower corridor, floor 0.3: 1.9 m from 0.1 % changes) → every
  comparison on a capture is a family of seven against the family of the same code without the layer.
- **Layers 1 and 2 may not be enough**, and layer 3 may not be justified → then the result of this change is a
  correct carry, the readings, and the measured statement of what is still wrong (D6 or the LiDAR side), for the
  user's decision on the next change.

## Migration Plan

1. J0 on the feature branch `honest-inertial-carry`; step checks: everything bit-identical.
2. Layers 1 and 2 (J2, J3, J4), each merged with its switch off after ownership check, step checks and the conclusion
   reviewer; each replayed (J5) and run once on Livox 3 before the next is judged. Numbers to the user.
3. Task 5.1; the user's decision on layer 3.
4. Full runs (J7), the user's look and decision.
5. On acceptance: values in `settings/`, golden TOML, baselines (44, 44_short, guards, captures re-made with the new
   carry), switches removed, docs and knowledge files. On rejection of a layer: its code removed, the finding recorded
   in the knowledge files.

Rollback at any point: the switches are off by default, so the main tree's results never changed.

## Open Questions

- What makes E2: the accelerometer, the hand-off, the LiDAR's confidence or the attitude? Tasks 1.6 and 5.1.
- Why does the LiDAR pull against travel in the first weak windows after a blind stretch (E8)? Not treated here; the
  tool reports it per exit.
- Does `imu_position_information_scale` still have a job once the edges follow the state? Settings audit, after this
  change.
