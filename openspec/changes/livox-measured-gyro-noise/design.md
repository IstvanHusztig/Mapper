# Design

## Where the values come from

- ARW 0.0094 / 0.0224 / 0.0144 °/√s: measured on `raw data/livox/4`, 2–20 s at rest (2026-11-15,
  `openspec/changes/isolate-and-replace-trajectory-method/checkpoint-mamba.md`, "State when stopped").
- Inflation k = 1: the measured noise is used as it is. Stage 8 replays: k = 1 alone (with the L2's ARW) still gives
  speed bursts to 5.5 m/s in the flyaway; the measured ARW with k = 1 gives 1.4 m/s.
- Floor 0.0002°: below the noise of one IMU step (0.0094 °/√s × √0.002 s = 0.00042°), so it never binds at 500 Hz; it
  only guards against a zero sigma.

## Evidence (stage 8, `experiments/stage8/`)

| | Shipped | Measured gyro noise |
|---|---|---|
| Livox 4: windows bent > 3° / largest speed / end height | 96 / 8.5 m/s / −78 m | 0 / 1.62 m/s / −0.7 m |
| Livox 4: loops accepted, largest correction | 23, 5.7 m | 66 loops kept, 4.36 m |
| Livox 3: largest bend / start-to-end before loop closure | 1.98° / 0.08 m | 0.30° / 0.07 m |
| The user's look | Livox 3 good, Livox 4 flies away | Livox 3 "crisp as expected", Livox 4 "a huge win" |

The run check (`tools/run_health.py`) on the two measured-gyro runs: bend and speed CLEAR on both; Livox 4 bias
estimate 0.58 °/s TRIGGER (two windows), velocity chi² TRIGGER on both (also with the shipped values).

## Decisions

1. **Livox file only.** The L2 settings keep their tuned inflation: the user's L2 has a faulty gyro, and its baselines
   are the regression reference of every refactor.
2. **The runs that become the reference** are made again from the committed settings file (not the scratch copies the
   user looked at, which also had the per-window logs on). The odometry must be identical to `full_gyro_livox3/4`:
   checked with `tools/trajectory_seams.py` (difference 0) before the manifests are written. The logs change no
   result, so this is expected.
3. **Old reference folders** are kept with the suffix `_2026-11-18` (CLAUDE.md, golden baseline).
4. **Docs:** the per-term table page's example reading was made with the old values; it keeps its numbers and says
   which settings they came from, rather than being redone.

## Not in this change

The rotation drift, the passage stall and the in-plane rule (stage 8's open questions).
