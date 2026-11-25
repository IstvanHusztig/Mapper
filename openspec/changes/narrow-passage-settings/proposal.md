# Proposal

## Why

The first recording of a narrow passage (`raw data/livox/5`, 2026-11-25: a long underground corridor with doors and
niches, walked out and back by hand at a steady pace, 110.7 s) comes out of dMapper collapsed. With the shipped Livox
settings the corridor is 12 m long in the map; by the time walked it is several times that. The files are clean
(`tools/imu_health.py`: no lost samples, no false gyro offsets), so the fault is in how dMapper treats the passage:

- **The scans do not see along the passage.** In the recorded frame the returns lie in a band from -7 to +53 degrees
  off the plane across the sensor's x axis, with none within 30 degrees of that axis, and the corridor runs along it.
  While walking, half the returns are within 1.15 m and 99 % within 2.74 m. A door or a niche is seen only while the
  sensor is beside it. 223 of 306 windows have an information ratio below 0.05 along the passage.
- **The odometry does not hold the travel.** Along the corridor its position goes 0, 7.2, 2.2, 5.3, 1.1, 4.7, 1.5, 7.9,
  6.0, 9.4 m and ends 6.5 m from the start, for a walk that went out once and came back once; its path is 49 m.
- **The loop closure makes it worse.** Every stretch of such a corridor looks like every other. 34 loops are accepted
  with corrections up to 8.3 m, none is dropped by the match check (the odometry steps are loosened along the passage
  by up to 48 m, so every match is consistent), and the corrected position jumps by about 10 m within a second.
- **The fine-time step does nothing**: three blind runs (sigmas 48, 14 and 22 m) lie so close that their stretches
  overlap, and all three are left as they are.

The target of dMapper is cave passages, which are this case. The user decided on 2026-11-25 to start with a settings
file of its own for narrow passages, and to keep every run on the way as data for the planned learned tuner.

## What Changes

- **A way to judge a passage walked out and back without knowing its length**: one tool that reads a result and
  gives, along the passage, whether the way back mirrors the way out, how even the pace is, and whether the surfaces
  across the passage (door frames, niche edges) seen on the way out lie where they are seen on the way back. Stated
  before any candidate is run.
- **A diagnosis first, with replays**: where the odometry loses the travel (the per-term table on a window capture of
  two blind stretches), which loop edges are false, and what the readings rule and the fine-time step do with stretches
  this close together. No setting is chosen before its numbers.
- **A listed set of candidate settings**, each answering one question (how far the accelerometer is trusted and how
  fast its offset may follow; the weight of the scan matching and of the degeneracy projection where the scans are
  blind; the loop closure off, stricter, or with a shorter search; the fine-time step's overlap), screened by replay and
  confirmed by full runs (7 minutes each on this recording), the survivor checked for robustness with the perturbation
  sweep.
- **`settings/livox-narrow.toml`**, shipped only after the user's look at the maps: the Livox Mid-360S in a passage
  where the scans do not see along the travel. Its header names the recording it was tuned on and what it was judged
  by. `settings/livox-vertical.toml` does not change.
- **Every run is a training record**: each candidate's settings, its per-window features and terms
  (`window_features.csv`, `window_terms.csv`), its log and its measures are kept together, with one table of all runs
  (settings that differ, measures) under `raw data/livox/ml_data/livox5/`.
- **What settings cannot do is said up front** (design D6): with the scans blind along the travel, the travel comes
  from the inertial terms, whose error grows with the square of the blind time. The file can make the map better, not
  right. Three things outside a settings file are named as follow-ups, each a change of its own, to be decided on this
  change's numbers: a speed from the walker's steps, loop edges that carry no information along the passage from scans
  that see nothing across it, and holding the sensor so that its band of view lies along the passage.

## Capabilities

### New Capabilities

### Modified Capabilities
- `sensor-settings`: a sensor may have a further shipped settings file for a kind of place; the Livox gets one for
  narrow passages.
- `verification-evidence`: a result of a passage walked out and back can be judged without a reference length.

## Impact

- New: `tools/passage_check.py` with tests (the verification region); `settings/livox-narrow.toml` and its golden TOML
  (after the user's decision); experiment folder `experiments/passages/livox5/`.
- Docs: the guide (which settings file for which place; what a narrow passage costs), `tools/README.md`,
  `checks/instruments.toml` (the new tool's section), `docs/regions/verification.md`, `CLAUDE.md`.
- Runs: one full run with a window capture and the per-term table; replays; about 10 to 20 full runs of 7 minutes
  through the gate, in the background, one at a time; one perturbation sweep (7 runs).
- No change to `src/`, to the shipped Livox or L2 files, or to any baseline. The check register: additions only.
