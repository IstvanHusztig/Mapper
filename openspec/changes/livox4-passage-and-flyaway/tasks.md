# Tasks

## 1. Captures

- [ ] 1.1 Full Livox 4 run with the shipped settings, every per-window log on, capture 95–125 s
  (`experiments/stage8/run_passage`). Verify: the capture exists; the run's verdict summary equals the stage 4 run's.
- [ ] 1.2 Full Livox 4 run, capture 195–285 s (`experiments/stage8/run_flyaway`; loop closure and consistency off).
  Verify: the capture exists.
- [ ] 1.3 Both captures replay identically with their own settings (`tools/run_replay.py`).

## 2. Reading

- [ ] 2.1 The passage (100–110 s): which term moves the windows, hold and pull per term, which term gave way.
- [ ] 2.2 The flyaway (onset 205–210 s, then 210–285 s): the same.
- [ ] 2.3 The velocity chi² question carried from stage 4: what the carried velocity's hold is made of.

## 3. Experiments

- [ ] 3.1 Region job lidar-matching: which part of the NDT term pulls the passage windows back (normal / in-plane,
  ahead / behind, per grid). Ownership check, verification, conclusion review.
- [ ] 3.2 Replays of both captures with the gyro's measured noise (ARW 0.0094 / 0.0224 / 0.0144 °/√s, k 1, floor
  0.0002°) and with k 1 alone: distances, bends, speeds, the per-term table where a window still goes wrong.
- [ ] 3.3 Further experiments the readings ask for (listed here as they are decided).

## 4. Report and decision

- [ ] 4.1 The numbers for the user, without a verdict; the conclusion reviewed (Sonnet) before it is stated.
- [ ] 4.2 The user's decision: which fix is proposed as its own change, or what is looked at next.
- [ ] 4.3 Knowledge entries of the regions that took part; tracker rows for defects found.

## Open, for a later look at higher effort

(Questions this change does not settle, with what was tried.)
