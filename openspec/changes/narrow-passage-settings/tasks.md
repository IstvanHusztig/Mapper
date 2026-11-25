# Tasks

Experiment folder: `C:\Users\User\Downloads\dLidar\experiments\passages\livox5\`. Runs go through the gate, one at a
time, in the background; never two jobs on `raw data/livox/5` at once. Release only. Numbers before verdicts; the user
judges the maps. No file under `settings/` changes before task 4.1.

## 1. The measures and the diagnosis (design D1, D2)

- [ ] 1.1 `tools/passage_check.py` (the verification region's agent): M1 to M4 of design D1 from a result folder and its
      odometry snapshot; tests on made-up trajectories and points (a folded passage, an even out and back, a passage
      without surfaces across it, door frames at even spacing: the tool reports what the design says for each). Done
      when the tests pass, the step checks hold (the register: additions only) and the tool's section is in
      `checks/instruments.toml` and `tools/README.md`.
- [ ] 1.2 The measures on what exists: `livox5_shipped` (final trajectory and odometry snapshot). Done when both rows
      are in `experiments/passages/livox5/results.md`, with the picture `livox5_top_view.png` beside them.
- [ ] 1.3 The run with the instruments on (shipped settings, `log_window_terms`, `log_window_noise`, window captures of
      the first blind stretch and of the stretch round the turning point, snapshot, loop closure capture). Done when its
      result is bit-identical in trajectory with `livox5_shipped` and the captures replay bit for bit.
- [ ] 1.4 The odometry alone (loop closure off, from the snapshot): M1 to M4. Done when its row is in `results.md`.
- [ ] 1.5 Where the travel goes in a blind window (design D2.3): the per-term table over the two captures, the honesty
      report, the placed speed along the passage against time. Done when `results.md` has the table and one paragraph of
      what was measured, without a verdict.
- [ ] 1.6 Which loops are false and why the stretches overlap (design D2.4, D2.5). Done when `results.md` lists every
      kept edge with its two times, its place on the way out and back, its readings, and the replays without each kind.
- [ ] 1.7 The conclusion reviewer on `results.md`; the numbers and the pictures to the user. **Stop: the user decides
      whether stage 2 runs as listed, and which follow-up of design D6 comes first.**

## 2. The candidates (design D3, D4; only after the user's go)

- [ ] 2.1 The record's frame: `experiments/passages/livox5/runs/`, `raw data/livox/ml_data/livox5/runs.csv` and its
      README (design D4), filled for the runs of group 1. Done when the base runs are in it.
- [ ] 2.2 Three repeats of the base, the machine quiet: the spread of M1 to M3. Done when the spread is in `results.md`.
- [ ] 2.3 The odometry candidates one at a time (window replays first, then full runs for those that move the placed
      travel in the blind stretches). Done when each row of design D3's table has its measures beside the base and its
      spread.
- [ ] 2.4 The loop closure candidates on the base's snapshot and on the best odometry candidate's. Done when each row
      of the table has its measures.
- [ ] 2.5 The combination of those beyond the spread; the perturbation sweep on it. Done when the sweep's table is in
      `results.md` and the combination's measures are given with the family's spread.
- [ ] 2.6 The survivor once on Livox 4 and Livox 3 (from their snapshots where the odometry settings allow, else a full
      run): position differences to their shipped results, as a reading for the guide. Done when both are in
      `results.md`.
- [ ] 2.7 The conclusion reviewer on the results; the maps of the base, the odometry alone and the survivor to the user
      (clouds and top views). **Stop for the user's decision on the file.**

## 3. The file (only on the user's decision)

- [ ] 3.1 `settings/livox-narrow.toml` with its header (design D5), the golden TOML in its own commit, the load-and-save
      test. Done when the unit tests pass and a run of Livox 5 with the file reproduces the survivor's result bit for bit.
- [ ] 3.2 Docs: the guide (which file for which place, in general words), `CLAUDE.md`, `docs/regions/` notes of the
      regions whose settings it changes, `tools/README.md`; the register recorded. Done when
      `python tools/build_docs_index.py --check` holds and `python checks/check_register.py` exits 0.

## 4. Close

- [ ] 4.1 The step checks and the end runs (the shipped files unchanged: all IDENTICAL); the master reviewer on the push.
- [ ] 4.2 The follow-ups of design D6 written into the regions' open questions, with this change's numbers; archive.
