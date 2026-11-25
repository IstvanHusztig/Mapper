# Design

## Context

See proposal.md for the motivation and the first run's numbers (`experiments/consistency/livox4_lever/fine_time/
livox5_shipped`, picture `livox5_top_view.png`). What shapes the approach:

- **The recording.** `raw data/livox/5`: 110.7 s, 306 windows of 0.36 s; at rest for about the first 12 s and the last
  10 s; walked out and back in between (the user: by hand, steady pace, doors and niches along the corridor; its length
  is not known). One full run takes 7 minutes (odometry 322 s, loop closure 50 s), so full runs are cheap here.
- **The view.** The sensor's band of view (-7 to +53 degrees off the plane across its x axis) leaves a cone of 37
  degrees round that axis unseen, and the passage runs along it. Walls at 1 m are seen up to about 1.3 m ahead. This is
  the same mount as the earlier Livox recordings; rooms gave those their surfaces across the travel.
- **What exists.** The readings rule and the match check, the fine-time placement, window capture and replay, loop
  closure capture and replay, the per-term window table, `run_health.py`, `honesty_report.py`, the perturbation sweep,
  the run gate. `tools/loop_fine_time.py` has the patches of surfaces across an axis. The folder
  `raw data/livox/ml_data/` holds per-window feature tables of two earlier recordings and a README.
- **Earlier findings that bear on it** (knowledge files): with the scans blind, the placed move follows the inertial
  guess; an attitude error of 1 degree leaks 0.17 m/s^2 of gravity into the travel; the gyro bias estimate is disturbed
  at such events (here it reaches 1.1 deg/s, with the raw IMU's last quiet block at -1.08 deg/s on z); on Livox 4 one
  loop edge from scans that saw nothing across the passage placed a stretch falsely (change `blind-stretch-zigzag`).
- **Rules that apply.** A value is not tuned to the noise: a candidate counts only beyond the spread of the
  perturbation family. The user judges the maps. Settings change only with the user's decision.

## Goals / Non-Goals

**Goals:**
- Measures for this kind of recording that need no length and are fixed before the candidates run.
- To know, with numbers, where the travel is lost and which loops are false, before any value is chosen.
- One settings file that makes this map as good as settings can, with its limits written into its header.
- Every run kept as a record a learned tuner can read.

**Non-Goals:**
- New estimator code in this change (see D6 for what is named and left).
- A file that is right for every passage: it is tuned on one recording and says so.
- Changing the shipped files, or the recorder.

## Decisions

### D1 The measures (fixed here, before any candidate runs)

All along the passage's main direction `a`: the horizontal direction from the start to the point of the trajectory
farthest from it, in the result that is judged. Positions are the nodes' after the stage that is judged (odometry
snapshot, or the final trajectory). "Moving windows" are those not at rest by the readings.

- **M1, the mirror.** The turning point is the node farthest along `a`. With `s(t)` the position along `a` and `t*` its
  time: the root mean square of `s(t* - u) - s(t* + u)` over the `u` both ways cover, and the largest. A walk out and
  back at a steady pace gives 0. Also: the passage's length in the result (`s` at the turning point) and `|s(end) -
  s(start)|`.
- **M2, the pace.** The travel along `a` per moving window, signed toward the turning point before `t*` and away from
  it after: its median, its spread (the median absolute deviation over the median), and the number of moving windows
  with a travel below a quarter of the median or negative ("stands or goes back").
- **M3, the surfaces across the passage.** The patches of `tools/loop_fine_time.py` (planes whose normal lies within 60
  degrees of `a`) of every moving window before `t*` and of every moving window after it, placed; their counts in bins
  of 0.1 m along `a`; the correlation of the two histograms at no shift, and the shift (within +-10 m) at which it is
  largest with its value. Fewer than 50 patches on either way: NOT MEASURED.
- **M4, readings without a limit here:** the run check's rows (bends, speed, gyro bias, chi-square, time-limit windows),
  the number of weak runs and their sigmas, the loops kept and the largest correction, the fall of every weak run.

What they assume and cannot tell: that the walk was out and back along one passage at a steady pace (the user's
word for this recording); M1 and M2 are zero for any placement that is symmetric and even, also a wrong one that is too
short by the same factor both ways; M3 can agree at a false shift where door frames repeat at even spacing, which is why
its value at no shift and its best shift are both given. None of them is accuracy. A length would decide much: if the
user can pace or measure the corridor later, it becomes the fourth measure.

- *Alternative: only the user's eye.* It stays the verdict, but twenty maps cannot be compared by eye alone.
- *Alternative: the distance between the two ends of the trajectory.* Zero after any loop closure that matches the
  end to the start; it is a reading, not a measure.

`tools/passage_check.py` computes them from a result folder (and its odometry snapshot when present), in pure Python
like the other tools, with tests on made-up trajectories and points (a folded passage; an even out and back; a passage
without surfaces across it; door frames at even spacing).

### D2 Diagnosis before candidates (stage 1)

1. **A run with the instruments on**: the shipped settings with `log_window_terms`, `log_window_noise`, a window
   capture of two blind stretches (the first weak run's start and the stretch round the turning point), the snapshot and
   the loop closure capture. Results unchanged by the logging; 9 % more time.
2. **The odometry alone** (loop closure off, from the snapshot): M1 to M4. This separates the two faults.
3. **Where the travel goes in a blind window**: `tools/window_terms.py` on the capture (which term places the move
   along the travel: the scan matching's leftover weight, the motion edges, the accelerometer term, the velocity and
   offset priors), `tools/honesty_report.py` (the carried velocity's sigma against its corrections through the blind
   stretches, the offset estimate against its prior), and the placed speed along `a` against time.
4. **Which loops are false**: for every kept edge, the two segments' times and where on the way out and back they lie
   (an edge between two places that M1's mirror puts metres apart is suspect), its time-support reading, the match
   check's value; the loop closure replayed with the edges of each kind left out (`--reject-tries`).
5. **Why the three stretches overlap**: the weak runs, the windows between them and their readings.

Stage 1 ends with a table and a stop: the numbers go to the user before a candidate is run.

### D3 The candidates (stage 2): each one question, listed before the runs

Odometry (screened by window replay on the two captures, then by full runs):

| Question | Setting | Shipped Livox value | Tried |
|---|---|---|---|
| Does a slower acceleration offset hold the travel better when nothing observes it? | `imu_acceleration_offset_rw_mps2_sqrt_s` | 0.01 | 0.001, 0.0001 |
| Is the accelerometer trusted too little against the vibration floor? | `imu_accel_floor_sigma_mps2` | 0.1 | 0.05, 0.03 |
| Does the scan matching still hold the travel back where it is blind? | `degenerate_projection_ratio` / `_full` | 0.02 / 0.04 | 0.05 / 0.10, 0.10 / 0.20 |
| Its weight against the inertial terms | `ndt_information_scale` | 0.1 | 0.03, 0.3 |
| The motion edges' position weight | `imu_position_information_scale` | 0.1 | 0.03, 0.3 |
| Finer or coarser cells for walls at 1 m | the indoor NDT resolution (`resolution_X/Y/Z`) | 0.15 m | 0.10, 0.20 |
| The attitude filter's feedback | `ahrs_gain` | 0 | 0.05 |

Loop closure (from the odometry snapshot, about a minute each; a loop closure replay where the capture allows it:
the search radius, the largest correction and the segment length need registrations the capture does not hold):

| Question | Setting | Shipped value | Tried |
|---|---|---|---|
| Is the map better with no loop closure at all? | `use_loop_closure` | true | false |
| Fewer far matches | `loop_closure_search_radius_m`, `loop_closure_max_correction_m` | 8, 6 | 3 and 2; 2 and 1 |
| Stricter acceptance | `loop_closure_min_fitness` | 0.5 | 0.7, 0.85 |
| Longer segments, so that a door falls into more of them | `loop_closure_segment_windows` | 6 | 12 |
| A tighter stretch for the fine-time step | `loop_closure_fine_time_min_sigma_m` | 1.0 | reported only: the overlap rule is in the code |

Rules of the stage:
- One at a time against the shipped values first; then the combination of those that move a measure beyond the spread
  of repeated identical runs (three repeats of the base give that spread; the machine otherwise quiet).
- A candidate that improves one measure and worsens another is reported as that; nothing is averaged into a score.
- The survivor gets the perturbation sweep (`tools/perturbation_sweep.py`, base plus six) and is called better only
  beyond the family's spread.
- The user looks at the maps of the base, the odometry alone and the survivor before the file is written.
- Values outside the table are tried only after they are written into this table with their question.

### D4 Every run a record

Folder per run under `experiments/passages/livox5/runs/<id>/`: `settings.toml`, the log, `window_features.csv`,
`window_terms.csv`, the trajectory files, `passage_check.txt`, `run_health.txt`. One table
`raw data/livox/ml_data/livox5/runs.csv`: the run's id, the settings that differ from the shipped file
(`key=value;...`), the stage (odometry, final), M1 to M4 as columns, the commit, and a README that says what each column
is, what the measures assume, and that the outcome columns are agreement readings and not truth. Clouds are kept only
for the runs the user looks at (88 MB each).

### D5 The file

`settings/livox-narrow.toml` = `settings/livox-vertical.toml` with the survivor's values, written only after the user's
decision. Its header: the Livox Mid-360S; a passage where the scans do not see along the travel; tuned on
`raw data/livox/5` (one recording); judged by the measures of `tools/passage_check.py` and the user's look; what it
cannot repair (D6). A golden TOML and the load-and-save test like the other two files. If no candidate is better than
the shipped values beyond the spread, no file is shipped and the change closes with its diagnosis.

### D6 What settings cannot do, and what is left for later

With the scans blind along the travel, the position along it is the double integral of the accelerometer between the
places where a door or niche is seen. An offset of 0.02 m/s^2, or 0.1 degree of tilt, is 4 m after 20 s. Settings choose
how the error is spread and how much the scans may pull where they see nothing; they do not remove it. So:

- **A speed from the walker's steps.** A hand-held sensor shows the steps; their rate gives a speed good to some tens
  of per cent without any drift. It would be a new term in the odometry (the inertial region), for hand-held use only.
- **Loop edges without information along the passage from scans that see nothing across it** (the graph-level repair
  read in `blind-stretch-zigzag`): in this recording every edge is of that kind, which is why the loop closure folds the
  corridor.
- **The sensor's band of view along the passage.** Held with its axis across the passage or upright, the sensor would
  see along it for tens of metres. That is how the recording is made, not dMapper's code; it is named here because it
  is the cause.

Each is a change of its own. Which comes first is decided on this change's stage 1 numbers.

## Risks / Trade-offs

- [The measures reward a wrong map that happens to be symmetric and even] → said with each measure; M3 and the user's
  look are the checks; a measured length later would settle it.
- [Tuning on one recording] → the file's header says so; the sweep guards against tuning to noise, not against tuning
  to this corridor.
- [Twenty runs on one input folder] → one at a time through the gate; a `--from-snapshot` run shares the recording's
  cache, so no two jobs on `raw data/livox/5` at once.
- [A candidate that only helps because loop closure is off] → the odometry-alone result is measured for every candidate,
  and loop closure settings are judged on the same snapshot.
- [The file invites use where it does not fit] → the guide says for which place it is meant and what it costs elsewhere
  (the candidates that survive are run once on Livox 4 and Livox 3 as a reading before the file ships).

## Open Questions

- The corridor's length, if the user can pace it: the fourth measure.
- Whether the second attempt at the recording can be repeated with the sensor held the other way, as a comparison of the
  view (the user's choice; not needed for this change).
