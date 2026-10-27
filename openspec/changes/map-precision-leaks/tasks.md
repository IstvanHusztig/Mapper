# Tasks

## 1. Locate the leaks

- [x] 1.1 `plan_section.py`: `--threshold` takes several values (one image each) and writes one combined image coloured by the highest threshold a line survives; README updated. Verify: the defaults reproduce today's `plan_section_+0.00_t0.03.png` pixel for pixel.
- [x] 1.2 Run the sweep 0.01 / 0.03 / 0.1 / 0.3 on `44_baseline` at heights −0.6, 0.0, +0.8. Verify: images written for every combination.
- [x] 1.3 List the suspect walls (map coordinates, what is wrong: faint, wavy, doubled, off the plan) that show at more than one height. Verify: the list and the images are shown to the user.

## 2. Diagnose each suspect wall

- [x] 2.1 `tools/wall_precision.py`: per ~0.5 m piece of section line, thickness, visits and their offsets, seen time, points, range, incidence, odometry windows; optional plan distance; one CSV row per piece and images coloured by thickness and by visit disagreement; README section. Verify: on `44_baseline` the lift shaft walls come out thin, the visit-offset measure's own noise (`half_split_m`, a visit's halves against each other) is well below the visit disagreement it is used to detect, the walls that vanish at threshold 0.15 come out with short seen time, and it runs without the plan file. (Done 2026-10-18: shaft thickness 2.5 cm; noise floor 1.2 cm p50 against visit spread 5.7 cm overall and 8.2 cm on the shaft — the shaft's visits do not agree, the 215–240 s visits stand out by 8–11 cm, a finding for 2.2; vanished at 0.15: seen 8.0 s vs 19.3 s.)
- [x] 2.2 Run it on `44_baseline` and collect the measures of the suspect walls of 1.3 with the diagnostics of their odometry windows. Verify: the table is shown to the user.

## 3. Separate the causes

- [x] 3.1 Make the comparison results myself (Release, `l2-vertical.toml`, window diagnostics on): an odometry-only result of 44 (loop closure and consistency off, odometry snapshot saved) and s0/s1/s2. Verify: the folders exist and the logs end without errors.
- [x] 3.2 Run the diagnostics on them (a research subagent if the data calls for it) and sort each suspect wall into sensor-side / pose error in one visit / drift between visits / noise. Verify: the subagent's numbers are checked against the CSVs.
- [ ] 3.3 Have the conclusions reviewed, write them into `findings.md`, show the numbers and images to the user, and ask how to proceed before any verdict or fix. Verify: `findings.md` exists and the user has answered.

## 4. Second round (user, 2026-10-18: "another round with xhigh", plus a code review)

- [x] 4.1 `wall_precision.py`: visits split by sensor side, and the visit disagreement reported in a form that does not grow with the number of visits (per visit offset, and the spread at equal visit count); README. Verify: on `44_baseline` the two-visit spread stays ~2 cm and the per-visit numbers don't depend on the visit count.
- [x] 4.2 Ablation of the post-processing on 44 from one odometry snapshot: full, loop closure only, consistency only, neither. Verify: the full run equals the 44 baseline and "neither" equals `44_odo`; jitter and visit disagreement per variant.
- [x] 4.3 Heading at 255–260 s from the raw LiDAR scans (registration of raw scans, independent of the odometry) against the gyro and the result. Verify: the registration is checked on a stretch where gyro and result agree.
- [x] 4.4 Code review of the critical parts (odometry LM loop, window hand-off, gyro bias, consistency, loop-closure corrections) by a review agent. Verify: every finding is checked in the code before it is reported.
- [x] 4.5 Research agent (xhigh): mechanism of the damping dependence (odometry only, 44_short) and of the consistency jitter. Verify: its numbers are checked and its conclusions reviewed before they are reported.
