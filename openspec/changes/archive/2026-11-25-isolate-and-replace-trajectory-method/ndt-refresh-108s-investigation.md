# Existing-data investigation of refreshed-guard rotation at108seconds

Scope: existing125s replay/source71e2fa5 only. No optimizer change, build or new
recording run. User requested independent review. High effort is sufficient.
Evidence: external `livox4-iteration-refresh-125s/event108-analysis.json`, generated
by `tools/analyse_ndt_refresh_event.py`; hashes cover log/snapshot/tool. Tool validates
100 admission records and99 turnover records per grid in each W296–313. Saved
snapshot structure/finiteness is validated by the existing reader. Python compile
and actual-data coverage checks passed.

## Observed sequence

| Window/log-relative time | Heading correction | Final yaw stiffness | Admitted unique sources i0 to i99 |
|---|---:|---:|---:|
| W298/106.92s | -2.247deg | 36160 cost/deg² | 10605 to10737 |
| W300/107.64s | -2.064deg | 6859 cost/deg² | 5854 to6154 |
| W301/108.00s | -8.600deg | 14480 cost/deg² | 7705 to9711 |
| W302/108.36s | -9.095deg | 12460 cost/deg² | 6419 to7531 |

Yaw stiffness is a local final-state diagnostic, not proof of rotational
observability or comparable calibrated uncertainty. Its drop from51810 atW297
to6859 atW300 is context for the event, not an isolated cause.

W301 fine/coarse matches grow4562/5608 to5301/7642. Across99 refresh boundaries,
fine/coarse reassignment counts sum3029/3380; W302 sums3965/2600. These are repeated
source reassignment events, not distinct bad points. Neither increased support
nor valid per-cell covariance proves the new correspondences are correct. The
same fixed source set is admitted again at the current changing pose.

The W301 endpoint translation correction is(-.022,+.060,-.002)m; W302 is
(+.015,+.042,+.004)m. Small translations can coexist with large angular updates.
Whole-run counters report zero solver steps skipped, zero rejected big-jump nodes,
and zero map rebuilds. All100iterations were performed in each event window.

## Code mechanism verified

Scratch `src/application/run_odometry.cpp:427` rebuilds guarded matches before
each iteration from the current poses. The set is immutable for that solve.
`src/domain/ndt_match_guard.cpp:94` rejects invalid/incompatible individual cells
and requires6unique returns; it does not establish global correctness, angular
observability or agreement with the gyro trajectory.

`src/domain/scan_to_map.cpp:258` adds translation AND Euler-angle increments.
`stepped_poses` compares only translation against the motion-model pose with a
1m threshold. `scan_to_map.cpp:1374` commits a successfully solved step if no
translation node trips that threshold. It does not require a full-objective cost
decrease or bound the angular increment. `next_lm_factor` changes damping on a
fixed schedule/rejection, not an actual/predicted cost-reduction ratio. These are
known optimizer acceptance weaknesses, not a newly discovered sign error.

Consequently a rotating solution can be committed, then have its matches rebuilt
around that new orientation. More locally valid matches can support the changed
orientation. That is a plausible reinforcing feedback mechanism compatible with
this run. It is not proven that any specific refreshed match caused the first
bad angular step. No complete comparable objective sequence was captured at
W301–302, so do not assert that their total objective increased.

After optimization, `run_odometry.cpp:1247` places the next window from the current
end pose, then inserts the optimized points into the map. Therefore accepted
orientation error can affect both later predictions and map targets. There was
no abrupt map rebuild at this event. This downstream coupling is verified code
behavior; the magnitude attributable to it here remains unisolated.

## Alternatives checked and limits

- Gyro bias Z changes0.086 to0.210 to0.341deg/s across W300–302. The increments
  would contribute only about0.045/0.047deg over0.36s if integrated directly,
  far below8–9deg. This excludes the bias increment alone as a direct explanation,
  not joint rotational/inertial coupling.
- Saved W301/302 attitudes have maximum absolute pitch2.31/2.34deg, determinant
  essentially1 and orthogonality errors below5e-16. No saved-state gimbal-lock
  proximity or invalid rotation matrix is evident. Intermediate states are not
  captured, so this is not a guarantee about every iteration.
- Maximum saved IMU node gaps are2.55/2.71ms. This checks node timing only, not
  all raw packet validity or exact internal LiDAR/IMU synchronization.
- Saved `motion_model_poses` equal final poses after handoff; do not treat these
  as a recoverable pre-optimization reference. Log-relative108.00s corresponds
  to first saved sensor timestamp108.35482573s forW301; keep clocks distinguished.
- The earlier W297–300 factor trace passes400iteration checks, but ends before
  W301. It cannot name the first factor/cell responsible forW301–302 orW312.
- Loop closure runs afterward; these corrections are already in odometry.

## Decision boundary

Best supported conclusion: whole-window freezing was too restrictive for
translation; refreshing restores motion correction but does not make the
optimizer's angular acceptance safe. Validity guards alone cannot solve this.
The exact W301 initiating factor remains NOT MEASURED.

The next design worth considering is objective-consistent trial acceptance with
rotation-aware trust-region/step control and refreshed associations only between
accepted iterations. This is a proposal, not an implemented or proven fix; an
angle clamp alone could conceal genuine turns, and changing matched-only costs
cannot serve as a consistent acceptance objective. A focused W301–302 trace would
need explicit new-run approval. Do not launch it automatically.

No new runs, no algorithm edits, no default/adoption decision. Charlie3/19.

Independent review completed (trace_review, Codex): confirms acceptance deficiency,
no stale-set/ownership bug, and correspondence feedback as a supported hypothesis
rather than established initiating cause. Zero rejected steps also excludes a
rejection-path covariance update as this run's trigger. No additional run or
algorithm change was performed by the reviewer.
