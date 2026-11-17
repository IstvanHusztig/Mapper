# Full Livox2: traced onset mechanism

Analysis uses the exactly reproduced run described in full-livox2-trace-results.md.
No additional SLAM run, setting change or proposed correction was applied.

## New observation: optimization changes tilt at the onset

At window754 (start271.08s), the endpoint orientation changes by15.189degrees
in tilt relative to its incoming orientation and the fixed incoming gravity
direction. This is an optimizer-induced change, not measured absolute attitude
error. All100 iterations are accepted; all three projection scales remain1
throughout this window. Projection suppression therefore does not initiate this
particular transition.

| Iteration | Endpoint tilt change from input | Endpoint speed |
|---|---:|---:|
|0|0.274degrees|0.475m/s|
|10|5.629degrees|0.483m/s|
|20|10.394degrees|0.669m/s|
|40|15.035degrees|0.858m/s|
|99|15.189degrees|0.890m/s|

For the same recorded IMU samples, unweighted node-mean rotated acceleration minus incoming
gravity changes from[-0.328,0.170,0.145] to[0.022,-1.469,0.005]m/s2 over the
window's optimization. The next window starts with mean[-0.046,-3.053,-0.650]
m/s2 in its predicted orientation. Window755 has new IMU samples: its value
cannot be attributed entirely to inherited rotation. Its endpoint speed reaches1.535m/s; window756
reaches1.873m/s,759 reaches2.617m/s and771 reaches5.243m/s.
The window754 estimated acceleration offset is only
[0.00053,-0.00049,0.00166]m/s2. Folding that offset into gravity is not large
enough to explain the within-window15-degree attitude change.

These means omit the estimated offset and are not the optimizer's integrated
or time-weighted residual. An independent raw-trace check of754 found left-time-
weighted means[-0.326823,0.173701,0.140681] before and
[0.021419,-1.458559,0.002113] after, confirming the substantial Y change.

This establishes a concrete propagation path: accepted attitude changes alter
the acceleration used by subsequent translation prediction, while endpoint
velocity carries forward. It does not establish true physical acceleration or
absolute attitude error without independent reference.

## Which observations favor the onset steps?

For the actual proposed joint update h, use B=b^T h, C=h^T H h and q=C-2B.
Positive B favors the initial direction; negative q favors the full proposed
step in the local quadratic model. These are joint pose/velocity/bias/offset
contributions, not isolated tilt forces or a nonlinear objective guarantee.

At window754 iteration20, q for NDT is-2480.42, motion+785.93,
accelerometer+11.50 and carried velocity-23.91. NDT favors the joint step while
motion and accelerometer factors resist it locally. Iteration0's large negative
acceleration contribution must not be misread as sole evidence that acceleration
initiates tilt: it also reconciles auxiliary endpoint velocities.

The actual projected NDT B by sensor-range bin at iteration20 is:

| Range | B |
|---|---:|
|0–2m|-245.09|
|2–5m|-280.48|
|5–15m|2572.26|
|15m and farther|0|

Thus the longer-range minority favors this accepted joint step while nearer
returns oppose it. The same sign pattern occurs at iteration10. There are897
source points in5–15m out of23470 (3.82%); initially145 indoor-grid matches and
275 outdoor-grid matches. The grids can use the same points, so these are not
420 unique points. Numerous associations change during optimization.
As a descriptive sum over relinearized iterations1–99, NDT q totals-62627,
motion+19117, accelerometer+340 and carry+1110. Range-bin q totals are
[+15241,+11231,-89099,0]. These sums are not one fixed objective difference;
they confirm the repeated direction of local preferences across the updates.

This corrects the earlier emphasis on close surfaces: close returns dominate
counts, but do not alone explain the onset. Range alone does not prove a point
is erroneous. Whether the5–15m contribution comes from legitimate geometry,
wrong associations, reflection, map distortion or disproportionate covariance
weighting remains unresolved. Do not select a range cutoff from this evidence.

## Second escalation and safeguards

Window863 and867 also show endpoint tilt changes of11.132 and11.121degrees
within optimization. Their later strong motion occurs amid already distorted
state/history; they are not independent reproductions of the first onset.
Projection suppression does occur later, but is absent throughout754.

Source checks confirm accepted steps require a successful solve and staying
within1m of the predicted trajectory; they do not require a complete nonlinear
objective decrease. End velocity and conditional covariance carry forward;
speed fallback does not restore earlier accepted poses or map history.

## Evidence and limits

`tools/trace_summary.py` streams the2.5GB trace into compact JSONL, preserving
repeated scalar records and explicitly reducing large matrices. It does not
run SLAM. External `full-livox2-trace/analysis/manifest.json` binds the original
trace hash, tool and summary. Source hash matches the validated replay.
All11600 iteration groups and116 windows are present. The four projected range
quadratics sum to the NDT factor for both C and B within numerical rounding.
Independent Codex review (not Sonnet) reproduced the tilt and acceleration
calculations, spot-checked the raw trace and checked signed factor/range sums.
Its corrections are incorporated: distinguish unweighted means from integrated
residuals, new-window IMU samples from inherited orientation, and joint-step
preferences from isolated tilt causation. No physical-error or range-filter
conclusion follows from these aggregates.

Tilt uses acos((R_before^T g_unit) dot (R_after^T g_unit)); acceleration uses
the same node samples and fixed incoming gravity for before/after comparison.

Next work should inspect the influential5–15m observations/associations and the
attitude constraints using existing evidence before choosing a fix. Individual
cell correspondences are not retained in these aggregate summaries; hashes
validate identity but cannot reveal physical correctness. No further replay is
authorized automatically. High remains useful for this causal/design work.
