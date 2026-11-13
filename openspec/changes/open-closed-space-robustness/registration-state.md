# Registration reconciliation — 2026-11-13

Tasks3.1/3.2: checked the existing audit against its actual source files. Cloud,
plan, prior fit and audit script SHA256 all match. New verification metadata is
open-closed-existing-data/registration-state.json, with the checking script.
No cloud points, source regions, scale or transforms were changed.

Original room-only transform:22.254087845degrees, translation(22.151655830,
13.683239830)m. Broader structural candidate:19.326900243degrees,
(22.246497705,13.654176396)m. Difference:-2.927187602degrees. Fit support20–120s,
held-out return215–245s, fixed worldZ0.45–0.55m. The prior limited-room alternative
near201.886degrees remains evidence of ambiguity; a broader fit does not prove
its structural correspondences or remove the original ambiguity by decree.

Existing held-out residuals are mixed, roughly0.1–0.4m on several surfaces, with
larger discrepancies and sparse support in others. The0.8m correspondence gate
can mix wall faces with furniture/roof/window returns. First fit used first0–15s;
broader method keeps equal per-surface robust fitting and fixed fit/held-out roles.
Both candidates use nominal scale and one rigid transform for every visit.

Status remains **provisional**. Two independently identified corresponding
structural corners and confirmation that plan faces represent the as-built walls
are missing; independent dimensional evidence is needed for scale calibration.
Do not force another fit or score user-marked mirror/window projections as drift.
Sloped-roof intersections in a height slice may differ from floor-plan wall faces.

Before/after artifact already prepared:
livox3-indoor/huber3-baseline/registration-audit/registration-before-after.png.
User review of that broader overlay is still pending (task3.3). Registration
acceptance does not block independent trajectory, covariance or sharpness checks.
