# Comparable SLAM problems: Terra research and source audit

2026-11-12. User requested GPT-6.1 Terra research while the second approved
experiment runs. Research is separate from permission to implement another
correction or run another experiment. Root checked the primary links below.

## Findings relevant to remaining doubled walls

1. **Local covariance can miss wrong convergence.** Bonnabel, Barczyk and
   Goulette explain why ignoring rematching can invalidate point-to-point ICP
   covariance, while establishing a point-to-plane result under their assumptions.
   Brossard, Bonnabel and Barrau explicitly include initialization uncertainty,
   wrong convergence and initial/final correlation. These are ICP results, not
   proof of dMapper NDT double-counting. They support investigating why a narrow
   conditional Hessian covariance accompanies different registration solutions.
   Sources: [On the Covariance of ICP-based Scan-matching Techniques](https://arxiv.org/abs/1410.7632),
   [A New Approach to 3D ICP Covariance Estimation](https://arxiv.org/abs/1909.05722).

2. **Association handling matters alongside weights.** KISS-ICP combines motion
   prediction/deskewing, adaptive correspondence thresholds and robust alignment
   (sectionsIII-A/D/E). It includes handheld Livox, not only spinning sensors or
   driving datasets. Its point-to-point, LiDAR-only method is not a drop-in NDT
   correction. For dMapper, compare residual distributions and spatial support
   of the same returns/visits rather than relying on counts or a shrinking cost
   over a changing subset. Source: [KISS-ICP](https://arxiv.org/html/2209.15397).

3. **Degeneracy projection is not a wrong-alignment detector.** LIO-SAM's
   scan optimizer eigendecomposes its normal matrix and projects out directions
   below thresholds at the referenced revision. This handles local weak
   directions; it does not establish that a locally constrained solution is the
   correct registration. dMapper's projection retained all directions during the traced failure;
   this is compatible with a wrong-association hypothesis. Do not import
   hardcoded thresholds across different state units and measurement models.
   Source: [LIO-SAM exact source revision, lines1112-1144](https://github.com/TixiaoShan/LIO-SAM/blob/08af3f32f01725372d4269838dc44c19c6d9e76b/src/mapOptmization.cpp#L1112-L1144).

4. **Motion continuity trades stabilization against trajectory flexibility.**
   CT-ICP uses elastic within-scan interpolation and a proximity constraint
   between successive scans, allowing discontinuity between scans. This is a
   LiDAR-only formulation with a different state and objective. It motivates
   examining the role/units of each continuity constraint, not copying its
   coefficients or claiming that time normalization fixes our current drift.
   Source: [CT-ICP, sectionIII-A](https://arxiv.org/html/2109.12979).

5. **Viewpoint changes can bias voxel matching.** Rife and McDermott analyze
   perspective/occlusion error using cylindrical columns and dual-wall corners.
   This is another plausible explanation to investigate for wall separation,
   not evidence that it caused this recording's doubled walls. Separate point
   contributions by viewpoint and visit before changing voxel statistics.
   Source: [Characterizing Perspective Error in Voxel-Based Lidar Scan Matching](https://arxiv.org/abs/2401.13416).

## Bounded next use of the evidence

Finish the already approved two-run comparison first. With existing outputs,
localize the visible doubled-wall region, split it by acquisition time/visit,
and compare that same region across retained controls and candidates. Distinguish
within-visit spread from separation between visits, and pre-loop trajectory
behavior from final geometry. The current narrow-slice aggregates do not identify
the user's particular doubled wall or cover all returns.

Only after that localization should a new proposal choose between information
model/carry uncertainty, association reliability, trajectory interpolation or
map representation. Frozen-window/map initialization perturbations could test
basin sensitivity, but require additional instrumentation/state not currently
retained and are not approved runs. Another broad weight sweep or algorithm
replacement is not recommended from these papers alone. Historical mixed
LM/unmatched-cost results remain relevant counterevidence to a simple solver fix.

## Audit corrections and limits

The initial Terra response overstated verification of two links; they were
subsequently opened and verified. An inaccessible Censi PDF citation was omitted.
An initial claim of recorded runaway after zeroing XYZ was incorrect: zeroing
was only a synthetic null-mode check; recorded runaway was alpha0.1/beta1.
The final research accounts for these corrections and the handheld Livox coverage
of KISS-ICP. No source change, mapper run or physical-noise calibration follows
from this research. Independent local trace review supports a contribution from
motion-prior strength, not a unique cause or a universally valid information ratio.
