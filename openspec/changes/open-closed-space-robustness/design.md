# Design

## Context

See proposal.md and the linked ndt-information-probe evidence. Root and scratch are separate working copies: numerical work is based on the experimental scratch branch, while this OpenSpec change is the planning source of truth. Freeze the implementation base and its relation to Huber source cbca36d before editing; do not overwrite prior bundles or merge failed Cauchy behavior into defaults.

The current geometric protection uses unweighted normal outer products and relative eigenvalue ratios for translation; rotation is unchanged by that projection. Legacy cell means/normals are not recomputed centroids/normals. These are audit limitations, not a proven complete outdoor cause. W415 thin-cell influence is already established by an exact control and ablation; do not repeat its discovery. Detailed trace does not cover the later sustained Huber W904–916 escalation.

## Goals / Non-Goals

**Goals:** bound excessive residual confidence with one isolated experimental change; preserve the crisp indoor reference; measure both stages and support; distinguish registration artifacts from map deformation.

**Non-Goals:** replacing the mapper, automatic indoor/outdoor classification, global Cauchy adoption, disabling online bias, changing stored map statistics/priors/projection, fitting scale or warping the cloud, and claiming cave readiness from an apartment scan. Support-aware acceptance or full-pose degeneracy changes require a revised design if selected later.

## Decisions

### 1. Resolve the remaining evidence gap without another general audit

Use existing W415/W754 results as local gates and existing later logs/snapshots to write a short mechanism decision. Derive an empirical residual-noise range from existing Livox3 stationary observations with declared physical surfaces, sensor ranges/incidences and support. Distinguish sensor noise from pose/deskew/map/plane-fit error; aggregate wall MAD alone is not point-noise calibration. Declare one candidate nonzero value/range and rationale before experiments; do not choose by return-error tuning. If later factor attribution requires uncaptured state, specify that state and request one scoped capture. Do not claim approximate restarts reproduce an exact checkpoint. If a defensible value or hypothesis cannot be established, stop before numerical application and report the missing evidence; do not switch branches silently.

### 2. Regularize residual covariance, retaining map geometry and Huber

Proposed key: `[lidar_odometry_control].ndt_residual_noise_floor_sigma_m`, finite nonnegative metres, default zero. Positive values must also have finite, nonzero representable sigma-squared; reject invalid scales before processing. A positive value sigma applies only to odometry matching. For a finite symmetric positive-definite cell covariance C = Q diag(lambda) Q^T, compute C_eff = Q diag(max(lambda_i, sigma^2)) Q^T and information C_eff^-1. Use this same information for both grids' cost, normal equations, robust Mahalanobis weights and exported diagnostic contributions. This is an eigenvalue lower bound, not addition of sigma^2 to every eigenvalue, not a plane-distance clip and not a physical calibration claim.

Disabled equivalence concerns numerical trajectories/clouds and legacy eligibility, not byte identity of saved TOML after adding a key. Check snapshot settings-hash compatibility explicitly; preserve old zero/omitted semantics or document a deliberate migration rather than silently reusing incompatible snapshots. Zero must branch to the exact legacy arithmetic and eligibility checks, without a new decomposition or changed operation order. Positive mode must reject invalid covariance/failed decomposition explicitly, never silently produce finite-looking information from NaNs. Retain legacy cell eligibility initially so the floor does not simultaneously revive previously skipped near-singular cells; record skip reasons. This bounds a previously overconfident accepted cell without broadening associations. Finite/SPD/support behavior is tested separately.

Store C, mean, normal and population unchanged. Do not floor the covariance used by normal estimation/projection. Preserve all other weights, robust cutoff, range filters, association rules and prior states. Make the enabled value and odometry-only scope identifiable in logs and saved TOML; omitted key and zero remain equivalent. Huber3 is the frozen experimental activation already in use, not a new robust preset or shipped-default change.

An additive physical covariance model, exact cell-statistics flag, robust kernel replacement and support-aware solver acceptance were considered. Each changes additional assumptions; do not bundle them into this first intervention. Full-objective acceptance cannot use the current reassociated matched-only cost as if it were a complete objective: support loss could falsely appear beneficial.

### 3. Use staged validation rather than accepting a single local turn

Before runs, meaningful synthetic tests cover rooms, open corridors/planes, controlled weak translation/yaw, overconfident incorrect cells and contaminated priors as distinct cases. Verify analytical weighting/cost consistency, rotation equivariance, finite/SPD handling, enabled floor behavior and disabled equivalence in both grids and TOML round trips. Release first; Debug only where configuration-dependent issues warrant it. Preserve known pre-existing test failures explicitly, without claiming a clean suite.

Each experiment needs fresh output, identified source/config/input, source-selection coverage and explicit user run approval. First reproduce the exact Huber W415 control and evaluate one enabled candidate, including signed influence, support and gyro/velocity collateral effects. Then evaluate a longer failure interval with control-equivalent incoming map/state, if available. A checkpoint improvement alone cannot advance straight to adoption. Missing state/control reproduction is a stop, not permission for a cold-start replacement.

Only a surviving candidate advances to separately approved full Livox2 and Livox3 runs. Compare pre-loop and final stages, the full trajectory, transitions, gyro/bias/velocity/support and actual clouds. Freeze physical region selection, retain missing/displaced surfaces and known mirror/window projections. Before ranking improvement, use the agreed perturbation contract in ../sensor-neutral-yardstick/criteria-proposal.md; never discard failed members. The old sensor regression gates remain independent of Livox calibration. A different recording cannot be used for index-based trajectory seams or a causal route-only comparison.

### 4. Keep registration a separate measurement product

Preserve original and broader Livox3 transforms and their source identities. Existing candidate changes 22.254 to 19.327 degrees at fixed nominal scale. Retain fit and held-out surfaces/visits; mixed residuals of roughly 0.1–0.4m remain. Do not pick another transform per visit/floor or remove bad regions to improve a score. Attic/sloped-ceiling/window/fixture returns may not correspond to floor-plan wall faces. If a defensible global tie is required, identify two distinct corresponding structural corners, check plan/as-built correspondence and use additional held-out surfaces; a scale calibration needs independent dimensional evidence. Without that information report provisional/ambiguous alignment. Do not make registration acceptance a prerequisite to reporting independent trajectory and sharpness observations.

### 5. Promotion and documentation

Keep default sigma zero throughout evaluation. Adoption requires the user's map review plus explicit evidence against agreed geometry, revisit, sharpness and stability criteria; a small endpoint or good indoor cloud alone is insufficient. Persist limitations where motion is not observable; do not promise drift-free IMU coasting. If accepted as a general correction, update existing docs HTML (and maintained translated counterparts as appropriate) with behavior, configuration and limits, linking detailed experiment evidence in OpenSpec. This reconciles the user's documentation request with the sensor-neutral documentation spec: do not paste recording-specific metrics/history into product guidance.

## Risks / Trade-offs

- [Larger floor weakens useful geometry] -> one justified value, preserve indoor references and region coverage; stop on supported regression.
- [Robust scale changes with C_eff] -> recompute weights/cost with exactly the same information and retain influence diagnostics.
- [Wrong mean/association survives] -> do not call a floor a universal fix; select a different branch only through a revised proposal.
- [Repeated points and two grids overstate information] -> report effective support and prior innovations; do not call covariance calibrated.
- [Initial registration regions contain furniture] -> label correspondence confidence and held-out results, preserve residual mismatches.
- [Missing later trace leads to another long instrumentation cycle] -> identify the precise missing state and runtime, request a scoped capture, avoid broad tooling work.

## Migration Plan

Implement in the isolated experimental checkout after apply is invoked. Default zero preserves behavior and rollback; original settings and data remain unchanged. Keep binary/config manifests for every experiment. Do not ship a new enabled default, merge or push from this planning request. If evidence rejects the candidate, retain results and reset experimental activation, not original artifacts. If acceptance supports production integration, synchronize relevant requirements and update HTML documentation before finalizing the change.

## Open Questions

Deferrable empirical quantities are the justified nonzero sigma, exact availability of a longer control-equivalent restart, and confirmed as-built structural anchors. Tasks explicitly gate candidate runs/registration acceptance on these answers. If the answers require a different model or solver, revise the proposal instead of silently widening scope.
