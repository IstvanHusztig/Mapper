## Purpose

> The residual-floor candidate was rejected on 2026-11-13. Independent provisional
> registration evidence remains; no registration acceptance or completed
> main-spec integration is claimed. See ../../rejection.md.
Provide reproducible house-plan overlays that distinguish registration uncertainty from map deformation and retain independent checking surfaces.

## ADDED Requirements

### Requirement: Plan registration preserves geometry and uncertainty
A registration report SHALL identify source cloud/plan, units, scale assumptions, selected regions, transform and alternative hypotheses. One fixed rigid transform SHALL apply across evaluated visits. Reports SHALL retain held-out surfaces, unmatched/displaced support and known reflection caveats. Scale fitting, reflection or nonrigid warping SHALL NOT be used to make this fixed-scale evaluation appear accurate.

#### Scenario: Partial-room fit is ambiguous
- **WHEN** materially different orientations fit the limited room support comparably
- **THEN** the report retains the alternatives and does not label the registration uniquely confirmed

#### Scenario: Broader fit leaves inconsistent surfaces
- **WHEN** structural fitting improves some held-out residuals and worsens others
- **THEN** the report preserves both outcomes and does not attribute all remaining mismatch to odometry

#### Scenario: Physical anchors unavailable
- **WHEN** structural correspondences or plan/as-built identity are unconfirmed
- **THEN** the overlay remains provisional and independent surveyed accuracy is not claimed
