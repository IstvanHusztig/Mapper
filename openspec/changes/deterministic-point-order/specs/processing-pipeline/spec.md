## ADDED Requirements

### Requirement: The result does not depend on the data after a window
Points with equal timestamps SHALL keep their order in the recording, and the voxel decimation SHALL keep the first point of each voxel in that order, so that the odometry of a window depends only on the data up to that window. A recording cut with `cut_recording` SHALL give the same trajectory as the full recording for the windows the cut contains, except the last window, which the cut may shorten.

#### Scenario: A recording cut short
- **WHEN** recording 44 and its first 95 s (cut with `cut_recording`) are processed with the same build and odometry settings, without loop closure and consistency
- **THEN** their trajectories agree for the windows before the last one of the cut
