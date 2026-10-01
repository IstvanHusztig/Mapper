#pragma once

#include "lidar_odometry_utils.h"

// Automatic loop closure on the finished odometry (before the consistency pass and the export):
//  1. the windows are grouped into segments of loop_closure_segment_windows windows,
//  2. revisits are segment pairs far apart in time but close in space,
//  3. each revisit is registered against the map around the earlier visit (coarse voxel search,
//     then point-to-distribution refinement) and accepted only if the overlap is high,
//  4. a pose graph over the segment poses (odometry edges + accepted loop edges) is optimized,
//  5. the correction is interpolated onto every trajectory node of every window.
// Returns the number of accepted loop edges.
int run_loop_closure(std::vector<WorkerData> &worker_data, const LidarOdometryParams &params);
