#pragma once

// The window loop of the LiDAR odometry (step 2): every window in order, each optimized against the map so far.

#include "lidar_odometry_utils.h"

bool optimize_windows(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress);

