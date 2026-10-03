#pragma once

// Consistency passes over the whole trajectory (run after the odometry when use_consistency is set).

#include "lidar_odometry_utils.h"

void Consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);
void Consistency2(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);
