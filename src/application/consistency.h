#pragma once

// Consistency passes over the whole trajectory (run after the odometry when use_consistency is set).

#include "phase_timing.h"
#include <vector>
#include "input_ports.h"
#include "odometry_config.h"
#include "run_reporting.h"
#include "window.h"

void Consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger, ConsistencyTiming *timing = nullptr);
void Consistency2(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);
