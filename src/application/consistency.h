#pragma once

// Consistency passes over the whole trajectory (run after the odometry when use_consistency is set).

#include "normal_equations.h"
#include "phase_timing.h"
#include <vector>
#include "input_ports.h"
#include "odometry_config.h"
#include "run_reporting.h"
#include "window.h"

// What stays the same between the passes of one run: the points with their nodes, split into chunks, and the Cholesky
// analysis (the pattern of the normal matrix is the same in every pass). Pass the same object to every call of a run; without
// it a call loads and analyses on its own.
struct ConsistencyData
{
    bool loaded = false;
    size_t chunk_points = 1000000;   // points of the loaded order per chunk (each chunk gets its own NDT map)
    std::vector<Point3Di> points;    // the points inside the trajectory, index_pose set
    std::vector<size_t> chunk_begin; // chunk c is points[chunk_begin[c] .. chunk_begin[c + 1])
    PatternCachedCholesky solver;
};

void Consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger, ConsistencyTiming *timing = nullptr,
                 ConsistencyData *shared = nullptr);
