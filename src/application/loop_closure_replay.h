#pragma once

// Loop closure replay: close_loops (loop_closure.h) on a capture (loop_closure_capture.h), with the registrations taken
// from the capture instead of the points. Which settings a replay may change is decided by the capture file's reader
// (loop_closure_capture_file.h): those that change the odometry, the segments or a registration are refused there.

#include "loop_closure.h"
#include "loop_closure_capture.h"
#include "output_ports.h"
#include "run_reporting.h"

#include <cstddef>
#include <string>
#include <vector>

struct LoopClosureReplayResult
{
    bool segments_match = false; // the capture's windows give the capture's segments with these settings (else nothing ran)
    std::vector<std::string> missing; // tries the settings asked for that the capture doesn't hold, e.g. "segments 3 -> 17"
    int loops = 0;
    LoopClosureReport report;
    std::vector<Window> windows; // the capture's windows after the replayed loop closure
    // against the poses the capturing run's loop closure left
    std::size_t nodes = 0;
    std::size_t identical_nodes = 0;
    double largest_position_difference_m = 0.0;
    double largest_rotation_difference_deg = 0.0;
};

LoopClosureReplayResult replay_loop_closure(const LoopClosureCapture &capture, const OdometryConfig &params, Logger &logger);

// the trajectory as save_result cuts it into output chunks (a chunk per 51 windows, the last one only if it has more than
// min_counter_concatenated_trajectory_nodes windows): each chunk's first pose and its rows relative to it
struct TrajectoryChunk
{
    Eigen::Affine3d first_pose = Eigen::Affine3d::Identity();
    std::vector<TrajectoryRow> rows;
};
std::vector<TrajectoryChunk> trajectory_chunks(const std::vector<Window> &windows, const OdometryConfig &params);
