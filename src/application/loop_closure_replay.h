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
#include <utility>
#include <vector>

// A try named by its segments i and j (earlier, later), as "i-j"; a comma-separated list of them (--reject-tries). false
// (and `why` names the bad entry) if the text isn't such a list.
bool parse_loop_try_names(const std::string &text, std::vector<std::pair<int, int>> &tries, std::string &why);
std::string loop_try_name(const std::pair<int, int> &t);

struct LoopClosureReplayResult
{
    bool segments_match = false; // the capture's windows give the capture's segments with these settings (else nothing ran)
    std::vector<std::string> missing; // tries the settings asked for that the capture doesn't hold, e.g. "segments 3 -> 17"
    // the tries named for rejection ("i-j"): those the capture doesn't hold (then nothing ran), those close_loops asked for
    // (and counted as rejected), and those it never asked for (they changed nothing)
    std::vector<std::string> rejected_not_in_capture;
    std::vector<std::string> rejected;
    std::vector<std::string> rejected_not_asked;
    int loops = 0;
    LoopClosureReport report;
    std::vector<Window> windows; // the capture's windows after the replayed loop closure
    // against the poses the capturing run's loop closure left
    std::size_t nodes = 0;
    std::size_t identical_nodes = 0;
    double largest_position_difference_m = 0.0;
    double largest_rotation_difference_deg = 0.0;
};

// reject: captured tries that count as rejected registrations, as if their acceptance had failed (either kind, same-floor
// or cross-floor, of that pair); close_loops then chooses as it would have (another slot, the next cross-floor candidate)
LoopClosureReplayResult replay_loop_closure(const LoopClosureCapture &capture, const OdometryConfig &params, Logger &logger,
                                            const std::vector<std::pair<int, int>> &reject = {});

// the trajectory as save_result cuts it into output chunks (a chunk per 51 windows, the last one only if it has more than
// min_counter_concatenated_trajectory_nodes windows): each chunk's first pose and its rows relative to it
struct TrajectoryChunk
{
    Eigen::Affine3d first_pose = Eigen::Affine3d::Identity();
    std::vector<TrajectoryRow> rows;
};
std::vector<TrajectoryChunk> trajectory_chunks(const std::vector<Window> &windows, const OdometryConfig &params);
