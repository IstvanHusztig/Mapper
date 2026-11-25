#pragma once

// Loop closure capture: what a run's loop closure worked from (capture_loop_closure): the windows as the odometry left them,
// the segments, every registration it made with its result (from format 3 on with its information per source window), and
// the poses it produced, and from format 4 on the fine-time step's score curves. A replay (loop_closure_replay.h)
// runs everything after the registrations again from it, in seconds and without the recording, bit for bit with the same
// settings, or with other selection, acceptance, weighting and pruning settings to see what they do.

#include "loop_closure.h"
#include "window.h"

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

struct CapturedLoopTry
{
    LoopClosureTry registration;
    bool accepted = false;          // by the capturing run's acceptance rule (loop_try_accepted)
    bool requested_by_run = false;  // false: registered only for the capture (capture_loop_closure_max_tries)
};

struct LoopClosureCapture
{
    std::vector<Window> windows;              // as the odometry left them: the loop closure's input
    std::vector<LoopClosureSegment> segments; // loop_closure_segments of those windows
    std::vector<CapturedLoopTry> tries;       // in the order registered
    std::vector<std::vector<Eigen::Affine3d>> result_poses; // each window's poses after the run's loop closure
    int loops = 0;                            // the loop edges the run applied
    // loop_closure_fine_time_placement (format 4 on): the score curves the fine-time step used, with the pose graph result and
    // the eligible runs they belong to; none when the step did not ask for curves (setting off, no eligible run, or an older
    // format)
    std::optional<LoopFineTimeCapture> fine_time;
};

class LoopClosureCaptureSink
{
public:
    virtual ~LoopClosureCaptureSink() = default;
    // false (and `why` says why) if the capture can't be kept
    virtual bool write(const LoopClosureCapture &capture, std::string &why) = 0;
};
