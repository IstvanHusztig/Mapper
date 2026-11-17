#pragma once

// Replay of a window capture (window_capture.h): windows first..last run again from the captured loop state, with the
// given settings. With the settings of the captured run, the iterations end where they ended there (the time limit is
// not checked), so every window must come out bit for bit as captured; with other settings the replay shows what they do
// in that stretch, starting from the run's own map and state (a full run confirms).

#include "odometry_config.h"
#include "run_reporting.h"
#include "window_capture.h"
#include "window_term_sink.h"

#include <cstddef>
#include <vector>

struct WindowReplayResult
{
    std::vector<WindowOutcome> outcomes;    // per window first..last; fewer if the replay was cancelled
    bool cancelled = false;                 // a window's distance was not a number (as in the run, the replay stops)
    std::size_t identical = 0;              // windows whose poses equal the captured ones bit for bit
    double largest_position_difference_m = 0.0; // over all nodes of the replayed windows, against the captured poses
    double largest_rotation_difference_deg = 0.0;
    int first_different_window = -1;        // index of the first window that differs; -1 if none
    std::vector<int> iteration_differences; // per window: replayed last iteration minus the captured one
};

// same_settings: the capture was made with these odometry settings, so the captured iteration counts are forced
// terms: log_window_terms' tables of the replayed windows (optional)
WindowReplayResult replay_window_capture(const WindowCapture &capture, const OdometryConfig &params, bool same_settings, Logger &logger,
                                         WindowTermSink *terms = nullptr);
