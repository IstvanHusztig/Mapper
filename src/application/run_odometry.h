#pragma once

// The window loop of the LiDAR odometry (step 2): every window in order, each optimized against the map so far.

#include <vector>
#include "input_ports.h"
#include "odometry_config.h"
#include "odometry_state.h"
#include "run_reporting.h"
#include "window.h"
#include "window_capture.h"
#include "window_term_sink.h"

// capture: with capture_windows_from_s / _to_s set, the loop's state before that range and what the run made of it go there
bool optimize_windows(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress,
                      WindowCaptureSink *capture = nullptr, WindowTermSink *terms = nullptr);

