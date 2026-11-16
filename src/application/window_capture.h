#pragma once

// Window capture: the window loop's state before a range of windows, the windows and their points, and what the run made
// of them (capture_windows_from_s / capture_windows_to_s). A replay (window_replay.h) runs the range again from it, in
// seconds instead of a whole run, bit for bit with the same settings, or with changed settings to see what they do there.

#include "odometry_state.h"
#include "window.h"
#include "window_loop.h"
#include <structures.h>

#include <cstddef>
#include <string>
#include <vector>

struct WindowCapture
{
    std::size_t n_windows = 0; // windows of the run
    std::size_t first = 0;     // the range: windows first..last
    std::size_t last = 0;
    Window first_window;       // the run's first window (the loop's settings are taken from it)
    // windows windows_begin .. min(last + 1, n_windows - 1) as they were before window `first`: the two before the range
    // are read by the range's first windows, the one after it is placed by its last
    std::size_t windows_begin = 0;
    std::vector<Window> windows;
    std::vector<std::vector<Point3Di>> points; // each window's points first..last, as the PointStore holds them
    OdometryState state;                      // the maps and the run state before window `first` (no initial points or paths)
    WindowLoopCarry carry;                    // what passes from window to window (without timing and convergence)
    std::vector<WindowOutcome> outcomes;      // what the run made of windows first..last
};

// the range of windows whose first node lies within [from_s, to_s] seconds of the first window's first node; false if the
// settings are off or no window starts there
bool capture_range(const std::vector<Window> &windows, double from_s, double to_s, std::size_t &first, std::size_t &last);

class WindowCaptureSink
{
public:
    virtual ~WindowCaptureSink() = default;
    // false (and `why` says why) if the capture can't be kept
    virtual bool write(const WindowCapture &capture, std::string &why) = 0;
};
