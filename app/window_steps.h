#pragma once

// Steps of the odometry window loop (optimize loop in run_odometry.cpp) that only need the window data:
// no map, no state, no logging. Each one is tested on synthetic windows (tests/window_steps_test.cpp).

#include "window.h"

// re-chain the window's poses so that its first node lies at start, keeping every relative pose
void place_window_after(Window &window, const Eigen::Affine3d &start);
