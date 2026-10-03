#pragma once

// Steps of the odometry window loop (optimize loop in run_odometry.cpp) that only need the window data:
// no map, no state, no logging. Each one is tested on synthetic windows (tests/window_steps_test.cpp).

#include "window.h"

// re-chain the window's poses so that its first node lies at start, keeping every relative pose
void place_window_after(Window &window, const Eigen::Affine3d &start);

// the previous window's final world-frame velocity, the prior of the next window
struct PreviousVelocity
{
    bool use = false;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // [m/s]
    double age_s = 0.0;   // from the centre of the velocity span to the window's first node
    bool dropped = false; // faster than the limit: not used
    double speed = 0.0;   // the dropped speed [m/s]
};

// final velocity of previous over its last span_nodes nodes, and its age at window's first node;
// a velocity faster than max_speed_mps is dropped
PreviousVelocity estimate_previous_velocity(const Window &previous, const Window &window, int span_nodes, double max_speed_mps);
