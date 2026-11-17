#pragma once

// Where the window loop hands the per-term table of each window (log_window_terms; window_term_table.h). The loop only
// observes: nothing here feeds back into the odometry.

#include "window_term_table.h"
#include "window_verdict.h"

#include <Eigen/Dense>
#include <array>

// one window's tables and the features a learned tuner would read
struct WindowTermRecord
{
    int window = 0;          // 1-based
    double time_s = 0.0;     // the window's first node, from the first window's first node
    double duration_s = 0.0; // first to last node
    int iterations = 0;
    double delta = 0.0;      // the last iteration's update
    double lm_factor = 0.0;  // the damping after the iterations
    bool at_rest = false;
    bool turn_window = false;
    bool acc_term = false;
    Eigen::Vector3d guess_move = Eigen::Vector3d::Zero();  // first to last node, world frame [m]
    Eigen::Vector3d result_move = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_start = Eigen::Vector3d::Zero(); // [m/s]
    Eigen::Vector3d velocity_end = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_deg_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_offset_mps2 = Eigen::Vector3d::Zero();
    // the window's change from guess to result in the twelve motion numbers (window_motion_change)
    std::array<std::array<double, kTermDirections>, kTermMotions> change{};
    WindowTermTable guess;  // at the state the first iteration starts from
    WindowTermTable result; // after the last iteration
    WindowVerdict verdict;  // the window verdict (window_verdict.h): flags and readings
};

class WindowTermSink
{
public:
    virtual ~WindowTermSink() = default;
    virtual void write(const WindowTermRecord &record) = 0;
};
