#pragma once

// Steps of the odometry window loop (optimize loop in run_odometry.cpp) that only need the window data:
// no map, no state, no logging. Each one is tested on synthetic windows (tests/window_steps_test.cpp).

#include "window.h"

// re-chain the window's poses so that its first node lies at start, keeping every relative pose
void place_window_after(Window &window, const Eigen::Affine3d &start);

// place windows[first], windows[first + 1], ... in order, each after the last node of the window before it;
// does nothing for first == 0 or first >= windows.size()
void place_windows_after(std::vector<Window> &windows, size_t first);

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

// how predict_window_poses guesses the motion of a window
struct PosePredictionOptions
{
    bool use_imu_pred = false;      // integrate the accelerometer from the previous velocity
    bool use_zupt = false;          // zero-velocity detection
    bool constant_velocity = false; // else: timestamp-aware constant velocity from the previous velocity
    double gravity_norm = 1.0;      // accelerometer reading of gravity [g]
    double acc_smoothing_half_width_s = 0.0;
    double zupt_acc_threshold_mps2 = 0.0;
    double zupt_gyro_threshold_rad_s = 0.0;
};

struct PosePrediction
{
    bool stationary = false;         // the window rests (zero-velocity detection)
    bool mean_shift_dropped = false; // the mean-shift fallback was implausible (> 1 m per node) and set to zero
};

// initial guess of window: its relative poses chained from previous's last node; the translations come from
// IMU integration (or rest), else constant velocity, else the mean shift between before_previous and previous
// (before_previous may be null); poses and motion_model_poses are both set to the guess
PosePrediction predict_window_poses(Window &window, const Window &previous, const Window *before_previous,
                                    const Eigen::Vector3d &prev_velocity, const PosePredictionOptions &options);

// gyro-bias prior of window: the carried estimate, its covariance grown by the bias random walk over the window;
// sets window.gyro_bias to it and removes the carried bias from the guess's rotation increments (translations unchanged);
// returns the prior
GyroBiasState apply_gyro_bias_prior(Window &window, const GyroBiasState &carried, double bias_rw_rad);

// true if the guess turns about the vertical faster than threshold_rad_s on average over the window
bool is_turn_window(const Window &window, double threshold_rad_s);

// window hand-off: the last tail_nodes nodes carry almost no points (the last one none), so their pose is
// re-propagated from the last well-constrained node with the IMU rotation (motion_model_poses, corrected by
// bias_update) and the velocity of the 20 nodes before it; the next window starts from the re-propagated last node.
// Does nothing for tail_nodes <= 0 or a window too short for the velocity span.
void repropagate_window_tail(Window &window, int tail_nodes, const Eigen::Vector3d &bias_update);
