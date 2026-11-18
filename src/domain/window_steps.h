#pragma once

// Steps of the odometry window loop (optimize loop in run_odometry.cpp) that only need the window data:
// no map, no state, no logging. Each one is tested on synthetic windows (tests/window_steps_test.cpp).

#include "window.h"

// re-chain the window's poses so that its first node lies at start advanced by window.imu_motion_from_previous (start
// is the previous window's last node), keeping every relative pose
void place_window_after(Window &window, const Eigen::Affine3d &start);

// place windows[first], windows[first + 1], ... in order, each after the last node of the window before it;
// does nothing for first == 0 or first >= windows.size()
void place_windows_after(std::vector<Window> &windows, size_t first);

// the previous window's final world-frame velocity, the prior of the next window
struct PreviousVelocity
{
    bool use = false;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // [m/s]
    bool dropped = false; // faster than the limit: not used
    double speed = 0.0;   // the dropped speed [m/s]
};

// final velocity of previous over its last span_nodes nodes; a velocity faster than max_speed_mps is dropped
PreviousVelocity estimate_previous_velocity(const Window &previous, int span_nodes, double max_speed_mps);

// the zero-velocity test of a window: the world-frame acceleration on a 50 ms average (the accelerometer noise of a 10 ms
// average already reaches ~0.3 m/s^2 at rest, while walking accelerations of 1-3 m/s^2 at ~2 Hz survive the longer
// average) and is_window_stationary on it, which reads the node rotations, not the positions. The caller checks that
// poses, timestamps, imu_acc and imu_angles have one entry per node.
constexpr double rest_smoothing_half_width_s = 0.025;
bool window_rests(const std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &timestamps,
                  const std::vector<Eigen::Vector3d> &imu_acc, const std::vector<Eigen::Vector3d> &imu_angles, double gravity_norm,
                  double acc_threshold_mps2, double gyro_threshold_rad_s);

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

// initial guess of window: its relative poses chained from previous's last node advanced by
// window.imu_motion_from_previous; the translations come from IMU integration (or rest), else constant velocity, else the
// mean shift between before_previous and previous (before_previous may be null); IMU integration and constant velocity
// also over the time from previous's last node to window's first; poses and motion_model_poses are both set to the guess
PosePrediction predict_window_poses(Window &window, const Window &previous, const Window *before_previous,
                                    const Eigen::Vector3d &prev_velocity, const PosePredictionOptions &options);

// the steps of predict_window_poses. mean_shift_of: the mean shift per node between before_previous's and previous's last
// nodes (zero without before_previous); above 1 m it is set to zero and dropped is set
Eigen::Vector3d mean_shift_of(const Window &previous, const Window *before_previous, bool &dropped);
// the translations of a chained guess: with the IMU (imu_sizes_ok; use_imu_pred or rest) integrated from prev_velocity (zero
// at rest) with the world accelerations a, else constant velocity from prev_velocity (constant_velocity and timestamps),
// else the mean shift per node; the first two also over gap_s before the first node
void guess_window_translations(std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &ts,
                               const std::vector<Eigen::Vector3d> &a, bool imu_sizes_ok, bool stationary, const Eigen::Vector3d &prev_velocity,
                               const Eigen::Vector3d &mean_shift, double gap_s, const PosePredictionOptions &options);

// gyro-bias prior of window: the carried estimate, its covariance grown by the bias random walk over the window;
// sets window.gyro_bias to it and removes the carried bias from the guess's rotation increments (translations unchanged);
// returns the prior
GyroBiasState apply_gyro_bias_prior(Window &window, const GyroBiasState &carried, double bias_rw_rad);

// true if the guess turns about the vertical faster than threshold_rad_s on average over the window
bool is_turn_window(const Window &window, double threshold_rad_s);

// imu_always_on: the window's guess follows the accelerometer, so that the translation motion model (a velocity prior on every
// node around the guess) agrees with the accelerometer term instead of holding the window's mean velocity at its start velocity.
// From the first node, with start_velocity, the translations are integrated with the accelerations (R_k f_k - gravity) * to_mps2
// of the guess's rotations (the accelerometer term's model); rotations and the first node are kept. Returns false (and changes
// nothing) without an accelerometer reading and a timestamp for every node.
bool integrate_window_translations(Window &window, const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &gravity, double to_mps2);

// window hand-off: the last tail_nodes nodes carry almost no points (the last one none), so their pose is
// re-propagated from the last well-constrained node with the IMU rotation (motion_model_poses, corrected by
// bias_update) and the velocity of the velocity_nodes nodes before it; the next window starts from the re-propagated last node.
// Does nothing for tail_nodes <= 0 or a window too short for the velocity span.
void repropagate_window_tail(Window &window, int tail_nodes, int velocity_nodes, const Eigen::Vector3d &bias_update);

// how a window's Levenberg-Marquardt iterations converged (diagnostics of log_iteration_convergence)
struct IterationConvergence
{
    int iterations = 0;                    // iterations that ran
    int settled_after = 0;                 // from this many iterations on, the poses stay within the settle limits of the last result
    int fast_stop_after = 0;               // the first iteration count with delta below fast_delta (0: never)
    double fast_stop_position_error_m = 0.0; // largest node distance between the poses at fast_stop_after and the last result
    double fast_stop_rotation_error_rad = 0.0;
};

// poses_after[i] / deltas[i]: the window's poses and delta after iteration i + 1; measured against the last entry
IterationConvergence convergence_of(const std::vector<std::vector<Eigen::Affine3d>> &poses_after, const std::vector<double> &deltas,
                                    double settle_position_m, double settle_rotation_rad, double fast_delta);
