#pragma once

// The state of the odometry's window loop (optimize_windows in run_odometry.cpp): the settings fixed for a run, what
// passes from one window to the next, and the accelerometer term of one window. In a header so that a window capture
// (window_capture.h) can hold the loop's state and a replay can continue from it.

#include "input_ports.h"
#include "odometry_config.h"
#include "odometry_state.h"
#include "optimizer_options.h"
#include "run_reporting.h"
#include "phase_timing.h"
#include "scan_to_map.h"
#include "window.h"
#include "window_steps.h"
#include <structures.h>

#include <chrono>
#include <vector>

// settings of the window loop, fixed for a run
struct OdometryRunSetup
{
    MotionModelRotationOptions rotation_options; // bias_prior_* are set per window
    ScanToMapOptions scan_to_map;
    MotionModelOptions motion_model;
    double bias_rw_rad = 0.0;
    double bias_sigma0_rad = 0.0;
    bool have_acc = false;     // raw accelerometer and AHRS orientation of every node
    bool use_imu_pred = false; // IMU-aided translation prediction
    bool use_zupt = false;     // zero-velocity detection
    double gravity_norm = 1.0; // accelerometer reading of gravity [g]
    bool always_on = false;    // accelerometer term in every window after the first, with velocity states (imu_always_on)
    double acc_offset_sigma0 = 0.0;
    bool use_acc_prior = false;               // velocity-continuity prior
    bool uses_previous_velocity = false;      // the acc prior, IMU translation or the accelerometer term need it
    PosePredictionOptions prediction;         // how the initial guess of a window is predicted
};

// log_stage_timing: where the window loop spends its time, summed over all windows
struct OdometryTiming
{
    ScanToMapTiming lm;      // inside optimize_lidar_odometry
    PhaseTime point_load;    // reading the window's points from the PointStore
    PhaseTime lm_iterations; // run_lm_iterations as a whole
    PhaseTime preview;       // send_preview
    PhaseTime add_to_map;    // add_window_to_map (update_rgd, sliding-map rebuild)
    PhaseTime window_total;  // one window, from loading its points to adding it to the map
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); // the breakdown is relative to the time since
};

// what the window loop carries from one window to the next
struct WindowLoopCarry
{
    GyroBiasState bias;                                     // the previous window's estimate, the prior of the next one
    Eigen::Vector3d acc_gravity = Eigen::Vector3d::Zero();  // gravity in the odometry frame [accelerometer units]
    Eigen::Matrix3d acc_offset_cov = Eigen::Matrix3d::Zero(); // prior of the next window's acceleration offset
    double acc_distance = 0.0;                              // trajectory length since the last sliding-window reset [m]
    std::vector<Point3Di> points_global;                    // map points since the last reset
    int stationary_windows = 0;
    int turn_windows = 0;
    int map_rebuilds = 0;                                   // sliding_window_trajectory_length_threshold reached
    int acc_term_windows = 0;
    int rest_windows = 0;                                        // imu_always_on: windows at rest
    bool velocity_valid = false;                                 // imu_always_on: the previous window estimated its end velocity
    Eigen::Vector3d velocity_mean = Eigen::Vector3d::Zero();     // ... the prior of the next window's start velocity
    Eigen::Matrix3d velocity_cov = Eigen::Matrix3d::Identity();
    double velocity_time_s = 0.0;                                // ... at the previous window's last node
    bool first_window_at_rest = false;
    std::vector<IterationConvergence> convergence; // per window, with log_iteration_convergence
    OdometryTiming timing;                         // with log_stage_timing
};

// the accelerometer term of one window and the motion-model options that depend on it
struct AccTerm
{
    MotionModelTranslationImuOptions options; // enabled only in windows that use the term
    MotionModelOptions motion_model;          // the window's translation sigmas (loosened while the term is active)
    Eigen::Matrix3d offset_cov;               // acceleration-offset covariance, updated by the optimization
    double window_duration_s = 0.0;
};

// the loop's state: what is fixed for the run, what passes from window to window, and the Cholesky analyses kept from
// window to window (they only save time: the results are the same without them)
struct WindowLoop
{
    OdometryRunSetup setup;
    WindowLoopCarry carry;
    ScanToMapWorkspace workspace;
};

// what the loop did with one window
struct WindowOutcome
{
    int last_iteration = 0;                 // index of the last Levenberg-Marquardt iteration
    std::vector<Eigen::Affine3d> poses;     // the window's final poses
};

// optional hooks of process_windows; none set in a normal run
struct WindowRange
{
    // per window from `first`: end the iterations after this one and don't check the time limit (a replay)
    const std::vector<int> *forced_last_iteration = nullptr;
    // filled per window from `first` with what the loop did
    std::vector<WindowOutcome> *outcomes = nullptr;
};

// sets the loop's settings and its state before the first window; first: the run's first window, already placed
void start_window_loop(WindowLoop &loop, const Window &first, const OdometryConfig &params, Logger &logger);

// windows first..last in order, continuing from `loop` and `state`. Reads windows[i - 1] and windows[i - 2] of each window
// i, places windows[i + 1] after it, and loads each window's points from point_store. false: the run was cancelled
// (a window's distance was not a number); the windows after it are then placed after its restored end.
bool process_windows(WindowLoop &loop, std::vector<Window> &windows, size_t first, size_t last, const OdometryConfig &params,
                     OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger,
                     ProgressReporter &progress, const WindowRange &range = {});
