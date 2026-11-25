#pragma once

// Where the window loop hands the per-term table of each window (log_window_terms; window_term_table.h). The loop only
// observes: nothing here feeds back into the odometry.

#include "window_term_table.h"
#include "window_noise_model.h"
#include "window_verdict.h"

#include <Eigen/Dense>
#include <array>
#include <optional>

// The state a window was prepared with, taken right after its preparation (consistency-harness D9), on the window's own
// row: the fixtures join adjacent rows for the hand-off. Each value is absent (nullopt) where the window has no such
// prior; absent is not zero.
struct WindowPreparedState
{
    std::optional<Eigen::Affine3d> first_pose;              // node 0 as prepared (world)
    std::optional<Eigen::Vector3d> velocity_prior_mean;     // start velocity prior [m/s] (world)
    std::optional<Eigen::Matrix3d> velocity_prior_cov;      // [(m/s)^2]
    std::optional<Eigen::Vector3d> offset_prior_mean;       // acceleration offset prior [m/s^2] (zero today: the offset is folded into gravity)
    std::optional<Eigen::Matrix3d> offset_prior_cov;        // [(m/s^2)^2]
    std::optional<Eigen::Matrix3d> velocity_offset_prior_cov; // rows start velocity, columns offset [m/s * m/s^2]; zero unless carried jointly
    std::optional<Eigen::Vector3d> gravity;                 // the gravity vector the window was prepared with [accelerometer units, g]
    std::optional<Eigen::Vector3d> bias_prior_mean_rad_s;   // gyro bias prior [rad/s]
    std::optional<Eigen::Matrix3d> bias_prior_cov;          // [(rad/s)^2]
};

// a node's pose and its time [s, from the first window's first node, as WindowTermRecord::time_s]
struct WindowNode
{
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    double time_s = 0.0;
};

// whether the record holds what the window handed to the next one (taken after the hand-off: the offset folded into
// gravity, the tail re-propagated)
enum class WindowHandOff
{
    not_recorded, // the record was written before the hand-off: nothing of WindowHandedOnState is valid
    done,
    cancelled // the run stopped at this window's hand-off: nothing of WindowHandedOnState is valid
};

struct WindowHandedOnState
{
    std::optional<Eigen::Vector3d> velocity_end;        // [m/s] (world)
    std::optional<Eigen::Matrix3d> velocity_end_cov;    // [(m/s)^2]
    std::optional<Eigen::Vector3d> gravity;             // the carried gravity vector after the fold [accelerometer units, g]
    std::optional<double> to_mps2_before_fold;          // the unit factor the window used, 9.80665 / |gravity before the fold|
    std::optional<Eigen::Matrix3d> offset_cov;          // the offset covariance carried on [(m/s^2)^2]
    std::optional<Eigen::Matrix3d> velocity_offset_cov; // carried cov(end velocity, offset) [m/s * m/s^2] (joint carry only)
    std::optional<Eigen::Vector3d> bias_mean_rad_s;     // gyro bias carried on [rad/s]
    std::optional<Eigen::Matrix3d> bias_cov;            // [(rad/s)^2]
    std::optional<WindowNode> last_node_solved;         // the last node after the iterations
    std::optional<WindowNode> last_node_handed_on;      // the last node after the tail re-propagation
};

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
    WindowNoiseSigmas noise; // the window's noise model as sigmas (window_noise_model.h)

    // consistency-harness D9 (absent unless the loop fills them)
    WindowPreparedState prepared;
    WindowHandOff handoff = WindowHandOff::not_recorded;
    WindowHandedOnState handed_on;                          // valid only with handoff == done
    std::optional<double> rotation_rate_rad_s;              // mean_rotation_rate (window_term_table.h) of the prepared poses
    std::optional<Eigen::Vector3d> start_acceleration_mps2; // start_world_acceleration (window_term_table.h), world frame
};

class WindowTermSink
{
public:
    virtual ~WindowTermSink() = default;
    virtual void write(const WindowTermRecord &record) = 0;
};
