#pragma once

// The per-term table of one window state (log_window_terms; docs/code-split-and-region-agents-plan.md, stage 3): what each
// term of the window's optimization costs, how strongly it holds the window against four motions, and where it pulls it.
// Built from the same term functions optimize_lidar_odometry calls, each on its own empty WindowEquations, on const inputs:
// computing it changes nothing the optimization reads.
//
// The four motions, each with amplitude 1 at the window's last node (tau = time since the first node / window duration):
//   shift:   every node moves by d [m];
//   stretch: node k moves by tau_k d, start and end velocity by d / duration [m];
//   curve:   node k moves by tau_k^2 d, end velocity by 2 d / duration [m];
//   turn:    node k's attitude turns by tau_k * 1 deg about the world axis d, positions unchanged [deg].
// d is along travel (horizontal), sideways (horizontal, across it) or vertical (world z). For a term with normal matrix H
// and right-hand side b and a motion as the vector m over the unknowns: weight w = m^T H m, pull p = m^T b / w (where the
// term alone would move the window along m, everything else held; empty when w = 0). Summed over the rows, sum(w p) / sum(w)
// is the step all terms together want along m, without damping.

#include "ndt_map.h"
#include "optimizer_options.h"
#include "window.h"
#include "window_noise_model.h"
#include <structures.h>

#include <Eigen/Dense>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

constexpr int kTermMotions = 4;    // shift, stretch, curve, turn
constexpr int kTermDirections = 3; // along travel, sideways, vertical
const char *term_motion_name(int motion);
const char *term_direction_name(int direction);

struct WindowTermRow
{
    std::string name;
    bool in_total = true;   // false: shown for information (NDT before the projection and its normal / in-plane parts, damping)
    double cost = 0.0;      // at the state the table is computed at (with its own weight)
    int residuals = 0;      // scalar residuals behind the cost
    std::array<std::array<double, kTermDirections>, kTermMotions> weight{}; // [motion][direction]
    std::array<std::array<double, kTermDirections>, kTermMotions> pull{};   // NaN where the weight is 0
};

struct WindowTermTable
{
    std::array<Eigen::Vector3d, kTermDirections> directions; // along travel, sideways, vertical (world frame)
    bool turn_defined = true;  // false: a node within 3 deg of fi = +-90 deg (the turn columns stay empty)
    std::string failure;       // not empty: the table could not be computed (the rows are then empty)
    // the window's translation information from the indoor grid's matched normals (sum n n^T, world frame), and the NDT point
    // terms in range that found a cell and that found none (both grids)
    Eigen::Matrix3d translation_information = Eigen::Matrix3d::Zero();
    int ndt_matched = 0;
    int ndt_unmatched = 0;
    std::vector<WindowTermRow> rows; // the terms present in the window, then "total", then "damping"
};

// what the table reads: the optimizer's inputs for one window state (the window's poses, velocities and bias are the state)
struct WindowTermInputs
{
    const std::vector<Point3Di> &points;
    const Window &window;
    const NDTBucketMapType &buckets_indoor;
    const NDTBucketMapType &buckets_outdoor;
    const ScanToMapOptions &scan_to_map;
    const MotionModelOptions &motion_model;
    const MotionModelRotationOptions &rotation_options;
    const MotionModelTranslationImuOptions &translation_imu_options;
    const WindowNoiseModel &noise; // the window's noise model: its fixed weights depend on the timestamps only, so the guess
                                   // and the result share it
    const Eigen::Vector3d &acc_offset;
    double lm_factor;         // the damping this state was (or would be) solved with
    Eigen::Vector3d along;    // horizontal unit vector of the travel direction
};

struct WindowUnknowns;

// the 12 motions as vectors over the window's unknowns, at index motion * 3 + direction (see the top of this file);
// turn_defined false: a node is within 3 deg of fi = +-90 deg and the turn vectors are zero
std::vector<Eigen::VectorXd> window_motion_vectors(const Window &window, const WindowUnknowns &unknowns,
                                                   const std::array<Eigen::Vector3d, kTermDirections> &directions, bool &turn_defined);

// the horizontal unit direction from the first to the last node; `fallback` when they are less than 1 cm apart horizontally
Eigen::Vector3d window_travel_direction(const std::vector<Eigen::Affine3d> &poses, const Eigen::Vector3d &fallback);

// the table; a std::exception from a term (a non-finite NDT distance with Huber on) is caught and reported in `failure`
WindowTermTable window_term_table(const WindowTermInputs &inputs);

// the change of a window from one state to another (same node count) in the twelve motion numbers: shift = the first
// node's displacement; stretch and curve = the least-squares a, c of (displacement_k - displacement_0) = a tau_k + c tau_k^2
// per direction; turn = the rotation change at the last node minus the first node's, about each direction [deg]
std::array<std::array<double, kTermDirections>, kTermMotions> window_motion_change(const Window &from, const Window &to,
                                                                                 const std::array<Eigen::Vector3d, kTermDirections> &directions);

// The window's mean rotation rate [rad/s] from its attitude increments: the sum over consecutive nodes of the angle of
// R_k^T R_(k+1), divided by the time from the first node to the last. Attitudes, not raw gyro values (a replay has none).
// nullopt: fewer than two nodes, poses and timestamps of different lengths, or no time passing.
std::optional<double> mean_rotation_rate(const std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &timestamps);

// the span of start_world_acceleration [s]
constexpr double kStartAccelerationSpanS = 0.030;

// The window's world-frame acceleration at its start [m/s^2], gravity removed: the mean of (R_k a_k - g) * 9.80665 / |g|
// over the nodes k with t_k - t_0 <= span_s (node 0 always), a_k the raw accelerometer [accelerometer units, g] and g the
// gravity vector in the same units (the accelerometer term's own conversion). nullopt: no nodes, imu_acc, poses and
// timestamps of different lengths, or g zero / not finite.
std::optional<Eigen::Vector3d> start_world_acceleration(const std::vector<Eigen::Affine3d> &poses,
                                                        const std::vector<std::pair<double, double>> &timestamps,
                                                        const std::vector<Eigen::Vector3d> &imu_acc, const Eigen::Vector3d &gravity,
                                                        double span_s = kStartAccelerationSpanS);
