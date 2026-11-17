#pragma once

// The noise model of one window (stage 5 of docs/code-split-and-region-agents-plan.md): every weight of the window's
// least-squares problem in one place, built once per window before its first iteration from the settings, the window's
// timestamps and the state carried from the previous window. The terms receive their weights from it and never compute
// them. Every weight is computed with the expression, operand order and condition the term used before, so the result is
// the same bits.
//
// Two kinds of content:
// - values fixed for the window (the motion edges, the velocity continuity, the accelerometer terms, the priors, the
//   origin fix): they depend only on the settings, the timestamps and the carried state, none of which changes during the
//   iterations;
// - the NDT term's rules, applied per point and per iteration because they depend on the current poses (the Huber
//   weight, the degeneracy projection) or on the map (the cell information): the model holds their parameters.

#include "optimizer_options.h"
#include "window.h"
#include "window_equations.h"

#include <array>
#include <vector>

// the time-scaled rotation sigma of a motion edge: the gyro's angle random walk over dt, inflated by k, floored for
// near-zero dt [rad]
double time_scaled_rotation_sigma(double inflation_k, double arw_rad_sqrt_s, double dt, double floor_rad);

// the weights of the consecutive-node edges: the position rows' (x/y/z in the first node's frame, scaled by
// imu_position_information_scale while the accelerometer term is active) and, per edge, the rotation rows' (om/fi/ka;
// time-scaled from the gyro's noise when set), and each edge's duration (with time-scaled sigmas or bias estimation,
// else 0). The edge block is linear in these weights: the position rows alone are the block with zero rotation weights,
// and the reverse.
struct MotionEdgeWeights
{
    double x = 0.0, y = 0.0, z = 0.0;
    std::vector<Eigen::Vector3d> rotation; // per edge
    std::vector<double> dt;                // per edge [s]
};
MotionEdgeWeights motion_edge_weights(const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                                      const MotionModelRotationOptions &rotation_options,
                                      const MotionModelTranslationImuOptions &translation_imu_options);

// one accelerometer term (imu_always_on): which kind, its nodes, its span tau [s] and its weight
// 1 / ((density^2 / tau + floor^2) tau^2). anchor_triple: anchors a, b, c; start_velocity: node 0 (a) and the first anchor
// (b); end_velocity: the last two anchors (a, b) and the last node (c).
enum class AccelerometerTermKind
{
    anchor_triple,
    start_velocity,
    end_velocity
};
struct AccelerometerTermSpan
{
    AccelerometerTermKind kind = AccelerometerTermKind::anchor_triple;
    int a = 0, b = 0, c = 0;
    double tau = 0.0;
    double weight = 0.0;
};

// the NDT term's rules for one grid: the range weight (use_norm) and the cap of the weight at 10, the fine grid only
struct NdtGridNoise
{
    bool range_weight = false;
    bool cap = false;
};
constexpr double ndt_weight_cap = 10.0;

// the NDT term's rules, applied per point in this order: the cell information (from the map), planarity, the range weight
// and its cap, the Huber weight, the information scale; then the degeneracy projection of the node blocks
struct NdtNoise
{
    bool planarity = false;
    NdtGridNoise indoor, outdoor;
    double huber_cutoff = 0.0;
    double information_scale = 1.0;
    double projection_ratio = 0.0;
    double projection_ratio_full = 0.0;
};

// the NDT term's rules from the run's scan-to-map settings
NdtNoise ndt_noise(const ScanToMapOptions &scan_to_map);

// the weak tie of the end velocity to the start velocity when the end has no accelerometer term (sigma 1 m/s)
constexpr double weak_end_velocity_tie_weight = 1.0;

struct WindowNoiseModel
{
    WindowUnknowns unknowns; // the layout: which unknowns and terms exist

    MotionEdgeWeights edges;

    // velocity continuity, only without the accelerometer term and with timestamps: one weight per node triple
    // (node i - 1, i, i + 1 for i = 1 .. n - 2), and the first step's tie to the previous window's velocity
    bool velocity_continuity = false;
    std::vector<double> velocity_continuity_weights;
    bool first_step_prior = false;
    double first_step_weight = 0.0;
    Eigen::Vector3d first_step_velocity = Eigen::Vector3d::Zero();

    // gyro-bias prior (with bias estimation)
    Eigen::Vector3d bias_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d bias_prior_cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d bias_prior_information = Eigen::Matrix3d::Zero();

    // the accelerometer term (imu_always_on): its terms in the order they are added, whether the end velocity has one
    std::vector<AccelerometerTermSpan> accelerometer_terms;
    bool end_velocity_tied = false;
    // the start velocity's prior, the zero velocity of a window at rest, the acceleration-offset prior
    Eigen::Vector3d velocity_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d velocity_prior_cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d velocity_prior_information = Eigen::Matrix3d::Zero();
    bool at_rest = false;
    double zero_velocity_weight = 0.0;
    Eigen::Vector3d offset_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d offset_prior_cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d offset_prior_information = Eigen::Matrix3d::Zero();

    // the origin fix: x, y, z, om, fi, ka. The rotation sigmas are used as they are set, in degrees, without conversion
    // to radians (1e-6 gives 1e12 instead of 3.3e15; both hold the first node practically fixed): kept as it was,
    // recorded in docs/code-review-tracker.md
    std::array<double, 6> origin{};

    NdtNoise ndt;
};

// a prior carried from the previous window, grown by its random walk over the window: carried + I rw^2 duration (the
// gyro bias, the acceleration offset)
Eigen::Matrix3d random_walk_prior_cov(const Eigen::Matrix3d &carried, double rw, double duration_s);
// the start velocity's prior from the previous window's end velocity, grown over the gap between the windows by the
// accelerometer's noise: carried + I (density^2 gap + floor^2 gap^2)
Eigen::Matrix3d velocity_prior_cov_over_gap(const Eigen::Matrix3d &carried, double density, double floor, double gap_s);

WindowNoiseModel build_window_noise_model(const Window &window, const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                                          const MotionModelRotationOptions &rotation_options,
                                          const MotionModelTranslationImuOptions &translation_imu_options, const VelocityPrior &velocity_prior);
