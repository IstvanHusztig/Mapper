#include "window_problem.h"
#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "ndt_term.h"
#include "window_solver.h"

#include <cassert>

WindowProblem::WindowProblem(std::vector<Point3Di> &points, Window &window, NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *translation_information, ScanToMapTiming *timing, ScanToMapWorkspace &workspace)
    : points_(points), window_(window), buckets_indoor_(buckets_indoor), buckets_outdoor_(buckets_outdoor), scan_to_map_(scan_to_map),
      motion_model_(motion_model), rotation_options_(rotation_options), translation_imu_options_(translation_imu_options),
      velocity_prior_(velocity_prior), acc_offset_(acc_offset), acc_offset_cov_(acc_offset_cov), logger_(logger),
      translation_information_(translation_information), timing_(timing), workspace_(workspace)
{
}

LmLinearization WindowProblem::linearize(bool with_cost)
{
    const auto phase = [this](PhaseTime ScanToMapTiming::*p) { return timing_ ? &(timing_->*p) : nullptr; };
    PhaseTimer setup_timer(phase(&ScanToMapTiming::setup));

    unknowns_ = layout_window_unknowns(window_, rotation_options_, translation_imu_options_);
    WindowEquations equations(unknowns_);
    equations.with_cost = with_cost;
    setup_timer.stop();

    NdtSums ndt = sum_ndt_point_terms(equations, window_, points_, buckets_indoor_, buckets_outdoor_, scan_to_map_,
                                      translation_information_ != nullptr, with_cost, timing_);

    PhaseTimer motion_terms_timer(phase(&ScanToMapTiming::motion_terms));

    add_projected_ndt_blocks(equations, ndt, scan_to_map_);
    // geometric translation constraint of this window (world frame, summed over the nodes)
    if (translation_information_)
        *translation_information_ = window_translation_information(ndt);

    add_motion_edge_terms(equations, unknowns_, window_, motion_model_, rotation_options_, translation_imu_options_);

    add_gyro_bias_prior(equations, unknowns_, window_, rotation_options_);
    add_velocity_continuity(equations, unknowns_, window_, motion_model_, velocity_prior_);

    const bool end_tied = add_acceleration_terms(equations, unknowns_, window_, translation_imu_options_, acc_offset_);
    add_velocity_priors(equations, unknowns_, window_, translation_imu_options_, end_tied);
    add_acceleration_offset_prior(equations, unknowns_, translation_imu_options_, acc_offset_);

    motion_terms_timer.stop();

    if (with_cost)
    {
        cost_ = ScanToMapCost{};
        for (size_t k = 0; k < ndt.node_cost.size(); k++)
        {
            cost_.ndt += ndt.node_cost[k];
            cost_.matched += ndt.node_matched[k];
            cost_.unmatched += ndt.node_unmatched[k];
        }
        cost_.total = cost_.ndt + equations.motion_cost;
    }
    PhaseTimer assembly_timer(phase(&ScanToMapTiming::assembly));

    fix_origin(equations, motion_model_);

    assert(equations.matrix.dropped_writes() == 0); // a write outside the band would mean the band was chosen too narrow
    LmLinearization lin;
    lin.AtPA = equations.matrix.to_sparse();
    lin.AtPB = equations.rhs.sparseView();
    lin.damped = unknowns_.n_pose_params;
    lin.cost = with_cost ? cost_.total : 0.0;
    return lin;
}

void WindowProblem::take_covariance(const Eigen::MatrixXd &cols)
{
    const int n_pose_params = unknowns_.n_pose_params;
    if (unknowns_.estimate_bias)
        window_.gyro_bias.covariance = cols.block<3, 3>(unknowns_.ib, unknowns_.ib - n_pose_params);
    if (unknowns_.acc_term)
    {
        acc_offset_cov_ = cols.block<3, 3>(unknowns_.io, unknowns_.io - n_pose_params);
        window_.velocity.end_covariance = cols.block<3, 3>(unknowns_.ie, unknowns_.ie - n_pose_params);
    }
}

bool WindowProblem::apply(const std::vector<double> &h_x, bool solved, double &delta)
{
    WindowStep step;
    step.solved = solved;
    step.h_x = h_x;
    return apply_window_step(window_, step, unknowns_, rotation_options_, acc_offset_, delta, workspace_, logger_);
}

WindowProblem::State WindowProblem::state() const
{
    return State{window_.poses, window_.velocity, window_.gyro_bias, acc_offset_, acc_offset_cov_,
                 translation_information_ ? *translation_information_ : Eigen::Matrix3d::Zero(), cost_};
}

void WindowProblem::set_state(const State &s)
{
    window_.poses = s.poses;
    window_.velocity = s.velocity;
    window_.gyro_bias = s.gyro_bias;
    acc_offset_ = s.acc_offset;
    acc_offset_cov_ = s.acc_offset_cov;
    if (translation_information_)
        *translation_information_ = s.translation_information;
    cost_ = s.cost;
}

void WindowProblem::snapshot() { candidate_ = state(); }

void WindowProblem::accept_snapshot() { accepted_ = std::move(candidate_); }

void WindowProblem::restore_accepted() { set_state(accepted_); }
