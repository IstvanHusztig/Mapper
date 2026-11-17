#include "scan_to_map.h"
#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "ndt_map.h"
#include "ndt_term.h"
#include "normal_equations.h"
#include "window_equations.h"
#include "window_solver.h"
#include <transformations.h>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace
{

}

SteppedPoses stepped_poses(const Window &window, const std::vector<double> &h_x, double max_from_guess_m, Logger &logger)
{
    SteppedPoses out;
    out.poses.reserve(window.poses.size());
    int counter = 0;
    for (size_t i = 0; i < window.poses.size(); i++)
    {
        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(window.poses[i]);
        TaitBryanPose prev_pose = pose_tait_bryan_from_affine_matrix(window.motion_model_poses[i]);
        pose.px += h_x[counter++];
        pose.py += h_x[counter++];
        pose.pz += h_x[counter++];
        pose.om += h_x[counter++];
        pose.fi += h_x[counter++];
        pose.ka += h_x[counter++];

        Eigen::Vector3d p1(prev_pose.px, prev_pose.py, prev_pose.pz);
        Eigen::Vector3d p2(pose.px, pose.py, pose.pz);
        if (!((p1 - p2).norm() < max_from_guess_m))
        {
            log_info(logger) << "big jump on trajectory: " << (p1 - p2).norm() << " (step not applied)";
            out.big_jump_nodes++;
        }
        out.poses.push_back(pose);
    }
    return out;
}

double next_lm_factor(double lm_factor, int iter, bool step_applied)
{
    if (!step_applied)
        lm_factor *= 10.0;
    if (iter % 10 == 0 && iter > 0)
        lm_factor *= 10.0;
    return lm_factor;
}

double ndt_window_cost(const std::vector<Point3Di> &intermediate_points, const std::vector<Eigen::Affine3d> &poses,
                       const NDTBucketMapType &buckets_indoor, const NDTBucketMapType &buckets_outdoor, const ScanToMapOptions &scan_to_map)
{
    double cost = 0.0;
    const auto grid_cost = [&](const NDTBucketMapType &buckets, const NdtGridParameters &grid, double min_distance)
    {
        const Eigen::Vector3d b(grid.resolution_X, grid.resolution_Y, grid.resolution_Z);
        for (const Point3Di &p : intermediate_points)
        {
            const double range = p.point.norm();
            if (range < min_distance || range > scan_to_map.max_distance)
                continue;
            const Eigen::Vector3d g = poses[p.index_pose] * p.point;
            const auto it = buckets.find(ndt_cell_index(g, b));
            if (it == buckets.end())
                continue;
            const NdtBucket &cell = it->second;
            Eigen::Matrix3d scratch;
            const Eigen::Matrix3d *inverse = ndt_cell_inverse(cell, scratch);
            if (inverse == nullptr)
                continue;
            if (scan_to_map.use_view_point_and_normal_vectors && cell.normal_vector.dot(poses[p.index_pose].translation() - cell.mean) < 0)
                continue;
            double w = 1.0;
            if (scan_to_map.use_planarity)
            {
                const Eigen::Vector3d ev = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(cell.cov).eigenvalues();
                const double sum = ev.sum();
                w *= 1 - ((3 * ev(0) / sum) * (3 * ev(1) / sum) * (3 * ev(2) / sum));
            }
            if (scan_to_map.use_norm)
                w *= range;
            w = std::min(w, 10.0);
            const Eigen::Vector3d r = g - cell.mean;
            if (scan_to_map.huber_cutoff > 0.0)
                cost += w * ndt_huber(r.dot(*inverse * r), scan_to_map.huber_cutoff).cost;
            else
                cost += w * r.dot(*inverse * r);
        }
    };
    grid_cost(buckets_indoor, scan_to_map.grid_indoor, scan_to_map.min_distance);
    if (scan_to_map.use_hierarchical_rgd)
        grid_cost(buckets_outdoor, scan_to_map.grid_outdoor, scan_to_map.min_distance_outdoor_grid);
    if (scan_to_map.information_scale != 1.0)
        cost *= scan_to_map.information_scale;
    return cost;
}

bool optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                             NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options,
                             const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                             Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *lidar_translation_information, ScanToMapTiming *timing, ScanToMapWorkspace *workspace,
                             ScanToMapCost *cost, bool apply_step)
{
    ScanToMapWorkspace local_workspace; // without a caller's workspace every call analyses its own patterns
    ScanToMapWorkspace &cache = workspace ? *workspace : local_workspace;
    const auto phase = [timing](PhaseTime ScanToMapTiming::*p) { return timing ? &(timing->*p) : nullptr; };
    PhaseTimer setup_timer(phase(&ScanToMapTiming::setup));

    const WindowUnknowns unknowns = layout_window_unknowns(window, rotation_options, translation_imu_options);
    WindowEquations equations(unknowns);
    equations.with_cost = cost != nullptr;
    setup_timer.stop();

    NdtSums ndt = sum_ndt_point_terms(equations, window, intermediate_points, buckets_indoor, buckets_outdoor, scan_to_map,
                                      lidar_translation_information != nullptr, cost != nullptr, timing);

    PhaseTimer motion_terms_timer(phase(&ScanToMapTiming::motion_terms));

    add_projected_ndt_blocks(equations, ndt, scan_to_map);
    // geometric translation constraint of this window (world frame, summed over the nodes)
    if (lidar_translation_information)
        *lidar_translation_information = window_translation_information(ndt);

    add_motion_edge_terms(equations, unknowns, window, motion_model, rotation_options, translation_imu_options);

    add_gyro_bias_prior(equations, unknowns, window, rotation_options);
    add_velocity_continuity(equations, unknowns, window, motion_model, velocity_prior);

    const bool end_tied = add_acceleration_terms(equations, unknowns, window, translation_imu_options, acc_offset);
    add_velocity_priors(equations, unknowns, window, translation_imu_options, end_tied);
    add_acceleration_offset_prior(equations, unknowns, translation_imu_options, acc_offset);

    motion_terms_timer.stop();

    if (cost)
    {
        *cost = ScanToMapCost{};
        for (size_t k = 0; k < ndt.node_cost.size(); k++)
        {
            cost->ndt += ndt.node_cost[k];
            cost->matched += ndt.node_matched[k];
            cost->unmatched += ndt.node_unmatched[k];
        }
        cost->total = cost->ndt + equations.motion_cost;
    }
    PhaseTimer assembly_timer(phase(&ScanToMapTiming::assembly));

    fix_origin(equations, motion_model);

    assert(equations.matrix.dropped_writes() == 0); // a write outside the band would mean the band was chosen too narrow
    Eigen::SparseMatrix<double> AtPA = equations.matrix.to_sparse();
    Eigen::SparseMatrix<double> AtPB = equations.rhs.sparseView();

    assembly_timer.stop();
    PhaseTimer covariance_timer(phase(&ScanToMapTiming::covariance_solve));
    const ExtraCovariances covariances = extra_unknowns_covariance(AtPA, unknowns, cache.covariance);
    if (covariances.solved)
    {
        if (unknowns.estimate_bias)
            window.gyro_bias.covariance = covariances.gyro_bias;
        if (unknowns.acc_term)
        {
            acc_offset_cov = covariances.acceleration_offset;
            window.velocity.end_covariance = covariances.end_velocity;
        }
    }
    covariance_timer.stop();
    if (!apply_step)
        return false;

    PhaseTimer solve_timer(phase(&ScanToMapTiming::solve));
    const WindowStep step = solve_window_step(AtPA, AtPB, unknowns, lm_factor, cache.damped);
    solve_timer.stop();
    PhaseTimer update_timer(phase(&ScanToMapTiming::update));
    return apply_window_step(window, step, unknowns, rotation_options, acc_offset, delta, cache, logger);
}
