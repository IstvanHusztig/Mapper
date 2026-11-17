#include "scan_to_map.h"
#include "lm_solver.h"
#include "ndt_map.h"
#include "ndt_term.h"
#include "window_problem.h"
#include <transformations.h>

#include <algorithm>
#include <cassert>
#include <cmath>

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
    const WindowNoiseModel noise =
        build_window_noise_model(window, scan_to_map, motion_model, rotation_options, translation_imu_options, velocity_prior);
    WindowProblem problem(intermediate_points, window, buckets_indoor, buckets_outdoor, scan_to_map, motion_model, rotation_options,
                          translation_imu_options, noise, acc_offset, acc_offset_cov, logger, lidar_translation_information,
                          timing, cache);
    const auto phase = [timing](PhaseTime ScanToMapTiming::*p) { return timing ? &(timing->*p) : nullptr; };
    const LmTiming lm_timing{phase(&ScanToMapTiming::covariance_solve), phase(&ScanToMapTiming::solve), phase(&ScanToMapTiming::update)};
    const LmIteration it = lm_iteration(problem, lm_factor, apply_step, cost != nullptr, false, delta,
                                        LmWorkspace{cache.covariance, cache.damped}, lm_timing);
    if (cost)
        *cost = problem.cost();
    return it.applied;
}
