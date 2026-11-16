#include "scan_to_map.h"
#include "ndt_map.h"
#include <execution>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include "angle_units.h"
#include "normal_equations.h"
#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "window_equations.h"
#include "window_solver.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{
    // Huber kernel on an NDT point term with squared Mahalanobis distance s = r^T Sigma^-1 r: weight 1 and cost s up to
    // the cutoff c, beyond it weight c / sqrt(s) (IRLS) and cost 2 c sqrt(s) - c^2. Same formula as Codex's diagnostic
    // (cbca36d, ndt_huber_evaluate).
    struct HuberTerm
    {
        double weight, cost;
    };
    HuberTerm ndt_huber(double s, double c)
    {
        if (c <= 0.0 || s <= c * c)
            return {1.0, s};
        const double d = std::sqrt(s);
        return {c / d, 2.0 * c * d - c * c};
    }

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
    const bool time_scaled_sigma = unknowns.time_scaled_sigma;
    const bool estimate_bias = unknowns.estimate_bias;
    const int n_pose_params = unknowns.n_pose_params;
    const bool acc_term = unknowns.acc_term;
    const int n_params = unknowns.n_params;
    const int ib = unknowns.ib, io = unknowns.io, iv = unknowns.iv, ie = unknowns.ie;

    const std::vector<int> &anchors = unknowns.anchors;

    WindowEquations equations(unknowns);
    equations.with_cost = cost != nullptr;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    Eigen::Vector3d b_indoor(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z);

    // sum of n * n^T over the surface normals the points are matched to (per node): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells
    std::vector<Eigen::Matrix3d> normal_information;
    // degeneracy-aware NDT terms: the point terms are first summed per node and enter the normal matrix only after their
    // translation part is projected away from the directions the geometry does not constrain (see the end of the NDT loops)
    const bool project_degenerate = scan_to_map.degenerate_projection_ratio_full > 0.0;
    if (lidar_translation_information || project_degenerate)
        normal_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());
    const bool keep_ndt_blocks = project_degenerate;
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ndt_blocks;
    std::vector<Eigen::Matrix<double, 6, 1>> ndt_rhs;
    if (keep_ndt_blocks)
    {
        ndt_blocks.assign(window.poses.size(), Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero());
        ndt_rhs.assign(window.poses.size(), Eigen::Matrix<double, 6, 1>::Zero());
    }

    // cost (lm_step_check): the NDT term per node (a point adds only to its own node's entry, see for_each_node), summed in
    // node order afterwards, so it is the same with or without threads
    std::vector<double> node_ndt_cost;
    std::vector<int> node_matched, node_unmatched;
    if (cost)
    {
        node_ndt_cost.assign(window.poses.size(), 0.0);
        node_matched.assign(window.poses.size(), 0);
        node_unmatched.assign(window.poses.size(), 0);
    }
    const auto count_unmatched = [&](const Point3Di &p)
    {
        if (cost)
            node_unmatched[p.index_pose]++;
    };
    const auto count_matched = [&](const Point3Di &p, const Eigen::Vector3d &point_global, const Eigen::Vector3d &mean,
                                   const Eigen::Matrix3d &infm, double w)
    {
        if (!cost)
            return;
        const Eigen::Vector3d r = point_global - mean;
        const double s = r.dot(infm * r);
        const double point_cost = w * (scan_to_map.huber_cutoff > 0.0 ? ndt_huber(s, scan_to_map.huber_cutoff).cost : s);
        node_ndt_cost[p.index_pose] += scan_to_map.information_scale != 1.0 ? scan_to_map.information_scale * point_cost : point_cost;
        node_matched[p.index_pose]++;
    };
    // ndt_huber_cutoff: the point's IRLS weight from its distance to the cell (1 when off or within the cutoff), then
    // ndt_information_scale; in this order, as in Codex's build (cbca36d: Huber weight, then scale_ndt_point)
    // A non-finite distance with Huber on sets invalid_ndt_distance (the run stops after the loop, as in Codex's cbca36d).
    std::atomic<bool> invalid_ndt_distance{false};
    const auto apply_huber_and_scale = [&](Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &AtPA, Eigen::Matrix<double, 6, 1> &AtPB,
                                           const Eigen::Vector3d &point_global, const Eigen::Vector3d &mean, const Eigen::Matrix3d &infm)
    {
        if (scan_to_map.huber_cutoff > 0.0)
        {
            const Eigen::Vector3d r = point_global - mean;
            const double s = r.dot(infm * r);
            if (!std::isfinite(s))
                invalid_ndt_distance.store(true, std::memory_order_relaxed);
            const double weight = ndt_huber(s, scan_to_map.huber_cutoff).weight;
            AtPA *= weight;
            AtPB *= weight;
        }
        if (scan_to_map.information_scale != 1.0)
        {
            AtPA *= scan_to_map.information_scale;
            AtPB *= scan_to_map.information_scale;
        }
    };
    double &motion_cost = equations.motion_cost;
    setup_timer.stop();

    const auto hessian_fun_indoor = [&](const Point3Di &intermediate_points_i, const TaitBryanPose &pose_s)
    {
        if (intermediate_points_i.point.norm() < scan_to_map.min_distance || intermediate_points_i.point.norm() > scan_to_map.max_distance)
            return;

        Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = ndt_cell_index(point_global, b_indoor);

        auto bucket_it = buckets_indoor.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets_indoor.end())
        {
            count_unmatched(intermediate_points_i);
            return;
        }

        auto &this_bucket = bucket_it->second;

        Eigen::Matrix3d inverse_scratch;
        const Eigen::Matrix3d *inverse = ndt_cell_inverse(this_bucket, inverse_scratch);
        if (inverse == nullptr)
        {
            count_unmatched(intermediate_points_i);
            return;
        }
        const Eigen::Matrix3d &infm = *inverse;

        if (scan_to_map.use_view_point_and_normal_vectors)
        {
            // check nv
            Eigen::Vector3d &nv = this_bucket.normal_vector;
            Eigen::Vector3d viewport = window.poses[intermediate_points_i.index_pose].translation();
            if (nv.dot(viewport - this_bucket.mean) < 0)
            {
                count_unmatched(intermediate_points_i);
                return;
            }
        }

        const Eigen::Vector3d &p_s = intermediate_points_i.point;

        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

        int c = intermediate_points_i.index_pose * 6;

        double planarity = 1.0;

        if (scan_to_map.use_planarity)
        {
            // planarity
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
            auto eigen_values = eigen_solver.eigenvalues();
            auto eigen_vectors = eigen_solver.eigenvectors();
            double ev1 = eigen_values.x();
            double ev2 = eigen_values.y();
            double ev3 = eigen_values.z();
            double sum_ev = ev1 + ev2 + ev3;
            planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));
        }

        double norm = 1.0;
        if (scan_to_map.use_norm)
            norm = p_s.norm();

        double w = planarity * norm;
        if (w > 10.0)
            w = 10.0;

        if (scan_to_map.use_planarity || scan_to_map.use_norm)
        {
            AtPA *= w;
            AtPB *= w;
        }
        count_matched(intermediate_points_i, point_global, this_bucket.mean, infm, (scan_to_map.use_planarity || scan_to_map.use_norm) ? w : 1.0);
        apply_huber_and_scale(AtPA, AtPB, point_global, this_bucket.mean, infm);

        // only this point's node writes this block (see for_each_node), so no lock is needed
        if (keep_ndt_blocks)
        {
            ndt_blocks[intermediate_points_i.index_pose] += AtPA;
            ndt_rhs[intermediate_points_i.index_pose] -= AtPB;
        }
        else
        {
            AtPAndt.add_diagonal_block6(c, AtPA);
            AtPBndt.block<6, 1>(c, 0) -= AtPB;
        }
        if (!normal_information.empty() && this_bucket.normal_vector.squaredNorm() > 0.5)
            normal_information[intermediate_points_i.index_pose] += this_bucket.normal_vector * this_bucket.normal_vector.transpose();
    };

    PhaseTimer indoor_timer(phase(&ScanToMapTiming::ndt_indoor));

    // The point terms are summed per node, in point order: a point writes only its own node's block (and normal_information
    // entry), so each node's points can run as one task, and the nodes in parallel. Every block then receives the same
    // additions in the same order as a loop over the points in order, so the result is the same with or without threads.
    // points_by_node lists the point indices of node k, in point order, at node_begin[k] .. node_begin[k + 1].
    const int n_nodes = static_cast<int>(window.poses.size());
    std::vector<int> node_begin(n_nodes + 1, 0);
    for (const Point3Di &p : intermediate_points)
        node_begin[p.index_pose + 1]++;
    std::partial_sum(node_begin.begin(), node_begin.end(), node_begin.begin());
    std::vector<int> points_by_node(intermediate_points.size());
    {
        std::vector<int> next = node_begin;
        for (int i = 0; i < static_cast<int>(intermediate_points.size()); i++)
            points_by_node[next[intermediate_points[i].index_pose]++] = i;
    }
    std::vector<int> nodes(n_nodes);
    std::iota(nodes.begin(), nodes.end(), 0);

    const auto for_each_node = [&](const auto &point_term)
    {
        const auto node_terms = [&](int k)
        {
            if (node_begin[k] == node_begin[k + 1])
                return;
            // the node's pose in Tait-Bryan form, once for all its points
            const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(window.poses[k]);
            for (int j = node_begin[k]; j < node_begin[k + 1]; j++)
                point_term(intermediate_points[points_by_node[j]], pose_s);
        };
        if (scan_to_map.multithread)
            std::for_each(std::execution::par, nodes.begin(), nodes.end(), node_terms);
        else
            std::for_each(nodes.begin(), nodes.end(), node_terms);
    };

    for_each_node(hessian_fun_indoor);
    if (invalid_ndt_distance.load(std::memory_order_relaxed))
        throw std::domain_error("non-finite NDT distance (ndt_huber_cutoff) in the indoor grid");
    indoor_timer.stop();

    if (scan_to_map.use_hierarchical_rgd)
    {
        Eigen::Vector3d b_outdoor(scan_to_map.grid_outdoor.resolution_X, scan_to_map.grid_outdoor.resolution_Y, scan_to_map.grid_outdoor.resolution_Z);

        const auto hessian_fun_outdoor = [&](const Point3Di &intermediate_points_i, const TaitBryanPose &pose_s)
        {
            if (intermediate_points_i.point.norm() < scan_to_map.min_distance_outdoor_grid ||
                intermediate_points_i.point.norm() > scan_to_map.max_distance)
                return;

            Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
            auto index_of_bucket = ndt_cell_index(point_global, b_outdoor);

            auto bucket_it = buckets_outdoor.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets_outdoor.end())
            {
                count_unmatched(intermediate_points_i);
                return;
            }

            auto &this_bucket = bucket_it->second;

            Eigen::Matrix3d inverse_scratch;
            const Eigen::Matrix3d *inverse = ndt_cell_inverse(this_bucket, inverse_scratch);
            if (inverse == nullptr)
            {
                count_unmatched(intermediate_points_i);
                return;
            }
            const Eigen::Matrix3d &infm = *inverse;

            if (scan_to_map.use_view_point_and_normal_vectors)
            {
                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = window.poses[intermediate_points_i.index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                {
                    count_unmatched(intermediate_points_i);
                    return;
                }
            }

            const Eigen::Vector3d &p_s = intermediate_points_i.point;

            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
            point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                AtPA,
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

            Eigen::Matrix<double, 6, 1> AtPB;
            point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                AtPB,
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

            int c = intermediate_points_i.index_pose * 6;

            double planarity = 1.0;

            if (scan_to_map.use_planarity)
            {
                // planarity
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
                auto eigen_values = eigen_solver.eigenvalues();
                auto eigen_vectors = eigen_solver.eigenvectors();
                double ev1 = eigen_values.x();
                double ev2 = eigen_values.y();
                double ev3 = eigen_values.z();
                double sum_ev = ev1 + ev2 + ev3;
                planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

                AtPA *= planarity;
                AtPB *= planarity;
            }
            count_matched(intermediate_points_i, point_global, this_bucket.mean, infm, planarity);
            apply_huber_and_scale(AtPA, AtPB, point_global, this_bucket.mean, infm);

            if (keep_ndt_blocks)
            {
                ndt_blocks[intermediate_points_i.index_pose] += AtPA;
                ndt_rhs[intermediate_points_i.index_pose] -= AtPB;
            }
            else
            {
                AtPAndt.add_diagonal_block6(c, AtPA);
                AtPBndt.block<6, 1>(c, 0) -= AtPB;
            }
        };

        PhaseTimer outdoor_timer(phase(&ScanToMapTiming::ndt_outdoor));
        for_each_node(hessian_fun_outdoor);
        if (invalid_ndt_distance.load(std::memory_order_relaxed))
            throw std::domain_error("non-finite NDT distance (ndt_huber_cutoff) in the outdoor grid");
    }

    PhaseTimer motion_terms_timer(phase(&ScanToMapTiming::motion_terms));

    // degeneracy-aware NDT terms: the window's geometric information sum(n * n^T) tells which translation directions the
    // matched surfaces constrain; along a direction with a small share (eigenvalue / largest) the NDT terms are scaled
    // down to nothing (scale s from 0 below degenerate_projection_ratio to 1 at degenerate_projection_ratio_full), so the
    // motion model alone places the window there. With M = diag(P, I), P = sum(s_k * v_k * v_k^T), a node's block B and
    // right-hand side g become M B M and M g. Nothing is multiplied when every direction keeps s = 1.
    if (project_degenerate)
    {
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        for (const auto &ni : normal_information)
            info += ni;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(info);
        const double largest = eig.eigenvalues().maxCoeff();
        const double r0 = scan_to_map.degenerate_projection_ratio;
        const double r1 = scan_to_map.degenerate_projection_ratio_full;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
        bool all_kept = true;
        for (int k = 0; k < 3; k++)
        {
            const double ratio = largest > 0.0 ? eig.eigenvalues()(k) / largest : 0.0;
            const double s = r1 > r0 ? std::clamp((ratio - r0) / (r1 - r0), 0.0, 1.0) : (ratio < r0 ? 0.0 : 1.0);
            all_kept = all_kept && s == 1.0;
            P += s * eig.eigenvectors().col(k) * eig.eigenvectors().col(k).transpose();
        }
        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Identity();
        M.topLeftCorner<3, 3>() = P;
        for (int k = 0; k < n_nodes; k++)
        {
            if (all_kept)
            {
                AtPAndt.add_diagonal_block6(k * 6, ndt_blocks[k]);
                AtPBndt.block<6, 1>(k * 6, 0) += ndt_rhs[k];
            }
            else
            {
                const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> projected = M * ndt_blocks[k] * M;
                AtPAndt.add_diagonal_block6(k * 6, projected);
                AtPBndt.block<6, 1>(k * 6, 0) += M * ndt_rhs[k];
            }
        }
    }

    // geometric translation constraint of this window (world frame, summed over the nodes)
    if (lidar_translation_information)
    {
        lidar_translation_information->setZero();
        for (const auto &ni : normal_information)
            *lidar_translation_information += ni;
    }

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
        for (size_t k = 0; k < node_ndt_cost.size(); k++)
        {
            cost->ndt += node_ndt_cost[k];
            cost->matched += node_matched[k];
            cost->unmatched += node_unmatched[k];
        }
        cost->total = cost->ndt + motion_cost;
    }
    PhaseTimer assembly_timer(phase(&ScanToMapTiming::assembly));

    fix_origin(equations, motion_model);

    assert(AtPAndt.dropped_writes() == 0); // a write outside the band would mean the band was chosen too narrow
    Eigen::SparseMatrix<double> AtPA = AtPAndt.to_sparse();
    Eigen::SparseMatrix<double> AtPB = AtPBndt.sparseView();

    assembly_timer.stop();
    PhaseTimer covariance_timer(phase(&ScanToMapTiming::covariance_solve));
    const ExtraCovariances covariances = extra_unknowns_covariance(AtPA, unknowns, cache.covariance);
    if (covariances.solved)
    {
        if (estimate_bias)
            window.gyro_bias.covariance = covariances.gyro_bias;
        if (acc_term)
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
