#include "ndt_term.h"
#include "ndt_map.h"
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <execution>
#include <numeric>
#include <stdexcept>

NdtSums sum_ndt_point_terms(WindowEquations &equations, const Window &window, const std::vector<Point3Di> &intermediate_points,
                            NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor, const ScanToMapOptions &scan_to_map,
                            bool normal_information_wanted, bool with_cost, ScanToMapTiming *timing)
{
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    const auto phase = [timing](PhaseTime ScanToMapTiming::*p) { return timing ? &(timing->*p) : nullptr; };
    NdtSums sums;
    Eigen::Vector3d b_indoor(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z);

    // sum of n * n^T over the surface normals the points are matched to (per node): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells
    std::vector<Eigen::Matrix3d> &normal_information = sums.normal_information;
    // degeneracy-aware NDT terms: the point terms are first summed per node and enter the normal matrix only after their
    // translation part is projected away from the directions the geometry does not constrain (see the end of the NDT loops)
    const bool project_degenerate = scan_to_map.degenerate_projection_ratio_full > 0.0;
    if (normal_information_wanted || project_degenerate)
        normal_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());
    const bool keep_ndt_blocks = project_degenerate;
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> &ndt_blocks = sums.blocks;
    std::vector<Eigen::Matrix<double, 6, 1>> &ndt_rhs = sums.rhs;
    if (keep_ndt_blocks)
    {
        ndt_blocks.assign(window.poses.size(), Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero());
        ndt_rhs.assign(window.poses.size(), Eigen::Matrix<double, 6, 1>::Zero());
    }

    // cost (lm_step_check): the NDT term per node (a point adds only to its own node's entry, see for_each_node), summed in
    // node order afterwards, so it is the same with or without threads
    std::vector<double> &node_ndt_cost = sums.node_cost;
    std::vector<int> &node_matched = sums.node_matched, &node_unmatched = sums.node_unmatched;
    if (with_cost)
    {
        node_ndt_cost.assign(window.poses.size(), 0.0);
        node_matched.assign(window.poses.size(), 0);
        node_unmatched.assign(window.poses.size(), 0);
    }
    const auto count_unmatched = [&](const Point3Di &p)
    {
        if (with_cost)
            node_unmatched[p.index_pose]++;
    };
    const auto count_matched = [&](const Point3Di &p, const Eigen::Vector3d &point_global, const Eigen::Vector3d &mean,
                                   const Eigen::Matrix3d &infm, double w)
    {
        if (!with_cost)
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

    // One point term for both grids. A grid has its cells, cell size and nearest range, and three things only the indoor
    // grid does: the range weight (use_norm), the cap of the weight at 10, and the normal information. For the outdoor grid
    // the weight is planarity * 1.0, which is planarity exactly, so both grids compute what their two copies computed.
    struct Grid
    {
        NDTBucketMapType &buckets;
        Eigen::Vector3d b;
        double min_distance;
        bool range_weight;
        bool cap;
        bool normal_information;
    };
    const auto point_term = [&](const Grid &grid, const Point3Di &intermediate_points_i, const TaitBryanPose &pose_s)
    {
        if (intermediate_points_i.point.norm() < grid.min_distance || intermediate_points_i.point.norm() > scan_to_map.max_distance)
            return;

        Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = ndt_cell_index(point_global, grid.b);

        auto bucket_it = grid.buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == grid.buckets.end())
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
        if (grid.range_weight)
            norm = p_s.norm();

        double w = planarity * norm;
        if (grid.cap && w > 10.0)
            w = 10.0;

        const bool weighted = scan_to_map.use_planarity || grid.range_weight;
        if (weighted)
        {
            AtPA *= w;
            AtPB *= w;
        }
        count_matched(intermediate_points_i, point_global, this_bucket.mean, infm, weighted ? w : 1.0);
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
        if (grid.normal_information && !normal_information.empty() && this_bucket.normal_vector.squaredNorm() > 0.5)
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

    const Grid indoor{buckets_indoor, b_indoor, scan_to_map.min_distance, scan_to_map.use_norm, true, true};
    for_each_node([&](const Point3Di &p, const TaitBryanPose &pose) { point_term(indoor, p, pose); });
    if (invalid_ndt_distance.load(std::memory_order_relaxed))
        throw std::domain_error("non-finite NDT distance (ndt_huber_cutoff) in the indoor grid");
    indoor_timer.stop();

    if (scan_to_map.use_hierarchical_rgd)
    {
        Eigen::Vector3d b_outdoor(scan_to_map.grid_outdoor.resolution_X, scan_to_map.grid_outdoor.resolution_Y, scan_to_map.grid_outdoor.resolution_Z);

        const Grid outdoor{buckets_outdoor, b_outdoor, scan_to_map.min_distance_outdoor_grid, false, false, false};

        PhaseTimer outdoor_timer(phase(&ScanToMapTiming::ndt_outdoor));
        for_each_node([&](const Point3Di &p, const TaitBryanPose &pose) { point_term(outdoor, p, pose); });
        if (invalid_ndt_distance.load(std::memory_order_relaxed))
            throw std::domain_error("non-finite NDT distance (ndt_huber_cutoff) in the outdoor grid");
    }

    return sums;
}

void add_projected_ndt_blocks(WindowEquations &equations, const NdtSums &sums, const ScanToMapOptions &scan_to_map)
{
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    // degeneracy-aware NDT terms: the window's geometric information sum(n * n^T) tells which translation directions the
    // matched surfaces constrain; along a direction with a small share (eigenvalue / largest) the NDT terms are scaled
    // down to nothing (scale s from 0 below degenerate_projection_ratio to 1 at degenerate_projection_ratio_full), so the
    // motion model alone places the window there. With M = diag(P, I), P = sum(s_k * v_k * v_k^T), a node's block B and
    // right-hand side g become M B M and M g. Nothing is multiplied when every direction keeps s = 1.
    if (!(scan_to_map.degenerate_projection_ratio_full > 0.0))
        return;
    {
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        for (const auto &ni : sums.normal_information)
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
        for (int k = 0; k < static_cast<int>(sums.blocks.size()); k++)
        {
            if (all_kept)
            {
                AtPAndt.add_diagonal_block6(k * 6, sums.blocks[k]);
                AtPBndt.block<6, 1>(k * 6, 0) += sums.rhs[k];
            }
            else
            {
                const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> projected = M * sums.blocks[k] * M;
                AtPAndt.add_diagonal_block6(k * 6, projected);
                AtPBndt.block<6, 1>(k * 6, 0) += M * sums.rhs[k];
            }
        }
    }
}

Eigen::Matrix3d window_translation_information(const NdtSums &sums)
{
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    for (const auto &ni : sums.normal_information)
        information += ni;
    return information;
}
