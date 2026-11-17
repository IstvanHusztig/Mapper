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
#include <string>

NdtGrid indoor_ndt_grid(const NDTBucketMapType &buckets, const ScanToMapOptions &scan_to_map)
{
    return NdtGrid{buckets,
                   Eigen::Vector3d(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z),
                   scan_to_map.min_distance, scan_to_map.use_norm, true, true, "indoor"};
}

NdtGrid outdoor_ndt_grid(const NDTBucketMapType &buckets, const ScanToMapOptions &scan_to_map)
{
    return NdtGrid{buckets,
                   Eigen::Vector3d(scan_to_map.grid_outdoor.resolution_X, scan_to_map.grid_outdoor.resolution_Y, scan_to_map.grid_outdoor.resolution_Z),
                   scan_to_map.min_distance_outdoor_grid, false, false, false, "outdoor"};
}

namespace
{
    // a window's points listed by node: the point indices of node k, in point order, at order[begin[k]] .. order[begin[k + 1] - 1]
    struct PointsByNode
    {
        std::vector<int> begin;
        std::vector<int> order;
    };

    PointsByNode index_points_by_node(const std::vector<Point3Di> &points, int n_nodes)
    {
        PointsByNode by_node;
        by_node.begin.assign(n_nodes + 1, 0);
        for (const Point3Di &p : points)
            by_node.begin[p.index_pose + 1]++;
        std::partial_sum(by_node.begin.begin(), by_node.begin.end(), by_node.begin.begin());
        by_node.order.resize(points.size());
        std::vector<int> next = by_node.begin;
        for (int i = 0; i < static_cast<int>(points.size()); i++)
            by_node.order[next[points[i].index_pose]++] = i;
        return by_node;
    }

    // where the point terms go: the node blocks of `sums`, or (equations not null) straight into the matrix; the normal
    // information and the cost counts always into `sums`
    struct NdtPointTarget
    {
        NdtSums &sums;
        WindowEquations *equations;
        bool with_cost;
        // a non-finite distance with Huber on (the run stops after the grid's loop, as in Codex's cbca36d)
        std::atomic<bool> invalid_distance{false};
    };

    // the per-node sums, sized for n nodes: the normal information (normals), the node blocks (blocks), the cost (with_cost)
    NdtSums empty_ndt_sums(size_t n, bool normals, bool blocks, bool with_cost)
    {
        NdtSums sums;
        if (normals)
            sums.normal_information.assign(n, Eigen::Matrix3d::Zero());
        if (blocks)
        {
            sums.blocks.assign(n, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero());
            sums.rhs.assign(n, Eigen::Matrix<double, 6, 1>::Zero());
        }
        if (with_cost)
        {
            sums.node_cost.assign(n, 0.0);
            sums.node_matched.assign(n, 0);
            sums.node_unmatched.assign(n, 0);
        }
        return sums;
    }

    void count_unmatched(const Point3Di &p, NdtPointTarget &t)
    {
        if (t.with_cost)
            t.sums.node_unmatched[p.index_pose]++;
    }

    // cost (lm_step_check): the point's w r^T M r (Huber cost when on), times the information scale
    void count_matched(const Point3Di &p, const Eigen::Vector3d &point_global, const Eigen::Vector3d &mean, const Eigen::Matrix3d &infm, double w,
                       const ScanToMapOptions &scan_to_map, NdtPointTarget &t)
    {
        if (!t.with_cost)
            return;
        const Eigen::Vector3d r = point_global - mean;
        const double s = r.dot(infm * r);
        const double point_cost = w * (scan_to_map.huber_cutoff > 0.0 ? ndt_huber(s, scan_to_map.huber_cutoff).cost : s);
        t.sums.node_cost[p.index_pose] += scan_to_map.information_scale != 1.0 ? scan_to_map.information_scale * point_cost : point_cost;
        t.sums.node_matched[p.index_pose]++;
    }

    // ndt_huber_cutoff: the point's IRLS weight from its distance to the cell (1 when off or within the cutoff), then
    // ndt_information_scale; in this order, as in Codex's build (cbca36d: Huber weight, then scale_ndt_point)
    void apply_huber_and_scale(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &AtPA, Eigen::Matrix<double, 6, 1> &AtPB, const Eigen::Vector3d &point_global,
                               const Eigen::Vector3d &mean, const Eigen::Matrix3d &infm, const ScanToMapOptions &scan_to_map, NdtPointTarget &t)
    {
        if (scan_to_map.huber_cutoff > 0.0)
        {
            const Eigen::Vector3d r = point_global - mean;
            const double s = r.dot(infm * r);
            if (!std::isfinite(s))
                t.invalid_distance.store(true, std::memory_order_relaxed);
            const double weight = ndt_huber(s, scan_to_map.huber_cutoff).weight;
            AtPA *= weight;
            AtPB *= weight;
        }
        if (scan_to_map.information_scale != 1.0)
        {
            AtPA *= scan_to_map.information_scale;
            AtPB *= scan_to_map.information_scale;
        }
    }

    // the cell a point falls into, when it can take part: in range, a cell with a usable inverse, seen from its front
    // (view point test); nullptr otherwise (counted as unmatched when in range)
    const NdtBucket *matched_cell(const NdtGrid &grid, const Point3Di &p, const Eigen::Vector3d &point_global, const Eigen::Affine3d &node_pose,
                                  const ScanToMapOptions &scan_to_map, Eigen::Matrix3d &inverse_scratch, const Eigen::Matrix3d *&inverse, NdtPointTarget &t)
    {
        auto bucket_it = grid.buckets.find(ndt_cell_index(point_global, grid.b));
        if (bucket_it == grid.buckets.end())
        {
            count_unmatched(p, t);
            return nullptr;
        }
        const NdtBucket &bucket = bucket_it->second;
        inverse = ndt_cell_inverse(bucket, inverse_scratch);
        if (inverse == nullptr)
        {
            count_unmatched(p, t);
            return nullptr;
        }
        if (scan_to_map.use_view_point_and_normal_vectors && bucket.normal_vector.dot(node_pose.translation() - bucket.mean) < 0)
        {
            count_unmatched(p, t);
            return nullptr;
        }
        return &bucket;
    }

    // the planarity / range weight of a point term (1 when neither is on)
    double ndt_point_weight(const NdtGrid &grid, const NdtBucket &bucket, const Eigen::Vector3d &p_s, const ScanToMapOptions &scan_to_map)
    {
        double planarity = 1.0;
        if (scan_to_map.use_planarity)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(bucket.cov, Eigen::ComputeEigenvectors);
            auto eigen_values = eigen_solver.eigenvalues();
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
        return w;
    }

    // one point against one grid: its 6x6 block and right-hand side, weighted, into its node's entry of the target. Only
    // this point's node writes there (see sum_ndt_grid), so no lock is needed
    void add_ndt_point_term(const NdtGrid &grid, const Point3Di &p, const TaitBryanPose &pose_s, const Eigen::Affine3d &node_pose,
                            const ScanToMapOptions &scan_to_map, NdtPointTarget &t)
    {
        if (p.point.norm() < grid.min_distance || p.point.norm() > scan_to_map.max_distance)
            return;
        const Eigen::Vector3d point_global = node_pose * p.point;
        Eigen::Matrix3d inverse_scratch;
        const Eigen::Matrix3d *inverse = nullptr;
        const NdtBucket *bucket = matched_cell(grid, p, point_global, node_pose, scan_to_map, inverse_scratch, inverse, t);
        if (bucket == nullptr)
            return;
        const Eigen::Matrix3d &infm = *inverse;
        const Eigen::Vector3d &p_s = p.point;

        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));
        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            bucket->mean.x(), bucket->mean.y(), bucket->mean.z());

        const double w = ndt_point_weight(grid, *bucket, p_s, scan_to_map);
        const bool weighted = scan_to_map.use_planarity || grid.range_weight;
        if (weighted)
        {
            AtPA *= w;
            AtPB *= w;
        }
        count_matched(p, point_global, bucket->mean, infm, weighted ? w : 1.0, scan_to_map, t);
        apply_huber_and_scale(AtPA, AtPB, point_global, bucket->mean, infm, scan_to_map, t);

        if (t.equations == nullptr)
        {
            t.sums.blocks[p.index_pose] += AtPA;
            t.sums.rhs[p.index_pose] -= AtPB;
        }
        else
        {
            t.equations->matrix.add_diagonal_block6(p.index_pose * 6, AtPA);
            t.equations->rhs.block<6, 1>(p.index_pose * 6, 0) -= AtPB;
        }
        if (grid.normal_information && !t.sums.normal_information.empty() && bucket->normal_vector.squaredNorm() > 0.5)
            t.sums.normal_information[p.index_pose] += bucket->normal_vector * bucket->normal_vector.transpose();
    }

    // one grid's point terms, per node in parallel: a point writes only its own node's entries, so each node's points run
    // as one task in point order, and every entry receives the same additions in the same order as a loop over the points.
    // Throws std::domain_error on a non-finite distance (Huber on).
    void sum_ndt_grid(const NdtGrid &grid, const Window &window, const std::vector<Point3Di> &points, const PointsByNode &by_node,
                      const ScanToMapOptions &scan_to_map, NdtPointTarget &target)
    {
        const int n_nodes = static_cast<int>(window.poses.size());
        std::vector<int> nodes(n_nodes);
        std::iota(nodes.begin(), nodes.end(), 0);
        const auto node_terms = [&](int k)
        {
            if (by_node.begin[k] == by_node.begin[k + 1])
                return;
            // the node's pose in Tait-Bryan form, once for all its points
            const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(window.poses[k]);
            for (int j = by_node.begin[k]; j < by_node.begin[k + 1]; j++)
                add_ndt_point_term(grid, points[by_node.order[j]], pose_s, window.poses[k], scan_to_map, target);
        };
        if (scan_to_map.multithread)
            std::for_each(std::execution::par, nodes.begin(), nodes.end(), node_terms);
        else
            std::for_each(nodes.begin(), nodes.end(), node_terms);
        if (target.invalid_distance.load(std::memory_order_relaxed))
            throw std::domain_error(std::string("non-finite NDT distance (ndt_huber_cutoff) in the ") + grid.name + " grid");
    }
}

NdtSums sum_ndt_point_terms(WindowEquations &equations, const Window &window, const std::vector<Point3Di> &intermediate_points,
                            const NDTBucketMapType &buckets_indoor, const NDTBucketMapType &buckets_outdoor, const ScanToMapOptions &scan_to_map,
                            bool normal_information_wanted, bool with_cost, ScanToMapTiming *timing)
{
    const auto phase = [timing](PhaseTime ScanToMapTiming::*p) { return timing ? &(timing->*p) : nullptr; };
    // degeneracy-aware NDT terms: the point terms are first summed per node and enter the normal matrix only after their
    // translation part is projected away from the directions the geometry does not constrain (add_projected_ndt_blocks)
    const bool project_degenerate = scan_to_map.degenerate_projection_ratio_full > 0.0;
    NdtSums sums = empty_ndt_sums(window.poses.size(), normal_information_wanted || project_degenerate, project_degenerate, with_cost);
    NdtPointTarget target{sums, project_degenerate ? nullptr : &equations, with_cost};

    PhaseTimer indoor_timer(phase(&ScanToMapTiming::ndt_indoor));
    const PointsByNode by_node = index_points_by_node(intermediate_points, static_cast<int>(window.poses.size()));
    sum_ndt_grid(indoor_ndt_grid(buckets_indoor, scan_to_map), window, intermediate_points, by_node, scan_to_map, target);
    indoor_timer.stop();

    if (scan_to_map.use_hierarchical_rgd)
    {
        PhaseTimer outdoor_timer(phase(&ScanToMapTiming::ndt_outdoor));
        sum_ndt_grid(outdoor_ndt_grid(buckets_outdoor, scan_to_map), window, intermediate_points, by_node, scan_to_map, target);
    }
    return sums;
}

NdtSums sum_ndt_grid_blocks(const NdtGrid &grid, const Window &window, const std::vector<Point3Di> &intermediate_points,
                            const ScanToMapOptions &scan_to_map, bool with_cost)
{
    NdtSums sums = empty_ndt_sums(window.poses.size(), grid.normal_information, true, with_cost);
    NdtPointTarget target{sums, nullptr, with_cost};
    const PointsByNode by_node = index_points_by_node(intermediate_points, static_cast<int>(window.poses.size()));
    sum_ndt_grid(grid, window, intermediate_points, by_node, scan_to_map, target);
    return sums;
}

NdtProjection ndt_degeneracy_projection(const Eigen::Matrix3d &information, const ScanToMapOptions &scan_to_map)
{
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(information);
    const double largest = eig.eigenvalues().maxCoeff();
    const double r0 = scan_to_map.degenerate_projection_ratio;
    const double r1 = scan_to_map.degenerate_projection_ratio_full;
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    NdtProjection projection;
    projection.all_kept = true;
    for (int k = 0; k < 3; k++)
    {
        const double ratio = largest > 0.0 ? eig.eigenvalues()(k) / largest : 0.0;
        const double s = r1 > r0 ? std::clamp((ratio - r0) / (r1 - r0), 0.0, 1.0) : (ratio < r0 ? 0.0 : 1.0);
        projection.all_kept = projection.all_kept && s == 1.0;
        P += s * eig.eigenvectors().col(k) * eig.eigenvectors().col(k).transpose();
    }
    projection.M = Eigen::Matrix<double, 6, 6>::Identity();
    projection.M.topLeftCorner<3, 3>() = P;
    return projection;
}

void add_ndt_blocks(WindowEquations &equations, const NdtSums &sums, const NdtProjection &projection)
{
    const Eigen::Matrix<double, 6, 6> &M = projection.M;
    for (int k = 0; k < static_cast<int>(sums.blocks.size()); k++)
    {
        if (projection.all_kept)
        {
            equations.matrix.add_diagonal_block6(k * 6, sums.blocks[k]);
            equations.rhs.block<6, 1>(k * 6, 0) += sums.rhs[k];
        }
        else
        {
            const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> projected = M * sums.blocks[k] * M;
            equations.matrix.add_diagonal_block6(k * 6, projected);
            equations.rhs.block<6, 1>(k * 6, 0) += M * sums.rhs[k];
        }
    }
}

void add_projected_ndt_blocks(WindowEquations &equations, const NdtSums &sums, const ScanToMapOptions &scan_to_map)
{
    // degeneracy-aware NDT terms: the window's geometric information sum(n * n^T) tells which translation directions the
    // matched surfaces constrain; along a direction with a small share (eigenvalue / largest) the NDT terms are scaled
    // down to nothing (scale s from 0 below degenerate_projection_ratio to 1 at degenerate_projection_ratio_full), so the
    // motion model alone places the window there. With M = diag(P, I), P = sum(s_k * v_k * v_k^T), a node's block B and
    // right-hand side g become M B M and M g. Nothing is multiplied when every direction keeps s = 1.
    if (!(scan_to_map.degenerate_projection_ratio_full > 0.0))
        return;
    add_ndt_blocks(equations, sums, ndt_degeneracy_projection(window_translation_information(sums), scan_to_map));
}

Eigen::Matrix3d window_translation_information(const NdtSums &sums)
{
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    for (const auto &ni : sums.normal_information)
        information += ni;
    return information;
}
