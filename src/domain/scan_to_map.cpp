#include "scan_to_map.h"
#include "ndt_map.h"
#include <execution>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include "angle_units.h"
#include "normal_equations.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{
    // d(world rotation increment) / d(om, fi, ka) at the node's attitude, by central differences (independent of the angle
    // convention): column j is the rotation vector of R(angles + h e_j) R(angles - h e_j)^T / 2h
    Eigen::Matrix3d tait_bryan_rotation_jacobian(const Eigen::Affine3d &m)
    {
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(m);
        constexpr double h = 1e-5;
        Eigen::Matrix3d e;
        for (int j = 0; j < 3; j++)
        {
            TaitBryanPose plus = p, minus = p;
            double *const angle_plus[3] = {&plus.om, &plus.fi, &plus.ka};
            double *const angle_minus[3] = {&minus.om, &minus.fi, &minus.ka};
            *angle_plus[j] += h;
            *angle_minus[j] -= h;
            const Eigen::Matrix3d d = affine_matrix_from_pose_tait_bryan(plus).linear() * affine_matrix_from_pose_tait_bryan(minus).linear().transpose();
            e.col(j) = Eigen::Vector3d(d(2, 1) - d(1, 2), d(0, 2) - d(2, 0), d(1, 0) - d(0, 1)) * 0.5 / (2.0 * h);
        }
        return e;
    }

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

    Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d s;
        s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return s;
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

    const bool have_dt = window.timestamps.size() == window.poses.size();
    const bool time_scaled_sigma = rotation_options.time_scaled_sigma && have_dt;
    const bool estimate_bias = rotation_options.estimate_bias && have_dt;
    const int n_pose_params = static_cast<int>(window.poses.size()) * 6;
    const bool acc_term = translation_imu_options.enabled && have_dt && translation_imu_options.acc &&
                          translation_imu_options.acc->size() == window.poses.size();
    // [pose_0 .. pose_n-1 | b_om b_fi b_ka (gyro bias) | e_x e_y e_z (acceleration offset) | v_start | v_end (accelerometer term)]
    const int n_params = n_pose_params + (estimate_bias ? 3 : 0) + (acc_term ? 9 : 0);
    const int ib = n_pose_params;                        // first bias column
    const int io = n_pose_params + (estimate_bias ? 3 : 0); // first acceleration offset column
    const int iv = io + 3;                               // start velocity (accelerometer term)
    const int ie = iv + 3;                               // end velocity (accelerometer term)

    double sigma_motion_model_om = motion_model.om_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_fi = motion_model.fi_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_ka = motion_model.ka_sigma_deg * DEG_TO_RAD;

    double w_motion_model_om = 1.0 / (sigma_motion_model_om * sigma_motion_model_om);
    double w_motion_model_fi = 1.0 / (sigma_motion_model_fi * sigma_motion_model_fi);
    double w_motion_model_ka = 1.0 / (sigma_motion_model_ka * sigma_motion_model_ka);

    double w_motion_model_x = 1.0 / (motion_model.x_sigma_m * motion_model.x_sigma_m);
    double w_motion_model_y = 1.0 / (motion_model.y_sigma_m * motion_model.y_sigma_m);
    double w_motion_model_z = 1.0 / (motion_model.z_sigma_m * motion_model.z_sigma_m);
    // imu_position_information_scale: the x/y/z rows (with their attitude coupling) weaker while the accelerometer term is
    // active (Codex's "beta", DMAPPER_EXPERIMENT_MOTION_XYZ_SCALE in cbca36d)
    if (acc_term && translation_imu_options.position_information_scale != 1.0)
    {
        w_motion_model_x *= translation_imu_options.position_information_scale;
        w_motion_model_y *= translation_imu_options.position_information_scale;
        w_motion_model_z *= translation_imu_options.position_information_scale;
    }

    // The accelerometer term's anchors (nodes about span_s apart) and the band of the normal matrix: the pose-pose coupling
    // reaches 17 parameters (two nodes) without the term, and (last - first node) * 6 + 5 for its widest term. Both depend
    // only on the timestamps, so the band is known before anything is added.
    std::vector<int> anchors;
    int pose_band_offset = 17;
    if (acc_term)
    {
        const int n = static_cast<int>(window.poses.size());
        anchors.push_back(0);
        for (int k = 1; k < n; k++)
            if (window.timestamps[k].first - window.timestamps[anchors.back()].first >= translation_imu_options.span_s)
                anchors.push_back(k);
        for (size_t j = 1; j + 1 < anchors.size(); j++)
            pose_band_offset = std::max(pose_band_offset, (anchors[j + 1] - anchors[j - 1]) * 6 + 5);
        if (anchors.size() > 1)
            pose_band_offset = std::max(pose_band_offset, anchors[1] * 6 + 5);
    }
    // the end velocity is tied to the last two anchors, with the accelerometer from their midpoint to the last node
    if (acc_term && anchors.size() > 1)
        pose_band_offset = std::max(pose_band_offset, (static_cast<int>(window.poses.size()) - 1 - anchors[anchors.size() - 2]) * 6 + 5);

    // the normal matrix, stored by band (only the band is ever used)
    BandedNormalMatrix AtPAndt(n_pose_params, n_params - n_pose_params, pose_band_offset);
    Eigen::MatrixXd AtPBndt(n_params, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b_indoor(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z);

    // sum of n * n^T over the surface normals the points are matched to (per node): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells
    std::vector<Eigen::Matrix3d> normal_information;
    // degeneracy-aware NDT terms: the point terms are first summed per node and enter the normal matrix only after their
    // translation part is projected away from the directions the geometry does not constrain (see the end of the NDT loops)
    const bool project_degenerate = scan_to_map.degenerate_projection_ratio_full > 0.0;
    if (lidar_translation_information || project_degenerate)
        normal_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());
    // carried_covariance_normal_information_only: the covariance of the extra unknowns counts the NDT terms only by
    // their normal-direction information. It needs the NDT terms summed per node as well, and per node the sum of
    // n n^T / (n^T C n) over the matched cells (C the cell's covariance, n its normal from all points): what the cells
    // say about a displacement along their normals. A point's NDT translation information is C^-1, never less than that.
    const bool normal_only_covariance = scan_to_map.carried_covariance_normal_information_only && n_params > n_pose_params;
    const bool keep_ndt_blocks = project_degenerate || normal_only_covariance;
    std::vector<Eigen::Matrix3d> normal_direction_information;
    if (normal_only_covariance)
        normal_direction_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());
    const auto add_normal_direction_information = [&](const NdtBucket &cell, double weight, int node)
    {
        const Eigen::Vector3d n = cell.all_points_normal.cast<double>();
        if (n.squaredNorm() < 0.5)
            return;
        const double variance = n.dot(cell.cov * n);
        if (variance > 0.0)
            normal_direction_information[node] += (weight / variance) * n * n.transpose();
    };
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
    // Returns the factor applied (Huber weight times scale; 1 when both are off), so the normal-direction information of
    // carried_covariance_normal_information_only counts the point as much as its block does. A non-finite distance with
    // Huber on sets invalid_ndt_distance (the run stops after the loop, as in Codex's cbca36d).
    std::atomic<bool> invalid_ndt_distance{false};
    const auto apply_huber_and_scale = [&](Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &AtPA, Eigen::Matrix<double, 6, 1> &AtPB,
                                           const Eigen::Vector3d &point_global, const Eigen::Vector3d &mean, const Eigen::Matrix3d &infm)
    {
        double factor = 1.0;
        if (scan_to_map.huber_cutoff > 0.0)
        {
            const Eigen::Vector3d r = point_global - mean;
            const double s = r.dot(infm * r);
            if (!std::isfinite(s))
                invalid_ndt_distance.store(true, std::memory_order_relaxed);
            const double weight = ndt_huber(s, scan_to_map.huber_cutoff).weight;
            AtPA *= weight;
            AtPB *= weight;
            factor = weight;
        }
        if (scan_to_map.information_scale != 1.0)
        {
            AtPA *= scan_to_map.information_scale;
            AtPB *= scan_to_map.information_scale;
            factor *= scan_to_map.information_scale;
        }
        return factor;
    };
    double motion_cost = 0.0; // the motion and IMU terms (only summed, never used in the solve)
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
        const double point_factor = apply_huber_and_scale(AtPA, AtPB, point_global, this_bucket.mean, infm);

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
        if (!normal_information.empty())
        {
            if (scan_to_map.degenerate_projection_all_point_normals)
            {
                const Eigen::Vector3d n = this_bucket.all_points_normal.cast<double>();
                if (n.squaredNorm() > 0.5)
                    normal_information[intermediate_points_i.index_pose] += n * n.transpose();
            }
            else if (this_bucket.normal_vector.squaredNorm() > 0.5)
                normal_information[intermediate_points_i.index_pose] += this_bucket.normal_vector * this_bucket.normal_vector.transpose();
        }
        if (normal_only_covariance)
            add_normal_direction_information(this_bucket, point_factor != 1.0 ? w * point_factor : w, intermediate_points_i.index_pose);
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
            const double point_factor = apply_huber_and_scale(AtPA, AtPB, point_global, this_bucket.mean, infm);

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
            if (normal_only_covariance)
                add_normal_direction_information(this_bucket, point_factor != 1.0 ? planarity * point_factor : planarity, intermediate_points_i.index_pose);
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
    Eigen::Matrix3d ndt_translation_information = Eigen::Matrix3d::Zero(); // sum of the nodes' NDT translation blocks, unprojected
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
            if (normal_only_covariance)
                ndt_translation_information += ndt_blocks[k].topLeftCorner<3, 3>();
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
                if (normal_only_covariance)
                    ndt_blocks[k] = projected; // the block as it stands in the normal matrix
            }
        }
    }
    else if (normal_only_covariance)
    {
        for (int k = 0; k < n_nodes; k++)
        {
            ndt_translation_information += ndt_blocks[k].topLeftCorner<3, 3>();
            AtPAndt.add_diagonal_block6(k * 6, ndt_blocks[k]);
            AtPBndt.block<6, 1>(k * 6, 0) += ndt_rhs[k];
        }
    }

    // geometric translation constraint of this window (world frame, summed over the nodes)
    if (lidar_translation_information)
    {
        lidar_translation_information->setZero();
        for (const auto &ni : normal_information)
            *lidar_translation_information += ni;
    }

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < window.poses.size(); i++)
        odo_edges.emplace_back(i - 1, i);

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < window.poses.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(window.poses[i]));

    for (size_t i = 0; i < window.motion_model_poses.size(); i++)
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(window.motion_model_poses[i]));

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
                                                       poses_desired[odo_edges[i].first].px,
                                                       poses_desired[odo_edges[i].first].py,
                                                       poses_desired[odo_edges[i].first].pz,
                                                       poses_desired[odo_edges[i].first].om,
                                                       poses_desired[odo_edges[i].first].fi,
                                                       poses_desired[odo_edges[i].first].ka,
                                                       poses_desired[odo_edges[i].second].px,
                                                       poses_desired[odo_edges[i].second].py,
                                                       poses_desired[odo_edges[i].second].pz,
                                                       poses_desired[odo_edges[i].second].om,
                                                       poses_desired[odo_edges[i].second].fi,
                                                       poses_desired[odo_edges[i].second].ka);

        relative_pose_measurement_odo(3, 0) += motion_model.correction.om * DEG_TO_RAD;
        relative_pose_measurement_odo(4, 0) += motion_model.correction.fi * DEG_TO_RAD;
        relative_pose_measurement_odo(5, 0) += motion_model.correction.ka * DEG_TO_RAD;

        double w_om_i = w_motion_model_om;
        double w_fi_i = w_motion_model_fi;
        double w_ka_i = w_motion_model_ka;
        double dt_i = 0.0;
        if (time_scaled_sigma || estimate_bias)
            dt_i = std::max(window.timestamps[odo_edges[i].second].first - window.timestamps[odo_edges[i].first].first, 0.0);

        if (time_scaled_sigma)
        {
            // gyro angle random walk over this edge, inflated by k, floored for near-zero dt
            const double sqrt_dt = std::sqrt(dt_i);
            const double s_om = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.x() * sqrt_dt, rotation_options.sigma_floor_rad);
            const double s_fi = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.y() * sqrt_dt, rotation_options.sigma_floor_rad);
            const double s_ka = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.z() * sqrt_dt, rotation_options.sigma_floor_rad);
            w_om_i = 1.0 / (s_om * s_om);
            w_fi_i = 1.0 / (s_fi * s_fi);
            w_ka_i = 1.0 / (s_ka * s_ka);
        }

        if (estimate_bias)
        {
            // the IMU increment contains bias * dt (body frame of node i, small angle); apply_gyro_bias_prior already
            // removed the carried bias (the prior mean) from motion_model_poses, so only the update is subtracted here
            const Eigen::Vector3d bias_update = window.gyro_bias.mean - rotation_options.bias_prior_mean;
            relative_pose_measurement_odo(3, 0) -= bias_update.x() * dt_i;
            relative_pose_measurement_odo(4, 0) -= bias_update.y() * dt_i;
            relative_pose_measurement_odo(5, 0) -= bias_update.z() * dt_i;
        }

        const double w_om_c = w_om_i;
        const double w_fi_c = w_fi_i;
        const double w_ka_c = w_ka_i;

        Eigen::Matrix<double, 12, 12>
            AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 w_motion_model_x,
                                                                 w_motion_model_y,
                                                                 w_motion_model_z,
                                                                 w_om_c,
                                                                 w_fi_c,
                                                                 w_ka_c);
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 relative_pose_measurement_odo(0, 0),
                                                                 relative_pose_measurement_odo(1, 0),
                                                                 relative_pose_measurement_odo(2, 0),
                                                                 relative_pose_measurement_odo(3, 0),
                                                                 relative_pose_measurement_odo(4, 0),
                                                                 relative_pose_measurement_odo(5, 0),
                                                                 w_motion_model_x,
                                                                 w_motion_model_y,
                                                                 w_motion_model_z,
                                                                 w_om_c,
                                                                 w_fi_c,
                                                                 w_ka_c);

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt.add(ic_1 + row, ic_1 + col, AtPAodo(row, col));
                AtPAndt.add(ic_1 + row, ic_2 + col, AtPAodo(row, col + 6));
                AtPAndt.add(ic_2 + row, ic_1 + col, AtPAodo(row + 6, col));
                AtPAndt.add(ic_2 + row, ic_2 + col, AtPAodo(row + 6, col + 6));
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }

        if (cost)
        {
            const TaitBryanPose &p1 = poses[odo_edges[i].first];
            const TaitBryanPose &p2 = poses[odo_edges[i].second];
            Eigen::Matrix<double, 6, 1> d;
            relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(d, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka,
                                                                  p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                                  relative_pose_measurement_odo(0, 0), relative_pose_measurement_odo(1, 0), relative_pose_measurement_odo(2, 0),
                                                                  relative_pose_measurement_odo(3, 0), relative_pose_measurement_odo(4, 0), relative_pose_measurement_odo(5, 0));
            motion_cost += w_motion_model_x * d(0) * d(0) + w_motion_model_y * d(1) * d(1) + w_motion_model_z * d(2) * d(2) +
                           w_om_c * d(3) * d(3) + w_fi_c * d(4) * d(4) + w_ka_c * d(5) * d(5);
        }

        if (estimate_bias)
        {
            // rotation rows of delta = (m - (b - b0)*dt) - f(pose_1, pose_2): d(delta)/d(pose) = J, d(delta)/d(b) = -dt * I
            Eigen::Matrix<double, 6, 12, Eigen::RowMajor> J;
            relative_pose_obs_eq_tait_bryan_wc_case1_jacobian_simplified_1(J,
                                                                           poses[odo_edges[i].first].px, poses[odo_edges[i].first].py, poses[odo_edges[i].first].pz,
                                                                           poses[odo_edges[i].first].om, poses[odo_edges[i].first].fi, poses[odo_edges[i].first].ka,
                                                                           poses[odo_edges[i].second].px, poses[odo_edges[i].second].py, poses[odo_edges[i].second].pz,
                                                                           poses[odo_edges[i].second].om, poses[odo_edges[i].second].fi, poses[odo_edges[i].second].ka);
            Eigen::Matrix<double, 6, 1> delta_odo;
            relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta_odo,
                                                                  poses[odo_edges[i].first].px, poses[odo_edges[i].first].py, poses[odo_edges[i].first].pz,
                                                                  poses[odo_edges[i].first].om, poses[odo_edges[i].first].fi, poses[odo_edges[i].first].ka,
                                                                  poses[odo_edges[i].second].px, poses[odo_edges[i].second].py, poses[odo_edges[i].second].pz,
                                                                  poses[odo_edges[i].second].om, poses[odo_edges[i].second].fi, poses[odo_edges[i].second].ka,
                                                                  relative_pose_measurement_odo(0, 0), relative_pose_measurement_odo(1, 0), relative_pose_measurement_odo(2, 0),
                                                                  relative_pose_measurement_odo(3, 0), relative_pose_measurement_odo(4, 0), relative_pose_measurement_odo(5, 0));

            const Eigen::Vector3d w_rot(w_om_c, w_fi_c, w_ka_c);
            for (int r = 0; r < 3; r++)
            {
                // pose/bias cross terms: J_rot^T * P_rot * (-dt)
                for (int col = 0; col < 6; col++)
                {
                    const double v1 = J(3 + r, col) * w_rot(r) * (-dt_i);
                    const double v2 = J(3 + r, col + 6) * w_rot(r) * (-dt_i);
                    AtPAndt.add(ic_1 + col, ib + r, v1);
                    AtPAndt.add(ib + r, ic_1 + col, v1);
                    AtPAndt.add(ic_2 + col, ib + r, v2);
                    AtPAndt.add(ib + r, ic_2 + col, v2);
                }
                AtPAndt.add(ib + r, ib + r, dt_i * dt_i * w_rot(r));
                AtPBndt(ib + r, 0) -= (-dt_i) * w_rot(r) * delta_odo(3 + r, 0);
            }
        }
    }

    if (estimate_bias)
    {
        // bias prior carried over from the previous window
        const Eigen::Matrix3d prior_inf = rotation_options.bias_prior_cov.inverse();
        const Eigen::Vector3d prior_delta = rotation_options.bias_prior_mean - window.gyro_bias.mean;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                AtPAndt.add(ib + r, ib + c, prior_inf(r, c));
        AtPBndt.block<3, 1>(ib, 0) += prior_inf * prior_delta;
        motion_cost += prior_delta.dot(prior_inf * prior_delta);
    }

    // velocity-continuity prior on world-frame translation (white-noise acceleration):
    // residual = change of velocity between consecutive steps [m/s], sigma = acc_sigma * dt
    if (motion_model.acc_sigma_mps2 > 0.0 && !acc_term && window.timestamps.size() == window.poses.size())
    {
        constexpr double min_dt = 1e-4;

        for (size_t i = 1; i + 1 < window.poses.size(); i++)
        {
            const double dt1 = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, min_dt);
            const double dt2 = std::max(window.timestamps[i + 1].first - window.timestamps[i].first, min_dt);
            const double sigma_dv = motion_model.acc_sigma_mps2 * 0.5 * (dt1 + dt2);
            const double w = 1.0 / (sigma_dv * sigma_dv);

            const Eigen::Vector3d r = (window.poses[i + 1].translation() - window.poses[i].translation()) / dt2 -
                                      (window.poses[i].translation() - window.poses[i - 1].translation()) / dt1;

            const int nodes[3] = {int(i) - 1, int(i), int(i) + 1};
            const double c[3] = {1.0 / dt1, -(1.0 / dt1 + 1.0 / dt2), 1.0 / dt2};
            motion_cost += w * r.squaredNorm();

            for (int k = 0; k < 3; k++)
            {
                for (int a = 0; a < 3; a++)
                {
                    for (int b = 0; b < 3; b++)
                        AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                    AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
                }
            }
        }

        // tie the first step of this window to the final velocity of the previous window
        if (velocity_prior.use && window.poses.size() > 1)
        {
            const double dt = std::max(window.timestamps[1].first - window.timestamps[0].first, min_dt);
            const double sigma_dv = motion_model.acc_sigma_mps2 * dt;
            const double w = 1.0 / (sigma_dv * sigma_dv);

            const Eigen::Vector3d r = (window.poses[1].translation() - window.poses[0].translation()) / dt - velocity_prior.velocity;

            const int nodes[2] = {0, 1};
            const double c[2] = {-1.0 / dt, 1.0 / dt};
            motion_cost += w * r.squaredNorm();

            for (int k = 0; k < 3; k++)
            {
                for (int a = 0; a < 2; a++)
                {
                    for (int b = 0; b < 2; b++)
                        AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                    AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
                }
            }
        }
    }

    // the accelerometer term with the node rotations, noise from the sensor and the start and end velocity as
    // unknowns. One term: residual r = sum(c_j * p_j) + c_v * v - sign * (mean acceleration - e) * tau [m/s], with mean
    // acceleration = time-weighted mean of (R_k f_k - g) * to_mps2 over the term's interval; the node rotations enter through
    // d(R f) = -[R f]x d(theta). Weight 1 / (sigma^2 tau^2) with sigma^2 = density^2 / tau + floor^2.
    if (acc_term)
    {
        const auto &acc = *translation_imu_options.acc;
        const int n = static_cast<int>(window.poses.size());
        const double s = translation_imu_options.to_mps2;
        std::vector<Eigen::Vector3d> rf(n), a(n);
        std::vector<Eigen::Matrix3d> e_rot(n);
        for (int k = 0; k < n; k++)
        {
            rf[k] = Eigen::Quaterniond(window.poses[k].linear()).normalized() * acc[k];
            a[k] = (rf[k] - translation_imu_options.gravity) * s;
            e_rot[k] = tait_bryan_rotation_jacobian(window.poses[k]);
        }
        const double density2 = translation_imu_options.noise_density_mps2_sqrt_hz * translation_imu_options.noise_density_mps2_sqrt_hz;
        const double floor2 = translation_imu_options.floor_sigma_mps2 * translation_imu_options.floor_sigma_mps2;
        const double max_dt = 2.0 * translation_imu_options.span_s; // a longer stretch has a gap in the IMU samples: its terms are dropped

        auto add_term = [&](const std::vector<std::pair<int, double>> &translation, int velocity_column, double c_v, double t0, double t1,
                            int first, int last, double sign, double tau)
        {
            // time weights of the nodes over [t0, t1)
            std::vector<std::pair<int, double>> weights;
            double weight = 0.0;
            for (int k = first; k <= last && k + 1 < n; k++)
            {
                const double s0 = std::max(window.timestamps[k].first, t0);
                const double s1 = std::min(window.timestamps[k + 1].first, t1);
                if (s1 > s0)
                {
                    weights.push_back({k, s1 - s0});
                    weight += s1 - s0;
                }
            }
            if (weight <= 0.0 || tau <= 0.0)
                return;
            Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
            for (auto &[k, wk] : weights)
            {
                wk /= weight;
                mean_acc += a[k] * wk;
            }
            Eigen::Vector3d r = -sign * (mean_acc - acc_offset) * tau;
            for (const auto &[node, c] : translation)
                r += c * window.poses[node].translation();
            if (velocity_column >= 0)
                r += c_v * (velocity_column == iv ? window.velocity.start : window.velocity.end);

            // Jacobian blocks (3x3 each, by parameter group)
            std::vector<std::pair<int, Eigen::Matrix3d>> blocks;
            for (const auto &[node, c] : translation)
                blocks.push_back({node * 6, Eigen::Matrix3d::Identity() * c});
            for (const auto &[k, wk] : weights)
                blocks.push_back({k * 6 + 3, sign * tau * wk * s * skew(rf[k]) * e_rot[k]});
            blocks.push_back({io, Eigen::Matrix3d::Identity() * (sign * tau)});
            if (velocity_column >= 0)
                blocks.push_back({velocity_column, Eigen::Matrix3d::Identity() * c_v});

            const double w = 1.0 / ((density2 / tau + floor2) * tau * tau);
            motion_cost += w * r.squaredNorm();
            for (const auto &[ca, ja] : blocks)
            {
                const Eigen::Vector3d g = w * ja.transpose() * r;
                for (int p = 0; p < 3; p++)
                    AtPBndt(ca + p, 0) -= g(p);
                for (const auto &[cb, jb] : blocks)
                {
                    const Eigen::Matrix3d h = w * ja.transpose() * jb;
                    for (int p = 0; p < 3; p++)
                        for (int q = 0; q < 3; q++)
                            AtPAndt.add(ca + p, cb + q, h(p, q));
                }
            }
        };

        for (size_t j = 1; j + 1 < anchors.size(); j++)
        {
            const int A = anchors[j - 1], B = anchors[j], C = anchors[j + 1];
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double dt1 = tB - tA, dt2 = tC - tB;
            if (dt1 > max_dt || dt2 > max_dt)
                continue;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC);
            add_term({{A, 1.0 / dt1}, {B, -(1.0 / dt1 + 1.0 / dt2)}, {C, 1.0 / dt2}}, -1, 0.0, m1, m2, A, C, 1.0, m2 - m1);
        }
        // start velocity: the mean velocity over [node 0, first anchor] is v_start + (mean acceleration over its first half) * dt / 2
        if (anchors.size() > 1)
        {
            const int B = anchors[1];
            const double t0 = window.timestamps[0].first, tB = window.timestamps[B].first, dt = tB - t0;
            if (dt <= max_dt)
                add_term({{0, -1.0 / dt}, {B, 1.0 / dt}}, iv, -1.0, t0, 0.5 * (t0 + tB), 0, B, 1.0, 0.5 * dt);
        }
        // end velocity: the mean velocity over the last two anchors is v_end - (mean acceleration from their midpoint to the last
        // node) * (time from the midpoint to the last node)
        bool end_tied = false;
        if (anchors.size() > 1)
        {
            const int P = anchors[anchors.size() - 2], Q = anchors.back(), N = n - 1;
            const double tP = window.timestamps[P].first, tQ = window.timestamps[Q].first, tN = window.timestamps[N].first, dt = tQ - tP;
            const double mid = 0.5 * (tP + tQ);
            if (dt <= max_dt && tN - tQ <= max_dt)
            {
                add_term({{P, -1.0 / dt}, {Q, 1.0 / dt}}, ie, -1.0, mid, tN, P, N, -1.0, tN - mid);
                end_tied = true;
            }
        }

        // start velocity prior: the previous window's end velocity
        const Eigen::Matrix3d velocity_inf = translation_imu_options.velocity_prior_cov.inverse();
        const Eigen::Vector3d velocity_rhs = velocity_inf * (translation_imu_options.velocity_prior_mean - window.velocity.start);
        motion_cost += (translation_imu_options.velocity_prior_mean - window.velocity.start).dot(velocity_rhs);
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
                AtPAndt.add(iv + r, iv + c, velocity_inf(r, c));
            AtPBndt(iv + r, 0) += velocity_rhs(r);
        }
        // at rest: both velocities are zero
        if (translation_imu_options.at_rest)
        {
            const double w = 1.0 / (translation_imu_options.zero_velocity_sigma_mps * translation_imu_options.zero_velocity_sigma_mps);
            for (int r = 0; r < 3; r++)
            {
                AtPAndt.add(iv + r, iv + r, w);
                AtPBndt(iv + r, 0) -= w * window.velocity.start(r);
                AtPAndt.add(ie + r, ie + r, w);
                AtPBndt(ie + r, 0) -= w * window.velocity.end(r);
            }
            motion_cost += w * (window.velocity.start.squaredNorm() + window.velocity.end.squaredNorm());
        }
        // the end velocity must be determined even without its term (a gap at the end): a weak prior at the start velocity
        if (!end_tied)
        {
            for (int r = 0; r < 3; r++)
            {
                AtPAndt.add(ie + r, ie + r, 1.0);
                AtPBndt(ie + r, 0) += window.velocity.start(r) - window.velocity.end(r);
            }
            motion_cost += (window.velocity.start - window.velocity.end).squaredNorm();
        }

        // offset prior carried over from the previous window
        const Eigen::Matrix3d prior_inf = translation_imu_options.offset_prior_cov.inverse();
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                AtPAndt.add(io + r, io + c, prior_inf(r, c));
        AtPBndt.block<3, 1>(io, 0) += prior_inf * (translation_imu_options.offset_prior_mean - acc_offset);
        motion_cost += (translation_imu_options.offset_prior_mean - acc_offset).dot(prior_inf * (translation_imu_options.offset_prior_mean - acc_offset));
    }

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
        // an off-map point costs as much as a matched point at that distance would: scaled like the matched terms
        const double unmatched = scan_to_map.unmatched_point_cost * cost->unmatched;
        cost->ndt += scan_to_map.information_scale != 1.0 ? scan_to_map.information_scale * unmatched : unmatched;
        cost->total = cost->ndt + motion_cost;
    }
    PhaseTimer assembly_timer(phase(&ScanToMapTiming::assembly));

    // fix the origin: weight 1 / sigma^2 on the first node's six parameters (its right-hand side is zero)
    const double fix_origin_weights[6] = {1.0 / (motion_model.fix_origin_x_sigma_m * motion_model.fix_origin_x_sigma_m),
                                          1.0 / (motion_model.fix_origin_y_sigma_m * motion_model.fix_origin_y_sigma_m),
                                          1.0 / (motion_model.fix_origin_z_sigma_m * motion_model.fix_origin_z_sigma_m),
                                          1.0 / (motion_model.fix_origin_om_sigma_deg * motion_model.fix_origin_om_sigma_deg),
                                          1.0 / (motion_model.fix_origin_fi_sigma_deg * motion_model.fix_origin_fi_sigma_deg),
                                          1.0 / (motion_model.fix_origin_ka_sigma_deg * motion_model.fix_origin_ka_sigma_deg)};
    for (int k = 0; k < 6; k++)
        AtPAndt.add(k, k, fix_origin_weights[k]);

    assert(AtPAndt.dropped_writes() == 0); // a write outside the band would mean the band was chosen too narrow
    Eigen::SparseMatrix<double> AtPA = AtPAndt.to_sparse();
    Eigen::SparseMatrix<double> AtPB = AtPBndt.sparseView();

    assembly_timer.stop();
    PhaseTimer covariance_timer(phase(&ScanToMapTiming::covariance_solve));

    const int n_extra = n_params - n_pose_params;
    if (n_extra > 0)
    {
        // marginal covariance of the extra unknowns (gyro bias, acceleration offset):
        // their block of the inverse of the undamped normal matrix
        if (normal_only_covariance)
        {
            // For this covariance the NDT terms count along each direction only by their share of normal-direction
            // information: share c_j = v_j^T R v_j / v_j^T T v_j along the eigenvectors v_j of R (the window's sum of
            // n n^T / (n^T C n)), T the sum of the NDT translation blocks. Each node's block B becomes Mc B Mc with
            // Mc = diag(sum sqrt(c_j) v_j v_j^T, I): the points' pull along a surface adds no confidence.
            Eigen::Matrix3d real = Eigen::Matrix3d::Zero();
            for (const auto &r : normal_direction_information)
                real += r;
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(real);
            Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 3; j++)
            {
                const Eigen::Vector3d v = eig.eigenvectors().col(j);
                const double total = v.dot(ndt_translation_information * v);
                const double share = total > 0.0 ? std::clamp(eig.eigenvalues()(j) / total, 0.0, 1.0) : 1.0;
                scale += std::sqrt(share) * v * v.transpose();
            }
            Eigen::Matrix<double, 6, 6> Mc = Eigen::Matrix<double, 6, 6>::Identity();
            Mc.topLeftCorner<3, 3>() = scale;
            std::vector<Eigen::Triplet<double>> difference;
            difference.reserve(static_cast<size_t>(n_nodes) * 36);
            for (int k = 0; k < n_nodes; k++)
            {
                const Eigen::Matrix<double, 6, 6> block = ndt_blocks[k];
                const Eigen::Matrix<double, 6, 6> d = Mc * block * Mc - block;
                for (int r = 0; r < 6; r++)
                    for (int c = 0; c < 6; c++)
                        difference.emplace_back(k * 6 + r, k * 6 + c, d(r, c));
            }
            Eigen::SparseMatrix<double> D(n_params, n_params);
            D.setFromTriplets(difference.begin(), difference.end());
            const Eigen::SparseMatrix<double> AtPA_covariance = AtPA + D;
            cache.covariance.compute(AtPA_covariance);
        }
        else
            cache.covariance.compute(AtPA);
        const PatternCachedCholesky::Solver &solver_cov = cache.covariance.solver();
        if (solver_cov.info() == Eigen::Success)
        {
            Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(n_params, n_extra);
            for (int r = 0; r < n_extra; r++)
                unit(n_pose_params + r, r) = 1.0;
            const Eigen::MatrixXd cols = solver_cov.solve(unit);
            if (solver_cov.info() == Eigen::Success)
            {
                if (estimate_bias)
                    window.gyro_bias.covariance = cols.block<3, 3>(ib, ib - n_pose_params);
                if (acc_term)
                {
                    acc_offset_cov = cols.block<3, 3>(io, io - n_pose_params);
                    window.velocity.end_covariance = cols.block<3, 3>(ie, ie - n_pose_params);
                }
            }
        }
    }

    covariance_timer.stop();
    if (!apply_step)
        return false;
    PhaseTimer solve_timer(phase(&ScanToMapTiming::solve));

    Eigen::SparseMatrix<double> AtPA_I(n_params, n_params);
    if (n_extra > 0)
    {
        // LM damping on the pose parameters only: lm_factor grows to ~1e10, which is negligible against the pose
        // curvature (~1e8+) but would freeze the bias (curvature ~1e3); the bias is kept well-posed by its prior
        std::vector<Eigen::Triplet<double>> damping;
        damping.reserve(n_pose_params);
        for (int k = 0; k < n_pose_params; k++)
            damping.emplace_back(k, k, 1.0);
        AtPA_I.setFromTriplets(damping.begin(), damping.end());
    }
    else
        AtPA_I.setIdentity();
    AtPA_I *= lm_factor;
    AtPA += (AtPA_I);

    cache.damped.compute(AtPA);
    Eigen::SparseMatrix<double> x = cache.damped.solver().solve(AtPB);
    const bool solved = cache.damped.solver().info() == Eigen::Success;
    const std::vector<double> h_x = solution_by_row(x, n_params);
    solve_timer.stop();
    PhaseTimer update_timer(phase(&ScanToMapTiming::update));

    delta = 1000000.0;
    // the nodes' new poses; a node that would end 1 m or more from its guess ("big jump") rejects the whole step
    SteppedPoses stepped;
    if (solved)
    {
        stepped = stepped_poses(window, h_x, 1.0, logger);
        cache.big_jump_nodes += stepped.big_jump_nodes;
    }
    if (solved && stepped.big_jump_nodes == 0)
    {
        if (acc_term)
            for (int r = 0; r < 3; r++)
                acc_offset(r) += h_x[io + r];
        if (acc_term)
            for (int r = 0; r < 3; r++)
            {
                window.velocity.start(r) += h_x[iv + r];
                window.velocity.end(r) += h_x[ie + r];
            }

        if (estimate_bias)
        {
            for (int r = 0; r < 3; r++)
            {
                window.gyro_bias.mean(r) += h_x[ib + r];
                if (rotation_options.bias_max_rad_s > 0.0)
                    window.gyro_bias.mean(r) = std::clamp(window.gyro_bias.mean(r), -rotation_options.bias_max_rad_s, rotation_options.bias_max_rad_s);
            }
        }

        for (size_t i = 0; i < window.poses.size(); i++)
            window.poses[i] = affine_matrix_from_pose_tait_bryan(stepped.poses[i]);
        delta = 0.0;
        for (int i = 0; i < h_x.size(); i++)
            delta += sqrt(h_x[i] * h_x[i]);
    }
    else if (!solved)
        cache.steps_skipped++;
    return solved && stepped.big_jump_nodes == 0;
}
