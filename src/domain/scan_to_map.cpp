#include "scan_to_map.h"
#include "ndt_map.h"
#include <execution>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <common/include/cauchy.h>

#include "angle_units.h"
#include "normal_equations.h"
#include <cassert>
#include <numeric>

void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                             NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options,
                             const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                             Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *lidar_translation_information, ScanToMapTiming *timing, ScanToMapWorkspace *workspace)
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
    // [pose_0 .. pose_n-1 | b_om b_fi b_ka (gyro bias) | e_x e_y e_z (acceleration offset)]
    const int n_params = n_pose_params + (estimate_bias ? 3 : 0) + (acc_term ? 3 : 0);
    const int ib = n_pose_params;                        // first bias column
    const int io = n_pose_params + (estimate_bias ? 3 : 0); // first acceleration offset column

    double sigma_motion_model_om = motion_model.om_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_fi = motion_model.fi_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_ka = motion_model.ka_sigma_deg * DEG_TO_RAD;

    double w_motion_model_om = 1.0 / (sigma_motion_model_om * sigma_motion_model_om);
    double w_motion_model_fi = 1.0 / (sigma_motion_model_fi * sigma_motion_model_fi);
    double w_motion_model_ka = 1.0 / (sigma_motion_model_ka * sigma_motion_model_ka);

    double w_motion_model_x = 1.0 / (motion_model.x_sigma_m * motion_model.x_sigma_m);
    double w_motion_model_y = 1.0 / (motion_model.y_sigma_m * motion_model.y_sigma_m);
    double w_motion_model_z = 1.0 / (motion_model.z_sigma_m * motion_model.z_sigma_m);

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
        if (velocity_prior.use && anchors.size() > 1)
            pose_band_offset = std::max(pose_band_offset, anchors[1] * 6 + 5);
    }

    // the normal matrix, stored by band (only the band is ever used)
    BandedNormalMatrix AtPAndt(n_pose_params, n_params - n_pose_params, pose_band_offset);
    Eigen::MatrixXd AtPBndt(n_params, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b_indoor(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z);

    // sum of n * n^T over the surface normals the points are matched to (per node): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells
    std::vector<Eigen::Matrix3d> normal_information;
    if (lidar_translation_information)
        normal_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());
    setup_timer.stop();

    const auto hessian_fun_indoor = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < scan_to_map.min_distance || intermediate_points_i.point.norm() > scan_to_map.max_distance)
            return;

        Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = ndt_cell_index(point_global, b_indoor);

        auto bucket_it = buckets_indoor.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets_indoor.end())
            return;

        auto &this_bucket = bucket_it->second;

        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 100000.0;

        if ((infm.array() > threshold).any())
            return;

        if ((infm.array() < -threshold).any())
            return;

        if (scan_to_map.use_view_point_and_normal_vectors)
        {
            // check nv
            Eigen::Vector3d &nv = this_bucket.normal_vector;
            Eigen::Vector3d viewport = window.poses[intermediate_points_i.index_pose].translation();
            if (nv.dot(viewport - this_bucket.mean) < 0)
                return;
        }

        const Eigen::Affine3d &m_pose = window.poses[intermediate_points_i.index_pose];
        const Eigen::Vector3d &p_s = intermediate_points_i.point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

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

        // only this point's node writes this block (see for_each_node), so no lock is needed
        AtPAndt.add_diagonal_block6(c, AtPA);
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
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
            for (int j = node_begin[k]; j < node_begin[k + 1]; j++)
                point_term(intermediate_points[points_by_node[j]]);
        };
        if (scan_to_map.multithread)
            std::for_each(std::execution::par, nodes.begin(), nodes.end(), node_terms);
        else
            std::for_each(nodes.begin(), nodes.end(), node_terms);
    };

    for_each_node(hessian_fun_indoor);
    indoor_timer.stop();

    if (scan_to_map.use_hierarchical_rgd)
    {
        Eigen::Vector3d b_outdoor(scan_to_map.grid_outdoor.resolution_X, scan_to_map.grid_outdoor.resolution_Y, scan_to_map.grid_outdoor.resolution_Z);

        const auto hessian_fun_outdoor = [&](const Point3Di &intermediate_points_i)
        {
            if (intermediate_points_i.point.norm() < scan_to_map.min_distance_outdoor_grid ||
                intermediate_points_i.point.norm() > scan_to_map.max_distance)
                return;

            Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
            auto index_of_bucket = ndt_cell_index(point_global, b_outdoor);

            auto bucket_it = buckets_outdoor.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets_outdoor.end())
                return;

            auto &this_bucket = bucket_it->second;

            const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
            const double threshold = 100000.0;

            if ((infm.array() > threshold).any())
                return;

            if ((infm.array() < -threshold).any())
                return;

            if (scan_to_map.use_view_point_and_normal_vectors)
            {
                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = window.poses[intermediate_points_i.index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                    return;
            }

            const Eigen::Affine3d &m_pose = window.poses[intermediate_points_i.index_pose];
            const Eigen::Vector3d &p_s = intermediate_points_i.point;
            const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

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

            AtPAndt.add_diagonal_block6(c, AtPA);
            AtPBndt.block<6, 1>(c, 0) -= AtPB;
        };

        PhaseTimer outdoor_timer(phase(&ScanToMapTiming::ndt_outdoor));
        for_each_node(hessian_fun_outdoor);
    }

    PhaseTimer motion_terms_timer(phase(&ScanToMapTiming::motion_terms));

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
            // the IMU increment contains bias * dt (body frame of node i, small angle)
            relative_pose_measurement_odo(3, 0) -= window.gyro_bias.mean.x() * dt_i;
            relative_pose_measurement_odo(4, 0) -= window.gyro_bias.mean.y() * dt_i;
            relative_pose_measurement_odo(5, 0) -= window.gyro_bias.mean.z() * dt_i;
        }

        const double w_om_c = w_om_i * cauchy(relative_pose_measurement_odo(3, 0), 1);
        const double w_fi_c = w_fi_i * cauchy(relative_pose_measurement_odo(4, 0), 1);
        const double w_ka_c = w_ka_i * cauchy(relative_pose_measurement_odo(5, 0), 1);

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

        if (estimate_bias)
        {
            // rotation rows of delta = (m - b*dt) - f(pose_1, pose_2): d(delta)/d(pose) = J, d(delta)/d(b) = -dt * I
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

    // accelerometer as a soft measurement of the change of world-frame velocity between node anchors ~span_s apart:
    // residual = sum(c_j * p_j) - (mean acceleration - e) * tau, with mean acceleration = mean(R f - g) * to_mps2
    // (the node rotations are taken as fixed here) and e the acceleration offset of this window
    if (acc_term)
    {
        const auto &acc = *translation_imu_options.acc;
        const int n = static_cast<int>(window.poses.size());
        std::vector<Eigen::Vector3d> a(n);
        for (int k = 0; k < n; k++)
            a[k] = (Eigen::Quaterniond(window.poses[k].linear()).normalized() * acc[k] - translation_imu_options.gravity) * translation_imu_options.to_mps2;

        // time-weighted mean acceleration over [t0, t1), nodes first..last searched
        auto mean_acceleration = [&](double t0, double t1, int first, int last)
        {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            double weight = 0.0;
            for (int k = first; k <= last && k + 1 < n; k++)
            {
                const double s0 = std::max(window.timestamps[k].first, t0);
                const double s1 = std::min(window.timestamps[k + 1].first, t1);
                if (s1 > s0)
                {
                    sum += a[k] * (s1 - s0);
                    weight += s1 - s0;
                }
            }
            return weight > 0.0 ? Eigen::Vector3d(sum / weight) : a[std::clamp(first, 0, n - 1)];
        };

        const double w_scale = 1.0 / (translation_imu_options.sigma_mps2 * translation_imu_options.sigma_mps2);
        auto add_term = [&](const std::vector<int> &nodes, const std::vector<double> &c, const Eigen::Vector3d &measured_dv, double tau)
        {
            const double w = w_scale / (tau * tau);
            Eigen::Vector3d r = -measured_dv;
            for (size_t j = 0; j < nodes.size(); j++)
                r += c[j] * window.poses[nodes[j]].translation();
            r += acc_offset * tau; // measured_dv = (mean acceleration - e) * tau
            for (int k = 0; k < 3; k++)
            {
                for (size_t ja = 0; ja < nodes.size(); ja++)
                {
                    const int ra = nodes[ja] * 6 + k;
                    for (size_t jb = 0; jb < nodes.size(); jb++)
                        AtPAndt.add(ra, nodes[jb] * 6 + k, w * c[ja] * c[jb]);
                    AtPAndt.add(ra, io + k, w * c[ja] * tau);
                    AtPAndt.add(io + k, ra, w * c[ja] * tau);
                    AtPBndt(ra, 0) -= w * c[ja] * r(k);
                }
                AtPAndt.add(io + k, io + k, w * tau * tau);
                AtPBndt(io + k, 0) -= w * tau * r(k);
            }
            assert((nodes.back() - nodes.front()) * 6 + 5 <= pose_band_offset); // the band computed at the top covers every term
        };

        for (size_t j = 1; j + 1 < anchors.size(); j++)
        {
            const int A = anchors[j - 1], B = anchors[j], C = anchors[j + 1];
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double dt1 = tB - tA, dt2 = tC - tB;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC);
            const double tau = m2 - m1;
            add_term({A, B, C}, {1.0 / dt1, -(1.0 / dt1 + 1.0 / dt2), 1.0 / dt2}, mean_acceleration(m1, m2, A, C) * tau, tau);
        }

        // tie the first velocity of this window to the final velocity of the previous window
        if (velocity_prior.use && anchors.size() > 1)
        {
            const int B = anchors[1];
            const double t0 = window.timestamps[0].first, tB = window.timestamps[B].first;
            const double dt = tB - t0;
            const double tau = 0.5 * dt + translation_imu_options.prev_velocity_age_s;
            // measured_dv relative to prev_velocity: move prev_velocity to the measured side
            const Eigen::Vector3d measured_dv = mean_acceleration(t0, 0.5 * (t0 + tB), 0, B) * tau + velocity_prior.velocity;
            add_term({0, B}, {-1.0 / dt, 1.0 / dt}, measured_dv, tau);
        }

        // offset prior carried over from the previous window
        const Eigen::Matrix3d prior_inf = translation_imu_options.offset_prior_cov.inverse();
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                AtPAndt.add(io + r, io + c, prior_inf(r, c));
        AtPBndt.block<3, 1>(io, 0) += prior_inf * (translation_imu_options.offset_prior_mean - acc_offset);
    }

    motion_terms_timer.stop();
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
                    acc_offset_cov = cols.block<3, 3>(io, io - n_pose_params);
            }
        }
    }

    covariance_timer.stop();
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
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());
    solve_timer.stop();
    PhaseTimer update_timer(phase(&ScanToMapTiming::update));

    delta = 1000000.0;
    if (h_x.size() == static_cast<size_t>(n_params))
    {
        int counter = 0;

        if (acc_term)
            for (int r = 0; r < 3; r++)
                acc_offset(r) += h_x[io + r];

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

            if ((p1 - p2).norm() < 1.0)
                window.poses[i] = affine_matrix_from_pose_tait_bryan(pose);
            else
                log_info(logger) << "big jump on trajectory: " << (p1 - p2).norm();
        }
        delta = 0.0;
        for (int i = 0; i < h_x.size(); i++)
            delta += sqrt(h_x[i] * h_x[i]);
    }
}
