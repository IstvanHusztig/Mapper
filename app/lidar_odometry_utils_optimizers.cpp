#include "lidar_odometry_utils.h"
#include "optimizer_options.h"
#include <hash_utils.h>
#include <mutex>
#include "normal_equations.h"
#include "imu_kinematics.h"

#include <export_laz.h>

#include "angle_units.h"

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &trajectory,
                  const std::vector<Eigen::Affine3d> &motion_model_trajectory,
                  const NDT::GridParameters &rgd_params, bool useMultithread, double wx,
                  double wy,
                  double wz,
                  double wom,
                  double wfi,
                  double wka)
{

    auto trj = trajectory;
    std::vector<Point3Di> point_cloud_global;
    std::vector<Eigen::Vector3d> point_cloud_global_sc;
    std::vector<int> indexes;

    for (int i = 0; i < intermediate_points.size(); i++)
    {
        Point3Di pg = intermediate_points[i];
        pg.point = trajectory[intermediate_points[i].index_pose] * pg.point;
        point_cloud_global.push_back(pg);
        double r_g = pg.point.norm();
        point_cloud_global_sc.emplace_back(r_g, atan2(pg.point.y(), pg.point.x()) * RAD_TO_DEG, acos(pg.point.z() / r_g) * RAD_TO_DEG);
        indexes.push_back(i);
    }

    NDTBucketMapType buckets;
    update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc);

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    const auto hessian_fun = [&](const int &indexes_i) -> Blocks
    {
        int c = intermediate_points[indexes_i].index_pose * 6;
        auto index_of_bucket = get_rgd_index(point_cloud_global_sc[indexes_i], b);
        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
        }
        auto &this_bucket = bucket_it->second;

        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 100000.0;

        if ((infm.array() > threshold).any())
            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};

        if ((infm.array() < -threshold).any())
            return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};

        const Eigen::Affine3d &m_pose = trajectory[intermediate_points[indexes_i].index_pose];
        const Eigen::Vector3d &p_s = intermediate_points[indexes_i].point;
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

        // planarity
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
        auto eigen_values = eigen_solver.eigenvalues();
        auto eigen_vectors = eigen_solver.eigenvectors();
        double ev1 = eigen_values.x();
        double ev2 = eigen_values.y();
        double ev3 = eigen_values.z();
        double sum_ev = ev1 + ev2 + ev3;
        auto planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

        double w = planarity;

        // Wisdom ahead (mpelka) :
        // The order of execution in summing matters, so using concurrent execution, we will get slightly different output.
        // We should compute every contibution to Hessian in parallel, then add those ouputs sequentially.
        return Blocks{AtPA * w, AtPB * w, c};
    };

    Eigen::MatrixXd AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(trajectory.size() * 6, 1);
    AtPBndt.setZero();

    std::vector<Blocks> AtPAndtBlocksToSum(intermediate_points.size());

    if (useMultithread)
        std::transform(std::execution::par_unseq, std::begin(indexes), std::end(indexes), std::begin(AtPAndtBlocksToSum), hessian_fun);
    else
        std::transform(std::begin(indexes), std::end(indexes), std::begin(AtPAndtBlocksToSum), hessian_fun);

    SumBlocks(AtPAndtBlocksToSum, AtPAndt, AtPBndt);

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < trajectory.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));

    for (size_t i = 0; i < motion_model_trajectory.size(); i++)
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(motion_model_trajectory[i]));

    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < trajectory.size(); i++)
        odo_edges.emplace_back(i - 1, i);

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 12, 12> AtPAodo;
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
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
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
                                                                 0, 0, 0, 0, 0, 0,
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

    tripletListP.emplace_back(ir, ir, 1000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += AtPAndt.sparseView();
    AtPB += AtPBndt.sparseView();

    Eigen::SparseMatrix<double> AtPA_I(trajectory.size() * 6, trajectory.size() * 6);
    AtPA_I.setIdentity();
    AtPA += AtPA_I;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
        solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());

    if (h_x.size() == 6 * trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            auto prev_pose = pose;
            pose.px += h_x[counter++] * 0.5;
            pose.py += h_x[counter++] * 0.5;
            pose.pz += h_x[counter++] * 0.5;
            pose.om += h_x[counter++] * 0.5;
            pose.fi += h_x[counter++] * 0.5;
            pose.ka += h_x[counter++] * 0.5;

            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }

        Eigen::Affine3d first = trj[0];
        std::vector<Eigen::Affine3d> out;
        out.push_back(first);

        for (int i = 1; i < trajectory.size(); i++)
        {
            auto update = trajectory[i - 1].inverse() * trajectory[i];

            first = first * update;
            out.push_back(first);
        }

        trajectory = out;
    }
}

void optimize_rigid_sf(
    const std::vector<Point3Di> &intermediate_points,
    std::vector<Eigen::Affine3d> &trajectory,
    std::vector<Eigen::Affine3d> &motion_model_trajectory,
    const NDTBucketMapType &buckets,
    double distance_bucket,
    double polar_angle_deg,
    double azimutal_angle_deg,
    int robust_and_accurate_lidar_odometry_sf_rigid_iterations,
    double max_distance_lidar_rigid_sf,
    bool useMultithread,
    double rgd_sf_sigma_x_m,
    double rgd_sf_sigma_y_m,
    double rgd_sf_sigma_z_m,
    double rgd_sf_sigma_om_deg,
    double rgd_sf_sigma_fi_deg,
    double rgd_sf_sigma_ka_deg)
{

    auto shift = trajectory[0].translation();

    auto _intermediate_trajectory = trajectory;
    auto _intermediate_trajectory_motion_model = motion_model_trajectory;
    auto _buckets = buckets;

    NDT::GridParameters rgd_params_sc;

    rgd_params_sc.resolution_X = distance_bucket;
    rgd_params_sc.resolution_Y = polar_angle_deg;
    rgd_params_sc.resolution_Z = azimutal_angle_deg;

    for (auto &t : _intermediate_trajectory)
        t.translation() -= shift;

    for (auto &t : _intermediate_trajectory_motion_model)
        t.translation() -= shift;

    for (auto &b : _buckets)
        b.second.mean -= shift;

    std::vector<Eigen::Vector3d> points_rgd;
    std::vector<Point3Di> point_cloud_global;
    std::vector<Eigen::Vector3d> point_cloud_global_sc;

    auto minv = _intermediate_trajectory[0].inverse();

    for (const auto &b : _buckets)
    {
        auto pinv = minv * b.second.mean;

        if (pinv.norm() < max_distance_lidar_rigid_sf)
            points_rgd.push_back(b.second.mean);
    }

    for (int i = 0; i < points_rgd.size(); i++)
    {
        double r_l = points_rgd[i].norm();
        double polar_angle_deg_l = atan2(points_rgd[i].y(), points_rgd[i].x()) * RAD_TO_DEG;
        double azimutal_angle_deg_l = acos(points_rgd[i].z() / r_l) * RAD_TO_DEG;

        point_cloud_global_sc.emplace_back(r_l, polar_angle_deg_l, azimutal_angle_deg_l);

        Point3Di p;
        p.point = points_rgd[i];
        point_cloud_global.push_back(p);
    }

    NDTBucketMapType buckets_sf;
    update_rgd_spherical_coordinates(rgd_params_sc, buckets_sf, point_cloud_global, point_cloud_global_sc);

    std::vector<Eigen::Vector3d> points_local;
    std::vector<int> indexes;

    auto first_pose_inverse = _intermediate_trajectory[0].inverse();
    auto first_pose = _intermediate_trajectory[0];
    for (int i = 0; i < intermediate_points.size(); i++)
    {
        Eigen::Vector3d p = _intermediate_trajectory[intermediate_points[i].index_pose] * intermediate_points[i].point;
        p = first_pose_inverse * p;
        points_local.push_back(p);
        indexes.push_back(i);
    }

    // ICP spherical coordinates

    for (int iter = 0; iter < robust_and_accurate_lidar_odometry_sf_rigid_iterations; iter++)
    {
        const auto hessian_fun = [&](const int &indexes_i) -> Blocks
        {
            int c = 0;
            const auto &point_local = points_local[indexes_i];
            auto point_global = first_pose * point_local;

            double r_l = point_global.norm();
            double polar_angle_deg_l = atan2(point_global.y(), point_global.x()) * RAD_TO_DEG;
            double azimutal_angle_deg_l = acos(point_global.z() / r_l) * RAD_TO_DEG;

            auto index_of_bucket = get_rgd_index({r_l, polar_angle_deg_l, azimutal_angle_deg_l}, {rgd_params_sc.resolution_X, rgd_params_sc.resolution_Y, rgd_params_sc.resolution_Z});

            auto bucket_it = buckets_sf.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets_sf.end())
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};

            auto &this_bucket = bucket_it->second;

            const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
            const double threshold = 100000.0;

            if ((infm.array() > threshold).any())
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};
            if ((infm.array() < -threshold).any())
                return {Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero(), Eigen::Matrix<double, 6, 1>::Zero(), c};

            const Eigen::Affine3d &m_pose = first_pose;
            const Eigen::Vector3d &p_s = points_local[indexes_i];
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

            // planarity
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
            auto eigen_values = eigen_solver.eigenvalues();
            auto eigen_vectors = eigen_solver.eigenvectors();
            double ev1 = eigen_values.x();
            double ev2 = eigen_values.y();
            double ev3 = eigen_values.z();
            double sum_ev = ev1 + ev2 + ev3;
            auto planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));

            double w = planarity;
            AtPA *= w;
            AtPB *= w;

            return {AtPA, AtPB, c};
        };

        std::vector<Blocks> blocks(indexes.size());

        if (useMultithread) // ToDo fix for this case
            std::transform(std::execution::par_unseq, std::begin(indexes), std::end(indexes), std::begin(blocks), hessian_fun);
        else
            std::transform(std::begin(indexes), std::end(indexes), std::begin(blocks), hessian_fun);

        Eigen::MatrixXd AtPAndt(6, 6);
        AtPAndt.setZero();
        Eigen::MatrixXd AtPBndt(6, 1);
        AtPBndt.setZero();

        SumBlocks(blocks, AtPAndt, AtPBndt);

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        int ic = 0;
        int ir = tripletListB.size();
        tripletListA.emplace_back(ir, ic * 6 + 0, 1);
        tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
        tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
        tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
        tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
        tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

        double wx = 1.0 / (rgd_sf_sigma_x_m * rgd_sf_sigma_x_m);
        double wy = 1.0 / (rgd_sf_sigma_y_m * rgd_sf_sigma_y_m);
        double wz = 1.0 / (rgd_sf_sigma_z_m * rgd_sf_sigma_z_m);

        double a_om = rgd_sf_sigma_om_deg * DEG_TO_RAD;
        double w_om = 1.0 / (a_om * a_om);

        double a_fi = rgd_sf_sigma_fi_deg * DEG_TO_RAD;
        double w_fi = 1.0 / (a_fi * a_fi);

        double a_ka = rgd_sf_sigma_ka_deg * DEG_TO_RAD;
        double w_ka = 1.0 / (a_ka * a_ka);

        tripletListP.emplace_back(ir, ir, wx);
        tripletListP.emplace_back(ir + 1, ir + 1, wy);
        tripletListP.emplace_back(ir + 2, ir + 2, wz);
        tripletListP.emplace_back(ir + 3, ir + 3, w_om);
        tripletListP.emplace_back(ir + 4, ir + 4, w_fi);
        tripletListP.emplace_back(ir + 5, ir + 5, w_ka);

        tripletListB.emplace_back(ir, 0, 0);
        tripletListB.emplace_back(ir + 1, 0, 0);
        tripletListB.emplace_back(ir + 2, 0, 0);
        tripletListB.emplace_back(ir + 3, 0, 0);
        tripletListB.emplace_back(ir + 4, 0, 0);
        tripletListB.emplace_back(ir + 5, 0, 0);

        Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(6, 6);
        Eigen::SparseMatrix<double> AtPB(6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        AtPA += AtPAndt.sparseView();
        AtPB += AtPBndt.sparseView();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);
        std::vector<double> h_x;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 6)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(first_pose);

            pose.px += h_x[0];
            pose.py += h_x[1];
            pose.pz += h_x[2];
            pose.om += h_x[3];
            pose.fi += h_x[4];
            pose.ka += h_x[5];

            first_pose = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    std::vector<Eigen::Affine3d> trj;
    trj.push_back(first_pose);

    for (int i = 1; i < _intermediate_trajectory.size(); i++)
    {
        auto update = _intermediate_trajectory[i - 1].inverse() * _intermediate_trajectory[i];
        first_pose = first_pose * update;
        trj.push_back(first_pose);
    }
    _intermediate_trajectory = trj;

    for (int i = 0; i < _intermediate_trajectory.size(); i++)
        _intermediate_trajectory[i].translation() += shift;

    trajectory = _intermediate_trajectory;
    motion_model_trajectory = _intermediate_trajectory;
}

void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                             NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options,
                             const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                             Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *lidar_translation_information)
{
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

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixXd AtPAndt(n_params, n_params);
    AtPAndt.setZero();
    Eigen::MatrixXd AtPBndt(n_params, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b_indoor(scan_to_map.grid_indoor.resolution_X, scan_to_map.grid_indoor.resolution_Y, scan_to_map.grid_indoor.resolution_Z);

    std::vector<std::mutex> mutexes(window.poses.size());
    // sum of n * n^T over the surface normals the points are matched to (per node): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells
    std::vector<Eigen::Matrix3d> normal_information;
    if (lidar_translation_information)
        normal_information.assign(window.poses.size(), Eigen::Matrix3d::Zero());

    const auto hessian_fun_indoor = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < scan_to_map.min_distance || intermediate_points_i.point.norm() > scan_to_map.max_distance)
            return;

        Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = get_rgd_index(point_global, b_indoor);

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

        std::mutex &m = mutexes[intermediate_points_i.index_pose];
        std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
        if (!normal_information.empty() && this_bucket.normal_vector.squaredNorm() > 0.5)
            normal_information[intermediate_points_i.index_pose] += this_bucket.normal_vector * this_bucket.normal_vector.transpose();
    };

    if (scan_to_map.multithread)
        std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_indoor);
    else
        std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_indoor);

    if (scan_to_map.use_hierarchical_rgd)
    {
        Eigen::Vector3d b_outdoor(scan_to_map.grid_outdoor.resolution_X, scan_to_map.grid_outdoor.resolution_Y, scan_to_map.grid_outdoor.resolution_Z);

        const auto hessian_fun_outdoor = [&](const Point3Di &intermediate_points_i)
        {
            if (intermediate_points_i.point.norm() < scan_to_map.min_distance_outdoor_grid ||
                intermediate_points_i.point.norm() > scan_to_map.max_distance)
                return;

            Eigen::Vector3d point_global = window.poses[intermediate_points_i.index_pose] * intermediate_points_i.point;
            auto index_of_bucket = get_rgd_index(point_global, b_outdoor);

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

            std::mutex &m = mutexes[intermediate_points_i.index_pose];
            std::unique_lock lck(m);
            AtPAndt.block<6, 6>(c, c) += AtPA;
            AtPBndt.block<6, 1>(c, 0) -= AtPB;
        };

        if (scan_to_map.multithread)
            std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_outdoor);
        else
            std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun_outdoor);
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
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
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
                    AtPAndt(ic_1 + col, ib + r) += v1;
                    AtPAndt(ib + r, ic_1 + col) += v1;
                    AtPAndt(ic_2 + col, ib + r) += v2;
                    AtPAndt(ib + r, ic_2 + col) += v2;
                }
                AtPAndt(ib + r, ib + r) += dt_i * dt_i * w_rot(r);
                AtPBndt(ib + r, 0) -= (-dt_i) * w_rot(r) * delta_odo(3 + r, 0);
            }
        }
    }

    if (estimate_bias)
    {
        // bias prior carried over from the previous window
        const Eigen::Matrix3d prior_inf = rotation_options.bias_prior_cov.inverse();
        const Eigen::Vector3d prior_delta = rotation_options.bias_prior_mean - window.gyro_bias.mean;
        AtPAndt.block<3, 3>(ib, ib) += prior_inf;
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
                        AtPAndt(nodes[a] * 6 + k, nodes[b] * 6 + k) += w * c[a] * c[b];
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
                        AtPAndt(nodes[a] * 6 + k, nodes[b] * 6 + k) += w * c[a] * c[b];
                    AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
                }
            }
        }
    }

    // accelerometer as a soft measurement of the change of world-frame velocity between node anchors ~span_s apart:
    // residual = sum(c_j * p_j) - (mean acceleration - e) * tau, with mean acceleration = mean(R f - g) * to_mps2
    // (the node rotations are taken as fixed here) and e the acceleration offset of this window
    int pose_band_offset = 17;
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
                        AtPAndt(ra, nodes[jb] * 6 + k) += w * c[ja] * c[jb];
                    AtPAndt(ra, io + k) += w * c[ja] * tau;
                    AtPAndt(io + k, ra) += w * c[ja] * tau;
                    AtPBndt(ra, 0) -= w * c[ja] * r(k);
                }
                AtPAndt(io + k, io + k) += w * tau * tau;
                AtPBndt(io + k, 0) -= w * tau * r(k);
            }
            pose_band_offset = std::max(pose_band_offset, (nodes.back() - nodes.front()) * 6 + 5);
        };

        std::vector<int> anchors{0};
        for (int k = 1; k < n; k++)
            if (window.timestamps[k].first - window.timestamps[anchors.back()].first >= translation_imu_options.span_s)
                anchors.push_back(k);

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
        AtPAndt.block<3, 3>(io, io) += prior_inf;
        AtPBndt.block<3, 1>(io, 0) += prior_inf * (translation_imu_options.offset_prior_mean - acc_offset);
    }

    int ic = 0;
    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, ic * 6 + 0, 1);
    tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
    tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
    tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
    tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
    tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

    tripletListP.emplace_back(ir, ir, 1.0 / (motion_model.fix_origin_x_sigma_m * motion_model.fix_origin_x_sigma_m));
    tripletListP.emplace_back(ir + 1, ir + 1, 1.0 / (motion_model.fix_origin_y_sigma_m * motion_model.fix_origin_y_sigma_m));
    tripletListP.emplace_back(ir + 2, ir + 2, 1.0 / (motion_model.fix_origin_z_sigma_m * motion_model.fix_origin_z_sigma_m));
    tripletListP.emplace_back(ir + 3, ir + 3, 1.0 / (motion_model.fix_origin_om_sigma_deg * motion_model.fix_origin_om_sigma_deg));
    tripletListP.emplace_back(ir + 4, ir + 4, 1.0 / (motion_model.fix_origin_fi_sigma_deg * motion_model.fix_origin_fi_sigma_deg));
    tripletListP.emplace_back(ir + 5, ir + 5, 1.0 / (motion_model.fix_origin_ka_sigma_deg * motion_model.fix_origin_ka_sigma_deg));

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), n_params);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(n_params, n_params);
    Eigen::SparseMatrix<double> AtPB(n_params, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += pose_band_to_sparse(AtPAndt, n_pose_params, pose_band_offset);
    AtPB += AtPBndt.sparseView();

    const int n_extra = n_params - n_pose_params;
    if (n_extra > 0)
    {
        // marginal covariance of the extra unknowns (gyro bias, acceleration offset):
        // their block of the inverse of the undamped normal matrix
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver_cov(AtPA);
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

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
        solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());

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

MotionModelRotationOptions get_motion_model_rotation_options(const OdometryConfig &params)
{
    MotionModelRotationOptions o;
    o.time_scaled_sigma = params.use_time_scaled_rotation_sigma;
    o.arw_rad_sqrt_s = Eigen::Vector3d(params.gyro_arw_x_deg_sqrt_s, params.gyro_arw_y_deg_sqrt_s, params.gyro_arw_z_deg_sqrt_s) * DEG_TO_RAD;
    o.inflation_k = params.rotation_sigma_inflation_k;
    o.sigma_floor_rad = params.rotation_sigma_floor_deg * DEG_TO_RAD;
    o.estimate_bias = params.use_gyro_bias_estimation;
    o.bias_max_rad_s = params.gyro_bias_max_deg_s * DEG_TO_RAD;
    return o;
}

bool compute_step_2(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress)
{
    if (windows.size() != 0)
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        double acc_distance = 0.0;
        std::vector<Point3Di> points_global;

        Eigen::Affine3d m_last = state.m_g;
        auto tmp = windows[0].poses;

        windows[0].poses[0] = m_last;
        for (int k = 1; k < tmp.size(); k++)
        {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            windows[0].poses[k] = m_last;
        }
        windows[0].motion_model_poses = windows[0].poses;

        auto pp = state.initial_points;
        for (int i = 0; i < pp.size(); i++)
            pp[i].point = state.m_g * pp[i].point;

        update_rgd(params.in_out_params_indoor, state.buckets_indoor, pp, state.m_g.translation());

        if (params.ablation_study_use_hierarchical_rgd){
            update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pp, state.m_g.translation());
        }

        MotionModelRotationOptions rotation_options = get_motion_model_rotation_options(params);
        const ScanToMapOptions scan_to_map = get_scan_to_map_options(params);
        const MotionModelOptions motion_model = get_motion_model_options(params);
        if (rotation_options.estimate_bias && params.use_robust_and_accurate_lidar_odometry)
        {
            log_info(logger) << "gyro bias estimation is not supported with use_robust_and_accurate_lidar_odometry, disabled for this run";
            rotation_options.estimate_bias = false;
        }
        const double bias_rw_rad = params.gyro_bias_rw_deg_s_sqrt_s * DEG_TO_RAD;
        const double bias_sigma0_rad = params.gyro_bias_initial_sigma_deg_s * DEG_TO_RAD;
        GyroBiasState carried_bias; // the previous window's estimate, the prior of the next one
        carried_bias.covariance = Eigen::Matrix3d::Identity() * (bias_sigma0_rad * bias_sigma0_rad);

        // IMU-aided translation prediction: raw accelerometer and AHRS orientation of every node are needed
        const bool have_acc = !windows.empty() && windows[0].imu_acc.size() == windows[0].poses.size() &&
                              windows[0].imu_angles.size() == windows[0].poses.size();
        const bool use_imu_pred = params.use_imu_translation_prediction && have_acc;
        const bool use_zupt = params.use_zero_velocity_detection && have_acc;
        if ((params.use_imu_translation_prediction || params.use_zero_velocity_detection) && !have_acc)
            log_info(logger) << "no accelerometer data per node: IMU translation prediction / zero-velocity detection disabled";
        // accelerometer reading of gravity [g]: mean magnitude over the first window (the recording starts at rest)
        double gravity_norm = 1.0;
        if (have_acc)
        {
            double sum = 0.0;
            for (const auto &a : windows[0].imu_acc)
                sum += a.norm();
            gravity_norm = sum / double(windows[0].imu_acc.size());
        }
        int stationary_windows = 0;
        int turn_windows = 0;

        // accelerometer term: gravity in the odometry frame [accelerometer units], taken from the first window
        // (the recording starts at rest); the acceleration offset e is folded into it after every window
        const bool use_acc_term = params.use_imu_acceleration_term && have_acc;
        Eigen::Vector3d acc_gravity = Eigen::Vector3d::Zero();
        if (use_acc_term)
        {
            for (size_t k = 0; k < windows[0].imu_acc.size(); k++)
                acc_gravity += rotation_of(windows[0].poses[k]) * windows[0].imu_acc[k];
            acc_gravity /= double(windows[0].imu_acc.size());
        }
        const double acc_offset_sigma0 = params.imu_acceleration_offset_initial_sigma_mps2;
        Eigen::Matrix3d carried_acc_offset_cov = Eigen::Matrix3d::Identity() * (acc_offset_sigma0 * acc_offset_sigma0);
        // gating to LiDAR-degenerate stretches: degeneracy of the previous window, time spent in the current stretch
        const bool acc_gated = use_acc_term && params.imu_acceleration_only_when_degenerate;
        bool previous_window_degenerate = false;
        double degenerate_time_s = 0.0;
        int acc_term_windows = 0;
        constexpr double gravity_calibration_time_constant_s = 2.0;

        for (int i = 0; i < windows.size(); i++)
        {
            std::vector<Point3Di> intermediate_points;
            point_store.load(PointSet::WindowIntermediate, i, intermediate_points);

            progress.checkpoint();

            // final world-frame velocity of the previous window (for the velocity-continuity prior)
            const bool use_acc_prior = params.lidar_odometry_motion_model_acc_1_sigma_mps2 > 0.0;
            bool use_prev_velocity = false;
            Eigen::Vector3d prev_velocity(0.0, 0.0, 0.0);
            const bool imu_translation = use_imu_pred || use_zupt;
            double prev_velocity_age_s = 0.0;
            if ((use_acc_prior || imu_translation || use_acc_term) && i > 0)
            {
                const auto &prev_tr = windows[i - 1].poses;
                const auto &prev_ts = windows[i - 1].timestamps;
                const int n = prev_tr.size();
                const int k = std::min(20, n - 1);
                if (k > 0 && prev_ts.size() == prev_tr.size())
                {
                    const double dt = prev_ts[n - 1].first - prev_ts[n - 1 - k].first;
                    if (dt > 0.0)
                    {
                        prev_velocity = (prev_tr[n - 1].translation() - prev_tr[n - 1 - k].translation()) / dt;
                        use_prev_velocity = true;
                        const auto &ts_i = windows[i].timestamps;
                        if (!ts_i.empty())
                            prev_velocity_age_s = std::max(ts_i[0].first - 0.5 * (prev_ts[n - 1].first + prev_ts[n - 1 - k].first), 0.0);
                    }
                }

                if (prev_velocity.norm() > params.max_prior_velocity_mps)
                {
                    log_info(logger) << "window " << i + 1 << ": previous window velocity " << prev_velocity.norm()
                                     << " m/s > max_prior_velocity_mps " << params.max_prior_velocity_mps << ": not used as prior";
                    prev_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
                    use_prev_velocity = false;
                }
            }

            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            bool stationary = false;
            if ((i > 1 || ((use_acc_prior || imu_translation || use_acc_term) && i > 0)) && params.use_motion_from_previous_step)
            {
                if (i > 1)
                {
                    mean_shift = windows[i - 1].poses[windows[i - 1].poses.size() - 1].translation() -
                                 windows[i - 2].poses[windows[i - 2].poses.size() - 1].translation();
                    mean_shift /= (windows[i - 1].poses.size());
                }

                if (mean_shift.norm() > 1.0)
                {
                    log_info(logger) << "!!!mean_shift.norm() > 1.0!!!";
                    mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
                }

                std::vector<Eigen::Affine3d> new_trajectory;
                Eigen::Affine3d current_node = windows[i - 1].poses[windows[i - 1].poses.size() - 1];
                new_trajectory.push_back(current_node);

                for (int tr = 1; tr < windows[i].poses.size(); tr++)
                {
                    auto update = windows[i].poses[tr - 1].inverse() * windows[i].poses[tr];
                    current_node = current_node * update;
                    new_trajectory.push_back(current_node);
                }

                const auto &ts = windows[i].timestamps;
                const auto &acc = windows[i].imu_acc;
                const bool imu_sizes_ok = imu_translation && ts.size() == new_trajectory.size() && acc.size() == new_trajectory.size() &&
                                          windows[i].imu_angles.size() == new_trajectory.size();
                std::vector<Eigen::Vector3d> a;
                if (imu_sizes_ok)
                {
                    a = world_acceleration(new_trajectory, ts, acc, windows[i].imu_angles, gravity_norm, 0.5 * params.imu_acc_smoothing_s);
                    // rest detection on a 50 ms average: the accelerometer noise of a 10 ms average already reaches ~0.3 m/s^2
                    // at rest, while walking accelerations (1-3 m/s^2 at ~2 Hz) survive the longer average
                    constexpr double zupt_smoothing_half_width_s = 0.025;
                    stationary = use_zupt && is_window_stationary(new_trajectory, ts,
                                                                  world_acceleration(new_trajectory, ts, acc, windows[i].imu_angles, gravity_norm, zupt_smoothing_half_width_s),
                                                                  params.zero_velocity_acc_threshold_mps2,
                                                                  params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
                }

                if (imu_sizes_ok && (use_imu_pred || stationary))
                {
                    // integrate the accelerometer from the previous window's final velocity;
                    // a window at rest (zero-velocity detection) is predicted without motion
                    Eigen::Vector3d v = stationary ? Eigen::Vector3d::Zero() : prev_velocity;
                    Eigen::Vector3d p = new_trajectory[0].translation();
                    for (size_t tr = 1; tr < new_trajectory.size(); tr++)
                    {
                        const double dt = std::max(ts[tr].first - ts[tr - 1].first, 0.0);
                        const Eigen::Vector3d a_k = (use_imu_pred && !stationary) ? a[tr - 1] : Eigen::Vector3d::Zero();
                        p += v * dt + 0.5 * a_k * dt * dt;
                        v += a_k * dt;
                        new_trajectory[tr].translation() = p;
                    }
                    if (stationary)
                    {
                        stationary_windows++;
                        if (debugMsg)
                            log_debug(logger) << "window " << i + 1 << " stationary (zero-velocity detection)\n";
                    }
                }
                else if ((use_acc_prior || use_acc_term) && ts.size() == new_trajectory.size())
                {
                    // timestamp-aware constant-velocity prediction from the previous window's final velocity
                    for (int tr = 0; tr < new_trajectory.size(); tr++)
                        new_trajectory[tr].translation() += prev_velocity * (ts[tr].first - ts[0].first);
                }
                else
                {
                    for (int tr = 0; tr < new_trajectory.size(); tr++)
                        new_trajectory[tr].translation() += mean_shift * tr;
                }

                windows[i].poses = new_trajectory;
                windows[i].motion_model_poses = new_trajectory;
            }

            // gyro bias: prior from the previous window, grown by the bias random walk over this window
            if (rotation_options.estimate_bias)
            {
                const auto &ts = windows[i].timestamps;
                const double window_duration = ts.size() > 1 ? ts.back().first - ts.front().first : 0.0;
                rotation_options.bias_prior_mean = carried_bias.mean;
                rotation_options.bias_prior_cov = carried_bias.covariance + Eigen::Matrix3d::Identity() * (bias_rw_rad * bias_rw_rad * window_duration);
                windows[i].gyro_bias.mean = carried_bias.mean;
                windows[i].gyro_bias.covariance = rotation_options.bias_prior_cov;

                // remove the carried bias from the IMU rotation increments of the initial guess (translations unchanged)
                remove_gyro_bias_from_rotations(windows[i].poses, ts, carried_bias.mean);
                windows[i].motion_model_poses = windows[i].poses;
            }

            // the first window is registered against the initial map, which is built from not yet optimized points
            // (motion during the first threshold_initial_points is not compensated): its misfit must not enter the bias,
            // so the bias is only estimated from the second window on
            MotionModelRotationOptions window_rotation_options = rotation_options;
            window_rotation_options.estimate_bias = rotation_options.estimate_bias && i > 0;

            // tight turns: trust the gyro more (rotation about the vertical from the IMU-based initial guess)
            if (params.rotation_sigma_inflation_k_turn > 0.0)
            {
                const auto &tr = windows[i].poses;
                const auto &ts = windows[i].timestamps;
                if (tr.size() > 1 && ts.size() == tr.size())
                {
                    double yaw = 0.0;
                    for (size_t k = 1; k < tr.size(); k++)
                    {
                        // world-frame rotation increment, its component about the vertical
                        const Eigen::AngleAxisd aa(Eigen::Matrix3d(tr[k].linear() * tr[k - 1].linear().transpose()));
                        yaw += aa.angle() * aa.axis().z();
                    }
                    const double duration = ts.back().first - ts.front().first;
                    if (duration > 0.0 && std::abs(yaw) / duration > params.turn_rate_threshold_deg_s * DEG_TO_RAD)
                    {
                        window_rotation_options.inflation_k = params.rotation_sigma_inflation_k_turn;
                        turn_windows++;
                    }
                }
            }

            // accelerometer term (from the second window on, like the bias): offset prior grown by its random walk
            MotionModelTranslationImuOptions translation_imu_options;
            Eigen::Matrix3d lidar_translation_information = Eigen::Matrix3d::Zero();
            Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
            Eigen::Matrix3d acc_offset_cov = carried_acc_offset_cov;
            const double window_duration_s = windows[i].timestamps.size() > 1
                                                 ? windows[i].timestamps.back().first - windows[i].timestamps.front().first
                                                 : 0.0;
            bool acc_term_active = use_acc_term && i > 0;
            if (acc_gated)
            {
                // a new degenerate stretch starts with the calibrated gravity and a fresh offset prior
                if (previous_window_degenerate && degenerate_time_s == 0.0)
                    carried_acc_offset_cov = Eigen::Matrix3d::Identity() * (acc_offset_sigma0 * acc_offset_sigma0);
                degenerate_time_s = previous_window_degenerate ? degenerate_time_s + window_duration_s : 0.0;
                acc_term_active = acc_term_active && previous_window_degenerate && degenerate_time_s <= params.imu_degenerate_max_duration_s;
            }
            double sigma_x_m = params.lidar_odometry_motion_model_x_1_sigma_m;
            double sigma_y_m = params.lidar_odometry_motion_model_y_1_sigma_m;
            double sigma_z_m = params.lidar_odometry_motion_model_z_1_sigma_m;
            if (acc_gated && acc_term_active)
                sigma_x_m = sigma_y_m = sigma_z_m = params.imu_degenerate_translation_sigma_m;
            MotionModelOptions window_motion_model = motion_model;
            window_motion_model.x_sigma_m = sigma_x_m;
            window_motion_model.y_sigma_m = sigma_y_m;
            window_motion_model.z_sigma_m = sigma_z_m;
            const VelocityPrior velocity_prior{use_prev_velocity, prev_velocity};

            if (acc_term_active && windows[i].imu_acc.size() == windows[i].poses.size())
            {
                const double window_duration = window_duration_s;
                const double rw = params.imu_acceleration_offset_rw_mps2_sqrt_s;
                translation_imu_options.enabled = true;
                translation_imu_options.acc = &windows[i].imu_acc;
                translation_imu_options.gravity = acc_gravity;
                translation_imu_options.to_mps2 = 9.80665 / acc_gravity.norm();
                translation_imu_options.sigma_mps2 = params.imu_acceleration_sigma_mps2;
                translation_imu_options.span_s = params.imu_acceleration_span_s;
                translation_imu_options.prev_velocity_age_s = prev_velocity_age_s;
                translation_imu_options.offset_prior_cov = carried_acc_offset_cov + Eigen::Matrix3d::Identity() * (rw * rw * window_duration);
                acc_offset_cov = translation_imu_options.offset_prior_cov;
            }

            std::chrono::time_point<std::chrono::system_clock> start1, end1;
            start1 = std::chrono::system_clock::now();

            auto tmp_worker_data = windows[i].poses;

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                auto tr = windows[i].poses;
                auto trmm = windows[i].motion_model_poses;

                auto firstm = tr[0];

                for (auto &t : tr)
                    t.translation() -= firstm.translation();

                for (auto &t : trmm)
                    t.translation() -= firstm.translation();

                NDT::GridParameters rgd_params_sc;

                rgd_params_sc.resolution_X = params.distance_bucket;
                rgd_params_sc.resolution_Y = params.polar_angle_deg;
                rgd_params_sc.resolution_Z = params.azimutal_angle_deg;

                std::vector<Point3Di> points_local_sf;
                std::vector<Point3Di> points_local;

                for (int ii = 0; ii < intermediate_points.size(); ii++)
                {
                    double r_l = intermediate_points[ii].point.norm();

                    if (r_l > 0.5 && intermediate_points[ii].index_pose != -1 && r_l < params.max_distance_lidar_rigid_sf)
                    {
                        double polar_angle_deg_l = atan2(intermediate_points[ii].point.y(), intermediate_points[ii].point.x()) * RAD_TO_DEG;
                        double azimutal_angle_deg_l = acos(intermediate_points[ii].point.z() / r_l) * RAD_TO_DEG;

                        points_local.push_back(intermediate_points[ii]);

                        Point3Di p_sl = intermediate_points[ii];
                        p_sl.point.x() = r_l;
                        p_sl.point.y() = polar_angle_deg_l;
                        p_sl.point.z() = azimutal_angle_deg_l;

                        points_local_sf.push_back(p_sl);
                    }
                }
                log_info(logger) << "optimize_sf2";

                double wx = 1000000;
                double wy = 1000000;
                double wz = 1000000;
                double angle_sigma_rad = 0.1 * DEG_TO_RAD;
                double wom = 1.0 / (angle_sigma_rad * angle_sigma_rad);
                double wfi = 1.0 / (angle_sigma_rad * angle_sigma_rad);
                double wka = 1.0 / (angle_sigma_rad * angle_sigma_rad);

                for (int iter = 0; iter < params.robust_and_accurate_lidar_odometry_iterations; iter++)
                    optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, params.useMultithread, wx, wy, wz, wom, wfi, wka);

                for (auto &t : tr)
                    t.translation() += firstm.translation();

                for (auto &t : trmm)
                    t.translation() += firstm.translation();

                windows[i].poses = tr;
                windows[i].motion_model_poses = tr;

                optimize_rigid_sf(intermediate_points,
                                  windows[i].poses,
                                  windows[i].motion_model_poses,
                                  state.buckets_indoor,
                                  params.distance_bucket_rigid_sf,
                                  params.polar_angle_deg_rigid_sf,
                                  params.azimutal_angle_deg_rigid_sf,
                                  params.robust_and_accurate_lidar_odometry_rigid_sf_iterations,
                                  params.max_distance_lidar_rigid_sf,
                                  params.useMultithread,
                                  params.rgd_sf_sigma_x_m,
                                  params.rgd_sf_sigma_y_m,
                                  params.rgd_sf_sigma_z_m,
                                  params.rgd_sf_sigma_om_deg,
                                  params.rgd_sf_sigma_fi_deg,
                                  params.rgd_sf_sigma_ka_deg);
            }

            windows[i].motion_model_poses = windows[i].poses;

            double delta = 100000.0;
            double lm_factor = params.lm_factor_initial;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            int iter_end = 0;

            for (int iter = 0; iter < params.nr_iter; iter++)
            {
                iter_end = iter;
                delta = 100000.0;
                optimize_lidar_odometry(intermediate_points, windows[i], state.buckets_indoor, state.buckets_outdoor,
                                        scan_to_map, window_motion_model, window_rotation_options, translation_imu_options,
                                        velocity_prior, lm_factor, delta, acc_offset, acc_offset_cov, logger,
                                        acc_gated ? &lidar_translation_information : nullptr);
                if (delta < params.convergence_delta_threshold)
                    break;

                if (iter % 10 == 0 && iter > 0)
                {
                    if (debugMsg)
                        log_debug(logger) << "\nlm_factor " << lm_factor << ", delta " << std::setprecision(10) << delta << "\n";

                    lm_factor *= 10.0;
                }

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;

                if (elapsed_seconds.count() > params.real_time_threshold_seconds)
                    break;
            }

            end1 = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsed_seconds1 = end1 - start1;

            {
                auto msg = log_info(logger);
                msg << "finished at iteration " << iter_end + 1 << "/" << params.nr_iter;
                msg << " optimizing window " << i + 1 << "/" << windows.size()
                    << " with acc_distance " << fixed << std::setprecision(2) << acc_distance << "[m] in "
                    << fixed << std::setprecision(2) << elapsed_seconds1.count()
                    << "[s], delta ";
                if (delta >= params.convergence_delta_threshold)
                    msg << std::setprecision(10) << delta << "!!!";
                else
                    msg << "< " << std::setprecision(1) << std::scientific << params.convergence_delta_threshold;
            }

            if (rotation_options.estimate_bias)
            {
                const Eigen::Vector3d b_deg = windows[i].gyro_bias.mean * RAD_TO_DEG;
                const Eigen::Vector3d s_deg = windows[i].gyro_bias.covariance.diagonal().cwiseMax(0.0).cwiseSqrt() * RAD_TO_DEG;
                log_info(logger) << "gyro bias [deg/s] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                 << b_deg.x() << " " << b_deg.y() << " " << b_deg.z() << " (1-sigma "
                                 << s_deg.x() << " " << s_deg.y() << " " << s_deg.z() << ")";
                if ((windows[i].gyro_bias.mean.cwiseAbs().array() >= rotation_options.bias_max_rad_s * (1.0 - 1e-9)).any())
                    log_info(logger) << "WARNING: gyro bias clamped to +-" << params.gyro_bias_max_deg_s << " deg/s in window " << i + 1 << "\n";

                carried_bias = windows[i].gyro_bias;
            }

            if (acc_gated)
            {
                // degeneracy of this window decides about the term in the next one
                const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
                const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
                previous_window_degenerate = ratio < params.imu_degenerate_information_ratio;
                const Eigen::Vector3d weak = eig.eigenvectors().col(0);
                log_info(logger) << "lidar translation information window " << i + 1 << ": ratio " << std::fixed << std::setprecision(4) << ratio
                                 << " weakest direction " << std::setprecision(2) << weak.x() << " " << weak.y() << " " << weak.z()
                                 << (translation_imu_options.enabled ? " | accelerometer term active" : "");

                // outside degenerate stretches: calibrate the gravity vector from the LiDAR-constrained trajectory
                Eigen::Vector3d observed;
                if (!translation_imu_options.enabled && !previous_window_degenerate &&
                    observed_acceleration_offset(windows[i].poses, windows[i].timestamps,
                                                 windows[i].imu_acc, acc_gravity, 9.80665 / acc_gravity.norm(), params.handoff_tail_nodes, observed))
                {
                    const double alpha = 1.0 - std::exp(-window_duration_s / gravity_calibration_time_constant_s);
                    acc_gravity += alpha * observed / (9.80665 / acc_gravity.norm());
                }
            }

            if (translation_imu_options.enabled)
            {
                acc_term_windows++;
                // fold the estimated offset into the gravity vector; its uncertainty carries over as the next prior
                acc_gravity += acc_offset / translation_imu_options.to_mps2;
                carried_acc_offset_cov = acc_offset_cov;
                {
                    const Eigen::Vector3d s = acc_offset_cov.diagonal().cwiseMax(0.0).cwiseSqrt();
                    log_info(logger) << "acceleration offset [m/s^2] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                     << acc_offset.x() << " " << acc_offset.y() << " " << acc_offset.z() << " (1-sigma "
                                     << s.x() << " " << s.y() << " " << s.z() << ")";
                }
            }

            // window hand-off: the last nodes carry almost no points (the last one none), so their pose is
            // re-propagated from the last well-constrained node with the IMU rotation and constant velocity;
            // the next window starts from the re-propagated last node
            {
                auto &tr = windows[i].poses;
                const auto &imu = windows[i].motion_model_poses;
                const auto &ts = windows[i].timestamps;
                const int n = static_cast<int>(tr.size());
                const int m = params.handoff_tail_nodes;
                const int a = n - 1 - m;          // last node kept from the optimization
                constexpr int velocity_nodes = 20; // same span as the prev_velocity estimate
                if (m > 0 && a >= velocity_nodes && ts.size() == tr.size() && imu.size() == tr.size())
                {
                    const double dt_v = ts[a].first - ts[a - velocity_nodes].first;
                    const Eigen::Vector3d v = dt_v > 0.0 ? Eigen::Vector3d((tr[a].translation() - tr[a - velocity_nodes].translation()) / dt_v)
                                                         : Eigen::Vector3d::Zero();
                    // the initial guess was corrected with the prior bias; apply the update estimated in this window
                    const Eigen::Vector3d db = rotation_options.estimate_bias ? Eigen::Vector3d(windows[i].gyro_bias.mean - rotation_options.bias_prior_mean)
                                                                              : Eigen::Vector3d::Zero();
                    Eigen::Quaterniond q = rotation_of(tr[a]);
                    for (int k = a + 1; k < n; k++)
                    {
                        const double dt = std::max(ts[k].first - ts[k - 1].first, 0.0);
                        q = (q * relative_rotation(imu[k - 1], imu[k]) * Eigen::Quaterniond(exp_so3(-db * dt))).normalized();
                        const Eigen::Vector3d t_new = tr[k - 1].translation() + v * dt;
                        tr[k].linear() = q.toRotationMatrix();
                        tr[k].translation() = t_new;
                    }
                }
            }

            progress.progress(i + 1, windows.size());

            // temp save
            if (i % 100 == 0)
            {
                std::vector<Eigen::Vector3d> global_pointcloud;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;
                points_to_vector(
                    intermediate_points, windows[i].poses,
                    0, nullptr, global_pointcloud, intensity, timestamps, false, params.save_index_pose);
                preview.window_cloud(i, global_pointcloud, intensity, timestamps);
            }
            auto acc_distance_tmp = acc_distance;
            acc_distance += ((windows[i].poses[0].inverse()) *
                             windows[i].poses[windows[i].poses.size() - 1])
                                .translation()
                                .norm();

            if (!(acc_distance == acc_distance)) // NaN check
            {
                windows[i].poses = tmp_worker_data;
                log_info(logger) << "CHALLENGING DATA OCCURED!!!";
                acc_distance = acc_distance_tmp;
                log_info(logger) << "please split data set into subsets";
                ts_failure = windows[i].timestamps[0].first;
                log_info(logger) << "calculations canceled for TIMESTAMP: " << (long long int)windows[i].timestamps[0].first;
                return false;
            }

            for (int j = i + 1; j < windows.size(); j++)
            {
                Eigen::Affine3d m_last = windows[j - 1].poses[windows[j - 1].poses.size() - 1];
                auto tmp = windows[j].poses;

                windows[j].poses[0] = m_last;

                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_last = m_last * m_update;
                    windows[j].poses[k] = m_last;
                }
            }

            for (int j = 0; j < intermediate_points.size(); j++)
            {
                Point3Di pp = intermediate_points[j];
                pp.point = windows[i].poses[intermediate_points[j].index_pose] * pp.point;
                points_global.push_back(pp);
            }

            if (acc_distance > params.sliding_window_trajectory_length_threshold)
            {
                if (state.reference_points.size() == 0)
                {
                    state.buckets_indoor.clear();
                    state.buckets_outdoor.clear();
                }

                std::vector<Point3Di> points_global_new;
                points_global_new.reserve(points_global.size() / 2 + 1);
                for (int k = points_global.size() / 2; k < points_global.size(); k++)
                    points_global_new.emplace_back(points_global[k]);

                acc_distance = 0;
                points_global = points_global_new;

                // decimate (decimate() returns the reduced cloud; its result used to be discarded)
                if (params.decimation > 0){
                    points_global = decimate(points_global, params.decimation, params.decimation, params.decimation);
                }

                update_rgd(params.in_out_params_indoor, state.buckets_indoor, points_global, windows[i].poses[0].translation());
                if (params.ablation_study_use_hierarchical_rgd)
                {
                    update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, points_global, windows[i].poses[0].translation());
                }
            }
            else
            {
                std::vector<Point3Di> pg;
                for (int j = 0; j < intermediate_points.size(); j++)
                {
                    Point3Di pp = intermediate_points[j];
                    pp.point = windows[i].poses[intermediate_points[j].index_pose] * pp.point;
                    pg.push_back(pp);
                }
                update_rgd(params.in_out_params_indoor, state.buckets_indoor, pg, windows[i].poses[0].translation());
                if (params.ablation_study_use_hierarchical_rgd)
                {
                    update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pg, windows[i].poses[0].translation());
                }
            }

            if (i > 1)
            {
                double translation = (windows[i - 1].poses[0].translation() -
                                      windows[i - 2].poses[0].translation())
                                         .norm();
                state.consecutive_distance += translation;
            }
        }

        for (int i = 0; i < windows.size(); i++)
            windows[i].motion_model_poses = windows[i].poses;

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

        std::tm local_tm = *std::localtime(&end_time);
        log_info(logger) << "finished computation at " << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                         << ", elapsed time: " << std::fixed << std::setprecision(2) << elapsed_seconds.count() << "s";

        state.total_length_of_calculated_trajectory = 0;
        for (int i = 1; i < windows.size(); i++)
            state.total_length_of_calculated_trajectory += (windows[i].poses[0].translation() - windows[i - 1].poses[0].translation()).norm();

        log_info(logger) << "total_length_of_calculated_trajectory: " << state.total_length_of_calculated_trajectory << " [m]";
        if (use_zupt)
            log_info(logger) << "zero-velocity detection: " << stationary_windows << " of " << windows.size() << " windows stationary";
        if (use_acc_term)
            log_info(logger) << "accelerometer term: active in " << acc_term_windows << " of " << windows.size() << " windows";
        if (params.rotation_sigma_inflation_k_turn > 0.0)
            log_info(logger) << "turn windows (gyro trusted more): " << turn_windows << " of " << windows.size();
    }

    return true;
}

void Consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = false;

    log_info(logger) << "preparing data START";
    for (int i = 0; i < windows.size(); i++)
    {
        for (int j = 0; j < windows[i].poses.size(); j++)
        {
            trajectory.push_back(windows[i].poses[j]);
            timestamps.push_back(windows[i].timestamps[j]);
            indexes.emplace_back(i, j);
        }
    }
    log_info(logger) << "preparing data FINISHED";

    for (int i = 0; i < windows.size(); i++)
    {
        std::vector<Point3Di> intermediate_points;
        if (!point_store.load(PointSet::WindowIntermediate, i, intermediate_points))
            log_info(logger) << "problem with point store: intermediate points of window " << i;

        for (int j = 0; j < intermediate_points.size(); j++)
            all_points_local.push_back(intermediate_points[j]);
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<std::mutex> my_mutex(1);

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));

    double angle = 0.1 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    // smoothness

    for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        const auto ir = tripletListB.size();

        const auto ic_1 = (i - 1) * 6;
        const auto ic_2 = i * 6;
        const auto ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, wangle);
        tripletListP.emplace_back(ir + 4, ir + 4, wangle);
        tripletListP.emplace_back(ir + 5, ir + 5, wangle);
    }

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

    tripletListP.emplace_back(ir, ir, 1000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    // ndt

    log_info(logger) << "NDT observations START";
    for (size_t index_point_begin = 0; index_point_begin < all_points_local.size(); index_point_begin += 1000000)
    {
        NDTBucketMapType buckets;
        std::vector<Point3Di> points_local;
        std::vector<Point3Di> points_global;

        for (int index = index_point_begin; index < index_point_begin + 1000000; index++)
        {
            if (index < all_points_local.size())
            {
                auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), all_points_local[index].timestamp,
                                              [](std::pair<double, double> lhs, double rhs) -> bool
                                              { return lhs.first < rhs; });
                int index_pose = std::distance(timestamps.begin(), lower) - 1;
                if (index_pose >= 0 && index_pose < trajectory.size())
                {
                    auto pl = all_points_local[index];
                    pl.index_pose = index_pose;

                    points_local.push_back(pl);
                    pl.point = trajectory[pl.index_pose] * pl.point;
                    points_global.push_back(pl);
                }
            }
        }

        if (points_global.size() > 10000)
        {
            update_rgd(params.in_out_params_indoor, buckets, points_global, trajectory[points_global[0].index_pose].translation());

            const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
            {
                if (intermediate_points_i.point.norm() < 1.0)
                    return;

                Eigen::Vector3d point_global = trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
                auto index_of_bucket = get_rgd_index(point_global, b);

                auto bucket_it = buckets.find(index_of_bucket);
                // no bucket found
                if (bucket_it == buckets.end())
                    return;

                auto &this_bucket = bucket_it->second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 100000.0;

                if ((infm.array() > threshold).any())
                    return;

                if ((infm.array() < -threshold).any())
                    return;

                const Eigen::Affine3d &m_pose = trajectory[intermediate_points_i.index_pose];
                const Eigen::Vector3d &p_s = intermediate_points_i.point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                double delta_x;
                double delta_y;
                double delta_z;

                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

                point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                              pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                              p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                       pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                       p_s.x(), p_s.y(), p_s.z());

                int c = intermediate_points_i.index_pose * 6;

                std::mutex &m = my_mutex[0];
                std::unique_lock lck(m);
                int ir = tripletListB.size();

                for (int row = 0; row < 3; row++)
                    for (int col = 0; col < 6; col++)
                        if (jacobian(row, col) != 0.0)
                            tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                tripletListP.emplace_back(ir, ir, infm(0, 0));
                tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
                tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
                tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
                tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
                tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
                tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
                tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
                tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));
            };

            if (multithread)
                std::for_each(std::execution::par_unseq, std::begin(points_local), std::end(points_local), hessian_fun);
            else
                std::for_each(std::begin(points_local), std::end(points_local), hessian_fun);

            Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
            Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

            matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

            Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
            Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

            {
                Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
                AtPAndt = (AtPndt)*matAndt;
                AtPBndt = (AtPndt)*matBndt;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();
            AtPA += AtPAndt;
            AtPB += AtPBndt;
        }
    }
    log_info(logger) << "NDT observations FINISHED";

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());

    if (h_x.size() == 6 * trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    for (int i = 0; i < indexes.size(); i++)
        windows[indexes[i].first].poses[indexes[i].second] = trajectory[i];
}

void Consistency2(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = true;

    log_info(logger) << "preparing data START";
    for (int i = 0; i < windows.size(); i++)
    {
        for (int j = 0; j < windows[i].poses.size(); j++)
        {
            trajectory.push_back(windows[i].poses[j]);
            timestamps.push_back(windows[i].timestamps[j]);
            indexes.emplace_back(i, j);
        }
    }
    log_info(logger) << "preparing data FINISHED";

    for (int i = 0; i < windows.size(); i++)
    {
        std::vector<Point3Di> intermediate_points;
        if (!point_store.load(PointSet::WindowIntermediate, i, intermediate_points))
            log_info(logger) << "problem with point store: intermediate points of window " << i;

        for (int j = 0; j < intermediate_points.size(); j++)
            all_points_local.push_back(intermediate_points[j]);
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<std::mutex> my_mutex(1);

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));

    double angle = 0.01 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    // smoothness
    for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 100000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, wangle);
        tripletListP.emplace_back(ir + 4, ir + 4, wangle);
        tripletListP.emplace_back(ir + 5, ir + 5, wangle);
    }

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

    tripletListP.emplace_back(ir, ir, 1000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    log_info(logger) << "reindex data START";

    std::vector<Point3Di> points_local;
    std::vector<Point3Di> points_global;

    points_local.reserve(all_points_local.size());
    points_global.reserve(all_points_local.size());

    for (int index = 0; index < all_points_local.size(); index++)
    {
        if (index < all_points_local.size())
        {
            auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), all_points_local[index].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });
            int index_pose = std::distance(timestamps.begin(), lower) - 1;
            if (index_pose >= 0 && index_pose < trajectory.size())
            {
                auto pl = all_points_local[index];
                pl.index_pose = index_pose;

                points_local.emplace_back(pl);
                pl.point = trajectory[pl.index_pose] * pl.point;
                points_global.emplace_back(pl);
            }
        }
    }

    int counter_index = 0;
    int counter = 0;
    for (int i = 0; i < points_global.size(); i++)
    {
        points_global[i].index_point = counter_index;

        if (counter > 1000000)
        {
            counter = 0;
            counter_index++;
        }
        counter++;
    }

    log_info(logger) << "reindex data FINISHED";

    log_info(logger) << "build regular grid decomposition START";

    std::vector<std::pair<unsigned long long int, unsigned int>> index_pairs;
    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);
        index_pairs.emplace_back(index_of_bucket, i);
    }
    std::sort(index_pairs.begin(), index_pairs.end(),
              [](const std::pair<unsigned long long int, unsigned int> &a, const std::pair<unsigned long long int, unsigned int> &b)
              { return a.first < b.first; });

    NDTBucketMapType2 buckets;

    for (unsigned int i = 0; i < index_pairs.size(); i++)
    {
        unsigned long long int index_of_bucket = index_pairs[i].first;
        if (buckets.contains(index_of_bucket))
            buckets[index_of_bucket].index_end_exclusive = i;
        else
        {
            buckets[index_of_bucket].index_begin_inclusive = i;
            buckets[index_of_bucket].index_end_exclusive = i;
        }
    }

    for (auto &b : buckets)
    {
        for (int i = b.second.index_begin_inclusive; i < b.second.index_end_exclusive; i++)
        {
            long long unsigned int index_point_segment = points_global[index_pairs[i].second].index_point;

            if (b.second.buckets.contains(index_point_segment))
                b.second.buckets[index_point_segment].point_indexes.push_back(index_pairs[i].second);
            else
            {
                NDT::BucketCoef bc;
                bc.point_indexes.push_back(index_pairs[i].second);
                b.second.buckets[index_point_segment] = bc;
            }
        }

        for (auto &bb : b.second.buckets)
        {
            if (bb.second.point_indexes.size() >= 5)
            {
                bb.second.valid = true;

                bb.second.mean = Eigen::Vector3d(0, 0, 0);
                for (const auto &p : bb.second.point_indexes)
                    bb.second.mean += points_global[p].point;

                bb.second.mean /= bb.second.point_indexes.size();

                bb.second.cov.setZero();
                for (const auto &p : bb.second.point_indexes)
                {
                    bb.second.cov(0, 0) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(0, 1) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(0, 2) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(1, 0) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(1, 1) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(1, 2) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(2, 0) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(2, 1) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(2, 2) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.z() - points_global[p].point.z());
                }
                bb.second.cov /= bb.second.point_indexes.size();

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(bb.second.cov, Eigen::ComputeEigenvectors);

                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                //  flip towards viewport
                const auto &m = trajectory[points_global[bb.second.point_indexes[0]].index_pose];

                if (nv.dot(m.translation() - bb.second.mean) < 0.0)
                    nv *= -1.0;

                bb.second.normal_vector = nv;

                bb.second.point_indexes.clear();
            }
            else
                bb.second.valid = false;
        }
    }
    log_info(logger) << "build regular grid decomposition FINISHED";

    log_info(logger) << "ndt observations start START";

    counter = 0;

    for (int i = 0; i < points_global.size(); i++)
    {
        if (i % 10000 == 0)
            log_info(logger) << "computing " << i << " of " << points_global.size();

        if (points_local[i].point.norm() < params.consistency_min_distance)
            continue;

        auto index_of_bucket = get_rgd_index(points_global[i].point, b);
        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
            continue;

        for (auto &bb : bucket_it->second.buckets)
        {
            if (bb.second.valid)
            {
                auto &this_bucket = bb.second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 100000.0;

                if ((infm.array() > threshold).any())
                    continue;

                if ((infm.array() < -threshold).any())
                    continue;

                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = trajectory[points_global[i].index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                    continue;

                const Eigen::Affine3d &m_pose = trajectory[points_global[i].index_pose];
                const Eigen::Vector3d &p_s = points_local[i].point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
                double delta_x;
                double delta_y;
                double delta_z;

                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

                point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                              pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                              p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

                point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                       pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                       p_s.x(), p_s.y(), p_s.z());

                int c = points_global[i].index_pose * 6;

                std::mutex &m = my_mutex[0];
                std::unique_lock lck(m);
                int ir = tripletListB.size();

                for (int row = 0; row < 3; row++)
                    for (int col = 0; col < 6; col++)
                        if (jacobian(row, col) != 0.0)
                            tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                tripletListP.emplace_back(ir, ir, infm(0, 0));
                tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
                tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
                tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
                tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
                tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
                tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
                tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
                tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));

                counter++;
                if (counter > 10000000)
                {
                    counter = 0;
                    Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
                    Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
                    Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

                    matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
                    matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
                    matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

                    Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
                    Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

                    {
                        Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
                        AtPAndt = (AtPndt)*matAndt;
                        AtPBndt = (AtPndt)*matBndt;
                    }

                    tripletListA.clear();
                    tripletListP.clear();
                    tripletListB.clear();

                    AtPA += AtPAndt;
                    AtPB += AtPBndt;
                }
            }
        }
    }

    if (counter > 0)
    {
        Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
        Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

        matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
        Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

        {
            Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
            AtPAndt = (AtPndt)*matAndt;
            AtPBndt = (AtPndt)*matBndt;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();
        AtPA += AtPAndt;
        AtPB += AtPBndt;
    }

    log_info(logger) << "ndt observations start FINISHED";

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());

    if (h_x.size() == 6 * trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    for (int i = 0; i < indexes.size(); i++)
        windows[indexes[i].first].poses[indexes[i].second] = trajectory[i];
}
