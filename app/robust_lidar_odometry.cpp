#include "robust_lidar_odometry.h"

#include "angle_units.h"
#include "normal_equations.h"
#include <hash_utils.h>
#include <mutex>

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
