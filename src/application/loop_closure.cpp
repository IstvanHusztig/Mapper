#include "loop_closure.h"
#include "ndt_map.h"
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include <algorithm>
#include <iomanip>
#include <cstdio>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace
{
    constexpr double DEG = M_PI / 180.0;

    struct Segment
    {
        int first_window = 0;
        int last_window = 0;                 // inclusive
        Eigen::Affine3d pose;                // world pose of the first node
        Eigen::Vector3d center;              // mean node position
        double t_begin = 0.0;
        double t_end = 0.0;
        std::vector<Eigen::Vector3d> points; // world coordinates, voxel-decimated
    };

    uint64_t voxel_key(const Eigen::Vector3d &p, double r)
    {
        const auto c = [r](double v) { return static_cast<uint64_t>(static_cast<int64_t>(std::floor(v / r)) + (1 << 20)) & 0x1FFFFF; };
        return (c(p.x()) << 42) | (c(p.y()) << 21) | c(p.z());
    }

    std::vector<Eigen::Vector3d> voxel_decimate(const std::vector<Eigen::Vector3d> &points, double r)
    {
        std::unordered_set<uint64_t> seen;
        std::vector<Eigen::Vector3d> out;
        for (const auto &p : points)
            if (seen.insert(voxel_key(p, r)).second)
                out.push_back(p);
        return out;
    }

    std::unordered_set<uint64_t> voxel_set(const std::vector<Eigen::Vector3d> &points, double r)
    {
        std::unordered_set<uint64_t> s;
        for (const auto &p : points)
            s.insert(voxel_key(p, r));
        return s;
    }

    // the points of `points` whose horizontal cell (with one cell of margin) also holds points of `other`: the footprint two
    // segments on different floors share, in practice the stairwell
    std::vector<Eigen::Vector3d> crop_to_shared_footprint(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &other,
                                                         double cell)
    {
        const auto key = [cell](double x, double y)
        {
            return (static_cast<uint64_t>(static_cast<uint32_t>(static_cast<int32_t>(std::floor(x / cell)))) << 32) |
                   static_cast<uint32_t>(static_cast<int32_t>(std::floor(y / cell)));
        };
        std::unordered_set<uint64_t> footprint;
        for (const auto &p : other)
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    footprint.insert(key(p.x() + dx * cell, p.y() + dy * cell));
        std::vector<Eigen::Vector3d> out;
        for (const auto &p : points)
            if (footprint.count(key(p.x(), p.y())))
                out.push_back(p);
        return out;
    }

    // how well a point set constrains every direction: smallest over largest eigenvalue of sum(n n^T) over the normals of
    // its planar cells (cells of `cell` m with at least 8 points, smallest spread under a fifth of the next)
    double information_ratio(const std::vector<Eigen::Vector3d> &points, double cell)
    {
        std::unordered_map<uint64_t, std::vector<Eigen::Vector3d>> cells;
        for (const auto &p : points)
            cells[voxel_key(p, cell)].push_back(p);
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        for (const auto &[key, ps] : cells)
        {
            if (ps.size() < 8)
                continue;
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto &p : ps)
                mean += p;
            mean /= double(ps.size());
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto &p : ps)
                cov += (p - mean) * (p - mean).transpose();
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
            if (eig.eigenvalues()(0) < 0.2 * eig.eigenvalues()(1))
            {
                const Eigen::Vector3d n = eig.eigenvectors().col(0);
                info += n * n.transpose();
            }
        }
        const Eigen::Vector3d ev = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(info).eigenvalues();
        return ev(2) > 0.0 ? ev(0) / ev(2) : 0.0;
    }

    double overlap(const std::vector<Eigen::Vector3d> &source, const Eigen::Affine3d &T, const std::unordered_set<uint64_t> &target, double r)
    {
        if (source.empty())
            return 0.0;
        int hits = 0;
        for (const auto &p : source)
            hits += target.count(voxel_key(T * p, r)) ? 1 : 0;
        return double(hits) / double(source.size());
    }

    // rotation about the world vertical through c, then translation
    Eigen::Affine3d yaw_about(const Eigen::Vector3d &c, double yaw, const Eigen::Vector3d &t)
    {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translate(c + t);
        T.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        T.translate(-c);
        return T;
    }

    // coarse registration: best overlap over yaw and translation on voxel grids (source moved onto target)
    Eigen::Affine3d coarse_search(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double range_m)
    {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto &p : source)
            c += p;
        c /= double(std::max<size_t>(source.size(), 1));

        // level 1: 0.5 m voxels, yaw +-10 deg, horizontal +-range, no vertical shift
        const double r1 = 0.5;
        const auto s1 = voxel_decimate(source, r1);
        const auto t1 = voxel_set(target, r1);
        double best = -1.0;
        double best_yaw = 0.0;
        Eigen::Vector3d best_t = Eigen::Vector3d::Zero();
        const int n_xy = static_cast<int>(std::ceil(range_m / r1));
        for (int iy = -4; iy <= 4; iy++)
        {
            const double yaw = iy * 2.5 * DEG;
            for (int ix = -n_xy; ix <= n_xy; ix++)
                for (int jy = -n_xy; jy <= n_xy; jy++)
                {
                    const Eigen::Vector3d t(ix * r1, jy * r1, 0.0);
                    const double o = overlap(s1, yaw_about(c, yaw, t), t1, r1);
                    if (o > best)
                    {
                        best = o;
                        best_yaw = yaw;
                        best_t = t;
                    }
                }
        }

        // level 2: 0.2 m voxels around the best candidate, including the vertical
        const double r2 = 0.2;
        const auto s2 = voxel_decimate(source, r2);
        const auto t2 = voxel_set(target, r2);
        best = -1.0;
        const double yaw0 = best_yaw;
        const Eigen::Vector3d tr0 = best_t;
        for (int iy = -3; iy <= 3; iy++)
            for (int ix = -3; ix <= 3; ix++)
                for (int jy = -3; jy <= 3; jy++)
                    for (int iz = -5; iz <= 5; iz++)
                    {
                        const double yaw = yaw0 + iy * 0.75 * DEG;
                        const Eigen::Vector3d t = tr0 + Eigen::Vector3d(ix * 0.1, jy * 0.1, iz * 0.2);
                        const double o = overlap(s2, yaw_about(c, yaw, t), t2, r2);
                        if (o > best)
                        {
                            best = o;
                            best_yaw = yaw;
                            best_t = t;
                        }
                    }
        return yaw_about(c, best_yaw, best_t);
    }

    // fine registration: rigid point-to-distribution alignment of the source to NDT cells of the target
    // information: the undamped normal matrix of the last iteration (at the finest resolution), over T's Tait-Bryan parameters
    Eigen::Affine3d ndt_refine(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, Eigen::Affine3d T,
                               bool exact_ndt_statistics, Eigen::Matrix<double, 6, 6> *information = nullptr)
    {
        std::vector<Point3Di> target_points(target.size());
        Eigen::Vector3d viewport = Eigen::Vector3d::Zero();
        for (size_t k = 0; k < target.size(); k++)
        {
            target_points[k].point = target[k];
            target_points[k].index_pose = 0;
            viewport += target[k];
        }
        viewport /= double(std::max<size_t>(target.size(), 1));

        for (double resolution : {0.5, 0.25})
        {
            NdtGridParameters rgd_params;
            rgd_params.resolution_X = rgd_params.resolution_Y = rgd_params.resolution_Z = resolution;
            NDTBucketMapType buckets;
            update_rgd(rgd_params, buckets, target_points, viewport, exact_ndt_statistics);
            const Eigen::Vector3d b(resolution, resolution, resolution);

            for (int iter = 0; iter < 15; iter++)
            {
                Eigen::Matrix<double, 6, 6> AtPA = Eigen::Matrix<double, 6, 6>::Zero();
                Eigen::Matrix<double, 6, 1> AtPB = Eigen::Matrix<double, 6, 1>::Zero();
                const TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(T);
                int used = 0;
                for (const auto &p : source)
                {
                    const auto it = buckets.find(ndt_cell_index(T * p, b));
                    if (it == buckets.end() || it->second.number_of_points < 3)
                        continue;
                    const Eigen::Matrix3d infm = it->second.cov.inverse();
                    if (!infm.allFinite() || (infm.array().abs() > 100000.0).any())
                        continue;
                    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> a;
                    point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                        a, pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka, p.x(), p.y(), p.z(),
                        infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));
                    Eigen::Matrix<double, 6, 1> v;
                    point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                        v, pose.px, pose.py, pose.pz, pose.om, pose.fi, pose.ka, p.x(), p.y(), p.z(),
                        infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                        it->second.mean.x(), it->second.mean.y(), it->second.mean.z());
                    AtPA += a;
                    AtPB -= v;
                    used++;
                }
                if (used < 100)
                    break;
                if (information)
                    *information = AtPA;
                AtPA += Eigen::Matrix<double, 6, 6>::Identity() * (1e-3 * AtPA.trace() / 6.0);
                const Eigen::Matrix<double, 6, 1> dx = AtPA.ldlt().solve(AtPB);
                if (!dx.allFinite())
                    break;
                TaitBryanPose updated = pose;
                updated.px += dx(0);
                updated.py += dx(1);
                updated.pz += dx(2);
                updated.om += dx(3);
                updated.fi += dx(4);
                updated.ka += dx(5);
                T = affine_matrix_from_pose_tait_bryan(updated);
                if (dx.head<3>().norm() < 1e-4 && dx.tail<3>().norm() < 1e-5)
                    break;
            }
        }
        return T;
    }

    void add_relative_pose_edge(std::vector<Eigen::Triplet<double>> &A, Eigen::VectorXd &b, const std::vector<TaitBryanPose> &poses,
                                int from, int to, const Eigen::Matrix<double, 6, 1> &measurement_in, double w_t, double w_r)
    {
        const TaitBryanPose &p1 = poses[from];
        const TaitBryanPose &p2 = poses[to];
        // the angle residuals wrapped into (-pi, pi] as in relative_pose_edge_cost: the measurement's angles are shifted by
        // whole turns, so an edge whose relative yaw is near +-180 deg doesn't see a residual of 360 deg after a small step
        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                              measurement_in(0), measurement_in(1), measurement_in(2),
                                                              measurement_in(3), measurement_in(4), measurement_in(5));
        Eigen::Matrix<double, 6, 1> measurement = measurement_in;
        for (int r = 3; r < 6; r++)
            measurement(r) -= delta(r) - std::remainder(delta(r), 2.0 * M_PI);
        Eigen::Matrix<double, 12, 12> AtPA;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPA, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka,
                                                                 p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka, w_t, w_t, w_t, w_r, w_r, w_r);
        Eigen::Matrix<double, 12, 1> AtPB;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPB, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka,
                                                                 p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                                 measurement(0), measurement(1), measurement(2),
                                                                 measurement(3), measurement(4), measurement(5), w_t, w_t, w_t, w_r, w_r, w_r);
        const int c[2] = {from * 6, to * 6};
        for (int r = 0; r < 12; r++)
        {
            for (int col = 0; col < 12; col++)
                A.emplace_back(c[r / 6] + r % 6, c[col / 6] + col % 6, AtPA(r, col));
            b(c[r / 6] + r % 6) -= AtPB(r);
        }
    }

    // weighted squared error of one relative-pose edge
    double relative_pose_edge_cost(const std::vector<TaitBryanPose> &poses, int from, int to, const Eigen::Matrix<double, 6, 1> &measurement,
                                   double w_t, double w_r)
    {
        const TaitBryanPose &p1 = poses[from];
        const TaitBryanPose &p2 = poses[to];
        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                              measurement(0), measurement(1), measurement(2), measurement(3), measurement(4), measurement(5));
        // the angle residuals wrapped into (-pi, pi]: a pose at yaw +-180 deg must not count as 360 deg off
        for (int r = 3; r < 6; r++)
            delta(r) = std::remainder(delta(r), 2.0 * M_PI);
        return w_t * delta.head<3>().squaredNorm() + w_r * delta.tail<3>().squaredNorm();
    }

    // an information edge's residual (measurement minus the relative pose of the two poses), angles wrapped into (-pi, pi]
    Eigen::Matrix<double, 6, 1> information_edge_residual(const TaitBryanPose &p1, const TaitBryanPose &p2, const Eigen::Matrix<double, 6, 1> &m)
    {
        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                              m(0), m(1), m(2), m(3), m(4), m(5));
        for (int r = 3; r < 6; r++)
            delta(r) = std::remainder(delta(r), 2.0 * M_PI);
        return delta;
    }

    // its Gauss-Newton terms: A += J^T omega J, b -= J^T omega delta (the generator's convention for AtPA / AtPB)
    void add_information_edge(std::vector<Eigen::Triplet<double>> &A, Eigen::VectorXd &b, const std::vector<TaitBryanPose> &poses, int from, int to,
                              const Eigen::Matrix<double, 6, 1> &measurement, const Eigen::Matrix<double, 6, 6> &omega)
    {
        const TaitBryanPose &p1 = poses[from];
        const TaitBryanPose &p2 = poses[to];
        const Eigen::Matrix<double, 6, 1> delta = information_edge_residual(p1, p2, measurement);
        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> J;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(J, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka);
        const Eigen::Matrix<double, 12, 12> H = J.transpose() * omega * J;
        const Eigen::Matrix<double, 12, 1> g = J.transpose() * omega * delta;
        const int c[2] = {from * 6, to * 6};
        for (int r = 0; r < 12; r++)
        {
            for (int col = 0; col < 12; col++)
                A.emplace_back(c[r / 6] + r % 6, c[col / 6] + col % 6, H(r, col));
            b(c[r / 6] + r % 6) -= g(r);
        }
    }

    // the Tait-Bryan parameters of a pose, as a vector
    Eigen::Matrix<double, 6, 1> pose_vector(const Eigen::Affine3d &T)
    {
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(T);
        Eigen::Matrix<double, 6, 1> v;
        v << p.px, p.py, p.pz, p.om, p.fi, p.ka;
        return v;
    }

    Eigen::Affine3d pose_from_vector(const Eigen::Matrix<double, 6, 1> &v)
    {
        TaitBryanPose p;
        p.px = v(0);
        p.py = v(1);
        p.pz = v(2);
        p.om = v(3);
        p.fi = v(4);
        p.ka = v(5);
        return affine_matrix_from_pose_tait_bryan(p);
    }

    // Jacobian (central differences) of a 6-vector function of a 6-parameter perturbation; angle outputs wrapped
    template <typename F>
    Eigen::Matrix<double, 6, 6> numeric_jacobian(const F &f)
    {
        constexpr double h = 1e-6;
        Eigen::Matrix<double, 6, 6> G;
        for (int k = 0; k < 6; k++)
        {
            Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Zero();
            e(k) = h;
            Eigen::Matrix<double, 6, 1> d = f(e) - f(-e);
            for (int r = 3; r < 6; r++)
                d(r) = std::remainder(d(r), 2.0 * M_PI);
            G.col(k) = d / (2.0 * h);
        }
        return G;
    }

    Eigen::Matrix<double, 6, 1> relative_measurement(const Eigen::Affine3d &a, const Eigen::Affine3d &b);

    // the edge's measurement as a function of a perturbation of the correction's Tait-Bryan parameters
    Eigen::Matrix<double, 6, 6> measurement_jacobian(const Eigen::Affine3d &pose_i, const Eigen::Affine3d &pose_j, const Eigen::Affine3d &correction)
    {
        const Eigen::Matrix<double, 6, 1> c = pose_vector(correction);
        return numeric_jacobian([&](const Eigen::Matrix<double, 6, 1> &e) { return relative_measurement(pose_i, pose_from_vector(c + e) * pose_j); });
    }

    // a symmetric matrix with its negative eigenvalues (rounding) set to zero
    Eigen::Matrix<double, 6, 6> clip_negative(const Eigen::Matrix<double, 6, 6> &M)
    {
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(0.5 * (M + M.transpose()));
        return eig.eigenvectors() * eig.eigenvalues().cwiseMax(0.0).asDiagonal() * eig.eigenvectors().transpose();
    }

    Eigen::Matrix<double, 6, 1> relative_measurement(const Eigen::Affine3d &a, const Eigen::Affine3d &b)
    {
        const TaitBryanPose p1 = pose_tait_bryan_from_affine_matrix(a);
        const TaitBryanPose p2 = pose_tait_bryan_from_affine_matrix(b);
        Eigen::Matrix<double, 6, 1> m;
        relative_pose_tait_bryan_wc_case1_simplified_1(m, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka);
        return m;
    }

    Eigen::Affine3d interpolate(const Eigen::Affine3d &a, const Eigen::Affine3d &b, double f)
    {
        const Eigen::Quaterniond qa(a.linear());
        const Eigen::Quaterniond qb(b.linear());
        Eigen::Affine3d r = Eigen::Affine3d::Identity();
        r.linear() = qa.normalized().slerp(f, qb.normalized()).toRotationMatrix();
        r.translation() = (1.0 - f) * a.translation() + f * b.translation();
        return r;
    }
} // namespace

Eigen::Matrix<double, 6, 6> loop_edge_information(const Eigen::Affine3d &pose_i, const Eigen::Affine3d &pose_j, const Eigen::Affine3d &correction,
                                                  const Eigen::Matrix<double, 6, 6> &H, bool free_height, double sigma_m, bool &ok)
{
    ok = false;
    const Eigen::Matrix<double, 6, 6> G = measurement_jacobian(pose_i, pose_j, correction);
    const Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu(G);
    if (!G.allFinite() || !H.allFinite() || lu.rcond() < 1e-9)
        return Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix<double, 6, 6> Gi = lu.inverse();
    Eigen::Matrix<double, 6, 6> omega = clip_negative(Gi.transpose() * H * Gi);
    if (free_height)
    {
        // the world-vertical shift of the source (the correction's pz) as a free nuisance parameter: Schur complement
        const Eigen::Matrix<double, 6, 1> u = G.col(2);
        const double uu = u.dot(omega * u);
        if (uu > 1e-12 * omega.trace())
        {
            const Eigen::Matrix<double, 6, 1> w = omega * u;
            omega = clip_negative(omega - w * w.transpose() / uu);
        }
    }
    const double best = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(omega.topLeftCorner<3, 3>()).eigenvalues()(2);
    if (!(best > 0.0) || !omega.allFinite())
        return Eigen::Matrix<double, 6, 6>::Zero();
    ok = true;
    return omega * (1.0 / (sigma_m * sigma_m) / best);
}

Eigen::Matrix<double, 6, 1> loop_edge_region_sigmas(const Eigen::Affine3d &pose_i, const Eigen::Affine3d &pose_j, const LoopClosureEdge &edge,
                                                    const Eigen::Vector3d &centre)
{
    // the correction perturbed by a move of the centre and a rotation about it (world axes)
    const auto measurement = [&](const Eigen::Matrix<double, 6, 1> &e)
    {
        const Eigen::Vector3d r = e.tail<3>();
        Eigen::Affine3d D = Eigen::Affine3d::Identity();
        D.translate(centre + e.head<3>());
        if (r.norm() > 0.0)
            D.rotate(Eigen::AngleAxisd(r.norm(), r.normalized()));
        D.translate(-centre);
        return relative_measurement(pose_i, D * edge.correction * pose_j);
    };
    const Eigen::Matrix<double, 6, 6> K = numeric_jacobian(measurement);
    const Eigen::Matrix<double, 6, 6> info = K.transpose() * edge.omega * K;
    // a tiny regularization keeps the free directions finite (and very large)
    const Eigen::Matrix<double, 6, 6> cov = (info + Eigen::Matrix<double, 6, 6>::Identity() * (1e-12 * std::max(info.trace(), 1e-30))).inverse();
    Eigen::Matrix<double, 6, 1> s;
    for (int k = 0; k < 6; k++)
        s(k) = std::sqrt(std::max(cov(k, k), 0.0)) / (k < 3 ? 1.0 : DEG);
    return s;
}

std::vector<Eigen::Affine3d> optimize_loop_pose_graph(const std::vector<Eigen::Affine3d> &initial, const std::vector<LoopClosureEdge> &loops,
                                                      const OdometryConfig &params)
{
    const int n = static_cast<int>(initial.size());
    std::vector<Eigen::Matrix<double, 6, 1>> odo(n > 0 ? n - 1 : 0);
    for (int k = 0; k + 1 < n; k++)
        odo[k] = relative_measurement(initial[k], initial[k + 1]);
    std::vector<Eigen::Matrix<double, 6, 1>> loop_meas;
    for (const auto &e : loops)
        loop_meas.push_back(relative_measurement(initial[e.i], e.correction * initial[e.j]));

    const double w_odo_t = 1.0 / std::pow(params.loop_closure_odometry_sigma_m, 2);
    const double w_odo_r = 1.0 / std::pow(params.loop_closure_odometry_sigma_deg * DEG, 2);
    const double w_loop_t = 1.0 / std::pow(params.loop_closure_edge_sigma_m, 2);
    const double w_loop_r = 1.0 / std::pow(params.loop_closure_edge_sigma_deg * DEG, 2);

    std::vector<TaitBryanPose> poses;
    for (const auto &p : initial)
        poses.push_back(pose_tait_bryan_from_affine_matrix(p));

    // robust kernel on the loop edges: s2 = the edge's squared residual in sigmas. Two stages: plain least squares first,
    // then the kernel from that solution. Reweighting from the odometry's poses would treat every true loop as an outlier:
    // before the solve each one is off by the whole drift (on 44 1-2 m, 20-40 sigmas)
    const double kernel_scale2 = params.loop_closure_kernel_scale * params.loop_closure_kernel_scale;
    double kernel2 = 0.0; // the kernel of the current stage (0: plain)
    // the edge's squared residual in sigmas
    const auto loop_edge_s2 = [&](const std::vector<TaitBryanPose> &ps, size_t e)
    {
        if (loops[e].information)
        {
            const Eigen::Matrix<double, 6, 1> d = information_edge_residual(ps[loops[e].i], ps[loops[e].j], loop_meas[e]);
            return d.dot(loops[e].omega * d);
        }
        return relative_pose_edge_cost(ps, loops[e].i, loops[e].j, loop_meas[e], w_loop_t, w_loop_r);
    };
    const auto loop_edge_cost = [&](const std::vector<TaitBryanPose> &ps, size_t e)
    {
        const double s2 = loop_edge_s2(ps, e);
        return kernel2 > 0.0 ? kernel2 * std::log1p(s2 / kernel2) : s2;
    };
    const auto loop_edge_weight = [&](const std::vector<TaitBryanPose> &ps, size_t e)
    {
        if (kernel2 <= 0.0)
            return 1.0;
        const double s2 = loop_edge_s2(ps, e);
        return 1.0 / (1.0 + s2 / kernel2);
    };

    const auto cost_of = [&](const std::vector<TaitBryanPose> &ps)
    {
        double c = 0.0;
        for (int k = 0; k + 1 < n; k++)
            c += relative_pose_edge_cost(ps, k, k + 1, odo[k], w_odo_t, w_odo_r);
        for (size_t e = 0; e < loops.size(); e++)
            c += loop_edge_cost(ps, e);
        return c;
    };

    const auto solve = [&]()
    {
        double cost = cost_of(poses);
        double lambda = 0.0; // 0: plain Gauss-Newton; raised only when a step fails to lower the cost
        int iter = 0;
        for (int attempt = 0; attempt < 60 && iter < 30; attempt++)
        {
            std::vector<Eigen::Triplet<double>> A;
            Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * n);
            for (int k = 0; k + 1 < n; k++)
                add_relative_pose_edge(A, b, poses, k, k + 1, odo[k], w_odo_t, w_odo_r);
            for (size_t e = 0; e < loops.size(); e++)
            {
                // the kernel's weight at the current poses (1 without the kernel): the reweighted quadratic bounds the robust
                // cost from above, so a step that lowers it lowers the cost, and the step check below stays valid
                const double f = loop_edge_weight(poses, e);
                if (loops[e].information)
                    add_information_edge(A, b, poses, loops[e].i, loops[e].j, loop_meas[e], loops[e].omega * f);
                else
                    add_relative_pose_edge(A, b, poses, loops[e].i, loops[e].j, loop_meas[e], w_loop_t * f, w_loop_r * f);
            }
            for (int r = 0; r < 6; r++)
                A.emplace_back(r, r, 1e12); // first segment pose fixed
            Eigen::SparseMatrix<double> AtPA(6 * n, 6 * n);
            AtPA.setFromTriplets(A.begin(), A.end());
            if (lambda > 0.0)
                for (int r = 0; r < 6 * n; r++)
                    AtPA.coeffRef(r, r) *= 1.0 + lambda;
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
            if (solver.info() != Eigen::Success)
                break;
            const Eigen::VectorXd dx = solver.solve(b);
            if (solver.info() != Eigen::Success || !dx.allFinite())
                break;
            std::vector<TaitBryanPose> trial = poses;
            for (int k = 0; k < n; k++)
            {
                trial[k].px += dx(6 * k + 0);
                trial[k].py += dx(6 * k + 1);
                trial[k].pz += dx(6 * k + 2);
                trial[k].om += dx(6 * k + 3);
                trial[k].fi += dx(6 * k + 4);
                trial[k].ka += dx(6 * k + 5);
            }
            const double trial_cost = cost_of(trial);
            // a step that doesn't raise the cost beyond rounding is kept; otherwise damp and try again from the same poses
            if (std::isfinite(trial_cost) && trial_cost <= cost * (1.0 + 1e-9) + 1e-12)
            {
                poses = trial;
                cost = trial_cost;
                lambda = lambda > 0.0 ? lambda * 0.1 : 0.0;
                if (lambda < 1e-9)
                    lambda = 0.0;
                iter++;
                if (dx.norm() < 1e-8)
                    break;
            }
            else
            {
                lambda = lambda > 0.0 ? lambda * 10.0 : 1e-3;
                if (lambda > 1e6)
                    break;
            }
        }
    };
    solve();
    if (kernel_scale2 > 0.0)
    {
        kernel2 = kernel_scale2;
        solve();
    }
    std::vector<Eigen::Affine3d> result;
    for (const auto &p : poses)
        result.push_back(affine_matrix_from_pose_tait_bryan(p));
    return result;
}

std::vector<LoopEdgeResidual> loop_edge_residuals(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                                                  const std::vector<LoopClosureEdge> &loops)
{
    std::vector<LoopEdgeResidual> out;
    for (const auto &e : loops)
    {
        const Eigen::Affine3d measured = initial[e.i].inverse() * e.correction * initial[e.j];
        const Eigen::Affine3d solved = optimized[e.i].inverse() * optimized[e.j];
        const Eigen::Affine3d d = measured.inverse() * solved;
        LoopEdgeResidual r{d.translation().norm(), Eigen::AngleAxisd(d.linear()).angle() / DEG};
        if (e.information)
        {
            const Eigen::Matrix<double, 6, 1> delta =
                information_edge_residual(pose_tait_bryan_from_affine_matrix(optimized[e.i]), pose_tait_bryan_from_affine_matrix(optimized[e.j]),
                                          relative_measurement(initial[e.i], e.correction * initial[e.j]));
            r.weighted = std::sqrt(std::max(delta.dot(e.omega * delta), 0.0));
        }
        out.push_back(r);
    }
    return out;
}

double loop_edge_drop_ratio(const LoopClosureEdge &edge, const LoopEdgeResidual &residual, const OdometryConfig &params)
{
    if (edge.information)
        return residual.weighted * params.loop_closure_edge_sigma_m / params.loop_closure_max_residual_m;
    return std::max(residual.translation_m / params.loop_closure_max_residual_m,
                    residual.angle_deg / params.loop_closure_max_residual_deg);
}

bool loop_pose_graph_plausible(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                               const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params, double &max_shift_m)
{
    // movements of the segment anchors (positions), not translations of world-frame transforms, which would also contain
    // the rotation about the world origin
    max_shift_m = 0.0;
    double largest_edge = 0.0;
    for (const auto &e : loops)
        if (e.j >= 0 && static_cast<size_t>(e.j) < initial.size())
            largest_edge = std::max(largest_edge, (e.correction * initial[e.j].translation() - initial[e.j].translation()).norm());
    bool finite = optimized.size() == initial.size();
    for (size_t k = 0; finite && k < optimized.size(); k++)
    {
        finite = optimized[k].matrix().allFinite();
        if (finite)
            max_shift_m = std::max(max_shift_m, (optimized[k].translation() - initial[k].translation()).norm());
    }
    return finite && max_shift_m <= params.loop_closure_max_correction_m + largest_edge;
}

int run_loop_closure(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    const int windows_per_segment = std::max(1, params.loop_closure_segment_windows);
    const int n_windows = static_cast<int>(windows.size());
    if (n_windows < 2 * windows_per_segment)
        return 0;
    log_info(logger) << "loop closure START";

    // 1. segments with their world points
    std::vector<Segment> segments;
    for (int w0 = 0; w0 < n_windows; w0 += windows_per_segment)
    {
        Segment s;
        s.first_window = w0;
        s.last_window = std::min(n_windows, w0 + windows_per_segment) - 1;
        s.pose = windows[w0].poses.front();
        s.t_begin = windows[w0].timestamps.front().first;
        s.t_end = windows[s.last_window].timestamps.back().first;
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
        std::vector<Eigen::Vector3d> points;
        for (int w = s.first_window; w <= s.last_window; w++)
        {
            const auto &tr = windows[w].poses;
            for (const auto &node : tr)
            {
                sum += node.translation();
                count++;
            }
            std::vector<Point3Di> window_points;
            if (!point_store.load(PointSet::WindowIntermediate, w, window_points))
                continue;
            for (const auto &p : window_points)
            {
                const double r = p.point.norm();
                if (r < params.min_distance_lidar || r > params.max_distance_lidar || p.index_pose < 0 || p.index_pose >= static_cast<int>(tr.size()))
                    continue;
                points.push_back(tr[p.index_pose] * p.point);
            }
        }
        s.center = count > 0 ? Eigen::Vector3d(sum / count) : s.pose.translation();
        s.points = voxel_decimate(points, 0.1);
        segments.push_back(std::move(s));
    }

    // 2.+3. revisit candidates and their registration
    std::vector<LoopClosureEdge> loops;
    const bool use_cross_floor = params.loop_closure_cross_floor_max_vertical_m > params.loop_closure_max_vertical_offset_m;
    // diagnostics: segment ranges whose cross-floor pairs are not tried (loop_closure_cross_floor_exclude_segments)
    std::vector<std::pair<int, int>> excluded_ranges;
    {
        std::stringstream list(params.loop_closure_cross_floor_exclude_segments);
        std::string item;
        while (std::getline(list, item, ','))
        {
            int a = 0, b = 0;
            if (std::sscanf(item.c_str(), "%d-%d", &a, &b) == 2)
                excluded_ranges.emplace_back(a, b);
            else if (std::sscanf(item.c_str(), "%d", &a) == 1)
                excluded_ranges.emplace_back(a, a);
        }
    }
    double near_x = 0.0, near_y = 0.0, near_r = -1.0; // loop_closure_cross_floor_only_near, r < 0: off
    std::sscanf(params.loop_closure_cross_floor_only_near.c_str(), "%lf %lf %lf", &near_x, &near_y, &near_r);
    const auto excluded = [&](int segment)
    {
        for (const auto &[a, b] : excluded_ranges)
            if (segment >= a && segment <= b)
                return true;
        return false;
    };
    int cross_floor_tries = 0;
    int cross_floor_accepted = 0;
    for (int j = 0; j < static_cast<int>(segments.size()); j++)
    {
        // earlier segments within the search radius, nearest first; when a registration is rejected the next one is tried
        // (the nearest centre is not always the same view: a short last segment can pick a wrong partner).
        // Pass 0: the same floor. Pass 1 (cross-floor loops): the floors above and below, as one more edge
        for (int pass = 0; pass < (use_cross_floor ? 2 : 1); pass++)
        {
            const bool cross_floor = pass == 1;
            std::vector<std::pair<double, int>> candidates;
            for (int i = 0; i < j; i++)
            {
                const double min_gap = cross_floor ? params.loop_closure_cross_floor_min_time_gap_s : params.loop_closure_min_time_gap_s;
                if (segments[j].t_begin - segments[i].t_end < min_gap)
                    continue;
                const Eigen::Vector3d d = segments[j].center - segments[i].center;
                if (cross_floor && (excluded(i) || excluded(j)))
                    continue;
                const double dz = std::abs(d.z());
                const bool height_fits = cross_floor ? dz > params.loop_closure_max_vertical_offset_m && dz <= params.loop_closure_cross_floor_max_vertical_m
                                                     : dz <= params.loop_closure_max_vertical_offset_m;
                if (!height_fits)
                    continue;
                const double dh = d.head<2>().norm();
                if (dh < params.loop_closure_search_radius_m)
                    candidates.emplace_back(dh, i);
            }
            std::sort(candidates.begin(), candidates.end());
            const int tries = std::min(static_cast<int>(candidates.size()), std::max(1, params.loop_closure_candidates));
            for (int c = 0; c < tries; c++)
            {
                const int best_i = candidates[c].second;

                // target: the earlier segment with its neighbours
                std::vector<Eigen::Vector3d> target;
                for (int i = std::max(0, best_i - 1); i <= std::min(j - 1, best_i + 1); i++)
                    target.insert(target.end(), segments[i].points.begin(), segments[i].points.end());
                std::vector<Eigen::Vector3d> source = segments[j].points;
                if (cross_floor)
                {
                    // only what both floors see (the shaft): the rest of each segment is ground the other never saw
                    const std::vector<Eigen::Vector3d> shared_target = crop_to_shared_footprint(target, source, 0.5);
                    source = crop_to_shared_footprint(source, target, 0.5);
                    target = shared_target;
                }
                if (source.size() < 500 || target.size() < 500)
                    continue;

                const auto fitness_grid = voxel_set(target, 0.2);
                const double before = overlap(source, Eigen::Affine3d::Identity(), fitness_grid, 0.2);
                Eigen::Affine3d T = coarse_search(source, target, cross_floor ? params.loop_closure_cross_floor_search_m
                                                                              : params.loop_closure_max_correction_m);
                Eigen::Matrix<double, 6, 6> registration_information = Eigen::Matrix<double, 6, 6>::Zero();
                T = ndt_refine(source, target, T, params.use_exact_ndt_bucket_statistics, &registration_information);
                const double after = overlap(source, T, fitness_grid, 0.2);
                // how far the correction moves the segment: its centre's movement (T's translation would also contain the
                // rotation about the world origin, growing with the distance from the start)
                Eigen::Vector3d source_centre = Eigen::Vector3d::Zero();
                for (const auto &p : source)
                    source_centre += p;
                source_centre /= static_cast<double>(source.size());
                const double shift = (T * source_centre - source_centre).norm();
                const double angle = Eigen::AngleAxisd(T.linear()).angle() / DEG;
                // between floors the shared region may be a single wall (a facade), which fixes only the direction across it
                const double info_ratio = cross_floor ? information_ratio(target, 0.2) : 1.0;
                // an already aligned revisit (no improvement possible) is a valid constraint as well. Between floors only
                // the shaft is shared: a lower overlap, and a correction no larger than the drift between floors can be
                constexpr double cross_floor_max_shift_m = 1.0; // a correction no larger than the drift between floors can be
                const bool accepted = cross_floor ? after >= params.loop_closure_cross_floor_min_fitness && after >= before - 0.02 &&
                                                        shift <= cross_floor_max_shift_m && angle <= params.loop_closure_cross_floor_max_angle_deg &&
                                                        info_ratio >= params.loop_closure_cross_floor_min_information_ratio
                                                  : after >= params.loop_closure_min_fitness && after >= before - 0.02 &&
                                                        shift <= params.loop_closure_max_correction_m + 1.0 && angle <= 15.0;
                if (cross_floor)
                    cross_floor_tries++;
                log_info(logger) << (cross_floor ? "cross-floor " : "") << "loop candidate segments " << best_i << " -> " << j << " (t "
                                 << std::fixed << std::setprecision(1) << segments[best_i].t_begin - segments[0].t_begin << " s / "
                                 << segments[j].t_begin - segments[0].t_begin << " s): overlap " << std::setprecision(2) << before << " -> "
                                 << after << ", correction " << shift << " m " << std::setprecision(1) << angle << " deg"
                                 << (cross_floor ? ", information ratio " + std::to_string(info_ratio) : std::string())
                                 << (accepted ? " ACCEPTED" : " rejected");
                if (accepted)
                {
                    LoopClosureEdge e;
                    e.i = best_i;
                    e.j = j;
                    e.correction = T;
                    e.fitness = after;
                    e.cross_floor = cross_floor;
                    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
                    for (const auto &p : source)
                        centre += p;
                    centre /= double(std::max<size_t>(source.size(), 1));
                    if (cross_floor && near_r >= 0.0 && std::hypot(centre.x() - near_x, centre.y() - near_y) > near_r)
                    {
                        log_info(logger) << "  cross-floor edge " << best_i << " -> " << j << " not used: shared region outside "
                                         << params.loop_closure_cross_floor_only_near;
                        continue; // the next candidate, as after a rejected registration
                    }
                    if (cross_floor)
                    {
                        const Eigen::Vector3d move = e.correction * centre - centre;
                        log_info(logger) << "  cross-floor edge " << best_i << " -> " << j << ": shared region centre " << std::fixed
                                         << std::setprecision(2) << centre.x() << " " << centre.y() << " " << centre.z() << ", its move "
                                         << move.x() << " " << move.y() << " " << move.z() << " m";
                    }
                    if (cross_floor)
                    {
                        // weighted by what the registration determines, without the height (a shaft fixes neither its
                        // height nor the turn about its axis)
                        bool ok = false;
                        e.omega = loop_edge_information(segments[best_i].pose, segments[j].pose, e.correction, registration_information, cross_floor,
                                                        params.loop_closure_edge_sigma_m, ok);
                        e.information = ok;
                        auto line = log_info(logger);
                        line << "  cross-floor edge " << best_i << " -> " << j;
                        if (!ok)
                        {
                            line << ": registration information unusable, edge not used";
                            continue; // the next candidate, as after a rejected registration
                        }
                        {
                            const Eigen::Matrix<double, 6, 1> s = loop_edge_region_sigmas(segments[best_i].pose, segments[j].pose, e, centre);
                            const auto put = [&](double v, double scale, double free_above)
                            {
                                if (v > free_above)
                                    line << " free";
                                else
                                    line << " " << v * scale;
                            };
                            line << ": information sigmas at the shared region, move x y z [cm]" << std::fixed << std::setprecision(1);
                            for (int k = 0; k < 3; k++)
                                put(s(k), 100.0, 10.0);
                            line << ", rotation roll pitch yaw [deg]";
                            for (int k = 3; k < 6; k++)
                                put(s(k), 1.0, 90.0);
                        }
                    }
                    loops.push_back(e);
                    if (cross_floor)
                        cross_floor_accepted++;
                    break;
                }
            }
        }
    }
    if (use_cross_floor)
        log_info(logger) << "loop closure: " << cross_floor_accepted << " of " << cross_floor_tries << " cross-floor registrations accepted";

    if (loops.empty())
    {
        log_info(logger) << "loop closure: no loops accepted, trajectory unchanged";
        return 0;
    }

    // 4. pose graph over the segment poses
    std::vector<Eigen::Affine3d> initial;
    for (const auto &s : segments)
        initial.push_back(s.pose);
    // a loop edge the solution disagrees with by more than loop_closure_max_residual_m / _deg is a false match that bends
    // the trajectory (a ghost wall at the revisit): the worst one is dropped and the graph solved again
    std::vector<Eigen::Affine3d> optimized;
    for (;;)
    {
        optimized = optimize_loop_pose_graph(initial, loops, params);
        const std::vector<LoopEdgeResidual> residuals = loop_edge_residuals(initial, optimized, loops);
        size_t worst = loops.size();
        double worst_ratio = 1.0;
        for (size_t e = 0; e < loops.size(); e++)
        {
            const double ratio = loop_edge_drop_ratio(loops[e], residuals[e], params);
            if (ratio > worst_ratio)
            {
                worst_ratio = ratio;
                worst = e;
            }
        }
        if (worst == loops.size())
        {
            auto line = log_info(logger);
            line << "loop closure: loop edge residuals [m/deg] (=> cross-floor)" << std::fixed;
            for (size_t e = 0; e < loops.size(); e++)
            {
                line << " " << loops[e].i << (loops[e].cross_floor ? "=>" : "->") << loops[e].j << " " << std::setprecision(2) << residuals[e].translation_m << "/"
                     << std::setprecision(1) << residuals[e].angle_deg;
                if (loops[e].information)
                    line << "/w" << std::setprecision(2) << residuals[e].weighted * params.loop_closure_edge_sigma_m;
            }
            break;
        }
        log_info(logger) << (loops[worst].cross_floor ? "cross-floor " : "") << "loop segments " << loops[worst].i << " -> " << loops[worst].j
                         << " dropped: residual " << std::fixed
                         << std::setprecision(2) << residuals[worst].translation_m << " m " << std::setprecision(1) << residuals[worst].angle_deg
                         << " deg"
                         << (loops[worst].information ? ", weighted " + std::to_string(residuals[worst].weighted * params.loop_closure_edge_sigma_m) + " m"
                                                      : std::string());
        loops.erase(loops.begin() + static_cast<std::ptrdiff_t>(worst));
        if (loops.empty())
        {
            log_info(logger) << "loop closure: no loops left, trajectory unchanged";
            return 0;
        }
    }
    double max_shift = 0.0;
    if (!loop_pose_graph_plausible(initial, optimized, loops, params, max_shift))
    {
        log_warning(logger) << "loop closure: the pose graph result is implausible (largest segment correction " << std::fixed << std::setprecision(2)
                            << max_shift << " m), discarded; trajectory unchanged";
        return 0;
    }

    // 5. interpolate the per-segment correction onto every node
    std::vector<Eigen::Affine3d> correction(segments.size());
    for (size_t k = 0; k < segments.size(); k++)
        correction[k] = optimized[k] * initial[k].inverse();
    for (size_t k = 0; k < segments.size(); k++)
    {
        const bool has_next = k + 1 < segments.size();
        const double t0 = segments[k].t_begin;
        const double t1 = has_next ? segments[k + 1].t_begin : segments[k].t_end;
        for (int w = segments[k].first_window; w <= segments[k].last_window; w++)
        {
            auto &tr = windows[w].poses;
            const auto &ts = windows[w].timestamps;
            for (size_t n = 0; n < tr.size(); n++)
            {
                const double f = (has_next && t1 > t0) ? std::clamp((ts[n].first - t0) / (t1 - t0), 0.0, 1.0) : 0.0;
                const Eigen::Affine3d C = has_next ? interpolate(correction[k], correction[k + 1], f) : correction[k];
                tr[n] = C * tr[n];
            }
            windows[w].motion_model_poses = tr;
        }
    }
    log_info(logger) << "loop closure: " << loops.size() << " loop(s) accepted, largest segment correction " << std::fixed << std::setprecision(2)
                     << max_shift << " m";
    return static_cast<int>(loops.size());
}
