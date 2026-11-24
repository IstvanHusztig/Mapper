#include "loop_closure.h"
#include "loop_closure_capture.h"
#include "holdout.h"
#include "ndt_map.h"
#include "chi_square.h"
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <set>
#include <optional>
#include <tuple>
#include <iomanip>
#include <limits>
#include <cstdio>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace
{
    constexpr double DEG = M_PI / 180.0;

    // the path walked between two segments and their horizontal centre distance, for the candidate log line
    std::string path_note(const LoopClosureSegment &earlier, const LoopClosureSegment &later)
    {
        const double walked = later.path_begin - earlier.path_end;
        const double distance = (later.center - earlier.center).head<2>().norm();
        std::ostringstream s;
        s << std::fixed << std::setprecision(1) << ", path " << walked << " m, distance " << distance << " m, ratio " << std::setprecision(2)
          << (walked > 0.0 ? distance / walked : 0.0);
        return s.str();
    }

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

LoopOdometryRule loop_odometry_rule(const OdometryConfig &params)
{
    LoopOdometryRule r;
    r.weak_ratio = params.loop_closure_odometry_weak_ratio;
    r.axis_tolerance_deg = params.loop_closure_odometry_axis_tolerance_deg;
    r.accelerometer_error_mps2 = params.loop_closure_odometry_accelerometer_error_mps2;
    r.sigma_m = params.loop_closure_odometry_sigma_m;
    r.sigma_deg = params.loop_closure_odometry_sigma_deg;
    return r;
}

namespace
{
    double median_of(std::vector<double> v)
    {
        std::sort(v.begin(), v.end());
        const size_t n = v.size();
        return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }

    // the weak runs of loop_odometry_step_information (its steps 1 to 3), without the steps' shares
    // (a window without readings is not readable: never weak, never bridged, never a pace reference; only readable windows
    // are read below)
    std::vector<LoopOdometryRun> weak_runs(const std::vector<std::optional<WindowReadings>> &readings, const LoopOdometryRule &rule)
    {
        const int W = static_cast<int>(readings.size());
        std::vector<char> readable(W), weak(W);
        for (int k = 0; k < W; k++)
        {
            const std::optional<WindowReadings> &x = readings[k];
            readable[k] = x && std::isfinite(x->information_ratio) && std::isfinite(x->duration_s) && x->duration_s > 0.0 &&
                          x->placed_move.allFinite() && x->weak_axis.allFinite() && !x->at_rest;
            weak[k] = readable[k] && x->information_ratio < rule.weak_ratio;
        }
        const auto rd = [&](int k) -> const WindowReadings & { return *readings[k]; };
        const double axis_cos = std::cos(rule.axis_tolerance_deg * DEG);
        const auto pace = [&](int k) { return rd(k).placed_move.norm() / rd(k).duration_s; };

        std::vector<LoopOdometryRun> runs;
        int k = 0;
        while (k < W)
        {
            if (!weak[k])
            {
                k++;
                continue;
            }
            const Eigen::Vector3d a0 = rd(k).weak_axis;
            const auto same = [&](int j) { return weak[j] && std::abs(rd(j).weak_axis.dot(a0)) >= axis_cos; };
            int e = k;
            for (;;)
            {
                if (e + 1 < W && same(e + 1))
                    e += 1;
                else if (e + 2 < W && readable[e + 1] && same(e + 2))
                    e += 2;
                else
                    break;
            }
            LoopOdometryRun r;
            r.first_window = k;
            r.last_window = e;
            Eigen::Vector3d axis = Eigen::Vector3d::Zero();
            Eigen::Vector3d placed = Eigen::Vector3d::Zero();
            for (int j = k; j <= e; j++)
            {
                placed += rd(j).placed_move;
                r.duration_s += rd(j).duration_s;
                if (!weak[j])
                    continue;
                const double d = rd(j).weak_axis.dot(a0);
                axis += rd(j).weak_axis * static_cast<double>((d > 0.0) - (d < 0.0));
                r.weak_windows++;
            }
            axis /= static_cast<double>(r.weak_windows);
            axis.normalize();
            if (placed.dot(axis) < 0.0)
                axis = -axis;
            r.axis = axis;

            // the pace before and after: the 3 nearest readable non-weak windows on each side
            std::vector<double> before, after;
            for (int j = k - 1; j >= 0 && before.size() < 3; j--)
                if (readable[j] && !weak[j])
                    before.push_back(pace(j));
            for (int j = e + 1; j < W && after.size() < 3; j++)
                if (readable[j] && !weak[j])
                    after.push_back(pace(j));
            double vb = before.empty() ? std::nan("") : median_of(before);
            double va = after.empty() ? std::nan("") : median_of(after);
            if (std::isnan(vb))
                vb = va;
            if (std::isnan(va))
                va = vb;
            const int n = e - k + 1;
            std::vector<double> v(n);
            for (int j = 0; j < n; j++)
                v[j] = rd(k + j).placed_move.dot(axis) / rd(k + j).duration_s;
            if (std::isnan(vb))
            {
                double mean = 0.0;
                for (double x : v)
                    mean += x;
                vb = va = mean / n;
            }
            r.pace_before_mps = vb;
            r.pace_after_mps = va;
            double elapsed = 0.0;
            for (int j = 0; j < n; j++)
            {
                const double dur = rd(k + j).duration_s;
                elapsed += dur;
                const double f = (elapsed - dur / 2.0) / r.duration_s;
                r.s2_m += std::abs(v[j] - (vb + (va - vb) * f)) * dur;
            }
            r.s1_m = 0.5 * rule.accelerometer_error_mps2 * r.duration_s * r.duration_s;
            r.sigma_m = std::hypot(r.s1_m, r.s2_m);
            runs.push_back(r);
            k = e + 1;
        }
        return runs;
    }
} // namespace

LoopOdometryInformation loop_odometry_step_information(const std::vector<std::optional<WindowReadings>> &readings,
                                                       const std::vector<LoopClosureSegment> &segments, const LoopOdometryRule &rule)
{
    LoopOdometryInformation out;
    out.runs = weak_runs(readings, rule);
    const int steps = std::max(0, static_cast<int>(segments.size()) - 1);
    std::vector<Eigen::Matrix3d> cov(steps, Eigen::Matrix3d::Identity() * rule.sigma_m * rule.sigma_m);
    for (LoopOdometryRun &r : out.runs)
    {
        for (int k = 0; k < static_cast<int>(segments.size()); k++)
        {
            const int lo = std::max(r.first_window, segments[k].first_window);
            const int hi = std::min(r.last_window, segments[k].last_window);
            if (lo > hi)
                continue;
            double in_segment = 0.0;
            for (int j = lo; j <= hi; j++)
                in_segment += readings[j]->duration_s; // a run's windows all have readings
            const double share = in_segment / r.duration_s;
            if (k == steps)
            {
                r.share_after_last_step += share;
                continue;
            }
            const Eigen::Vector3d along = segments[k].pose.linear().transpose() * r.axis;
            cov[k] += r.sigma_m * r.sigma_m * share * along * along.transpose();
            r.step_shares.emplace_back(k, share);
        }
    }
    const double w_r = 1.0 / std::pow(rule.sigma_deg * DEG, 2);
    for (int k = 0; k < steps; k++)
    {
        Eigen::Matrix<double, 6, 6> omega = Eigen::Matrix<double, 6, 6>::Zero();
        omega.topLeftCorner<3, 3>() = cov[k].inverse();
        omega.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * w_r;
        out.information.push_back(omega);
    }
    return out;
}

namespace
{
    // one line per run of loop_odometry_step_information (windows numbered from 0, as the segments; times from the first
    // window's first node, as time_s of window_features.csv), and a summary
    void log_loop_odometry_runs(const LoopOdometryInformation &steps, const std::vector<Window> &windows, const OdometryConfig &params, Logger &logger)
    {
        const double t0 = windows.front().timestamps.front().first;
        const LoopOdometryRun *largest = nullptr;
        int weak = 0;
        std::set<int> loosened;
        for (const LoopOdometryRun &r : steps.runs)
        {
            weak += r.weak_windows;
            if (!largest || r.sigma_m > largest->sigma_m)
                largest = &r;
            auto line = log_info(logger);
            line << "loop closure odometry run: windows " << r.first_window << "-" << r.last_window << ", t " << std::fixed << std::setprecision(2)
                 << windows[r.first_window].timestamps.front().first - t0 << "-" << windows[r.last_window].timestamps.back().first - t0 << " s, T "
                 << r.duration_s << " s, " << r.weak_windows << " weak, axis " << r.axis.x() << " " << r.axis.y() << " " << r.axis.z()
                 << ", pace before " << r.pace_before_mps << " / after " << r.pace_after_mps << " m/s, S1 " << r.s1_m << " m, S2 " << r.s2_m
                 << " m, sigma " << r.sigma_m << " m, steps";
            for (const auto &[k, share] : r.step_shares)
            {
                line << " " << k << " " << share;
                loosened.insert(k);
            }
            if (r.share_after_last_step > 0.0)
                line << ", after the last step " << r.share_after_last_step << " (not added)";
        }
        auto line = log_info(logger);
        line << "loop closure odometry from readings: " << steps.runs.size() << " run(s), " << weak << " weak window(s) (information ratio below "
             << params.loop_closure_odometry_weak_ratio << ", axes within " << params.loop_closure_odometry_axis_tolerance_deg
             << " deg, accelerometer error " << params.loop_closure_odometry_accelerometer_error_mps2 << " m/s^2), " << loosened.size() << " of "
             << steps.information.size() << " odometry steps loosened";
        if (largest)
            line << ", largest sigma " << std::fixed << std::setprecision(2) << largest->sigma_m << " m (windows " << largest->first_window << "-"
                 << largest->last_window << ")";
    }
} // namespace

std::vector<Eigen::Affine3d> optimize_loop_pose_graph(const std::vector<Eigen::Affine3d> &initial, const std::vector<LoopClosureEdge> &loops,
                                                      const OdometryConfig &params, const LoopOdometryEdgeInformation *odometry_information)
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
    // before the solve each one is off by the whole drift (metres on a long recording, tens of sigmas)
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
            if (odometry_information)
            {
                const Eigen::Matrix<double, 6, 1> d = information_edge_residual(ps[k], ps[k + 1], odo[k]);
                c += d.dot((*odometry_information)[k] * d);
            }
            else
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
                if (odometry_information)
                    add_information_edge(A, b, poses, k, k + 1, odo[k], (*odometry_information)[k]);
                else
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

double loop_pose_graph_cost(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                            const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params,
                            const LoopOdometryEdgeInformation *odometry_information)
{
    std::vector<TaitBryanPose> ps;
    for (const auto &p : optimized)
        ps.push_back(pose_tait_bryan_from_affine_matrix(p));
    const double w_odo_t = 1.0 / std::pow(params.loop_closure_odometry_sigma_m, 2);
    const double w_odo_r = 1.0 / std::pow(params.loop_closure_odometry_sigma_deg * DEG, 2);
    double cost = 0.0;
    for (size_t k = 0; k + 1 < initial.size(); k++)
        if (odometry_information)
        {
            const Eigen::Matrix<double, 6, 1> d = information_edge_residual(ps[k], ps[k + 1], relative_measurement(initial[k], initial[k + 1]));
            cost += d.dot((*odometry_information)[k] * d);
        }
        else
            cost += relative_pose_edge_cost(ps, static_cast<int>(k), static_cast<int>(k + 1), relative_measurement(initial[k], initial[k + 1]), w_odo_t,
                                            w_odo_r);
    const double w_loop_t = 1.0 / std::pow(params.loop_closure_edge_sigma_m, 2);
    const double w_loop_r = 1.0 / std::pow(params.loop_closure_edge_sigma_deg * DEG, 2);
    for (const auto &e : loops)
    {
        const Eigen::Matrix<double, 6, 1> m = relative_measurement(initial[e.i], e.correction * initial[e.j]);
        if (e.information)
        {
            const Eigen::Matrix<double, 6, 1> d = information_edge_residual(ps[e.i], ps[e.j], m);
            cost += d.dot(e.omega * d);
        }
        else
            cost += relative_pose_edge_cost(ps, e.i, e.j, m, w_loop_t, w_loop_r);
    }
    return cost;
}

size_t loop_edge_to_drop(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                         const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params,
                         const LoopOdometryEdgeInformation *odometry_information)
{
    const std::vector<LoopEdgeResidual> residuals = loop_edge_residuals(initial, optimized, loops);
    std::vector<size_t> beyond;
    for (size_t e = 0; e < loops.size(); e++)
        if (loop_edge_drop_ratio(loops[e], residuals[e], params) > 1.0)
            beyond.push_back(e);
    if (beyond.empty())
        return loops.size();
    if (beyond.size() == 1)
        return beyond.front();
    size_t best = beyond.front();
    double best_cost = std::numeric_limits<double>::infinity();
    for (const size_t e : beyond)
    {
        std::vector<LoopClosureEdge> without = loops;
        without.erase(without.begin() + static_cast<std::ptrdiff_t>(e));
        const double cost = loop_pose_graph_cost(initial, optimize_loop_pose_graph(initial, without, params, odometry_information), without, params,
                                                 odometry_information);
        if (cost < best_cost) // ties keep the earlier edge (deterministic)
        {
            best_cost = cost;
            best = e;
        }
    }
    return best;
}

double loop_edge_drop_ratio(const LoopClosureEdge &edge, const LoopEdgeResidual &residual, const OdometryConfig &params)
{
    if (edge.information)
        return residual.weighted * params.loop_closure_edge_sigma_m / params.loop_closure_max_residual_m;
    return std::max(residual.translation_m / params.loop_closure_max_residual_m,
                    residual.angle_deg / params.loop_closure_max_residual_deg);
}

bool loop_edges_share_scans(const LoopClosureEdge &e, const LoopClosureEdge &f)
{
    return f.j == e.j || std::abs(f.i - e.i) <= 2 || std::abs(f.j - e.i) <= 1 || std::abs(e.j - f.i) <= 1;
}

int loop_edge_dof(const LoopClosureEdge &e)
{
    if (!e.information)
        return 6;
    const double largest = e.omega.cwiseAbs().maxCoeff();
    if (!(largest > 0.0))
        return 0;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(e.omega);
    int rank = 0;
    for (int k = 0; k < 6; k++)
        if (svd.singularValues()(k) > 1e-6 * largest)
            rank++;
    return rank;
}

std::vector<LoopMatchTest> loop_match_tests(const std::vector<Eigen::Affine3d> &initial, const std::vector<LoopClosureEdge> &loops,
                                            const OdometryConfig &params, const LoopOdometryEdgeInformation *odometry_information)
{
    std::vector<LoopMatchTest> out;
    for (size_t e = 0; e < loops.size(); e++)
    {
        LoopMatchTest t;
        std::vector<LoopClosureEdge> rest;
        for (size_t f = 0; f < loops.size(); f++)
        {
            if (f == e)
                continue;
            if (loop_edges_share_scans(loops[e], loops[f]))
                t.held_out++;
            else
                rest.push_back(loops[f]);
        }
        const std::vector<Eigen::Affine3d> without = optimize_loop_pose_graph(initial, rest, params, odometry_information);
        const double c0 = loop_pose_graph_cost(initial, without, rest, params, odometry_information);
        std::vector<LoopClosureEdge> with = rest;
        with.push_back(loops[e]);
        const double c1 = loop_pose_graph_cost(initial, optimize_loop_pose_graph(initial, with, params, odometry_information), with, params,
                                               odometry_information);
        t.t = c1 - c0;
        t.dof = loop_edge_dof(loops[e]);
        t.limit = chi_square_quantile(params.loop_closure_match_check_quantile, t.dof);
        const LoopClosureEdge &x = loops[e];
        const Eigen::Matrix<double, 6, 1> d =
            information_edge_residual(pose_tait_bryan_from_affine_matrix(without[x.i]), pose_tait_bryan_from_affine_matrix(without[x.j]),
                                      relative_measurement(initial[x.i], x.correction * initial[x.j]));
        t.innovation_m = d.head<3>().norm();
        out.push_back(t);
    }
    return out;
}

namespace
{
    std::string edge_name(const LoopClosureEdge &e)
    {
        return std::to_string(e.i) + (e.cross_floor ? "=>" : "->") + std::to_string(e.j);
    }

    // loop_edge_to_drop until no edge is beyond the limits; false: no edge left (outcome all_dropped)
    bool prune_loop_pose_graph(const std::vector<Eigen::Affine3d> &initial, std::vector<LoopClosureEdge> &loops, const OdometryConfig &params,
                               const LoopOdometryEdgeInformation *odometry_information, Logger &logger, LoopClosureReport &rep,
                               std::vector<Eigen::Affine3d> &optimized, std::vector<LoopEdgeResidual> &residuals)
    {
        // a loop edge the solution disagrees with by more than loop_closure_max_residual_m / _deg is a false match that bends
        // the trajectory (a ghost wall at the revisit): the worst one is dropped and the graph solved again
        for (;;)
        {
            optimized = optimize_loop_pose_graph(initial, loops, params, odometry_information);
            residuals = loop_edge_residuals(initial, optimized, loops);
            if (rep.first_solve.empty())
            {
                rep.first_solve = loops;
                rep.first_solve_residuals = residuals;
            }
            const size_t worst = loop_edge_to_drop(initial, optimized, loops, params, odometry_information);
            if (worst == loops.size())
                return true;
            log_info(logger) << (loops[worst].cross_floor ? "cross-floor " : "") << "loop segments " << loops[worst].i << " -> " << loops[worst].j
                             << " dropped: residual " << std::fixed
                             << std::setprecision(2) << residuals[worst].translation_m << " m " << std::setprecision(1) << residuals[worst].angle_deg
                             << " deg"
                             << (loops[worst].information ? ", weighted " + std::to_string(residuals[worst].weighted * params.loop_closure_edge_sigma_m) + " m"
                                                          : std::string());
            rep.dropped.push_back(loops[worst]);
            rep.dropped_residuals.push_back(residuals[worst]);
            rep.dropped_match_tests.push_back(std::nullopt);
            loops.erase(loops.begin() + static_cast<std::ptrdiff_t>(worst));
            if (loops.empty())
            {
                rep.outcome = "all_dropped";
                log_info(logger) << "loop closure: no loops left, trajectory unchanged";
                return false;
            }
        }
    }

    // an edge whose test means something: at least one degree of freedom, a finite T and limit
    bool match_testable(const LoopMatchTest &t)
    {
        return t.dof > 0 && std::isfinite(t.t) && std::isfinite(t.limit) && t.limit > 0.0;
    }

    // the testable edges, by T / limit, largest first (the earlier one on ties)
    std::vector<size_t> match_tests_by_ratio(const std::vector<LoopMatchTest> &tests)
    {
        std::vector<size_t> order;
        for (size_t e = 0; e < tests.size(); e++)
            if (match_testable(tests[e]))
                order.push_back(e);
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) { return tests[a].t / tests[a].limit > tests[b].t / tests[b].limit; });
        return order;
    }
} // namespace

std::optional<std::pair<std::string, std::string>> loop_match_check_refusal(const OdometryConfig &params)
{
    if (!params.loop_closure_match_check)
        return std::nullopt;
    const double q = params.loop_closure_match_check_quantile;
    if (!(std::isfinite(q) && q > 0.0 && q < 1.0))
    {
        std::ostringstream why;
        why << "loop_closure_match_check_quantile = " << q << " is not a probability between 0 and 1 (exclusive)";
        return std::make_pair(std::string("match_check_bad_quantile"), why.str());
    }
    if (params.loop_closure_kernel_scale > 0.0)
    {
        std::ostringstream why;
        why << "loop_closure_match_check with loop_closure_kernel_scale = " << params.loop_closure_kernel_scale
            << ": the graphs would be solved with the kernel and scored without it, so the test value is not the held-out innovation";
        return std::make_pair(std::string("match_check_with_kernel"), why.str());
    }
    if (!params.loop_closure_odometry_from_readings)
        return std::make_pair(std::string("match_check_without_readings"),
                              std::string("loop_closure_match_check needs loop_closure_odometry_from_readings (on uniform odometry steps it "
                                          "rates true edges across a blind stretch as false)"));
    return std::nullopt;
}

bool solve_and_prune_loop_pose_graph(const std::vector<Eigen::Affine3d> &initial, std::vector<LoopClosureEdge> &loops, const OdometryConfig &params,
                                     const LoopOdometryEdgeInformation *odometry_information, Logger &logger, LoopClosureReport &rep,
                                     std::vector<Eigen::Affine3d> &optimized, std::vector<LoopEdgeResidual> &residuals)
{
    if (params.loop_closure_match_check)
        if (const auto refused = loop_match_check_refusal(params); refused && refused->first != "match_check_without_readings")
        {
            rep.outcome = refused->first;
            log_error(logger) << "loop closure: " << refused->second << "; loop closure not run, trajectory unchanged";
            return false;
        }
    if (!prune_loop_pose_graph(initial, loops, params, odometry_information, logger, rep, optimized, residuals))
        return false;
    if (params.loop_closure_match_check)
    {
        int rounds = 0, dropped = 0;
        std::vector<LoopMatchTest> tests;
        std::vector<size_t> order;
        for (;;)
        {
            tests = loop_match_tests(initial, loops, params, odometry_information);
            rounds++;
            order = match_tests_by_ratio(tests);
            if (order.empty())
                break;
            const size_t worst = order.front();
            const LoopMatchTest &t = tests[worst];
            if (!(t.t > t.limit))
                break;
            log_info(logger) << "loop closure match check drops " << edge_name(loops[worst]) << ": T " << std::fixed << std::setprecision(2) << t.t
                             << " (limit " << t.limit << ", " << t.dof << " dof), " << t.held_out << " held out, innovation " << t.innovation_m << " m";
            rep.dropped.push_back(loops[worst]);
            rep.dropped_residuals.push_back(residuals[worst]);
            rep.dropped_match_tests.push_back(t);
            loops.erase(loops.begin() + static_cast<std::ptrdiff_t>(worst));
            dropped++;
            if (loops.empty())
            {
                rep.outcome = "all_dropped";
                log_info(logger) << "loop closure: no loops left, trajectory unchanged";
                return false;
            }
            if (!prune_loop_pose_graph(initial, loops, params, odometry_information, logger, rep, optimized, residuals))
                return false;
        }
        rep.kept_match_tests = tests;
        // the largest values left, by T / limit; the edges that could not be tested (kept)
        auto line = log_info(logger);
        line << "loop closure match check: " << rounds << " round(s), " << order.size() << " edges tested in the last, " << dropped
             << " dropped; largest T left" << std::fixed << std::setprecision(2);
        for (size_t k = 0; k < std::min<size_t>(4, order.size()); k++)
            line << " " << edge_name(loops[order[k]]) << " " << tests[order[k]].t << " (limit " << tests[order[k]].limit << ")";
        if (order.size() < tests.size())
        {
            line << "; " << tests.size() - order.size() << " not testable (no degree of freedom or no finite test value), kept:";
            for (size_t e = 0; e < tests.size(); e++)
                if (!match_testable(tests[e]))
                    line << " " << edge_name(loops[e]);
        }
    }
    auto line = log_info(logger);
    line << "loop closure: loop edge residuals [m/deg] (=> cross-floor)" << std::fixed;
    for (size_t e = 0; e < loops.size(); e++)
    {
        line << " " << loops[e].i << (loops[e].cross_floor ? "=>" : "->") << loops[e].j << " " << std::setprecision(2) << residuals[e].translation_m << "/"
             << std::setprecision(1) << residuals[e].angle_deg;
        if (loops[e].information)
            line << "/w" << std::setprecision(2) << residuals[e].weighted * params.loop_closure_edge_sigma_m;
    }
    return true;
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

std::vector<double> path_at_windows(const std::vector<Window> &windows)
{
    constexpr double min_step_m = 0.1;
    std::vector<double> path(windows.size(), 0.0);
    double walked = 0.0;
    Eigen::Vector3d last = Eigen::Vector3d::Zero();
    bool have_last = false;
    for (size_t w = 0; w < windows.size(); w++)
    {
        if (!windows[w].poses.empty())
        {
            const Eigen::Vector3d p = windows[w].poses.front().translation();
            if (!have_last)
            {
                last = p;
                have_last = true;
            }
            else if ((p - last).norm() >= min_step_m)
            {
                walked += (p - last).norm();
                last = p;
            }
        }
        path[w] = walked;
    }
    return path;
}

std::vector<int> same_floor_revisits(const std::vector<RevisitSegment> &segments, int j, const OdometryConfig &params)
{
    std::vector<std::pair<double, int>> qualifying; // (path between, i)
    for (int i = 0; i < j; i++)
    {
        const double path = segments[j].path_begin - segments[i].path_end;
        if (path < params.loop_closure_min_path_m)
            continue;
        const Eigen::Vector3d d = segments[j].center - segments[i].center;
        if (std::abs(d.z()) > params.loop_closure_max_vertical_offset_m)
            continue;
        if (d.head<2>().norm() > std::min(params.loop_closure_search_radius_m, params.loop_closure_radius_per_path * path))
            continue;
        qualifying.emplace_back(path, i);
    }
    // longest path first; equal paths by the earlier segment (deterministic)
    std::sort(qualifying.begin(), qualifying.end(), [](const auto &a, const auto &b) { return a.first != b.first ? a.first > b.first : a.second < b.second; });
    std::vector<int> out;
    for (const auto &[path, i] : qualifying)
        out.push_back(i);
    return out;
}

// the same-floor revisits of segment j, horizontally nearest first (equal distances in the order of same_floor_revisits)
static std::vector<int> nearest_same_floor_revisits(const std::vector<RevisitSegment> &segments, int j, const std::vector<int> &by_path)
{
    std::vector<int> by_distance = by_path;
    std::stable_sort(by_distance.begin(), by_distance.end(), [&](int a, int b) {
        return (segments[j].center - segments[a].center).head<2>().norm() < (segments[j].center - segments[b].center).head<2>().norm();
    });
    return by_distance;
}

std::vector<RevisitTry> revisit_tries(const std::vector<RevisitSegment> &segments, int j, const OdometryConfig &params)
{
    const std::vector<int> by_path = same_floor_revisits(segments, j, params); // longest path first
    const std::vector<int> by_distance = nearest_same_floor_revisits(segments, j, by_path);
    // the first of an ordered list, then the first at least 3 segments away from it
    const auto two_apart = [](const std::vector<int> &ordered) {
        std::vector<int> out;
        for (const int i : ordered)
            if (out.empty() || std::abs(i - out.front()) >= 3)
            {
                out.push_back(i);
                if (out.size() == 2)
                    break;
            }
        return out;
    };
    std::vector<RevisitTry> tries;
    const auto add = [&](int i, bool near) {
        for (auto &t : tries)
            if (t.i == i)
            {
                (near ? t.near : t.far) = true;
                return;
            }
        RevisitTry t;
        t.i = i;
        (near ? t.near : t.far) = true;
        tries.push_back(t);
    };
    // interleaved (nearest, longest path, second nearest, second longest path), so that a cap below 4 keeps both classes
    const std::vector<int> near = two_apart(by_distance);
    const std::vector<int> far = two_apart(by_path);
    for (size_t k = 0; k < 2; k++)
    {
        if (k < near.size())
            add(near[k], true);
        if (k < far.size())
            add(far[k], false);
    }
    if (static_cast<int>(tries.size()) > std::max(1, params.loop_closure_candidates))
        tries.resize(static_cast<size_t>(std::max(1, params.loop_closure_candidates)));
    return tries;
}

std::vector<LoopClosureEdge> revisit_slots(const std::vector<AcceptedRevisit> &accepted, int edges_per_segment)
{
    const auto best_of = [&](bool want_near, bool want_far) {
        const AcceptedRevisit *best = nullptr;
        for (const auto &a : accepted)
            if ((!want_near || a.near) && (!want_far || a.far) && (!best || a.edge.fitness > best->edge.fitness))
                best = &a;
        return best;
    };
    std::vector<LoopClosureEdge> out;
    const AcceptedRevisit *overall = best_of(false, false);
    if (!overall)
        return out;
    if (edges_per_segment < 2)
        return {overall->edge};
    const AcceptedRevisit *near = best_of(true, false);
    const AcceptedRevisit *far = best_of(false, true);
    if (near && far && near != far && std::abs(near->edge.i - far->edge.i) >= 3)
        return {near->edge, far->edge};
    return {overall->edge};
}

std::vector<LoopClosureSegment> loop_closure_segments(const std::vector<Window> &windows, const OdometryConfig &params)
{
    const int windows_per_segment = std::max(1, params.loop_closure_segment_windows);
    const int n_windows = static_cast<int>(windows.size());
    const std::vector<double> path = path_at_windows(windows);
    std::vector<LoopClosureSegment> segments;
    for (int w0 = 0; w0 < n_windows; w0 += windows_per_segment)
    {
        LoopClosureSegment s;
        s.first_window = w0;
        s.last_window = std::min(n_windows, w0 + windows_per_segment) - 1;
        s.path_begin = path[w0];
        s.path_end = path[std::min(n_windows - 1, s.last_window + 1)];
        s.pose = windows[w0].poses.front();
        s.t_begin = windows[w0].timestamps.front().first;
        s.t_end = windows[s.last_window].timestamps.back().first;
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
        for (int w = s.first_window; w <= s.last_window; w++)
            for (const auto &node : windows[w].poses)
            {
                sum += node.translation();
                count++;
            }
        s.center = count > 0 ? Eigen::Vector3d(sum / count) : s.pose.translation();
        segments.push_back(s);
    }
    return segments;
}

bool loop_try_accepted(const LoopClosureTry &t, const OdometryConfig &params)
{
    if (!t.registered)
        return false;
    // an already aligned revisit (no improvement possible) is a valid constraint as well. Between floors only the shaft is
    // shared: a lower overlap, and a correction no larger than the drift between floors can be
    constexpr double cross_floor_max_shift_m = 1.0;
    // between floors the shared region may be a single wall (a facade), which fixes only the direction across it
    return t.cross_floor ? t.overlap_after >= params.loop_closure_cross_floor_min_fitness && t.overlap_after >= t.overlap_before - 0.02 &&
                               t.shift_m <= cross_floor_max_shift_m && t.angle_deg <= params.loop_closure_cross_floor_max_angle_deg &&
                               t.information_ratio >= params.loop_closure_cross_floor_min_information_ratio
                         : t.overlap_after >= params.loop_closure_min_fitness && t.overlap_after >= t.overlap_before - 0.02 &&
                               t.shift_m <= params.loop_closure_max_correction_m + 1.0 && t.angle_deg <= 15.0;
}

std::vector<int> cross_floor_revisits(const std::vector<LoopClosureSegment> &segments, int j, const OdometryConfig &params)
{
    if (!(params.loop_closure_cross_floor_max_vertical_m > params.loop_closure_max_vertical_offset_m))
        return {};
    // segment ranges whose cross-floor pairs are not tried (loop_closure_cross_floor_exclude_segments)
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
    const auto excluded = [&](int segment)
    {
        for (const auto &[a, b] : excluded_ranges)
            if (segment >= a && segment <= b)
                return true;
        return false;
    };
    std::vector<std::pair<double, int>> nearest;
    for (int i = 0; i < j; i++)
    {
        if (segments[j].t_begin - segments[i].t_end < params.loop_closure_cross_floor_min_time_gap_s)
            continue;
        if (excluded(i) || excluded(j))
            continue;
        const Eigen::Vector3d d = segments[j].center - segments[i].center;
        const double dz = std::abs(d.z());
        if (!(dz > params.loop_closure_max_vertical_offset_m && dz <= params.loop_closure_cross_floor_max_vertical_m))
            continue;
        const double dh = d.head<2>().norm();
        if (dh < params.loop_closure_search_radius_m)
            nearest.emplace_back(dh, i);
    }
    std::sort(nearest.begin(), nearest.end());
    std::vector<int> out;
    for (const auto &[dh, i] : nearest)
        out.push_back(i);
    return out;
}

int close_loops(std::vector<Window> &windows, const std::vector<LoopClosureSegment> &segments, LoopClosureRegistrations &registrations,
                const OdometryConfig &params, Logger &logger, LoopClosureReport *report)
{
    LoopClosureReport local;
    LoopClosureReport &rep = report ? *report : local;
    rep = LoopClosureReport{};
    const int windows_per_segment = std::max(1, params.loop_closure_segment_windows);
    const int n_windows = static_cast<int>(windows.size());
    if (n_windows < 2 * windows_per_segment)
        return 0;
    log_info(logger) << "loop closure START";

    if (const auto refused = loop_match_check_refusal(params))
    {
        rep.outcome = refused->first;
        log_error(logger) << "loop closure: " << refused->second << "; loop closure not run, trajectory unchanged";
        return 0;
    }

    // the odometry steps' information from the windows' readings (loop_closure_odometry_from_readings)
    std::optional<LoopOdometryInformation> odometry_steps;
    if (params.loop_closure_odometry_from_readings)
    {
        std::vector<std::optional<WindowReadings>> readings;
        std::vector<std::pair<int, int>> without; // ranges of windows without readings
        for (int w = 0; w < n_windows; w++)
        {
            readings.push_back(windows[w].readings);
            if (windows[w].readings)
                continue;
            if (!without.empty() && without.back().second == w - 1)
                without.back().second = w;
            else
                without.emplace_back(w, w);
        }
        int missing = 0;
        for (const auto &[a, b] : without)
            missing += b - a + 1;
        rep.windows_without_readings = missing;
        if (missing == n_windows)
        {
            rep.outcome = "readings_missing";
            log_error(logger) << "loop closure: loop_closure_odometry_from_readings is on, but none of the " << n_windows
                              << " windows has readings (a snapshot or capture written before they were kept); loop closure not run, "
                              << "trajectory unchanged";
            return 0;
        }
        if (missing > 0)
        {
            auto line = log_warning(logger);
            line << "loop closure: " << missing << " of " << n_windows << " windows have no readings (not solved by the odometry): windows";
            for (size_t k = 0; k < without.size(); k++)
                line << (k ? ", " : " ") << without[k].first << "-" << without[k].second;
            line << " (from 0); the rule doesn't read them, their steps keep the fixed sigmas unless a run's time lies in them";
        }
        odometry_steps = loop_odometry_step_information(readings, segments, loop_odometry_rule(params));
        log_loop_odometry_runs(*odometry_steps, windows, params, logger);
        rep.odometry_runs = odometry_steps->runs;
    }
    const LoopOdometryEdgeInformation *odometry_information = odometry_steps ? &odometry_steps->information : nullptr;

    // 2.+3. revisit candidates and their registration
    std::vector<LoopClosureEdge> loops;
    const bool use_cross_floor = params.loop_closure_cross_floor_max_vertical_m > params.loop_closure_max_vertical_offset_m;
    double near_x = 0.0, near_y = 0.0, near_r = -1.0; // loop_closure_cross_floor_only_near, r < 0: off
    std::sscanf(params.loop_closure_cross_floor_only_near.c_str(), "%lf %lf %lf", &near_x, &near_y, &near_r);
    int cross_floor_tries = 0;
    int cross_floor_accepted = 0;
    std::vector<RevisitSegment> revisit_segments;
    for (const auto &s : segments)
        revisit_segments.push_back({s.center, s.path_begin, s.path_end});
    for (int j = 0; j < static_cast<int>(segments.size()); j++)
    {
        // the earlier segments that qualify. Pass 0: the same floor, every try of revisit_tries registered and the edges
        // chosen afterwards (revisit_slots). Pass 1 (cross-floor loops): the floors above and below, nearest first, the
        // next one tried when a registration is rejected, as one more edge
        for (int pass = 0; pass < (use_cross_floor ? 2 : 1); pass++)
        {
            const bool cross_floor = pass == 1;
            std::vector<int> candidates;
            std::vector<RevisitTry> same_floor_tries;
            std::vector<AcceptedRevisit> same_floor_accepted;
            if (!cross_floor)
            {
                same_floor_tries = revisit_tries(revisit_segments, j, params);
                for (const auto &t : same_floor_tries)
                    candidates.push_back(t.i);
            }
            else
                candidates = cross_floor_revisits(segments, j, params);
            const int tries = std::min(static_cast<int>(candidates.size()), std::max(1, params.loop_closure_candidates));
            for (int c = 0; c < tries; c++)
            {
                const int best_i = candidates[c];
                const LoopClosureTry *registration = registrations.registration(best_i, j, cross_floor);
                if (!registration || !registration->registered)
                    continue;
                const LoopClosureTry &r = *registration;
                const bool named = registrations.rejected(best_i, j, cross_floor);
                const bool accepted = !named && loop_try_accepted(r, params);
                if (cross_floor)
                    cross_floor_tries++;
                log_info(logger) << (cross_floor ? "cross-floor " : "") << "loop candidate segments " << best_i << " -> " << j << " (t "
                                 << std::fixed << std::setprecision(1) << segments[best_i].t_begin - segments[0].t_begin << " s / "
                                 << segments[j].t_begin - segments[0].t_begin << " s): overlap " << std::setprecision(2) << r.overlap_before
                                 << " -> " << r.overlap_after << ", correction " << r.shift_m << " m " << std::setprecision(1) << r.angle_deg
                                 << " deg"
                                 << (cross_floor ? ", information ratio " + std::to_string(r.information_ratio) : path_note(segments[best_i], segments[j]))
                                 << (accepted ? " ACCEPTED" : named ? " rejected (named for rejection)" : " rejected");
                if (accepted)
                {
                    LoopClosureEdge e;
                    e.i = best_i;
                    e.j = j;
                    e.correction = r.correction;
                    e.fitness = r.overlap_after;
                    e.cross_floor = cross_floor;
                    e.shift_m = r.shift_m;
                    e.angle_deg = r.angle_deg;
                    const Eigen::Vector3d &centre = r.centre;
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
                    if (!cross_floor)
                    {
                        // weighted by what the registration determines (height kept); a registration whose information is
                        // unusable keeps the fixed sigmas loop_closure_edge_sigma_m / _deg
                        bool ok = false;
                        e.omega = loop_edge_information(segments[best_i].pose, segments[j].pose, e.correction, r.information, false,
                                                        params.loop_closure_edge_sigma_m, ok);
                        e.information = ok;
                    }
                    if (cross_floor)
                    {
                        // weighted by what the registration determines, without the height (a shaft fixes neither its
                        // height nor the turn about its axis)
                        bool ok = false;
                        e.omega = loop_edge_information(segments[best_i].pose, segments[j].pose, e.correction, r.information, cross_floor,
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
                    if (cross_floor)
                    {
                        loops.push_back(e);
                        cross_floor_accepted++;
                        break;
                    }
                    // G2b: all tries are registered, the slots are chosen afterwards
                    same_floor_accepted.push_back({e, same_floor_tries[c].near, same_floor_tries[c].far});
                }
            }
            if (!cross_floor)
                for (const auto &e : revisit_slots(same_floor_accepted, params.loop_closure_edges_per_segment))
                {
                    log_info(logger) << "  loop edge " << e.i << " -> " << j << " kept (overlap " << std::fixed << std::setprecision(2) << e.fitness << ")";
                    loops.push_back(e);
                }
        }
    }
    if (use_cross_floor)
        log_info(logger) << "loop closure: " << cross_floor_accepted << " of " << cross_floor_tries << " cross-floor registrations accepted";

    if (loops.empty())
    {
        rep.outcome = "no_loops";
        log_info(logger) << "loop closure: no loops accepted, trajectory unchanged";
        return 0;
    }

    // 4. pose graph over the segment poses
    std::vector<Eigen::Affine3d> initial;
    for (const auto &s : segments)
        initial.push_back(s.pose);
    std::vector<Eigen::Affine3d> optimized;
    std::vector<LoopEdgeResidual> residuals;
    if (!solve_and_prune_loop_pose_graph(initial, loops, params, odometry_information, logger, rep, optimized, residuals))
        return 0;
    rep.kept = loops;
    rep.kept_residuals = residuals;
    double max_shift = 0.0;
    const bool plausible = loop_pose_graph_plausible(initial, optimized, loops, params, max_shift);
    rep.largest_segment_correction_m = max_shift;
    if (!plausible)
    {
        rep.outcome = "implausible";
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
    rep.outcome = "applied";
    log_info(logger) << "loop closure: " << loops.size() << " loop(s) accepted, largest segment correction " << std::fixed << std::setprecision(2)
                     << max_shift << " m";
    return static_cast<int>(loops.size());
}

namespace
{
    // a segment's points in world coordinates (as the odometry placed them), 0.1 m voxel-decimated
    std::vector<Eigen::Vector3d> segment_points(const std::vector<Window> &windows, const LoopClosureSegment &s, const OdometryConfig &params,
                                                PointStore &point_store)
    {
        std::vector<Eigen::Vector3d> points;
        for (int w = s.first_window; w <= s.last_window; w++)
        {
            const auto &tr = windows[w].poses;
            std::vector<Point3Di> window_points;
            if (!point_store.load(PointSet::WindowIntermediate, w, window_points))
                continue;
            remove_held_out(window_points, params.holdout_block_s);
            for (const auto &p : window_points)
            {
                const double r = p.point.norm();
                if (r < params.min_distance_lidar || r > params.max_distance_lidar || p.index_pose < 0 || p.index_pose >= static_cast<int>(tr.size()))
                    continue;
                points.push_back(tr[p.index_pose] * p.point);
            }
        }
        return voxel_decimate(points, 0.1);
    }

    // step 3 on the points: each pair registered once, kept in the order registered
    class PointRegistrations : public LoopClosureRegistrations
    {
    public:
        PointRegistrations(std::vector<std::vector<Eigen::Vector3d>> points, const OdometryConfig &params) : points_(std::move(points)), params_(params) {}

        const LoopClosureTry *registration(int i, int j, bool cross_floor) override
        {
            const auto key = std::make_tuple(i, j, cross_floor);
            if (const auto it = index_.find(key); it != index_.end())
                return &tries_[it->second];
            index_[key] = tries_.size();
            tries_.push_back(register_pair(i, j, cross_floor));
            return &tries_.back();
        }
        bool has(int i, int j, bool cross_floor) const { return index_.count(std::make_tuple(i, j, cross_floor)) > 0; }
        // the registrations of segment j of one kind so far
        int count(int j, bool cross_floor) const
        {
            int n = 0;
            for (const auto &t : tries_)
                n += t.j == j && t.cross_floor == cross_floor;
            return n;
        }
        const std::deque<LoopClosureTry> &tries() const { return tries_; }

    private:
        LoopClosureTry register_pair(int i, int j, bool cross_floor) const
        {
            LoopClosureTry t;
            t.i = i;
            t.j = j;
            t.cross_floor = cross_floor;
            // target: the earlier segment with its neighbours
            std::vector<Eigen::Vector3d> target;
            for (int k = std::max(0, i - 1); k <= std::min(j - 1, i + 1); k++)
                target.insert(target.end(), points_[k].begin(), points_[k].end());
            std::vector<Eigen::Vector3d> source = points_[j];
            if (cross_floor)
            {
                // only what both floors see (the shaft): the rest of each segment is ground the other never saw
                const std::vector<Eigen::Vector3d> shared_target = crop_to_shared_footprint(target, source, 0.5);
                source = crop_to_shared_footprint(source, target, 0.5);
                target = shared_target;
            }
            if (source.size() < 500 || target.size() < 500)
                return t;
            t.registered = true;

            const auto fitness_grid = voxel_set(target, 0.2);
            t.overlap_before = overlap(source, Eigen::Affine3d::Identity(), fitness_grid, 0.2);
            Eigen::Affine3d T = coarse_search(source, target, cross_floor ? params_.loop_closure_cross_floor_search_m : params_.loop_closure_max_correction_m);
            T = ndt_refine(source, target, T, params_.use_exact_ndt_bucket_statistics, &t.information);
            t.overlap_after = overlap(source, T, fitness_grid, 0.2);
            // how far the correction moves the segment: its centre's movement (T's translation would also contain the
            // rotation about the world origin, growing with the distance from the start)
            Eigen::Vector3d centre = Eigen::Vector3d::Zero();
            for (const auto &p : source)
                centre += p;
            centre /= static_cast<double>(source.size());
            t.centre = centre;
            t.correction = T;
            t.shift_m = (T * centre - centre).norm();
            t.angle_deg = Eigen::AngleAxisd(T.linear()).angle() / DEG;
            if (cross_floor)
                t.information_ratio = information_ratio(target, 0.2);
            return t;
        }

        std::vector<std::vector<Eigen::Vector3d>> points_;
        const OdometryConfig &params_;
        std::deque<LoopClosureTry> tries_; // a deque: the pointers handed out stay valid while it grows
        std::map<std::tuple<int, int, bool>, size_t> index_;
    };

    // capture_loop_closure_max_tries: per segment and kind, the qualifying candidates not yet registered (nearest first)
    // until that many registrations exist; for replays with rules that would ask for them. Returns how many were added.
    int register_for_capture(const std::vector<LoopClosureSegment> &segments, PointRegistrations &registrations, const OdometryConfig &params)
    {
        const int cap = params.capture_loop_closure_max_tries;
        int added = 0;
        std::vector<RevisitSegment> revisit_segments;
        for (const auto &s : segments)
            revisit_segments.push_back({s.center, s.path_begin, s.path_end});
        for (int j = 0; j < static_cast<int>(segments.size()); j++)
            for (const bool cross_floor : {false, true})
            {
                const std::vector<int> qualifying =
                    cross_floor ? cross_floor_revisits(segments, j, params)
                                : nearest_same_floor_revisits(revisit_segments, j, same_floor_revisits(revisit_segments, j, params));
                int have = registrations.count(j, cross_floor);
                for (const int i : qualifying)
                {
                    if (have >= cap)
                        break;
                    if (registrations.has(i, j, cross_floor))
                        continue;
                    registrations.registration(i, j, cross_floor);
                    have++;
                    added++;
                }
            }
        return added;
    }
} // namespace

int run_loop_closure(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger,
                     LoopClosureCaptureSink *capture)
{
    const std::vector<LoopClosureSegment> segments = loop_closure_segments(windows, params);
    // 1. the segments' world points (the only step besides the registrations that reads points)
    // (none with too few windows: close_loops then registers nothing)
    const bool enough_windows = static_cast<int>(windows.size()) >= 2 * std::max(1, params.loop_closure_segment_windows);
    std::vector<std::vector<Eigen::Vector3d>> points;
    if (enough_windows)
        for (const auto &s : segments)
            points.push_back(segment_points(windows, s, params, point_store));
    PointRegistrations registrations(std::move(points), params);

    LoopClosureCapture captured;
    if (capture)
        captured.windows = windows;
    const int loops = close_loops(windows, segments, registrations, params, logger);
    if (!capture)
        return loops;

    const size_t requested = registrations.tries().size();
    const int extra = params.capture_loop_closure_max_tries > 0 && enough_windows ? register_for_capture(segments, registrations, params) : 0;
    captured.segments = segments;
    for (size_t k = 0; k < registrations.tries().size(); k++)
    {
        const LoopClosureTry &t = registrations.tries()[k];
        captured.tries.push_back({t, loop_try_accepted(t, params), k < requested});
    }
    for (const auto &w : windows)
        captured.result_poses.push_back(w.poses);
    captured.loops = loops;
    std::string why;
    if (capture->write(captured, why))
        log_info(logger) << "loop closure capture: " << captured.segments.size() << " segments, " << captured.tries.size() << " registrations ("
                         << extra << " only for the capture), written";
    else
        log_error(logger) << "loop closure capture not written: " << why;
    return loops;
}
