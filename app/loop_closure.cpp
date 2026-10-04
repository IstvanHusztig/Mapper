#include "loop_closure.h"

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
    Eigen::Affine3d ndt_refine(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, Eigen::Affine3d T,
                               bool exact_ndt_statistics)
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
                    const auto it = buckets.find(get_rgd_index(T * p, b));
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

    struct LoopEdge
    {
        int i = 0;                                       // earlier segment
        int j = 0;                                       // later segment
        Eigen::Affine3d correction = Eigen::Affine3d::Identity(); // world transform moving segment j onto i
        double fitness = 0.0;
    };

    void add_relative_pose_edge(std::vector<Eigen::Triplet<double>> &A, Eigen::VectorXd &b, const std::vector<TaitBryanPose> &poses,
                                int from, int to, const Eigen::Matrix<double, 6, 1> &measurement, double w_t, double w_r)
    {
        const TaitBryanPose &p1 = poses[from];
        const TaitBryanPose &p2 = poses[to];
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

    Eigen::Matrix<double, 6, 1> relative_measurement(const Eigen::Affine3d &a, const Eigen::Affine3d &b)
    {
        const TaitBryanPose p1 = pose_tait_bryan_from_affine_matrix(a);
        const TaitBryanPose p2 = pose_tait_bryan_from_affine_matrix(b);
        Eigen::Matrix<double, 6, 1> m;
        relative_pose_tait_bryan_wc_case1_simplified_1(m, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka);
        return m;
    }

    // Gauss-Newton on the segment poses: odometry edges between consecutive segments, loop edges, first pose fixed
    std::vector<Eigen::Affine3d> optimize_pose_graph(const std::vector<Eigen::Affine3d> &initial, const std::vector<LoopEdge> &loops,
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

        for (int iter = 0; iter < 30; iter++)
        {
            std::vector<Eigen::Triplet<double>> A;
            Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * n);
            for (int k = 0; k + 1 < n; k++)
                add_relative_pose_edge(A, b, poses, k, k + 1, odo[k], w_odo_t, w_odo_r);
            for (size_t e = 0; e < loops.size(); e++)
                add_relative_pose_edge(A, b, poses, loops[e].i, loops[e].j, loop_meas[e], w_loop_t, w_loop_r);
            for (int r = 0; r < 6; r++)
                A.emplace_back(r, r, 1e12); // first segment pose fixed
            Eigen::SparseMatrix<double> AtPA(6 * n, 6 * n);
            AtPA.setFromTriplets(A.begin(), A.end());
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
            if (solver.info() != Eigen::Success)
                break;
            const Eigen::VectorXd dx = solver.solve(b);
            if (solver.info() != Eigen::Success || !dx.allFinite())
                break;
            for (int k = 0; k < n; k++)
            {
                poses[k].px += dx(6 * k + 0);
                poses[k].py += dx(6 * k + 1);
                poses[k].pz += dx(6 * k + 2);
                poses[k].om += dx(6 * k + 3);
                poses[k].fi += dx(6 * k + 4);
                poses[k].ka += dx(6 * k + 5);
            }
            if (dx.norm() < 1e-8)
                break;
        }
        std::vector<Eigen::Affine3d> result;
        for (const auto &p : poses)
            result.push_back(affine_matrix_from_pose_tait_bryan(p));
        return result;
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
    std::vector<LoopEdge> loops;
    for (int j = 0; j < static_cast<int>(segments.size()); j++)
    {
        int best_i = -1;
        double best_d = params.loop_closure_search_radius_m;
        for (int i = 0; i < j; i++)
        {
            if (segments[j].t_begin - segments[i].t_end < params.loop_closure_min_time_gap_s)
                continue;
            const Eigen::Vector3d d = segments[j].center - segments[i].center;
            if (std::abs(d.z()) > params.loop_closure_max_vertical_offset_m)
                continue;
            const double dh = d.head<2>().norm();
            if (dh < best_d)
            {
                best_d = dh;
                best_i = i;
            }
        }
        if (best_i < 0)
            continue;

        // target: the earlier segment with its neighbours
        std::vector<Eigen::Vector3d> target;
        for (int i = std::max(0, best_i - 1); i <= std::min(j - 1, best_i + 1); i++)
            target.insert(target.end(), segments[i].points.begin(), segments[i].points.end());
        const auto &source = segments[j].points;
        if (source.size() < 500 || target.size() < 500)
            continue;

        const auto fitness_grid = voxel_set(target, 0.2);
        const double before = overlap(source, Eigen::Affine3d::Identity(), fitness_grid, 0.2);
        Eigen::Affine3d T = coarse_search(source, target, params.loop_closure_max_correction_m);
        T = ndt_refine(source, target, T, params.use_exact_ndt_bucket_statistics);
        const double after = overlap(source, T, fitness_grid, 0.2);
        const double shift = T.translation().norm();
        const double angle = Eigen::AngleAxisd(T.linear()).angle() / DEG;
        // an already aligned revisit (no improvement possible) is a valid constraint as well
        const bool accepted = after >= params.loop_closure_min_fitness && after >= before - 0.02 &&
                              shift <= params.loop_closure_max_correction_m + 1.0 && angle <= 15.0;
        log_info(logger) << "loop candidate segments " << best_i << " -> " << j << " (t " << std::fixed << std::setprecision(1)
                         << segments[best_i].t_begin - segments[0].t_begin << " s / " << segments[j].t_begin - segments[0].t_begin
                         << " s): overlap " << std::setprecision(2) << before << " -> " << after << ", correction " << shift << " m "
                         << std::setprecision(1) << angle << " deg" << (accepted ? " ACCEPTED" : " rejected");
        if (accepted)
        {
            LoopEdge e;
            e.i = best_i;
            e.j = j;
            e.correction = T;
            e.fitness = after;
            loops.push_back(e);
        }
    }

    if (loops.empty())
    {
        log_info(logger) << "loop closure: no loops accepted, trajectory unchanged";
        return 0;
    }

    // 4. pose graph over the segment poses
    std::vector<Eigen::Affine3d> initial;
    for (const auto &s : segments)
        initial.push_back(s.pose);
    std::vector<Eigen::Affine3d> optimized = optimize_pose_graph(initial, loops, params);

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
    double max_shift = 0.0;
    for (const auto &c : correction)
        max_shift = std::max(max_shift, c.translation().norm());
    log_info(logger) << "loop closure: " << loops.size() << " loop(s) accepted, largest segment correction " << std::fixed << std::setprecision(2)
                     << max_shift << " m";
    return static_cast<int>(loops.size());
}
