#include "window_term_table.h"
#include "accelerometer_term.h"
#include "angle_units.h"
#include "motion_model_terms.h"
#include "ndt_term.h"
#include "window_equations.h"
#include "window_solver.h"
#include <transformations.h>

#include <cmath>
#include <exception>
#include <limits>

namespace
{
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kTurnLimitDeg = 87.0; // within 3 deg of fi = +-90 deg the angle Jacobian has no usable inverse

    // tau_k = time since the first node / duration (node index / (n - 1) without timestamps)
    std::vector<double> node_tau(const Window &window, double &duration)
    {
        const size_t n = window.poses.size();
        std::vector<double> tau(n, 0.0);
        duration = 0.0;
        if (n < 2)
            return tau;
        if (window.timestamps.size() == n)
            duration = window.timestamps.back().first - window.timestamps.front().first;
        for (size_t k = 0; k < n; k++)
            tau[k] = duration > 0.0 ? (window.timestamps[k].first - window.timestamps.front().first) / duration : double(k) / double(n - 1);
        return tau;
    }
}

std::vector<Eigen::VectorXd> window_motion_vectors(const Window &window, const WindowUnknowns &u, const std::array<Eigen::Vector3d, kTermDirections> &dirs,
                                                   bool &turn_defined)
{
    double duration = 0.0;
    const std::vector<double> tau = node_tau(window, duration);
    const int n = static_cast<int>(window.poses.size());
    std::vector<Eigen::Matrix3d> inverse_jacobian(n);
    turn_defined = true;
    for (int k = 0; k < n; k++)
    {
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(window.poses[k]);
        if (std::abs(p.fi) * RAD_TO_DEG > kTurnLimitDeg)
            turn_defined = false;
        else
            inverse_jacobian[k] = tait_bryan_rotation_jacobian(window.poses[k]).inverse();
    }
    std::vector<Eigen::VectorXd> m(kTermMotions * kTermDirections, Eigen::VectorXd::Zero(u.n_params));
    for (int d = 0; d < kTermDirections; d++)
    {
        const Eigen::Vector3d &dir = dirs[d];
        Eigen::VectorXd &shift = m[0 * 3 + d], &stretch = m[1 * 3 + d], &curve = m[2 * 3 + d], &turn = m[3 * 3 + d];
        for (int k = 0; k < n; k++)
        {
            shift.segment<3>(6 * k) = dir;
            stretch.segment<3>(6 * k) = tau[k] * dir;
            curve.segment<3>(6 * k) = tau[k] * tau[k] * dir;
            if (turn_defined)
                turn.segment<3>(6 * k + 3) = inverse_jacobian[k] * (dir * (DEG_TO_RAD * tau[k]));
        }
        if (u.acc_term && duration > 0.0)
        {
            stretch.segment<3>(u.iv) = dir / duration;
            stretch.segment<3>(u.ie) = dir / duration;
            curve.segment<3>(u.ie) = 2.0 * dir / duration;
        }
    }
    return m;
}

namespace
{
    struct Reducer
    {
        const std::vector<Eigen::VectorXd> &m;

        // weight and pull of one term's normal equations along every motion
        WindowTermRow row(const std::string &name, const WindowEquations &e, double cost, int residuals, bool in_total = true) const
        {
            WindowTermRow r;
            r.name = name;
            r.in_total = in_total;
            r.cost = cost;
            r.residuals = residuals;
            const Eigen::SparseMatrix<double> H = e.matrix.to_sparse();
            std::array<double, kTermMotions * kTermDirections> w{}, c{};
            double largest = 0.0;
            for (int i = 0; i < kTermMotions * kTermDirections; i++)
            {
                w[i] = m[i].dot(H * m[i]);
                c[i] = m[i].dot(e.rhs.col(0));
                largest = std::max(largest, w[i]);
            }
            // a weight below 1e-9 of the row's largest is rounding (a motion the term doesn't see): no pull
            for (int i = 0; i < kTermMotions * kTermDirections; i++)
            {
                r.weight[i / 3][i % 3] = w[i];
                r.pull[i / 3][i % 3] = w[i] > 1e-9 * largest ? c[i] / w[i] : kNaN;
            }
            return r;
        }
    };

    WindowTermRow total_row(const std::vector<WindowTermRow> &rows)
    {
        WindowTermRow t;
        t.name = "total";
        t.in_total = false;
        std::array<std::array<double, kTermDirections>, kTermMotions> c{};
        for (const WindowTermRow &r : rows)
        {
            if (!r.in_total)
                continue;
            t.cost += r.cost;
            t.residuals += r.residuals;
            for (int a = 0; a < kTermMotions; a++)
                for (int d = 0; d < kTermDirections; d++)
                {
                    t.weight[a][d] += r.weight[a][d];
                    if (std::isfinite(r.pull[a][d]))
                        c[a][d] += r.pull[a][d] * r.weight[a][d];
                }
        }
        for (int a = 0; a < kTermMotions; a++)
            for (int d = 0; d < kTermDirections; d++)
                t.pull[a][d] = t.weight[a][d] > 0.0 ? c[a][d] / t.weight[a][d] : kNaN;
        return t;
    }

    // the damping as a weight: lm_factor on every pose parameter
    WindowTermRow damping_row(const std::vector<Eigen::VectorXd> &m, int n_pose_params, double lm_factor)
    {
        WindowTermRow r;
        r.name = "damping";
        r.in_total = false;
        for (int i = 0; i < kTermMotions * kTermDirections; i++)
        {
            r.weight[i / 3][i % 3] = lm_factor * m[i].head(n_pose_params).squaredNorm();
            r.pull[i / 3][i % 3] = kNaN;
        }
        return r;
    }

    NdtProjection no_projection()
    {
        NdtProjection p;
        p.M = Eigen::Matrix<double, 6, 6>::Identity();
        p.all_kept = true;
        return p;
    }

    double sum_of(const std::vector<double> &v)
    {
        double s = 0.0;
        for (double x : v)
            s += x;
        return s;
    }

    int sum_of(const std::vector<int> &v)
    {
        int s = 0;
        for (int x : v)
            s += x;
        return s;
    }

    // ndt_unprojected split by each cell's information: along the cell's normal (ndt_normal) and the rest (ndt_in_plane),
    // both grids, before the projection; the two rows' weights and weight * pull add up to ndt_unprojected's
    void add_ndt_part_rows(const WindowTermInputs &in, const WindowNoiseModel &noise, const Reducer &reduce, std::vector<WindowTermRow> &rows)
    {
        const ScanToMapOptions &o = in.scan_to_map;
        for (const auto &[name, part] : {std::pair<const char *, NdtInformationPart>{"ndt_normal", NdtInformationPart::normal},
                                         {"ndt_in_plane", NdtInformationPart::in_plane}})
        {
            WindowEquations e(noise.unknowns);
            NdtGrid indoor_grid = indoor_ndt_grid(in.buckets_indoor, o, noise.ndt);
            indoor_grid.part = part;
            const NdtSums indoor = sum_ndt_grid_blocks(indoor_grid, in.window, in.points, o, true);
            add_ndt_blocks(e, indoor, no_projection());
            double cost = sum_of(indoor.node_cost);
            int matched = sum_of(indoor.node_matched);
            if (o.use_hierarchical_rgd)
            {
                NdtGrid outdoor_grid = outdoor_ndt_grid(in.buckets_outdoor, o, noise.ndt);
                outdoor_grid.part = part;
                const NdtSums outdoor = sum_ndt_grid_blocks(outdoor_grid, in.window, in.points, o, true);
                add_ndt_blocks(e, outdoor, no_projection());
                cost += sum_of(outdoor.node_cost);
                matched += sum_of(outdoor.node_matched);
            }
            rows.push_back(reduce.row(name, e, cost, 3 * matched, false));
        }
    }

    void add_ndt_rows(const WindowTermInputs &in, const WindowNoiseModel &noise, const Reducer &reduce, std::vector<WindowTermRow> &rows,
                      WindowTermTable &table)
    {
        const WindowUnknowns &u = noise.unknowns;
        const ScanToMapOptions &o = in.scan_to_map;
        const NdtSums indoor = sum_ndt_grid_blocks(indoor_ndt_grid(in.buckets_indoor, o, noise.ndt), in.window, in.points, o, true);
        NdtSums outdoor;
        if (o.use_hierarchical_rgd)
            outdoor = sum_ndt_grid_blocks(outdoor_ndt_grid(in.buckets_outdoor, o, noise.ndt), in.window, in.points, o, true);
        // the solver's projection: from the indoor grid's normals, for both grids
        const NdtProjection projection = noise.ndt.projection_ratio_full > 0.0
                                             ? ndt_degeneracy_projection(window_translation_information(indoor), noise.ndt)
                                             : no_projection();
        table.translation_information = window_translation_information(indoor);
        table.ndt_matched = sum_of(indoor.node_matched) + sum_of(outdoor.node_matched);
        table.ndt_unmatched = sum_of(indoor.node_unmatched) + sum_of(outdoor.node_unmatched);

        WindowEquations e_in(u);
        add_ndt_blocks(e_in, indoor, projection);
        rows.push_back(reduce.row("ndt_indoor", e_in, sum_of(indoor.node_cost), 3 * sum_of(indoor.node_matched)));
        if (o.use_hierarchical_rgd)
        {
            WindowEquations e_out(u);
            add_ndt_blocks(e_out, outdoor, projection);
            rows.push_back(reduce.row("ndt_outdoor", e_out, sum_of(outdoor.node_cost), 3 * sum_of(outdoor.node_matched)));
        }
        WindowEquations e_raw(u);
        add_ndt_blocks(e_raw, indoor, no_projection());
        if (o.use_hierarchical_rgd)
            add_ndt_blocks(e_raw, outdoor, no_projection());
        rows.push_back(reduce.row("ndt_unprojected", e_raw, sum_of(indoor.node_cost) + sum_of(outdoor.node_cost),
                                  3 * (sum_of(indoor.node_matched) + sum_of(outdoor.node_matched)), false));
        add_ndt_part_rows(in, noise, reduce, rows);
    }

    void add_edge_rows(const WindowTermInputs &in, const WindowNoiseModel &noise, const Reducer &reduce, std::vector<WindowTermRow> &rows)
    {
        if (in.window.poses.size() < 2)
            return;
        const WindowUnknowns &u = noise.unknowns;
        const int edges = static_cast<int>(in.window.poses.size()) - 1;
        const MotionEdgeWeights &weights = noise.edges;
        MotionEdgeWeights position = weights;
        for (Eigen::Vector3d &r : position.rotation)
            r.setZero();
        MotionEdgeWeights rotation = weights;
        rotation.x = rotation.y = rotation.z = 0.0;
        for (const auto &[name, w] : {std::pair<const char *, const MotionEdgeWeights *>{"edge_position", &position}, {"edge_rotation", &rotation}})
        {
            WindowEquations e(u);
            e.with_cost = true;
            add_motion_edges(e, u, in.window, in.motion_model, in.rotation_options, in.translation_imu_options, in.acc_offset, *w);
            rows.push_back(reduce.row(name, e, e.motion_cost, 3 * edges));
        }
    }

    // a motion or IMU term on its own equations: a row when it added a residual
    template <typename Add>
    void add_term_row(const char *name, const WindowUnknowns &u, const Reducer &reduce, std::vector<WindowTermRow> &rows, Add add)
    {
        WindowEquations e(u);
        e.with_cost = true;
        add(e);
        if (e.residuals > 0)
            rows.push_back(reduce.row(name, e, e.motion_cost, e.residuals));
    }

    std::vector<WindowTermRow> term_rows(const WindowTermInputs &in, const WindowNoiseModel &noise, const std::vector<Eigen::VectorXd> &m,
                                         WindowTermTable &table)
    {
        const WindowUnknowns &u = noise.unknowns;
        const Reducer reduce{m};
        std::vector<WindowTermRow> rows;
        add_ndt_rows(in, noise, reduce, rows, table);
        add_edge_rows(in, noise, reduce, rows);
        const Window &w = in.window;
        add_term_row("gyro_bias_prior", u, reduce, rows, [&](WindowEquations &e) { add_gyro_bias_prior(e, noise, w); });
        add_term_row("velocity_continuity", u, reduce, rows,
                     [&](WindowEquations &e) { add_velocity_continuity(e, noise, w); });
        bool end_tied = false;
        add_term_row("accelerometer", u, reduce, rows,
                     [&](WindowEquations &e) { end_tied = add_acceleration_terms(e, noise, w, in.translation_imu_options, in.acc_offset); });
        add_term_row("velocity_prior", u, reduce, rows, [&](WindowEquations &e) { add_start_velocity_prior(e, noise, w); });
        add_term_row("zero_velocity", u, reduce, rows, [&](WindowEquations &e) { add_zero_velocity(e, noise, w); });
        add_term_row("end_velocity_tie", u, reduce, rows, [&](WindowEquations &e) { add_weak_end_velocity_tie(e, noise, w, end_tied); });
        add_term_row("offset_prior", u, reduce, rows,
                     [&](WindowEquations &e) { add_acceleration_offset_prior(e, noise, in.acc_offset); });
        WindowEquations origin(u);
        fix_origin(origin, noise);
        rows.push_back(reduce.row("origin_fix", origin, 0.0, 6));
        return rows;
    }
}

const char *term_motion_name(int motion)
{
    static const char *names[kTermMotions] = {"shift", "stretch", "curve", "turn"};
    return names[motion];
}

const char *term_direction_name(int direction)
{
    static const char *names[kTermDirections] = {"along", "sideways", "vertical"};
    return names[direction];
}

Eigen::Vector3d window_travel_direction(const std::vector<Eigen::Affine3d> &poses, const Eigen::Vector3d &fallback)
{
    if (poses.size() < 2)
        return fallback;
    Eigen::Vector3d d = poses.back().translation() - poses.front().translation();
    d.z() = 0.0;
    return d.norm() < 0.01 ? fallback : Eigen::Vector3d(d.normalized());
}

WindowTermTable window_term_table(const WindowTermInputs &in)
{
    WindowTermTable table;
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    table.directions = {in.along, up.cross(in.along).normalized(), up};
    try
    {
        const WindowNoiseModel &noise = in.noise;
        const WindowUnknowns &u = noise.unknowns;
        const std::vector<Eigen::VectorXd> m = window_motion_vectors(in.window, u, table.directions, table.turn_defined);
        table.rows = term_rows(in, noise, m, table);
        table.rows.push_back(total_row(table.rows));
        table.rows.push_back(damping_row(m, u.n_pose_params, in.lm_factor));
        if (!table.turn_defined)
            for (WindowTermRow &r : table.rows)
                for (int d = 0; d < kTermDirections; d++)
                    r.weight[3][d] = r.pull[3][d] = kNaN;
    }
    catch (const std::exception &e)
    {
        table.rows.clear();
        table.failure = e.what();
    }
    return table;
}

std::array<std::array<double, kTermDirections>, kTermMotions> window_motion_change(const Window &from, const Window &to,
                                                                                 const std::array<Eigen::Vector3d, kTermDirections> &dirs)
{
    std::array<std::array<double, kTermDirections>, kTermMotions> change;
    for (auto &a : change)
        a.fill(kNaN);
    const size_t n = from.poses.size();
    if (n == 0 || to.poses.size() != n)
        return change;
    double duration = 0.0;
    const std::vector<double> tau = node_tau(to, duration);
    const Eigen::Vector3d d0 = to.poses[0].translation() - from.poses[0].translation();
    // least squares of y_k = a tau_k + c tau_k^2
    double s2 = 0.0, s3 = 0.0, s4 = 0.0;
    for (double t : tau)
    {
        s2 += t * t;
        s3 += t * t * t;
        s4 += t * t * t * t;
    }
    const double det = s2 * s4 - s3 * s3;
    const auto rotation_change = [&](size_t k)
    {
        const Eigen::AngleAxisd aa(to.poses[k].linear() * from.poses[k].linear().transpose());
        return Eigen::Vector3d(aa.axis() * aa.angle());
    };
    const Eigen::Vector3d turn = (rotation_change(n - 1) - rotation_change(0)) * RAD_TO_DEG;
    for (int d = 0; d < kTermDirections; d++)
    {
        change[0][d] = d0.dot(dirs[d]);
        double ty = 0.0, t2y = 0.0;
        for (size_t k = 0; k < n; k++)
        {
            const double y = (to.poses[k].translation() - from.poses[k].translation() - d0).dot(dirs[d]);
            ty += tau[k] * y;
            t2y += tau[k] * tau[k] * y;
        }
        if (std::abs(det) > 1e-12)
        {
            change[1][d] = (s4 * ty - s3 * t2y) / det;
            change[2][d] = (s2 * t2y - s3 * ty) / det;
        }
        change[3][d] = turn.dot(dirs[d]);
    }
    return change;
}

std::optional<double> mean_rotation_rate(const std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &timestamps)
{
    if (poses.size() < 2 || poses.size() != timestamps.size())
        return std::nullopt;
    const double duration = timestamps.back().first - timestamps.front().first;
    if (!(duration > 0.0))
        return std::nullopt;
    double angle = 0.0;
    for (size_t k = 0; k + 1 < poses.size(); k++)
        angle += Eigen::AngleAxisd(poses[k].linear().transpose() * poses[k + 1].linear()).angle();
    return angle / duration;
}

std::optional<Eigen::Vector3d> start_world_acceleration(const std::vector<Eigen::Affine3d> &poses,
                                                        const std::vector<std::pair<double, double>> &timestamps,
                                                        const std::vector<Eigen::Vector3d> &imu_acc, const Eigen::Vector3d &gravity,
                                                        double span_s)
{
    if (poses.empty() || poses.size() != timestamps.size() || poses.size() != imu_acc.size())
        return std::nullopt;
    const double g = gravity.norm();
    if (!(g > 0.0) || !std::isfinite(g))
        return std::nullopt;
    const double to_mps2 = 9.80665 / g;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int n = 0;
    for (size_t k = 0; k < poses.size() && (k == 0 || timestamps[k].first - timestamps.front().first <= span_s); k++, n++)
        sum += (poses[k].linear() * imu_acc[k] - gravity) * to_mps2;
    return Eigen::Vector3d(sum / double(n));
}
