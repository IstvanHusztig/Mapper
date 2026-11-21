#include "synthetic_loop.h"

#include "ndt_map.h"
#include "run_odometry.h"
#include "toml_config_repository.h"
#include "truth_statistics.h"
#include "window_capture.h"
#include "window_steps.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace synthetic
{
namespace
{
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    class NoPreview : public PreviewSink
    {
    public:
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &, const std::vector<double> &) override {}
        std::string location() const override { return "none"; }
    };

    class NoProgress : public ProgressReporter
    {
    public:
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    class KeepTerms : public WindowTermSink
    {
    public:
        void write(const WindowTermRecord &record) override { records.push_back(record); }
        std::vector<WindowTermRecord> records;
    };

    // keeps the indoor map as it was before the captured range
    class KeepMapBefore : public WindowCaptureSink
    {
    public:
        bool write(const WindowCapture &capture, std::string &) override
        {
            buckets_indoor = capture.state.buckets_indoor;
            first = capture.first;
            kept = true;
            return true;
        }
        NDTBucketMapType buckets_indoor;
        std::size_t first = 0;
        bool kept = false;
    };

    // the loop's own lines that a precondition reads: the map rebuilds (finish_run's "silent events" line, and each
    // rebuild's warning); everything else is dropped
    class RunLog : public Logger
    {
    public:
        void write(LogLevel, std::string_view message) override
        {
            if (message.find("reference map rebuilt") != std::string_view::npos)
                rebuild_warnings++;
            const std::string_view key = "silent events: map rebuilds ";
            const std::size_t at = message.find(key);
            if (at != std::string_view::npos)
            {
                std::size_t i = at + key.size();
                int n = 0;
                bool digits = false;
                while (i < message.size() && message[i] >= '0' && message[i] <= '9')
                {
                    n = n * 10 + (message[i] - '0');
                    digits = true;
                    i++;
                }
                if (digits)
                    map_rebuilds = n;
            }
        }
        int map_rebuilds = -1;
        int rebuild_warnings = 0;
    };

    double heading_of(const Eigen::Matrix3d &r) { return std::atan2(r(1, 0), r(0, 0)); }

    double wrapped(double a)
    {
        const double pi = 3.14159265358979323846;
        while (a > pi)
            a -= 2.0 * pi;
        while (a < -pi)
            a += 2.0 * pi;
        return a;
    }

    // the route's heading at motion time t without the gait's yaw twist: the body heading averaged over one twist period
    // (2 / step_hz, the twist goes at half the step rate), centred on t and clamped to the motion
    double route_heading(const InertialRecording &rec, double t)
    {
        const Motion &m = rec.route.motion;
        const double period = 2.0 / m.gait().step_hz;
        const double t0 = std::clamp(t - 0.5 * period, 0.0, std::max(0.0, m.duration() - period));
        const double h0 = heading_of(m.state(t0).rotation);
        double sum = 0.0;
        constexpr int samples = 50;
        for (int i = 0; i < samples; i++)
            sum += wrapped(heading_of(m.state(t0 + (i + 0.5) * period / samples).rotation) - h0);
        return h0 + sum / samples;
    }

    Eigen::Affine3d truth_pose(const InertialRecording &rec, int node)
    {
        const MotionState m = rec.imu.truth_at_node(node).motion;
        Eigen::Affine3d p = Eigen::Affine3d::Identity();
        p.linear() = m.rotation;
        p.translation() = m.position;
        return p;
    }

    Eigen::Vector3d cell_size(const OdometryConfig &c)
    {
        return Eigen::Vector3d(c.in_out_params_indoor.resolution_X, c.in_out_params_indoor.resolution_Y, c.in_out_params_indoor.resolution_Z);
    }

    // a point's match as the NDT term accepts it (ndt_term.cpp, matched_cell and add_ndt_point_term): in range, a cell with a
    // usable inverse, seen from its front (with the view point test on)
    const NdtBucket *accepted_cell(const NDTBucketMapType &buckets, const Point3Di &p, const Eigen::Affine3d &node_pose, const OdometryConfig &c,
                                   std::uint64_t &key)
    {
        const double r = p.point.norm();
        if (r < c.min_distance_lidar || r > c.max_distance_lidar)
            return nullptr;
        const Eigen::Vector3d g = node_pose * p.point;
        key = ndt_cell_index(g, cell_size(c));
        const auto it = buckets.find(key);
        if (it == buckets.end())
            return nullptr;
        Eigen::Matrix3d scratch;
        if (ndt_cell_inverse(it->second, scratch) == nullptr)
            return nullptr;
        if (c.ablation_study_use_view_point_and_normal_vectors && it->second.normal_vector.dot(node_pose.translation() - it->second.mean) < 0)
            return nullptr;
        return &it->second;
    }

    void add(LoopRun &run, const std::string &name, int window, double value, double tolerance, bool at_least)
    {
        Precondition p;
        p.name = name;
        p.window = window;
        p.value = value;
        p.tolerance = tolerance;
        p.at_least = at_least;
        p.holds = std::isfinite(value) && (at_least ? value >= tolerance : value <= tolerance);
        run.preconditions.push_back(p);
    }

    // the window index (0-based) of a recording node, -1 before window 0 or beyond the last
    int window_of_node(const InertialRecording &rec, int node, int n_windows)
    {
        if (node < rec.window_origin_node)
            return -1;
        const int w = (node - rec.window_origin_node) / rec.sensor.nodes_per_window;
        return w < n_windows ? w : -1;
    }

    // R2's geometry (synthetic_routes.cpp, route_r2): room A x < 0, the corridor [0, 20], room B x > 20, all moved by grid_offset
    bool in_room_a(const Eigen::Vector3d &p) { return p.x() < grid_offset.x() + 1e-6; }
    bool in_room_b(const Eigen::Vector3d &p) { return p.x() > 20.0 + grid_offset.x() - 1e-6; }

    // the R2 design from the generator's hits and the truth alone (before the run): the turn, the exits, the landmark cells
    void design_r2(LoopRun &run, const InertialRecording &rec, const Eigen::Affine3d &align, const OdometryConfig &c, const LoopTolerances &tol,
                   std::vector<int> &exit_windows)
    {
        R2Design &d = run.r2;
        d.present = true;
        const int n = static_cast<int>(run.windows.size());
        for (int w = 0; w < n && d.turn_window == 0; w++)
            if (std::cos(heading_of(align.linear().transpose() * run.windows[w].truth.poses.front().linear())) < -0.5)
                d.turn_window = w + 1;
        // x-facing face hits per window in room A and in room B
        std::vector<int> end_a(n, 0), end_b(n, 0);
        for (const Hit &h : rec.hits)
        {
            const int w = window_of_node(rec, h.node, n);
            if (w < 0 || h.face < 0 || rec.route.scene.faces[h.face].axis != 0)
                continue;
            if (in_room_a(h.latent_world))
                end_a[w]++;
            if (in_room_b(h.latent_world))
                end_b[w]++;
        }
        const auto blind = [&](int w) { return run.windows[w].truth.x_min >= rec.route.blind_x_lo && run.windows[w].truth.x_max <= rec.route.blind_x_hi; };
        // the outbound exit: the first window after an outbound blind window with enough hits on room B's end walls
        const int return_first = d.turn_window > 0 ? d.turn_window - 1 : n;
        bool was_blind = false;
        for (int w = 0; w < return_first; w++)
        {
            was_blind = was_blind || blind(w);
            if (was_blind && !blind(w) && end_b[w] >= tol.exit_min_end_wall_hits)
            {
                exit_windows.push_back(w + 1);
                break;
            }
        }
        was_blind = false;
        for (int w = return_first; w < n; w++)
        {
            was_blind = was_blind || blind(w);
            if (was_blind && !blind(w) && end_a[w] >= tol.exit_min_end_wall_hits)
            {
                d.exit_window = w + 1;
                d.exit_end_wall_hits = end_a[w];
                exit_windows.push_back(w + 1);
                break;
            }
        }
        if (d.exit_window == 0)
            return;
        // the landmark cells: room A's cells hit outbound, minus every cell the return leg hit before the exit window
        const Eigen::Vector3d b = cell_size(c);
        std::set<std::uint64_t> outbound, returned;
        for (const Hit &h : rec.hits)
        {
            const int w = window_of_node(rec, h.node, n);
            if (w < 0)
                continue;
            const std::uint64_t key = ndt_cell_index(align * h.latent_world, b);
            if (w < return_first && in_room_a(h.latent_world))
                outbound.insert(key);
            else if (w >= return_first && w < d.exit_window - 1)
                returned.insert(key);
        }
        for (const std::uint64_t key : outbound)
            if (!returned.count(key))
                d.landmark_cells.insert(key);
    }

    WindowClass classify(const LoopWindow &w, const InertialRecording &rec, const std::vector<int> &exits, const LoopTolerances &tol)
    {
        const double t0 = rec.imu.node_time(w.truth.first_node);
        const double t1 = rec.imu.node_time(w.truth.first_node + static_cast<int>(w.truth.poses.size()) - 1);
        if (rec.route.motion.at_rest(t0, t1))
            return WindowClass::rest;
        if (w.truth.duration_s > 0.0 && std::abs(w.truth.heading_change_rad) / w.truth.duration_s > tol.turn_rate_rad_s)
            return WindowClass::turn;
        if (std::find(exits.begin(), exits.end(), w.window) != exits.end())
            return WindowClass::exit;
        if (rec.route.blind_x_hi > rec.route.blind_x_lo && w.truth.x_min >= rec.route.blind_x_lo && w.truth.x_max <= rec.route.blind_x_hi)
            return WindowClass::blind;
        return WindowClass::seen;
    }

    double projected_ndt_weight(const WindowTermTable &t, int direction)
    {
        double s = 0.0;
        for (const WindowTermRow &row : t.rows)
            if (row.name == "ndt_indoor" || row.name == "ndt_outdoor")
                s += row.weight[0][direction];
        return s;
    }

    void blind_preconditions(LoopRun &run, const LoopWindow &w, const Eigen::Affine3d &align, const LoopTolerances &tol)
    {
        const std::string at = " (blind)";
        if (!w.record || !w.record->result.failure.empty())
        {
            add(run, "blind window has its result table" + at, w.window, 0.0, 1.0, true);
            return;
        }
        const WindowTermTable &t = w.record->result;
        const Eigen::Matrix3d &I = t.translation_information;
        const double lmax = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(I, Eigen::EigenvaluesOnly).eigenvalues().maxCoeff();
        const Eigen::Vector3d axis = align.linear() * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d side = align.linear() * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d up = align.linear() * Eigen::Vector3d::UnitZ();
        const double share = [&](const Eigen::Vector3d &u) { return lmax > 0.0 ? u.dot(I * u) / lmax : nan; }(axis);
        add(run, "along-axis information share" + at, w.window, share, tol.blind_max_along_share, false);
        const double along_w = projected_ndt_weight(t, 0), side_w = projected_ndt_weight(t, 1);
        add(run, "projected NDT weight along / sideways" + at, w.window, side_w > 0.0 ? along_w / side_w : nan, tol.blind_max_projected_along, false);
        add(run, "sideways information share" + at, w.window, lmax > 0.0 ? side.dot(I * side) / lmax : nan, tol.blind_min_side_share, true);
        add(run, "vertical information share" + at, w.window, lmax > 0.0 ? up.dot(I * up) / lmax : nan, tol.blind_min_up_share, true);
        add(run, "matched side normals" + at, w.window, side.dot(I * side), tol.blind_min_side_normals, true);
        add(run, "matched floor normals" + at, w.window, up.dot(I * up), tol.blind_min_floor_normals, true);
        const double c = std::min(1.0, std::abs(t.directions[0].dot(axis)));
        add(run, "travel direction against the corridor axis [deg]" + at, w.window, std::acos(c) * 180.0 / 3.14159265358979323846, tol.blind_max_travel_angle_deg, false);
    }

    void exit_preconditions(LoopRun &run, Cut &cut, const KeepMapBefore &map_before, const Eigen::Affine3d &align, const LoopTolerances &tol)
    {
        R2Design &d = run.r2;
        const int e = d.exit_window;
        const std::string at = " (exit)";
        add(run, "designated landmark cells", e, double(d.landmark_cells.size()), double(tol.min_landmark_cells), true);
        if (!map_before.kept || map_before.first != std::size_t(e - 1))
        {
            add(run, "map before the exit window kept", e, 0.0, 1.0, true);
            return;
        }
        for (const std::uint64_t key : d.landmark_cells)
            d.landmark_cells_mapped += map_before.buckets_indoor.count(key) ? 1 : 0;
        add(run, "landmark cells in the map before the exit (share)", e,
            d.landmark_cells.empty() ? nan : double(d.landmark_cells_mapped) / double(d.landmark_cells.size()), tol.min_landmark_cells_mapped, true);
        std::vector<Point3Di> points;
        const LoopWindow &w = run.windows[e - 1];
        if (!cut.store.load(PointSet::WindowIntermediate, std::size_t(e - 1), points))
        {
            add(run, "exit window's points readable", e, 0.0, 1.0, true);
            return;
        }
        for (const Point3Di &p : points)
        {
            if (p.index_pose < 0 || p.index_pose >= static_cast<int>(w.poses.size()))
                continue;
            const double r = p.point.norm();
            if (r < cut.config.min_distance_lidar || r > cut.config.max_distance_lidar)
                continue;
            d.exit_points++;
            std::uint64_t key = 0;
            const bool accepted = accepted_cell(map_before.buckets_indoor, p, w.poses[p.index_pose], cut.config, key) != nullptr;
            key = ndt_cell_index(w.poses[p.index_pose] * p.point, cell_size(cut.config));
            const bool landmark = d.landmark_cells.count(key) > 0;
            d.exit_points_in_landmark_keys += landmark;
            d.exit_points_accepted += accepted;
            d.exit_landmark_matches += accepted && landmark;
        }
        add(run, "accepted matches to the landmark cells" + at, e, double(d.exit_landmark_matches), double(tol.exit_min_landmark_matches), true);
        if (!w.record || !w.record->result.failure.empty())
        {
            add(run, "exit window has its result table" + at, e, 0.0, 1.0, true);
            return;
        }
        const Eigen::Matrix3d &I = w.record->result.translation_information;
        const double lmax = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(I, Eigen::EigenvaluesOnly).eigenvalues().maxCoeff();
        const Eigen::Vector3d axis = align.linear() * Eigen::Vector3d::UnitX();
        add(run, "along-axis information share (full support)" + at, e, lmax > 0.0 ? axis.dot(I * axis) / lmax : nan,
            cut.config.degenerate_projection_ratio_full, true);
    }

    void count_unavailable(LoopRun &run)
    {
        Unavailable &u = run.unavailable;
        for (const LoopWindow &w : run.windows)
        {
            if (!w.record)
            {
                u.records_missing++;
                continue;
            }
            const WindowTermRecord &r = *w.record;
            const WindowEvidence &e = r.verdict.evidence;
            u.handoff_not_done += r.handoff != WindowHandOff::done;
            u.heading_bend += !e.heading_bend_deg;
            u.speed += !e.speed_mps;
            u.max_bias += !e.max_bias_deg_s;
            u.velocity_chi2 += !e.velocity_chi2;
            u.information_ratio += !e.information_ratio;
            u.remaining_step += !e.remaining_step_m;
            u.lidar_share += !e.lidar_share_along;
            u.bent_check += r.verdict.bent == Check::not_measured;
            u.fast_check += r.verdict.fast == Check::not_measured;
            u.bias_check += r.verdict.bias_high == Check::not_measured;
            const NormalisedMove *moves[3] = {&e.moves.start_velocity, &e.moves.joint, &e.moves.gyro_bias};
            for (int m = 0; m < 3; m++)
                if (!moves[m]->value)
                    u.move_reasons[m][int(moves[m]->reason)]++;
            u.move_velocity += !e.moves.start_velocity.value;
            u.move_joint += !e.moves.joint.value;
            u.move_bias += !e.moves.gyro_bias.value;
            u.rotation_rate += !r.rotation_rate_rad_s;
            u.start_acceleration += !r.start_acceleration_mps2;
            u.velocity_end_normalised += !w.errors.velocity_end_normalised;
        }
    }

    void push(std::vector<double> &v, const Eigen::Vector3d &x)
    {
        v.insert(v.end(), {x.x(), x.y(), x.z()});
    }

    // the declared deterministic fields of a run, flattened, with a name per value
    void deterministic_fields(const LoopRun &run, std::vector<double> &values, std::vector<std::string> &names)
    {
        const auto mark = [&](const std::string &name)
        {
            while (names.size() < values.size())
                names.push_back(name);
        };
        values.push_back(double(run.windows.size()));
        mark("window count");
        for (const LoopWindow &w : run.windows)
        {
            const std::string at = "window " + std::to_string(w.window) + ": ";
            for (const Eigen::Affine3d &p : w.poses)
            {
                push(values, p.translation());
                for (int i = 0; i < 9; i++)
                    values.push_back(p.linear().data()[i]);
            }
            mark(at + "solved poses");
            if (!w.record)
            {
                values.push_back(nan);
                mark(at + "record");
                continue;
            }
            const WindowTermRecord &r = *w.record;
            values.insert(values.end(), {double(r.iterations), r.delta, r.lm_factor, double(int(r.verdict.evidence.stop))});
            mark(at + "iterations, delta, damping, stop");
            push(values, r.velocity_start);
            push(values, r.velocity_end);
            mark(at + "velocity start / end");
            push(values, r.gyro_bias_deg_s);
            push(values, r.acc_offset_mps2);
            mark(at + "gyro bias / acceleration offset");
            values.push_back(double(int(r.handoff)));
            if (r.handed_on.velocity_end)
                push(values, *r.handed_on.velocity_end);
            if (r.handed_on.bias_mean_rad_s)
                push(values, *r.handed_on.bias_mean_rad_s);
            mark(at + "handed-on velocity / bias");
            for (const NormalisedMove *m : {&r.verdict.evidence.moves.start_velocity, &r.verdict.evidence.moves.joint, &r.verdict.evidence.moves.gyro_bias})
                values.push_back(m->value ? *m->value : -1.0 - double(int(m->reason)));
            mark(at + "normalised moves");
        }
        for (const Precondition &p : run.preconditions)
        {
            values.push_back(p.value);
            mark("precondition " + p.name + " w" + std::to_string(p.window));
        }
    }
}

OdometryConfig loop_candidate_config(const SensorProfile &sensor, bool switches)
{
    OdometryConfig c;
    RunPaths paths;
    GuiPreferences gui;
    TomlConfigRepository repo;
    if (!repo.load(std::string(LO_REPO_DIR) + "/settings/livox-vertical.toml", c, paths, gui))
        throw std::runtime_error("settings/livox-vertical.toml not loadable");
    c.use_exact_ndt_bucket_statistics = true;
    c.degenerate_projection_ratio = 0.02;
    c.degenerate_projection_ratio_full = 0.04;
    c.imu_acceleration_offset_rw_mps2_sqrt_s = 0.01;
    c.imu_edges_follow_state = switches;
    c.imu_carry_joint_covariance = switches;
    // the front end's fields (front_end_config): the cut must be the generator's
    const OdometryConfig f = front_end_config(sensor);
    c.threshold_initial_points = f.threshold_initial_points;
    c.threshold_nr_poses = f.threshold_nr_poses;
    c.decimation = f.decimation;
    c.filter_threshold_xy_inner = f.filter_threshold_xy_inner;
    c.filter_threshold_xy_outer = f.filter_threshold_xy_outer;
    c.lidar_imu_lever_arm_x_m = f.lidar_imu_lever_arm_x_m;
    c.lidar_imu_lever_arm_y_m = f.lidar_imu_lever_arm_y_m;
    c.lidar_imu_lever_arm_z_m = f.lidar_imu_lever_arm_z_m;
    c.ahrs_gain = f.ahrs_gain;
    c.use_remove_imu_bias_from_first_stationary_scan = f.use_remove_imu_bias_from_first_stationary_scan;
    c.startup_gyro_bias_nr_samples = f.startup_gyro_bias_nr_samples;
    c.startup_gyro_bias_max_variance_deg2_s2 = f.startup_gyro_bias_max_variance_deg2_s2;
    c.log_window_terms = true;
    c.real_time_threshold_seconds = 1000.0;
    return c;
}

const char *window_class_name(WindowClass c)
{
    switch (c)
    {
    case WindowClass::rest: return "rest";
    case WindowClass::turn: return "turn";
    case WindowClass::exit: return "exit";
    case WindowClass::blind: return "blind";
    case WindowClass::seen: return "seen";
    }
    return "?";
}

int Unavailable::total() const
{
    return records_missing + handoff_not_done + heading_bend + speed + max_bias + velocity_chi2 + information_ratio + remaining_step + lidar_share +
           bent_check + fast_check + bias_check + move_velocity + move_joint + move_bias + rotation_rate + start_acceleration + velocity_end_normalised;
}

std::string Unavailable::text() const
{
    std::ostringstream s;
    s << "records " << records_missing << ", hand-off " << handoff_not_done << ", bend " << heading_bend << ", speed " << speed << ", bias "
      << max_bias << ", chi2 " << velocity_chi2 << ", information ratio " << information_ratio << ", remaining step " << remaining_step
      << ", lidar share " << lidar_share << ", checks bent/fast/bias " << bent_check << "/" << fast_check << "/" << bias_check
      << ", moves velocity/joint/bias " << move_velocity << "/" << move_joint << "/" << move_bias;
    const char *move_names[3] = {"velocity", "joint", "bias"};
    for (int m = 0; m < 3; m++)
    {
        bool any = false;
        for (int r = 0; r < kMoveNotMeasuredReasons; r++)
            if (move_reasons[m][r] > 0)
            {
                s << (any ? ", " : " [" + std::string(move_names[m]) + ": ") << move_not_measured_name(MoveNotMeasured(r)) << " " << move_reasons[m][r];
                any = true;
            }
        if (any)
            s << "]";
    }
    s << ", rotation rate " << rotation_rate
      << ", start acceleration " << start_acceleration << ", end velocity normalised error " << velocity_end_normalised << " (total " << total() << ")";
    return s.str();
}

bool LoopRun::valid() const
{
    if (!completed)
        return false;
    for (const Precondition &p : preconditions)
        if (!p.holds)
            return false;
    return true;
}

std::vector<std::string> LoopRun::failed() const
{
    std::vector<std::string> out;
    if (!completed)
        out.push_back("the loop did not complete");
    for (const Precondition &p : preconditions)
        if (!p.holds)
        {
            std::ostringstream s;
            s << p.name << (p.window ? " window " + std::to_string(p.window) : std::string()) << ": " << p.value << (p.at_least ? " < " : " > ")
              << p.tolerance;
            out.push_back(s.str());
        }
    return out;
}

std::string LoopRun::summary() const
{
    std::ostringstream s;
    s << route << (switches ? " switches on" : " switches off") << ": " << windows_run << " of " << windows_cut << " windows, " << std::fixed
      << std::setprecision(1) << seconds << " s, time limit " << time_limit_windows << ", map rebuilds " << map_rebuilds << ", moving windows judged at rest "
      << moving_judged_at_rest << " (first " << first_moving_judged_at_rest << "); classes";
    for (int c = 0; c < kWindowClasses; c++)
        s << " " << window_class_name(WindowClass(c)) << " " << class_windows[c] << " (" << std::setprecision(2) << class_seconds[c] << " s)";
    int holds = 0;
    for (const Precondition &p : preconditions)
        holds += p.holds;
    s << "; preconditions " << holds << " of " << preconditions.size() << " hold, " << not_evaluated.size() << " groups not evaluated; unavailable "
      << unavailable.total();
    return s.str();
}

LoopRun run_loop(const InertialRecording &rec, const OdometryConfig &config_in, int max_windows, const LoopTolerances &tol)
{
    LoopRun run;
    run.route = rec.route.name;
    run.switches = config_in.imu_edges_follow_state || config_in.imu_carry_joint_covariance;
    Cut cut;
    cut_into_windows(cut, rec, config_in);
    run.windows_cut = static_cast<int>(cut.windows.size());
    add(run, "front end cut the recording", 0, cut.ok ? 1.0 : 0.0, 1.0, true);
    add(run, "sensor jitter sigma [m] (K9)", 0, rec.sensor.jitter_sigma_m, tol.min_jitter_sigma_m, true);
    if (!cut.ok)
        return run;
    // the cut is the generator's: the predicted count, and each window's nodes at the predicted timestamps
    const int T = rec.sensor.nodes_per_window;
    int mismatched = std::abs(run.windows_cut - rec.n_windows);
    for (int w = 0; w < std::min(run.windows_cut, rec.n_windows); w++)
    {
        const Window &win = cut.windows[w];
        bool same = static_cast<int>(win.timestamps.size()) == T;
        for (int k = 0; same && k < T; k++)
            same = win.timestamps[k].first == rec.node_time(rec.window_origin_node + w * T + k);
        mismatched += !same;
    }
    add(run, "windows cut otherwise than generated", 0, double(mismatched), 0.0, false);
    if (mismatched != 0)
        return run;
    // the truth of every window cut (the R2 design reads the whole recording); the run keeps the first max_windows
    const int n_cut = run.windows_cut;
    const int n = max_windows > 0 && max_windows < n_cut ? max_windows : n_cut;
    run.windows_run = n;

    // the alignment, at window 0's first node as the loop will place it (conditioned on: D5), before the run
    Window first = cut.windows[0];
    place_window_after(first, cut.state.m_g);
    const Eigen::Affine3d align = first.poses[0] * truth_pose(rec, rec.window_origin_node).inverse();

    run.windows.resize(n_cut);
    for (int w = 0; w < n_cut; w++)
    {
        LoopWindow &lw = run.windows[w];
        lw.window = w + 1;
        WindowTruth &t = lw.truth;
        t.first_node = rec.window_origin_node + w * T;
        t.x_min = 1e300;
        t.x_max = -1e300;
        for (int k = 0; k < T; k++)
        {
            const Eigen::Affine3d p = truth_pose(rec, t.first_node + k);
            t.x_min = std::min(t.x_min, p.translation().x());
            t.x_max = std::max(t.x_max, p.translation().x());
            t.poses.push_back(align * p);
        }
        const Truth a = rec.imu.truth_at_node(t.first_node), b = rec.imu.truth_at_node(t.first_node + T - 1);
        t.velocity_first = align.linear() * a.motion.velocity;
        t.velocity_last = align.linear() * b.motion.velocity;
        t.gyro_bias_rad_s = a.gyro_bias;
        t.accel_offset_mps2 = a.accel_offset;
        t.heading_change_rad = wrapped(route_heading(rec, rec.imu.node_time(t.first_node + T - 1)) - route_heading(rec, rec.imu.node_time(t.first_node)));
        t.duration_s = rec.node_time(t.first_node + T - 1) - rec.node_time(t.first_node);
    }
    std::vector<int> exits;
    const bool r2 = rec.route.name == "R2";
    if (r2)
        design_r2(run, rec, align, config_in, tol, exits);
    run.windows.resize(n);
    cut.windows.resize(n);
    run.r2.exit_in_run = run.r2.exit_window > 0 && run.r2.exit_window <= n;

    OdometryConfig config = config_in;
    KeepMapBefore map_before;
    WindowCaptureSink *capture = nullptr;
    if (r2 && run.r2.exit_in_run)
    {
        const double t = cut.windows[run.r2.exit_window - 1].timestamps[0].first - cut.windows[0].timestamps[0].first;
        config.capture_windows_from_s = t - 0.01;
        config.capture_windows_to_s = t + 0.01;
        capture = &map_before;
    }
    cut.config = config;

    KeepTerms terms;
    RunLog log;
    NoPreview preview;
    NoProgress progress;
    double ts_failure = 0.0;
    const auto start = std::chrono::steady_clock::now();
    run.completed = optimize_windows(cut.windows, config, cut.state, ts_failure, false, cut.store, preview, log, progress, capture, &terms);
    run.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    run.map_rebuilds = log.map_rebuilds;

    for (const WindowTermRecord &r : terms.records)
        if (r.window >= 1 && r.window <= n)
            run.windows[r.window - 1].record = r;
    for (int w = 0; w < n; w++)
    {
        LoopWindow &lw = run.windows[w];
        lw.poses = cut.windows[w].poses;
        WindowErrors &e = lw.errors;
        if (lw.poses.size() == lw.truth.poses.size() && !lw.poses.empty())
        {
            e.last_position = lw.poses.back().translation() - lw.truth.poses.back().translation();
            e.last_rotation_rad = Eigen::AngleAxisd(lw.truth.poses.back().linear().transpose() * lw.poses.back().linear()).angle();
        }
        if (lw.record && lw.record->handoff == WindowHandOff::done && lw.record->handed_on.velocity_end && lw.record->handed_on.velocity_end_cov)
        {
            e.velocity_end = *lw.record->handed_on.velocity_end - lw.truth.velocity_last;
            try
            {
                e.velocity_end_normalised = truth_stats::normalised_error(*e.velocity_end, *lw.record->handed_on.velocity_end_cov);
            }
            catch (const std::runtime_error &)
            {
            }
        }
        if (lw.record && lw.record->verdict.evidence.stop == LmStop::time_limit)
            run.time_limit_windows++;
        lw.cls = classify(lw, rec, exits, tol);
        if (lw.cls != WindowClass::rest && lw.record && lw.record->at_rest)
        {
            run.moving_judged_at_rest++;
            if (run.first_moving_judged_at_rest == 0)
                run.first_moving_judged_at_rest = lw.window;
        }
        run.class_windows[int(lw.cls)]++;
        run.class_seconds[int(lw.cls)] += lw.truth.duration_s;
    }

    // every run
    int missing = 0;
    for (const LoopWindow &lw : run.windows)
        missing += !lw.record;
    add(run, "windows without a record", 0, double(missing), 0.0, false);
    add(run, "windows stopped at the wall-clock limit", 0, double(run.time_limit_windows), double(tol.time_limit_windows), false);
    add(run, "map rebuilds", 0, run.map_rebuilds < 0 ? nan : double(std::max(run.map_rebuilds, log.rebuild_warnings)), double(tol.map_rebuilds), false);
    add(run, "origin moved by the run (largest pose matrix entry)", 0, (cut.windows[0].poses[0].matrix() - first.poses[0].matrix()).cwiseAbs().maxCoeff(),
        tol.origin_moved, false);

    // R2
    if (r2)
    {
        int return_blind = 0;
        for (const LoopWindow &lw : run.windows)
            if (lw.cls == WindowClass::blind)
            {
                blind_preconditions(run, lw, align, tol);
                return_blind += run.r2.turn_window > 0 && lw.window >= run.r2.turn_window;
            }
        add(run, "nominated exit in the recording", 0, double(run.r2.exit_window), 1.0, true);
        // the return leg and its exit only where the run reaches the exit (a shortened run lists them as not evaluated)
        if (run.r2.exit_in_run)
        {
            add(run, "return leg's blind windows", 0, double(return_blind), double(tol.min_blind_windows_return), true);
            exit_preconditions(run, cut, map_before, align, tol);
        }
        else
            run.not_evaluated.push_back("the return leg's blind windows and the exit's preconditions (exit window " + std::to_string(run.r2.exit_window) +
                                        " beyond the run's " + std::to_string(n) + ")");
    }
    count_unavailable(run);
    return run;
}

bool same_deterministic_fields(const LoopRun &a, const LoopRun &b, std::string &difference)
{
    if (a.time_limit_windows > 0 || b.time_limit_windows > 0)
    {
        difference = "a run reached the wall-clock limit: not comparable";
        return false;
    }
    std::vector<double> va, vb;
    std::vector<std::string> na, nb;
    deterministic_fields(a, va, na);
    deterministic_fields(b, vb, nb);
    if (va.size() != vb.size())
    {
        difference = "field count " + std::to_string(va.size()) + " vs " + std::to_string(vb.size());
        return false;
    }
    for (std::size_t i = 0; i < va.size(); i++)
        if (std::bit_cast<std::uint64_t>(va[i]) != std::bit_cast<std::uint64_t>(vb[i]))
        {
            std::ostringstream s;
            s << na[i] << ": " << std::setprecision(17) << va[i] << " vs " << vb[i];
            difference = s.str();
            return false;
        }
    difference.clear();
    return true;
}
}
