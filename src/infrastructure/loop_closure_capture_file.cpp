#include "loop_closure_capture_file.h"
#include "binary_io.h"
#include "odometry_snapshot_file.h"
#include "toml_io.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <type_traits>
#include <variant>

namespace
{
    using binary_io::Reader;
    using binary_io::Writer;

    constexpr char MAGIC[8] = {'D', 'M', 'L', 'C', 'C', 'A', 'P', '1'};
    constexpr std::uint32_t FORMAT_VERSION = 4;          // 2: the windows' readings after the loop count; 3: then each try's
                                                         // information per source window (LoopClosureTry::window_information);
                                                         // 4: then the fine-time step's curves (LoopClosureCapture::fine_time)
    constexpr std::uint32_t FIRST_READABLE_VERSION = 1; // 1: without readings

    // the odometry settings hash (odometry_snapshot_file.h) as one more setting: it also covers what the TOML keys don't
    // (the NDT grids), and it counts as an odometry setting
    const char *const ODOMETRY_HASH_KEY = "odometry_settings_hash";

    // [loop_closure] settings that change the segments or a registration: a capture holds neither the points nor the
    // registrations they would give (next to the odometry settings hash's added_later list in odometry_snapshot_file.cpp)
    const std::set<std::string> &registration_settings()
    {
        static const std::set<std::string> keys = {"loop_closure_segment_windows", "loop_closure_max_correction_m",
                                                   "loop_closure_cross_floor_search_m"};
        return keys;
    }

    // categories whose settings neither the odometry nor the loop closure reads
    const std::set<std::string> &output_only_categories()
    {
        static const std::set<std::string> c = {"consistency", "lidar_odometry_debug_info", "paths", "misc", "version_info"};
        return c;
    }

    void write_text(Writer &w, const std::string &s)
    {
        w.value(static_cast<std::uint64_t>(s.size()));
        for (const char c : s)
            w.value(c);
    }

    bool read_text(Reader &r, std::string &s)
    {
        std::uint64_t n = 0;
        r.value(n);
        if (!r.ok() || n > 1000000ULL)
            return false;
        s.resize(static_cast<size_t>(n));
        for (char &c : s)
            r.value(c);
        return r.ok();
    }

    void write_segment(Writer &w, const LoopClosureSegment &s)
    {
        w.value(s.first_window);
        w.value(s.last_window);
        w.matrix(s.pose.matrix());
        w.matrix(s.center);
        w.value(s.path_begin);
        w.value(s.path_end);
        w.value(s.t_begin);
        w.value(s.t_end);
    }

    void read_segment(Reader &r, LoopClosureSegment &s)
    {
        r.value(s.first_window);
        r.value(s.last_window);
        r.matrix(s.pose.matrix());
        r.matrix(s.center);
        r.value(s.path_begin);
        r.value(s.path_end);
        r.value(s.t_begin);
        r.value(s.t_end);
    }

    void write_try(Writer &w, const CapturedLoopTry &c)
    {
        const LoopClosureTry &t = c.registration;
        w.value(t.i);
        w.value(t.j);
        w.value(static_cast<std::uint8_t>(t.cross_floor));
        w.value(static_cast<std::uint8_t>(t.registered));
        w.value(t.overlap_before);
        w.value(t.overlap_after);
        w.matrix(t.correction.matrix());
        w.matrix(t.information);
        w.value(t.shift_m);
        w.value(t.angle_deg);
        w.value(t.information_ratio);
        w.matrix(t.centre);
        w.value(static_cast<std::uint8_t>(c.accepted));
        w.value(static_cast<std::uint8_t>(c.requested_by_run));
    }

    void read_try(Reader &r, CapturedLoopTry &c)
    {
        LoopClosureTry &t = c.registration;
        std::uint8_t flag = 0;
        r.value(t.i);
        r.value(t.j);
        r.value(flag);
        t.cross_floor = flag != 0;
        r.value(flag);
        t.registered = flag != 0;
        r.value(t.overlap_before);
        r.value(t.overlap_after);
        r.matrix(t.correction.matrix());
        r.matrix(t.information);
        r.value(t.shift_m);
        r.value(t.angle_deg);
        r.value(t.information_ratio);
        r.matrix(t.centre);
        r.value(flag);
        c.accepted = flag != 0;
        r.value(flag);
        c.requested_by_run = flag != 0;
    }

    void write_fine_time_run(Writer &w, const LoopFineTimeRun &r)
    {
        w.value(r.run);
        w.value(r.first_window);
        w.value(r.last_window);
        w.value(r.support_first);
        w.value(r.support_last);
        w.value(static_cast<std::uint8_t>(r.overlap));
        w.value(r.sigma_m);
        w.value(r.pin_begin_s);
        w.value(r.pin_end_s);
        w.matrix(r.axis_odometry);
        w.matrix(r.axis);
    }

    void read_fine_time_run(Reader &r, LoopFineTimeRun &f)
    {
        std::uint8_t flag = 0;
        r.value(f.run);
        r.value(f.first_window);
        r.value(f.last_window);
        r.value(f.support_first);
        r.value(f.support_last);
        r.value(flag);
        f.overlap = flag != 0;
        r.value(f.sigma_m);
        r.value(f.pin_begin_s);
        r.value(f.pin_end_s);
        r.matrix(f.axis_odometry);
        r.matrix(f.axis);
    }

    // format 4: a flag, then the segment poses, the eligible runs and the curves
    void write_fine_time(Writer &w, const std::optional<LoopFineTimeCapture> &f)
    {
        w.value(static_cast<std::uint8_t>(f ? 1 : 0));
        if (!f)
            return;
        w.list(f->segment_poses, [&](const Eigen::Affine3d &p) { w.matrix(p.matrix()); });
        w.list(f->eligible, [&](const LoopFineTimeRun &r) { write_fine_time_run(w, r); });
        w.list(f->curves, [&](const LoopFineTimeCurves &c) {
            w.value(c.support_first);
            w.value(c.support_last);
            w.value(c.bound_m);
            w.list(c.grid, [&](double v) { w.value(v); });
            w.value(c.reference_patches);
            w.list(c.windows, [&](const LoopFineTimeWindowCurves &x) {
                w.value(x.training_patches);
                w.value(x.held_out_patches);
                w.list(x.training, [&](double v) { w.value(v); });
                w.list(x.training_matched, [&](int v) { w.value(v); });
                w.list(x.held_out, [&](double v) { w.value(v); });
                w.list(x.held_out_matched, [&](int v) { w.value(v); });
            });
        });
    }

    bool read_fine_time(Reader &r, std::optional<LoopFineTimeCapture> &f)
    {
        f.reset();
        std::uint8_t present = 0;
        r.value(present);
        if (!r.ok() || present > 1)
            return false;
        if (!present)
            return true;
        LoopFineTimeCapture c;
        const std::uint64_t many = 100000000ULL;
        bool inner = true; // the lists inside the curves
        const bool outer =
            r.list(c.segment_poses, [&](Eigen::Affine3d &p) { r.matrix(p.matrix()); }, many) &&
            r.list(c.eligible, [&](LoopFineTimeRun &run) { read_fine_time_run(r, run); }, many) &&
            r.list(c.curves, [&](LoopFineTimeCurves &x) {
                r.value(x.support_first);
                r.value(x.support_last);
                r.value(x.bound_m);
                inner = inner && r.list(x.grid, [&](double &v) { r.value(v); }, many);
                r.value(x.reference_patches);
                inner = inner && r.list(x.windows, [&](LoopFineTimeWindowCurves &y) {
                    r.value(y.training_patches);
                    r.value(y.held_out_patches);
                    inner = inner && r.list(y.training, [&](double &v) { r.value(v); }, many) &&
                            r.list(y.training_matched, [&](int &v) { r.value(v); }, many) &&
                            r.list(y.held_out, [&](double &v) { r.value(v); }, many) &&
                            r.list(y.held_out_matched, [&](int &v) { r.value(v); }, many);
                }, many);
            }, many);
        if (!outer || !inner || !r.ok())
            return false;
        f = std::move(c);
        return true;
    }

    // NaN and infinities as null
    nlohmann::json number(double v)
    {
        return std::isfinite(v) ? nlohmann::json(v) : nlohmann::json(nullptr);
    }

    // the fine-time step's report (design D8): per eligible run what was done and why, and rule 7's edge table
    nlohmann::json fine_time_json(const LoopClosureReport &report)
    {
        const LoopFineTimeReport &f = report.fine_time;
        const double t0 = f.time_origin_s;
        nlohmann::json j;
        j["seconds"] = f.seconds;
        nlohmann::json runs = nlohmann::json::array();
        for (const LoopFineTimeRunReport &rr : f.runs)
        {
            nlohmann::json r;
            const LoopFineTimeRun &run = rr.run;
            r["windows"] = {run.first_window, run.last_window};
            r["support"] = {run.support_first, run.support_last};
            r["t_s"] = {run.pin_begin_s - t0, run.pin_end_s - t0};
            r["sigma_m"] = run.sigma_m;
            r["axis"] = {run.axis.x(), run.axis.y(), run.axis.z()};
            r["axis_odometry"] = {run.axis_odometry.x(), run.axis_odometry.y(), run.axis_odometry.z()};
            r["largest_axis_angle_deg"] = rr.largest_axis_angle_deg;
            r["status"] = rr.status;
            r["cause"] = rr.cause;
            if (rr.scored)
            {
                r["bound_m"] = rr.curves.bound_m;
                r["reference_patches"] = rr.curves.reference_patches;
                r["supported_in_run"] = rr.supported_in_run;
                r["run_windows"] = rr.run_windows;
                r["longest_unsupported_s"] = rr.longest_unsupported_s;
                r["largest_offset_m"] = rr.largest_offset_m;
                r["unsupported_moved"] = rr.unsupported_moved;
                r["unsupported_largest_m"] = rr.unsupported_largest_m;
                r["held_out_windows"] = rr.held_out_windows;
                r["held_out_before_m"] = number(std::sqrt(rr.held_out_before)); // RMS
                r["held_out_after_m"] = number(std::sqrt(rr.held_out_after));
                r["offsets_m"] = rr.offsets_m;
                std::vector<int> supported(rr.supported.begin(), rr.supported.end());
                r["supported"] = supported;
            }
            runs.push_back(r);
        }
        j["runs"] = runs;
        nlohmann::json edges = nlohmann::json::array();
        for (const LoopFineTimeEdgeRow &row : f.edges)
        {
            nlohmann::json e;
            e["i"] = row.i;
            e["j"] = row.j;
            e["cross_floor"] = row.cross_floor;
            const LoopOdometryRun &run = report.odometry_runs[row.run];
            e["run_windows"] = {run.first_window, run.last_window};
            e["eligible"] = row.eligible;
            e["side"] = row.target ? "target" : "source";
            e["source_windows"] = {row.first_window, row.last_window};
            e["first_pose_m"] = row.first_pose_m;
            e["rho_before_m"] = number(row.rho_before_m);
            e["rho_after_m"] = number(row.rho_after_m);
            e["limit_m"] = number(row.limit_m);
            e["sigma_a_m"] = number(row.sigma_a_m);
            e["informed_share"] = number(row.informed_share);
            e["class"] = fine_time_edge_class_name(row.cls);
            e["stopped_run"] = row.stopped_run;
            e["shares"] = row.shares;
            edges.push_back(e);
        }
        j["edges"] = edges;
        return j;
    }

    nlohmann::json edge_json(const LoopClosureEdge &e, const LoopEdgeResidual &r, const OdometryConfig &config)
    {
        nlohmann::json j;
        j["i"] = e.i;
        j["j"] = e.j;
        j["cross_floor"] = e.cross_floor;
        j["fitness"] = e.fitness;
        j["information_weighted"] = e.information;
        j["residual_m"] = r.translation_m;
        j["residual_deg"] = r.angle_deg;
        if (e.information)
            j["weighted_residual_m"] = r.weighted * config.loop_closure_edge_sigma_m;
        j["drop_ratio"] = loop_edge_drop_ratio(e, r, config);
        j["shift_m"] = e.shift_m;
        j["angle_deg"] = e.angle_deg;
        if (e.information)
        {
            // the eigenvalues of the edge's information (omega), ascending: its translation block [1/m^2] in segment i's
            // frame, its rotation block [1/rad^2]
            const auto eigenvalues = [](const Eigen::Matrix3d &m)
            {
                const Eigen::Vector3d v = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(m, Eigen::EigenvaluesOnly).eigenvalues();
                return std::vector<double>{v(0), v(1), v(2)};
            };
            j["information_translation_eigenvalues"] = eigenvalues(e.omega.topLeftCorner<3, 3>());
            j["information_rotation_eigenvalues"] = eigenvalues(e.omega.bottomRightCorner<3, 3>());
        }
        return j;
    }

    // the residual of an edge at the first solve, before any edge was dropped
    void add_first_solve_residual(nlohmann::json &j, const LoopClosureEdge &e, const LoopClosureReport &report, const OdometryConfig &config)
    {
        for (size_t k = 0; k < report.first_solve.size(); k++)
        {
            const LoopClosureEdge &f = report.first_solve[k];
            if (f.i != e.i || f.j != e.j || f.cross_floor != e.cross_floor)
                continue;
            const LoopEdgeResidual &r = report.first_solve_residuals[k];
            j["first_solve_residual_m"] = r.translation_m;
            j["first_solve_residual_deg"] = r.angle_deg;
            if (f.information)
                j["first_solve_weighted_residual_m"] = r.weighted * config.loop_closure_edge_sigma_m;
            return;
        }
    }
} // namespace

std::map<std::string, std::string> settings_as_text(const OdometryConfig &config)
{
    TomlIO io;
    std::map<std::string, std::string> out;
    for (const auto &[name, member] : io.POINTERS)
        std::visit(
            [&](auto pointer)
            {
                const auto &v = config.*pointer;
                using T = std::decay_t<decltype(v)>;
                std::ostringstream s;
                if constexpr (std::is_same_v<T, std::string>)
                    s << v;
                else if constexpr (std::is_same_v<T, bool>)
                    s << (v ? "true" : "false");
                else
                    s << std::setprecision(17) << v;
                out[name] = s.str();
            },
            member);
    out[ODOMETRY_HASH_KEY] = std::to_string(odometry_settings_hash(config));
    return out;
}

LoopClosureSettingsDifference loop_closure_replay_settings(const std::map<std::string, std::string> &captured, const OdometryConfig &config)
{
    TomlIO io;
    std::map<std::string, std::string> category_of;
    for (const auto &[category, keys] : io.CATEGORIES)
        for (const auto &key : keys)
            category_of[key] = category;
    const std::map<std::string, std::string> now = settings_as_text(config);
    const std::map<std::string, std::string> defaults = settings_as_text(OdometryConfig{});

    LoopClosureSettingsDifference d;
    for (const auto &[name, value] : now)
    {
        const auto it = captured.find(name);
        const std::string &then = it != captured.end() ? it->second : defaults.at(name);
        if (then == value)
            continue;
        const auto c = category_of.find(name);
        const std::string category = c != category_of.end() ? c->second : std::string();
        if (output_only_categories().count(category))
            continue;
        if (category == "loop_closure" && !registration_settings().count(name))
            d.changed.push_back(name);
        else
            d.refused.push_back(name);
    }
    return d;
}

bool write_loop_closure_capture(const std::filesystem::path &file, const LoopClosureCapture &capture, const OdometryConfig &config, std::string &why)
{
    std::error_code ec;
    if (file.has_parent_path())
        std::filesystem::create_directories(file.parent_path(), ec);
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    if (!out)
    {
        why = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    Writer w(out);
    out.write(MAGIC, sizeof(MAGIC));
    w.value(FORMAT_VERSION);
    const std::map<std::string, std::string> settings = settings_as_text(config);
    w.value(static_cast<std::uint64_t>(settings.size()));
    for (const auto &[name, value] : settings)
    {
        write_text(w, name);
        write_text(w, value);
    }
    w.list(capture.windows, [&](const Window &win) { binary_io::write_window(w, win); });
    w.list(capture.segments, [&](const LoopClosureSegment &s) { write_segment(w, s); });
    w.list(capture.tries, [&](const CapturedLoopTry &t) { write_try(w, t); });
    w.list(capture.result_poses, [&](const std::vector<Eigen::Affine3d> &poses) {
        w.list(poses, [&](const Eigen::Affine3d &p) { w.matrix(p.matrix()); });
    });
    w.value(capture.loops);
    write_window_readings(w, capture.windows);
    w.list(capture.tries, [&](const CapturedLoopTry &t) {
        w.list(t.registration.window_information, [&](const Eigen::Matrix3d &m) { w.matrix(m); });
    });
    write_fine_time(w, capture.fine_time);
    out.flush();
    if (!out)
    {
        why = "writing '" + file.string() + "' failed";
        return false;
    }
    return true;
}

bool read_loop_closure_capture(const std::filesystem::path &file, LoopClosureCapture &capture, std::map<std::string, std::string> &settings,
                               std::string &why)
{
    std::ifstream in(file, std::ios::binary);
    if (!in)
    {
        why = "cannot open '" + file.string() + "'";
        return false;
    }
    Reader r(in);
    char magic[sizeof(MAGIC)] = {};
    in.read(magic, sizeof(magic));
    std::uint32_t version = 0;
    r.value(version);
    if (!in || std::memcmp(magic, MAGIC, sizeof(MAGIC)) != 0 || version < FIRST_READABLE_VERSION || version > FORMAT_VERSION)
    {
        why = "'" + file.string() + "' is not a loop closure capture of format " + std::to_string(FIRST_READABLE_VERSION) + " to " +
              std::to_string(FORMAT_VERSION);
        return false;
    }
    const auto damaged = [&](const char *part)
    {
        why = "'" + file.string() + "' is damaged (" + part + ")";
        return false;
    };
    std::uint64_t n_settings = 0;
    r.value(n_settings);
    if (!r.ok() || n_settings > 100000ULL)
        return damaged("settings");
    std::map<std::string, std::string> s;
    for (std::uint64_t k = 0; k < n_settings; k++)
    {
        std::string name, value;
        if (!read_text(r, name) || !read_text(r, value))
            return damaged("settings");
        s[name] = value;
    }
    LoopClosureCapture c;
    if (!r.list(c.windows, [&](Window &win) { binary_io::read_window(r, win); }, 10000000ULL))
        return damaged("windows");
    if (!r.list(c.segments, [&](LoopClosureSegment &seg) { read_segment(r, seg); }, 10000000ULL))
        return damaged("segments");
    if (!r.list(c.tries, [&](CapturedLoopTry &t) { read_try(r, t); }, 100000000ULL))
        return damaged("registrations");
    if (!r.list(c.result_poses, [&](std::vector<Eigen::Affine3d> &poses) {
            r.list(poses, [&](Eigen::Affine3d &p) { r.matrix(p.matrix()); });
        }, 10000000ULL))
        return damaged("result poses");
    r.value(c.loops);
    if (!r.ok())
        return damaged("loop count");
    if (version >= 2 && !read_window_readings(r, c.windows))
        return damaged("window readings");
    if (version >= 3)
    {
        std::vector<std::vector<Eigen::Matrix3d>> blocks;
        if (!r.list(blocks, [&](std::vector<Eigen::Matrix3d> &b) { r.list(b, [&](Eigen::Matrix3d &m) { r.matrix(m); }, 100000ULL); },
                    100000000ULL) ||
            blocks.size() != c.tries.size())
            return damaged("per-window information");
        for (size_t k = 0; k < blocks.size(); k++)
            c.tries[k].registration.window_information = std::move(blocks[k]);
    }
    if (version >= 4 && !read_fine_time(r, c.fine_time))
        return damaged("fine-time curves");
    capture = std::move(c);
    settings = std::move(s);
    return true;
}

bool write_loop_closure_replay_json(const std::filesystem::path &file, const LoopClosureReplayResult &result,
                                    const LoopClosureSettingsDifference &settings, const OdometryConfig &config, std::string &why)
{
    nlohmann::json j;
    j["outcome"] = result.segments_match ? result.report.outcome : "segments_differ";
    j["loops"] = result.loops;
    j["largest_segment_correction_m"] = result.report.largest_segment_correction_m;
    if (result.report.windows_without_readings > 0) // loop_closure_odometry_from_readings: windows the rule could not read
        j["windows_without_readings"] = result.report.windows_without_readings;
    j["own_settings"] = settings.refused.empty() && settings.changed.empty();
    j["settings_changed"] = settings.changed;
    j["missing_tries"] = result.missing;
    j["rejected_tries"] = result.rejected;
    j["rejected_tries_not_asked"] = result.rejected_not_asked;
    j["rejected_tries_not_in_capture"] = result.rejected_not_in_capture;
    nlohmann::json kept = nlohmann::json::array();
    const auto match_json = [](const LoopMatchTest &t)
    {
        nlohmann::json m;
        m["t"] = t.t;
        m["limit"] = t.limit;
        m["dof"] = t.dof;
        m["held_out"] = t.held_out;
        m["innovation_m"] = t.innovation_m;
        return m;
    };
    for (size_t k = 0; k < result.report.kept.size(); k++)
    {
        kept.push_back(edge_json(result.report.kept[k], result.report.kept_residuals[k], config));
        add_first_solve_residual(kept.back(), result.report.kept[k], result.report, config);
        if (k < result.report.kept_match_tests.size())
            kept.back()["match_check"] = match_json(result.report.kept_match_tests[k]); // its last test (loop_closure_match_check)
    }
    j["edges_kept"] = kept;
    nlohmann::json dropped = nlohmann::json::array();
    for (size_t k = 0; k < result.report.dropped.size(); k++)
    {
        dropped.push_back(edge_json(result.report.dropped[k], result.report.dropped_residuals[k], config));
        add_first_solve_residual(dropped.back(), result.report.dropped[k], result.report, config);
        const bool by_match_check = k < result.report.dropped_match_tests.size() && result.report.dropped_match_tests[k];
        dropped.back()["dropped_by"] = by_match_check ? "match_check" : "pruning";
        if (by_match_check)
            dropped.back()["match_check"] = match_json(*result.report.dropped_match_tests[k]);
    }
    j["edges_dropped"] = dropped;
    // the kept edges read at the times of their information along each weak run's axis (loop_closure_odometry_from_readings)
    nlohmann::json time_support = nlohmann::json::array();
    for (const LoopEdgeTimeSupportReading &t : result.report.edge_time_support)
    {
        nlohmann::json e;
        e["i"] = t.i;
        e["j"] = t.j;
        e["cross_floor"] = t.cross_floor;
        const LoopOdometryRun &run = result.report.odometry_runs[t.run];
        e["run_windows"] = {run.first_window, run.last_window};
        e["run_sigma_m"] = run.sigma_m;
        e["source_windows"] = {t.first_window, t.last_window};
        e["axis"] = {t.axis.x(), t.axis.y(), t.axis.z()};
        e["first_pose_residual_m"] = t.support.first_pose_residual_m;
        e["per_window_information"] = t.window_information; // false: a capture of format 1 or 2
        if (t.window_information)
            e["information_along_axis"] = t.support.along_axis;
        if (t.support.along_axis)
        {
            e["time_support_residual_m"] = t.support.residual_m;
            e["shares"] = t.support.shares;
        }
        else
            e["time_support_residual_m"] = nullptr;
        time_support.push_back(e);
    }
    j["edge_time_support"] = time_support;
    if (result.report.fine_time.ran)
        j["fine_time"] = fine_time_json(result.report);
    nlohmann::json against;
    against["nodes"] = result.nodes;
    against["identical_nodes"] = result.identical_nodes;
    against["largest_position_difference_m"] = result.largest_position_difference_m;
    against["largest_rotation_difference_deg"] = result.largest_rotation_difference_deg;
    j["against_capture"] = against;
    std::ofstream out(file);
    if (!out)
    {
        why = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << j.dump(2) << "\n";
    if (!out)
    {
        why = "writing '" + file.string() + "' failed";
        return false;
    }
    return true;
}

bool write_loop_fine_time_csv(const std::filesystem::path &file, const LoopFineTimeReport &report, std::string &why)
{
    std::ofstream out(file);
    if (!out)
    {
        why = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    const LoopFineTimeRule rule;
    const auto fixed = [](double v, int digits)
    {
        std::ostringstream s;
        if (std::isfinite(v))
            s << std::fixed << std::setprecision(digits) << v;
        return s.str();
    };
    const auto rms = [&](const std::vector<double> &score, int k) { return score.empty() ? std::string() : fixed(std::sqrt(score[k]), 4); };
    out << "run,window,in_run,start_s,mid_s,offset_m,supported,train_patches,held_patches,matched_most,train_before_m,train_after_m,"
           "held_before_m,held_after_m,alone_m\n";
    for (const LoopFineTimeRunReport &rr : report.runs)
    {
        if (!rr.scored)
            continue;
        const LoopFineTimeCurves &c = rr.curves;
        int zero = 0;
        for (int k = 1; k < static_cast<int>(c.grid.size()); k++)
            if (std::abs(c.grid[k]) < std::abs(c.grid[zero]))
                zero = k;
        for (size_t j = 0; j < c.windows.size(); j++)
        {
            const LoopFineTimeWindowCurves &wc = c.windows[j];
            const int w = rr.run.support_first + static_cast<int>(j);
            const int k = rr.offset_index[j];
            const int matched = wc.training_matched.empty() ? 0 : *std::max_element(wc.training_matched.begin(), wc.training_matched.end());
            out << rr.run.first_window << "-" << rr.run.last_window << "," << w << "," << (w >= rr.run.first_window && w <= rr.run.last_window)
                << "," << fixed(rr.start_s[j] - report.time_origin_s, 4) << "," << fixed(rr.middle_s[j] - report.time_origin_s, 4) << ","
                << fixed(rr.offsets_m[j], 3) << "," << static_cast<int>(rr.supported[j] != 0) << "," << wc.training_patches << ","
                << wc.held_out_patches << "," << matched << "," << rms(wc.training, zero) << "," << rms(wc.training, k) << ","
                << rms(wc.held_out, zero) << "," << rms(wc.held_out, k) << ","
                << (wc.training.empty() ? std::string() : fixed(fine_time_own_offset(wc.training, c.grid, rule), 3)) << "\n";
        }
    }
    if (!out)
    {
        why = "writing '" + file.string() + "' failed";
        return false;
    }
    return true;
}

bool FolderLoopClosureCaptureSink::write(const LoopClosureCapture &capture, std::string &why)
{
    return write_loop_closure_capture(file(), capture, config_, why);
}
