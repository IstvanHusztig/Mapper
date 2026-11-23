#include "loop_closure_capture_file.h"
#include "binary_io.h"
#include "odometry_snapshot_file.h"
#include "toml_io.h"

#include <nlohmann/json.hpp>

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
    constexpr std::uint32_t FORMAT_VERSION = 1;

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
        return j;
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
    if (!in || std::memcmp(magic, MAGIC, sizeof(MAGIC)) != 0 || version != FORMAT_VERSION)
    {
        why = "'" + file.string() + "' is not a loop closure capture of format " + std::to_string(FORMAT_VERSION);
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
    j["own_settings"] = settings.refused.empty() && settings.changed.empty();
    j["settings_changed"] = settings.changed;
    j["missing_tries"] = result.missing;
    nlohmann::json kept = nlohmann::json::array();
    for (size_t k = 0; k < result.report.kept.size(); k++)
        kept.push_back(edge_json(result.report.kept[k], result.report.kept_residuals[k], config));
    j["edges_kept"] = kept;
    nlohmann::json dropped = nlohmann::json::array();
    for (size_t k = 0; k < result.report.dropped.size(); k++)
        dropped.push_back(edge_json(result.report.dropped[k], result.report.dropped_residuals[k], config));
    j["edges_dropped"] = dropped;
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

bool FolderLoopClosureCaptureSink::write(const LoopClosureCapture &capture, std::string &why)
{
    return write_loop_closure_capture(file(), capture, config_, why);
}
