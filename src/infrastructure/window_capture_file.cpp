#include "window_capture_file.h"
#include "binary_io.h"
#include "odometry_snapshot_file.h"

#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace
{
    using binary_io::Reader;
    using binary_io::Writer;

    constexpr char MAGIC[8] = {'D', 'M', 'C', 'A', 'P', 'T', '0', '1'};
    // 2: the carry's velocity-offset cross block (change honest-inertial-carry; its layer 2 was removed 2026-11-25, the
    // block is written as zeros and dropped on reading); format 1 is still read
    constexpr std::uint32_t FORMAT_VERSION = 2;
    constexpr std::uint32_t OLDEST_READ_VERSION = 1;

    // everything a window needs, beyond what the snapshot's window record holds
    void write_full_window(Writer &w, const Window &win)
    {
        binary_io::write_window(w, win);
        w.matrix(win.imu_motion_from_previous.matrix());
    }

    bool read_full_window(Reader &r, Window &win)
    {
        if (!binary_io::read_window(r, win))
            return false;
        r.matrix(win.imu_motion_from_previous.matrix());
        return r.ok();
    }

    void write_point(Writer &w, const Point3Di &p)
    {
        w.matrix(p.point);
        w.value(p.timestamp);
        w.value(p.intensity);
        w.value(p.index_pose);
        w.value(p.lidarid);
        w.value(p.index_point);
    }

    void read_point(Reader &r, Point3Di &p)
    {
        r.matrix(p.point);
        r.value(p.timestamp);
        r.value(p.intensity);
        r.value(p.index_pose);
        r.value(p.lidarid);
        r.value(p.index_point);
    }

    // a cell with every stored field, so a replay matches without recomputing anything
    void write_buckets(Writer &w, const NDTBucketMapType &buckets)
    {
        w.value(static_cast<std::uint64_t>(buckets.size()));
        for (const auto &[key, b] : buckets)
        {
            w.value(key);
            w.value(b.index_begin);
            w.value(b.index_end);
            w.value(b.number_of_points);
            w.matrix(b.mean);
            w.matrix(b.cov);
            w.matrix(b.normal_vector);
            w.matrix(b.cov_inverse);
            w.value(b.inverse_computed);
            w.value(b.inverse_usable);
        }
    }

    bool read_buckets(Reader &r, NDTBucketMapType &buckets)
    {
        std::uint64_t n = 0;
        r.value(n);
        if (!r.ok() || n > 1000000000ULL)
            return false;
        buckets.clear();
        buckets.reserve(static_cast<size_t>(n));
        for (std::uint64_t i = 0; i < n; i++)
        {
            std::uint64_t key = 0;
            NdtBucket b;
            r.value(key);
            r.value(b.index_begin);
            r.value(b.index_end);
            r.value(b.number_of_points);
            r.matrix(b.mean);
            r.matrix(b.cov);
            r.matrix(b.normal_vector);
            r.matrix(b.cov_inverse);
            r.value(b.inverse_computed);
            r.value(b.inverse_usable);
            buckets.emplace(key, b);
        }
        return r.ok();
    }

    void write_carry(Writer &w, const WindowLoopCarry &c, std::uint32_t version)
    {
        w.matrix(c.bias.mean);
        w.matrix(c.bias.covariance);
        w.matrix(c.acc_gravity);
        w.matrix(c.acc_offset_cov);
        w.value(c.acc_distance);
        w.list(c.points_global, [&](const Point3Di &p) { write_point(w, p); });
        // the second count is the retired turn-window count, kept as 0 so the format does not change
        for (int v : {c.stationary_windows, 0, c.map_rebuilds, c.acc_term_windows, c.rest_windows})
            w.value(v);
        w.value(c.velocity_valid);
        w.matrix(c.velocity_mean);
        w.matrix(c.velocity_cov);
        w.value(c.velocity_time_s);
        w.value(c.first_window_at_rest);
        // format 2 on: the retired cross block cov(end velocity, offset) of honest-inertial-carry layer 2 (removed
        // 2026-11-25), kept as zeros so the format does not change
        const Eigen::Matrix3d retired_velocity_offset_cov = Eigen::Matrix3d::Zero();
        if (version >= 2)
            w.matrix(retired_velocity_offset_cov);
    }

    bool read_carry(Reader &r, WindowLoopCarry &c, std::uint32_t version)
    {
        r.matrix(c.bias.mean);
        r.matrix(c.bias.covariance);
        r.matrix(c.acc_gravity);
        r.matrix(c.acc_offset_cov);
        r.value(c.acc_distance);
        if (!r.list(c.points_global, [&](Point3Di &p) { read_point(r, p); }))
            return false;
        int retired_turn_windows = 0;
        for (int *v : {&c.stationary_windows, &retired_turn_windows, &c.map_rebuilds, &c.acc_term_windows, &c.rest_windows})
            r.value(*v);
        r.value(c.velocity_valid);
        r.matrix(c.velocity_mean);
        r.matrix(c.velocity_cov);
        r.value(c.velocity_time_s);
        r.value(c.first_window_at_rest);
        Eigen::Matrix3d retired_velocity_offset_cov; // format 2 on: the retired cross block, read and dropped
        if (version >= 2)
            r.matrix(retired_velocity_offset_cov);
        return r.ok();
    }
} // namespace

bool write_window_capture(const std::filesystem::path &file, const WindowCapture &capture, const OdometryConfig &config, std::string &why,
                          std::uint32_t format_version)
{
    const std::uint32_t version = format_version == 0 ? FORMAT_VERSION : format_version;
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
    w.value(version);
    w.value(odometry_settings_hash(config));
    for (std::uint64_t v : {std::uint64_t(capture.n_windows), std::uint64_t(capture.first), std::uint64_t(capture.last),
                            std::uint64_t(capture.windows_begin)})
        w.value(v);
    write_full_window(w, capture.first_window);
    w.list(capture.windows, [&](const Window &win) { write_full_window(w, win); });
    w.list(capture.points, [&](const std::vector<Point3Di> &points) { w.list(points, [&](const Point3Di &p) { write_point(w, p); }); });
    w.matrix(capture.state.m_g.matrix());
    w.value(capture.state.consecutive_distance);
    w.value(capture.state.total_length_of_calculated_trajectory);
    w.list(capture.state.reference_points, [&](const Point3Di &p) { write_point(w, p); });
    write_buckets(w, capture.state.buckets_indoor);
    write_buckets(w, capture.state.buckets_outdoor);
    write_carry(w, capture.carry, version);
    w.list(capture.outcomes, [&](const WindowOutcome &o)
           {
               w.value(o.last_iteration);
               w.list(o.poses, [&](const Eigen::Affine3d &p) { w.matrix(p.matrix()); });
           });
    out.flush();
    if (!out)
    {
        why = "writing '" + file.string() + "' failed";
        return false;
    }
    return true;
}

bool read_window_capture(const std::filesystem::path &file, WindowCapture &capture, std::uint64_t &settings_hash, std::string &why)
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
    if (!in || std::memcmp(magic, MAGIC, sizeof(MAGIC)) != 0 || version < OLDEST_READ_VERSION || version > FORMAT_VERSION)
    {
        why = "'" + file.string() + "' is not a window capture of format " + std::to_string(OLDEST_READ_VERSION) + " to " + std::to_string(FORMAT_VERSION);
        return false;
    }
    WindowCapture c;
    r.value(settings_hash);
    std::uint64_t n_windows = 0, first = 0, last = 0, windows_begin = 0;
    r.value(n_windows);
    r.value(first);
    r.value(last);
    r.value(windows_begin);
    c.n_windows = n_windows;
    c.first = first;
    c.last = last;
    c.windows_begin = windows_begin;
    const auto damaged = [&](const char *part)
    {
        why = "'" + file.string() + "' is damaged (" + part + ")";
        return false;
    };
    if (!r.ok() || first > last || last >= n_windows || windows_begin > first)
        return damaged("range");
    if (!read_full_window(r, c.first_window))
        return damaged("first window");
    bool ok = true;
    if (!r.list(c.windows, [&](Window &win) { ok = read_full_window(r, win) && ok; }, 1000000ULL) || !ok ||
        c.windows.size() != std::min(last + 1, n_windows - 1) - windows_begin + 1)
        return damaged("windows");
    if (!r.list(c.points, [&](std::vector<Point3Di> &points) { ok = r.list(points, [&](Point3Di &p) { read_point(r, p); }) && ok; }, 1000000ULL) ||
        !ok || c.points.size() != last - first + 1)
        return damaged("points");
    r.matrix(c.state.m_g.matrix());
    r.value(c.state.consecutive_distance);
    r.value(c.state.total_length_of_calculated_trajectory);
    if (!r.list(c.state.reference_points, [&](Point3Di &p) { read_point(r, p); }) || !read_buckets(r, c.state.buckets_indoor) ||
        !read_buckets(r, c.state.buckets_outdoor))
        return damaged("map");
    if (!read_carry(r, c.carry, version))
        return damaged("loop state");
    if (!r.list(c.outcomes, [&](WindowOutcome &o)
                {
                    r.value(o.last_iteration);
                    ok = r.list(o.poses, [&](Eigen::Affine3d &p) { r.matrix(p.matrix()); }) && ok;
                }, 1000000ULL) || !ok)
        return damaged("outcomes");
    capture = std::move(c);
    return true;
}

std::filesystem::path FolderWindowCaptureSink::file() const
{
    std::ostringstream name;
    name << "window_capture_" << std::fixed << std::setprecision(1) << config_.capture_windows_from_s << "-" << config_.capture_windows_to_s
         << "s.bin";
    return folder_ / name.str();
}

bool FolderWindowCaptureSink::write(const WindowCapture &capture, std::string &why)
{
    return write_window_capture(file(), capture, config_, why);
}
