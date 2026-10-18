#include "odometry_snapshot_file.h"
#include "toml_io.h"

#include <cstring>
#include <fstream>
#include <set>
#include <type_traits>
#include <variant>

namespace
{
    constexpr char MAGIC[8] = {'D', 'M', 'S', 'N', 'A', 'P', '0', '1'};
    constexpr std::uint32_t FORMAT_VERSION = 1;

    // FNV-1a over raw bytes
    struct Hash
    {
        std::uint64_t h = 1469598103934665603ULL;
        void bytes(const void *data, size_t n)
        {
            const unsigned char *p = static_cast<const unsigned char *>(data);
            for (size_t i = 0; i < n; i++)
            {
                h ^= p[i];
                h *= 1099511628211ULL;
            }
        }
        template <typename T>
        void value(const T &v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            bytes(&v, sizeof(T));
        }
        void text(const std::string &s)
        {
            value(static_cast<std::uint64_t>(s.size()));
            bytes(s.data(), s.size());
        }
    };

    void hash_grid(Hash &h, const char *name, const NdtGridParameters &g)
    {
        h.text(name);
        for (double v : {g.bounding_box_min_X, g.bounding_box_min_Y, g.bounding_box_min_Z, g.bounding_box_max_X, g.bounding_box_max_Y,
                         g.bounding_box_max_Z, g.bounding_box_extension, g.resolution_X, g.resolution_Y, g.resolution_Z})
            h.value(v);
        for (int v : {g.number_of_buckets_X, g.number_of_buckets_Y, g.number_of_buckets_Z})
            h.value(v);
        h.value(g.number_of_buckets);
    }

    // writing and reading plain values and Eigen matrices
    class Writer
    {
    public:
        explicit Writer(std::ofstream &out) : out_(out) {}
        template <typename T>
        void value(const T &v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            out_.write(reinterpret_cast<const char *>(&v), sizeof(T));
        }
        template <typename M>
        void matrix(const M &m)
        {
            for (int i = 0; i < m.size(); i++)
                value(m.data()[i]);
        }
        template <typename V, typename F>
        void list(const std::vector<V> &v, F each)
        {
            value(static_cast<std::uint64_t>(v.size()));
            for (const auto &e : v)
                each(e);
        }

    private:
        std::ofstream &out_;
    };

    class Reader
    {
    public:
        explicit Reader(std::ifstream &in) : in_(in) {}
        bool ok() const { return static_cast<bool>(in_); }
        template <typename T>
        void value(T &v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            in_.read(reinterpret_cast<char *>(&v), sizeof(T));
        }
        template <typename M>
        void matrix(M &m)
        {
            for (int i = 0; i < m.size(); i++)
                value(m.data()[i]);
        }
        // a list of at most `limit` elements (guards against a damaged count)
        template <typename V, typename F>
        bool list(std::vector<V> &v, F each, std::uint64_t limit = 100000000ULL)
        {
            std::uint64_t n = 0;
            value(n);
            if (!in_ || n > limit)
                return false;
            v.resize(static_cast<size_t>(n));
            for (auto &e : v)
                each(e);
            return static_cast<bool>(in_);
        }

    private:
        std::ifstream &in_;
    };

    void write_window(Writer &w, const Window &win)
    {
        w.list(win.poses, [&](const Eigen::Affine3d &p) { w.matrix(p.matrix()); });
        w.list(win.motion_model_poses, [&](const Eigen::Affine3d &p) { w.matrix(p.matrix()); });
        w.list(win.timestamps, [&](const std::pair<double, double> &t) { w.value(t.first); w.value(t.second); });
        w.list(win.imu_angles, [&](const Eigen::Vector3d &v) { w.matrix(v); });
        w.list(win.imu_acc, [&](const Eigen::Vector3d &v) { w.matrix(v); });
        w.matrix(win.gyro_bias.mean);
        w.matrix(win.gyro_bias.covariance);
        w.matrix(win.velocity.start);
        w.matrix(win.velocity.end);
        w.matrix(win.velocity.end_covariance);
    }

    bool read_window(Reader &r, Window &win)
    {
        const auto affine = [&](Eigen::Affine3d &p) { r.matrix(p.matrix()); };
        const auto vec3 = [&](Eigen::Vector3d &v) { r.matrix(v); };
        if (!r.list(win.poses, affine) || !r.list(win.motion_model_poses, affine) ||
            !r.list(win.timestamps, [&](std::pair<double, double> &t) { r.value(t.first); r.value(t.second); }) ||
            !r.list(win.imu_angles, vec3) || !r.list(win.imu_acc, vec3))
            return false;
        r.matrix(win.gyro_bias.mean);
        r.matrix(win.gyro_bias.covariance);
        r.matrix(win.velocity.start);
        r.matrix(win.velocity.end);
        r.matrix(win.velocity.end_covariance);
        return r.ok();
    }
} // namespace

std::uint64_t odometry_settings_hash(const OdometryConfig &config)
{
    // categories whose settings the odometry doesn't read
    static const std::set<std::string> not_odometry = {"loop_closure", "consistency", "lidar_odometry_debug_info", "paths", "misc", "version_info"};
    TomlIO io;
    std::set<std::string> excluded;
    for (const auto &[category, keys] : io.CATEGORIES)
        if (not_odometry.count(category))
            excluded.insert(keys.begin(), keys.end());

    Hash h;
    for (const auto &[name, member] : io.POINTERS) // a std::map: always in the same order
    {
        if (excluded.count(name))
            continue;
        // the hold-out mode changes the odometry, but off it adds nothing, so snapshots made before it existed still match
        if (name == "holdout_block_s" && config.holdout_block_s == 0.0)
            continue;
        h.text(name);
        std::visit(
            [&](auto pointer)
            {
                const auto &v = config.*pointer;
                if constexpr (std::is_same_v<std::decay_t<decltype(v)>, std::string>)
                    h.text(v);
                else
                    h.value(v);
            },
            member);
    }
    hash_grid(h, "in_out_params_indoor", config.in_out_params_indoor);
    hash_grid(h, "in_out_params_outdoor", config.in_out_params_outdoor);
    h.text("motion_model_correction");
    for (double v : {config.motion_model_correction.px, config.motion_model_correction.py, config.motion_model_correction.pz,
                     config.motion_model_correction.om, config.motion_model_correction.fi, config.motion_model_correction.ka})
        h.value(v);
    return h.h;
}

bool write_odometry_snapshot(const std::filesystem::path &file, const OdometrySnapshot &snapshot, const OdometryConfig &config, std::string &why)
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
    w.value(odometry_settings_hash(config));
    w.list(snapshot.windows, [&](const Window &win) { write_window(w, win); });
    w.matrix(snapshot.m_g.matrix());
    w.value(snapshot.total_length_of_calculated_trajectory);
    w.value(snapshot.consecutive_distance);
    out.flush();
    if (!out)
    {
        why = "writing '" + file.string() + "' failed";
        return false;
    }
    return true;
}

bool read_odometry_snapshot(const std::filesystem::path &file, const OdometryConfig &config, OdometrySnapshot &snapshot, std::string &why)
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
        why = "'" + file.string() + "' is not an odometry snapshot of format " + std::to_string(FORMAT_VERSION);
        return false;
    }
    std::uint64_t hash = 0;
    r.value(hash);
    if (hash != odometry_settings_hash(config))
    {
        why = "the snapshot was made with other odometry settings than the given parameters (only loop closure, consistency, "
              "debug logging and output paths may differ)";
        return false;
    }
    OdometrySnapshot s;
    if (!r.list(s.windows, [&](Window &win) { read_window(r, win); }, 10000000ULL))
    {
        why = "'" + file.string() + "' is damaged (window list)";
        return false;
    }
    r.matrix(s.m_g.matrix());
    r.value(s.total_length_of_calculated_trajectory);
    r.value(s.consecutive_distance);
    if (!r.ok())
    {
        why = "'" + file.string() + "' is damaged (run state)";
        return false;
    }
    snapshot = std::move(s);
    return true;
}
