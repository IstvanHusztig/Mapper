#pragma once

// Binary files of the run (odometry snapshot, window capture): plain values, Eigen matrices, lists and windows, written in
// the machine's byte order (little-endian on x86/x64), so that doubles come back bit for bit.

#include "window.h"

#include <cstdint>
#include <fstream>
#include <type_traits>
#include <vector>

namespace binary_io
{
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

inline void write_window(Writer &w, const Window &win)
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

inline bool read_window(Reader &r, Window &win)
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
} // namespace binary_io
