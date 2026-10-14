#include "synthetic_recording.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_set>

namespace synthetic
{
// a box with the given corners; open_x leaves out the two faces normal to x (a corridor)
Scene box_scene(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, bool open_x)
{
    Scene s;
    for (int axis = 0; axis < 3; axis++)
    {
        if (open_x && axis == 0)
            continue;
        s.faces.push_back({axis, lo[axis], lo, hi});
        s.faces.push_back({axis, hi[axis], lo, hi});
    }
    s.centre = 0.5 * (lo + hi);
    return s;
}

Scene shifted(const Scene &s, const Eigen::Vector3d &offset)
{
    Scene out = s;
    for (Face &f : out.faces)
    {
        f.value += offset[f.axis];
        f.lo += offset;
        f.hi += offset;
    }
    out.centre += offset;
    return out;
}

// a point on face f at in-plane coordinates (u, v) in [0, 1], moved by jitter along the face normal
Eigen::Vector3d face_point(const Face &f, double u, double v, double jitter)
{
    const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
    Eigen::Vector3d p;
    p[f.axis] = f.value + jitter;
    p[a] = f.lo[a] + u * (f.hi[a] - f.lo[a]);
    p[b] = f.lo[b] + v * (f.hi[b] - f.lo[b]);
    return p;
}

double face_area(const Face &f)
{
    const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
    return (f.hi[a] - f.lo[a]) * (f.hi[b] - f.lo[b]);
}

// the scene sampled on a 5 cm grid, with a +-5 mm jitter along the face normal (fixed seed)
std::vector<Point3Di> sample_scene(const Scene &s)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> jitter(-0.005, 0.005);
    std::vector<Point3Di> points;
    for (const Face &f : s.faces)
    {
        const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
        const int na = static_cast<int>(std::round((f.hi[a] - f.lo[a]) / 0.05));
        const int nb = static_cast<int>(std::round((f.hi[b] - f.lo[b]) / 0.05));
        for (int i = 0; i <= na; i++)
            for (int j = 0; j <= nb; j++)
            {
                Point3Di p{};
                p.point = face_point(f, double(i) / na, double(j) / nb, jitter(rng));
                p.index_point = static_cast<int>(points.size());
                points.push_back(p);
            }
    }
    return points;
}

Eigen::Affine3d pose(const Eigen::Vector3d &position, double yaw_rad)
{
    Eigen::Affine3d m = Eigen::Affine3d::Identity();
    m.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    m.translation() = position;
    return m;
}

PoseError max_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
{
    PoseError e;
    for (size_t k = 0; k < a.size(); k++)
    {
        e.position = std::max(e.position, (a[k].translation() - b[k].translation()).norm());
        e.rotation = std::max(e.rotation, Eigen::AngleAxisd(a[k].linear() * b[k].linear().transpose()).angle());
    }
    return e;
}

// ---- the loop recording ----

namespace
{
    void add_box(Scene &s, const Eigen::Vector3d &lo, const Eigen::Vector3d &hi)
    {
        const Scene b = box_scene(lo, hi, false);
        s.faces.insert(s.faces.end(), b.faces.begin(), b.faces.end());
    }

    constexpr double pi = 3.14159265358979323846;

    // A corner turns left by 90 deg: the curvature ramps linearly from 0 to 1 / corner_radius over corner_ramp_m,
    // stays there, and ramps back to 0 over corner_ramp_m (clothoid transitions), so the turn rate never jumps.
    constexpr double corner_radius = 2.0;
    constexpr double corner_ramp_m = 1.0; // 0.5 s at 2 m/s
    constexpr double corner_arc_m = 0.5 * pi * corner_radius - corner_ramp_m;
    constexpr double corner_length = corner_arc_m + 2.0 * corner_ramp_m;

    // heading change [rad] after arc length u into a corner
    double corner_heading(double u)
    {
        const double k = 1.0 / corner_radius;
        if (u <= corner_ramp_m)
            return 0.5 * k * u * u / corner_ramp_m;
        if (u <= corner_ramp_m + corner_arc_m)
            return 0.5 * k * corner_ramp_m + k * (u - corner_ramp_m);
        const double rest = corner_length - u;
        return 0.5 * pi - 0.5 * k * rest * rest / corner_ramp_m;
    }

    // position after arc length u into a corner that starts at the origin heading along x: Simpson's rule on each of the
    // three sections, where the heading is smooth
    Eigen::Vector2d corner_offset(double u)
    {
        const auto simpson = [](double from, double to) {
            const int n = 64;
            const double h = (to - from) / n;
            Eigen::Vector2d sum = Eigen::Vector2d::Zero();
            for (int i = 0; i <= n; i++)
            {
                const double a = corner_heading(from + i * h);
                const double w = (i == 0 || i == n) ? 1.0 : (i % 2 == 1 ? 4.0 : 2.0);
                sum += w * Eigen::Vector2d(std::cos(a), std::sin(a));
            }
            return Eigen::Vector2d(sum * h / 3.0);
        };
        const double bounds[] = {0.0, corner_ramp_m, corner_ramp_m + corner_arc_m, corner_length};
        Eigen::Vector2d offset = Eigen::Vector2d::Zero();
        for (int i = 0; i < 3 && u > bounds[i]; i++)
            offset += simpson(bounds[i], std::min(u, bounds[i + 1]));
        return offset;
    }

    // how far a corner advances along each axis (symmetric about its bisector)
    double corner_extent() { return corner_offset(corner_length).x(); }

    // the centreline: straights along y = -4, x = 8, y = 4 and x = -8, joined by the corners around the block's
    // corners, counter-clockwise from (0, -4)
    double path_length()
    {
        const double d = corner_extent();
        return 2 * (16.0 - 2 * d) + 2 * (8.0 - 2 * d) + 4 * corner_length;
    }

    struct PathPoint
    {
        Eigen::Vector2d position;
        double heading;
    };

    // position and heading at arc length s (wrapped to one lap)
    PathPoint path_at(double s)
    {
        s = std::fmod(s, path_length());
        const double d = corner_extent();
        struct Piece
        {
            double length;
            Eigen::Vector2d start;
            double heading;  // at the start
            bool is_corner; // turning left
        };
        const Piece pieces[] = {
            {8.0 - d, {0.0, -4.0}, 0.0, false},
            {corner_length, {8.0 - d, -4.0}, 0.0, true},
            {8.0 - 2 * d, {8.0, -4.0 + d}, 0.5 * pi, false},
            {corner_length, {8.0, 4.0 - d}, 0.5 * pi, true},
            {16.0 - 2 * d, {8.0 - d, 4.0}, pi, false},
            {corner_length, {-8.0 + d, 4.0}, pi, true},
            {8.0 - 2 * d, {-8.0, 4.0 - d}, 1.5 * pi, false},
            {corner_length, {-8.0, -4.0 + d}, 1.5 * pi, true},
            {8.0 - d, {-8.0 + d, -4.0}, 2.0 * pi, false},
        };
        for (const Piece &p : pieces)
        {
            if (s > p.length)
            {
                s -= p.length;
                continue;
            }
            const Eigen::Vector2d dir(std::cos(p.heading), std::sin(p.heading));
            if (!p.is_corner)
                return {p.start + s * dir, p.heading};
            const Eigen::Vector2d left(-dir.y(), dir.x());
            const Eigen::Vector2d o = corner_offset(s);
            return {p.start + o.x() * dir + o.y() * left, p.heading + corner_heading(s)};
        }
        return {{0.0, -4.0}, 2.0 * pi};
    }
}

Scene loop_scene()
{
    Scene s = box_scene({-10.0, -6.0, 0.0}, {10.0, 6.0, 3.0}, false); // outer walls, floor, ceiling
    add_box(s, {-6.0, -2.0, 0.0}, {6.0, 2.0, 3.0});                    // the block
    // pillars (0.3 m, full height) against the outer walls and the block, irregularly spaced
    for (double x : {-8.7, -5.1, -1.6, 0.9, 4.4, 7.8})
        add_box(s, {x, -6.0, 0.0}, {x + 0.3, -5.7, 3.0});
    for (double x : {-7.4, -3.3, 2.2, 5.6})
        add_box(s, {x, 5.7, 0.0}, {x + 0.3, 6.0, 3.0});
    for (double y : {-4.6, -0.7, 3.1})
        add_box(s, {9.7, y, 0.0}, {10.0, y + 0.3, 3.0});
    for (double y : {-3.2, 1.4})
        add_box(s, {-10.0, y, 0.0}, {-9.7, y + 0.3, 3.0});
    for (double x : {-4.2, 1.7, 3.9})
        add_box(s, {x, -2.3, 0.0}, {x + 0.3, -2.0, 3.0});
    for (double x : {-2.6, 0.3})
        add_box(s, {x, 2.0, 0.0}, {x + 0.3, 2.3, 3.0});
    // boxes on the floor (0.6 m x 0.6 m x 1 m)
    add_box(s, {-3.0, -5.6, 0.0}, {-2.4, -5.0, 1.0});
    add_box(s, {8.9, 0.8, 0.0}, {9.5, 1.4, 1.0});
    add_box(s, {-1.2, 5.0, 0.0}, {-0.6, 5.6, 1.0});
    add_box(s, {-9.6, -0.6, 0.0}, {-9.0, 0.0, 1.0});
    s.centre = Eigen::Vector3d(0.0, 0.0, 1.5);
    return shifted(s, grid_offset);
}

Recording loop_recording(int laps)
{
    constexpr double speed = 2.0;  // [m/s]
    constexpr double height = 1.5; // [m]
    constexpr int nodes_per_window = 20;
    constexpr int points_per_node = 150;
    constexpr double min_range = 1.0, max_range = 15.0;

    Recording r;
    r.scene = loop_scene();
    r.node_dt = 0.05;
    r.lap_time_s = path_length() / speed;
    const int n_windows = static_cast<int>(laps * r.lap_time_s / r.node_dt) / nodes_per_window;

    std::vector<double> areas;
    for (const Face &f : r.scene.faces)
        areas.push_back(face_area(f));
    std::mt19937 rng(11);
    std::discrete_distribution<int> pick_face(areas.begin(), areas.end());
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> jitter(-0.005, 0.005);

    for (int w = 0; w < n_windows; w++)
    {
        Window window;
        std::vector<Eigen::Affine3d> truth;
        std::vector<Point3Di> points;
        for (int k = 0; k < nodes_per_window; k++)
        {
            const double t = (w * nodes_per_window + k) * r.node_dt;
            const PathPoint pp = path_at(speed * t);
            const Eigen::Affine3d m = pose(Eigen::Vector3d(pp.position.x(), pp.position.y(), height) + grid_offset, pp.heading);
            truth.push_back(m);
            window.timestamps.push_back({t, t});

            const Eigen::Affine3d to_node = m.inverse();
            int accepted = 0;
            for (int attempt = 0; accepted < points_per_node && attempt < 100 * points_per_node; attempt++)
            {
                const Face &f = r.scene.faces[pick_face(rng)];
                const Eigen::Vector3d world = face_point(f, unit(rng), unit(rng), jitter(rng));
                const double range = (world - m.translation()).norm();
                if (range < min_range || range > max_range)
                    continue;
                Point3Di p{};
                p.point = to_node * world;
                p.timestamp = t + 0.5 * r.node_dt;
                p.intensity = 100.0f;
                p.index_pose = k;
                p.index_point = static_cast<int>(points.size());
                points.push_back(p);
                accepted++;
            }
        }
        window.poses = truth;
        window.motion_model_poses = truth;
        r.windows.push_back(std::move(window));
        r.truth.push_back(std::move(truth));
        r.points.push_back(std::move(points));
    }
    return r;
}

std::vector<Eigen::Affine3d> all_poses(const std::vector<Window> &windows)
{
    std::vector<Eigen::Affine3d> all;
    for (const Window &w : windows)
        all.insert(all.end(), w.poses.begin(), w.poses.end());
    return all;
}

std::vector<Eigen::Affine3d> all_poses(const std::vector<std::vector<Eigen::Affine3d>> &per_window)
{
    std::vector<Eigen::Affine3d> all;
    for (const auto &w : per_window)
        all.insert(all.end(), w.begin(), w.end());
    return all;
}

PoseError mean_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
{
    PoseError e;
    for (size_t k = 0; k < a.size(); k++)
    {
        e.position += (a[k].translation() - b[k].translation()).norm();
        e.rotation += Eigen::AngleAxisd(a[k].linear() * b[k].linear().transpose()).angle();
    }
    if (!a.empty())
    {
        e.position /= static_cast<double>(a.size());
        e.rotation /= static_cast<double>(a.size());
    }
    return e;
}

size_t occupied_voxels(const std::vector<Window> &windows, const std::vector<std::vector<Point3Di>> &points, double voxel)
{
    const auto cell = [voxel](double c) { return static_cast<uint64_t>(static_cast<int64_t>(std::floor(c / voxel)) + (1 << 20)) & 0x1FFFFF; };
    std::unordered_set<uint64_t> occupied;
    for (size_t w = 0; w < windows.size(); w++)
        for (const Point3Di &p : points[w])
        {
            const Eigen::Vector3d g = windows[w].poses[p.index_pose] * p.point;
            occupied.insert((cell(g.x()) << 42) | (cell(g.y()) << 21) | cell(g.z()));
        }
    return occupied.size();
}

double distance_to_scene(const Scene &s, const Eigen::Vector3d &p)
{
    double best = 1e9;
    for (const Face &f : s.faces)
    {
        const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
        if (p[a] < f.lo[a] - 1e-9 || p[a] > f.hi[a] + 1e-9 || p[b] < f.lo[b] - 1e-9 || p[b] > f.hi[b] + 1e-9)
            continue;
        best = std::min(best, std::abs(p[f.axis] - f.value));
    }
    return best;
}
}
