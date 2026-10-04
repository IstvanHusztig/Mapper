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
    constexpr double corner_radius = 2.0;

    // the centreline: straights along y = -4, x = 8, y = 4 and x = -8, joined by quarter circles of radius 2 around the
    // block's corners, counter-clockwise from (0, -4)
    double path_length() { return 2 * 12.0 + 2 * 4.0 + 2 * pi * corner_radius; }

    struct PathPoint
    {
        Eigen::Vector2d position;
        double heading;
    };

    // position and heading at arc length s (wrapped to one lap)
    PathPoint path_at(double s)
    {
        s = std::fmod(s, path_length());
        const double arc = 0.5 * pi * corner_radius;
        struct Piece
        {
            double length;
            Eigen::Vector2d start;
            double heading; // at the start
            bool is_arc;    // turning left
        };
        const Piece pieces[] = {
            {6.0, {0.0, -4.0}, 0.0, false},
            {arc, {6.0, -4.0}, 0.0, true},
            {4.0, {8.0, -2.0}, 0.5 * pi, false},
            {arc, {8.0, 2.0}, 0.5 * pi, true},
            {12.0, {6.0, 4.0}, pi, false},
            {arc, {-6.0, 4.0}, pi, true},
            {4.0, {-8.0, 2.0}, 1.5 * pi, false},
            {arc, {-8.0, -2.0}, 1.5 * pi, true},
            {6.0, {-6.0, -4.0}, 2.0 * pi, false},
        };
        for (const Piece &p : pieces)
        {
            if (s > p.length)
            {
                s -= p.length;
                continue;
            }
            const Eigen::Vector2d dir(std::cos(p.heading), std::sin(p.heading));
            if (!p.is_arc)
                return {p.start + s * dir, p.heading};
            const Eigen::Vector2d left(-dir.y(), dir.x());
            const Eigen::Vector2d centre = p.start + corner_radius * left;
            const double a = s / corner_radius;
            const Eigen::Vector2d r0 = p.start - centre;
            const Eigen::Vector2d r(std::cos(a) * r0.x() - std::sin(a) * r0.y(), std::sin(a) * r0.x() + std::cos(a) * r0.y());
            return {centre + r, p.heading + a};
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
    return s;
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
            const Eigen::Affine3d m = pose({pp.position.x(), pp.position.y(), height}, pp.heading);
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
