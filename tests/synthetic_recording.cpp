#include "synthetic_recording.h"

#include <algorithm>
#include <cmath>
#include <random>

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

}
