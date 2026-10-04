#pragma once

// The synthetic scenes and recordings the unit tests run on: axis-aligned plane faces, their sampling, and the pose
// errors against a known truth (shared by scan_to_map_test, loop_closure_test and consistency_test).

#include "run_reporting.h"

#include <Eigen/Dense>
#include <structures.h>

#include <vector>

namespace synthetic
{
struct NullLogger : Logger
{
    int messages = 0;
    void write(LogLevel, std::string_view) override { messages++; }
};

// an axis-aligned plane face of the scene: points p with p[axis] == value, within [lo, hi] on the other two axes
struct Face
{
    int axis;
    double value;
    Eigen::Vector3d lo;
    Eigen::Vector3d hi;
};

struct Scene
{
    std::vector<Face> faces;
    Eigen::Vector3d centre;
};

// largest position [m] and rotation [rad] difference between two trajectories
struct PoseError
{
    double position = 0.0;
    double rotation = 0.0;
};

// a box with the given corners; open_x leaves out the two faces normal to x (a corridor)
Scene box_scene(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi, bool open_x);

// a point on face f at in-plane coordinates (u, v) in [0, 1], moved by jitter along the face normal
Eigen::Vector3d face_point(const Face &f, double u, double v, double jitter);

double face_area(const Face &f);

// the scene sampled on a 5 cm grid, with a +-5 mm jitter along the face normal (fixed seed)
std::vector<Point3Di> sample_scene(const Scene &s);

Eigen::Affine3d pose(const Eigen::Vector3d &position, double yaw_rad);

PoseError max_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b);
}
