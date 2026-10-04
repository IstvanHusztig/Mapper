#pragma once

// The synthetic scenes and recordings the unit tests run on: axis-aligned plane faces, their sampling, and the pose
// errors against a known truth (shared by scan_to_map_test, loop_closure_test and consistency_test).

#include "run_reporting.h"
#include "window.h"

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

// ---- a recording that drives laps around a block (loop closure and consistency tests) ----

// A floor of 20 m x 12 m x 3 m with a 12 m x 4 m block in the middle, so that four corridors, 4 m wide, form a loop.
// Pillars and boxes stand along the walls at irregular spacing, so that no corridor is featureless (a straight corridor
// with plain walls is degenerate along its length) and no stretch repeats another.
Scene loop_scene();

// One run through loop_scene: the true node poses and the points each node sees, cut into windows.
struct Recording
{
    Scene scene;
    double node_dt = 0.0;                            // [s]
    double lap_time_s = 0.0;                         // time for one lap [s]
    std::vector<Window> windows;                     // poses and motion_model_poses = the truth, timestamps per node
    std::vector<std::vector<Eigen::Affine3d>> truth; // the true poses, per window and node
    std::vector<std::vector<Point3Di>> points;       // per window: points in their node's frame, index_pose = node in window
};

// laps around the block's centreline (a rounded rectangle, 44.4 m) at 2 m/s, 1.5 m above the floor, moving forward
// along the body x axis; in each corner the turn rate ramps up to 1 rad/s over 0.5 s and back down over 0.5 s, as a
// real turn does (no step). Nodes every 50 ms, windows of 20 nodes (1 s). Each node sees 150 points sampled on the faces
// between 1 m and 15 m away (no occlusion), with +-5 mm noise along the face normal, from fixed seeds. A point's
// timestamp is its node's time plus half a node interval, so that the consistency pass assigns it to the same node.
Recording loop_recording(int laps);

// all node poses, in order
std::vector<Eigen::Affine3d> all_poses(const std::vector<Window> &windows);
std::vector<Eigen::Affine3d> all_poses(const std::vector<std::vector<Eigen::Affine3d>> &per_window);

// mean position [m] and rotation [rad] difference between two trajectories
PoseError mean_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b);

// map sharpness as tools/eval_odometry.py measures it: the number of occupied voxels of edge `voxel` [m] when every
// window's points are placed with its poses
size_t occupied_voxels(const std::vector<Window> &windows, const std::vector<std::vector<Point3Di>> &points, double voxel);

// distance of p to the nearest face of the scene that contains it in its extent
double distance_to_scene(const Scene &s, const Eigen::Vector3d &p);
}
