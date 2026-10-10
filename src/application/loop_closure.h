#pragma once

#include <Eigen/Dense>
#include <vector>
#include "input_ports.h"
#include "odometry_config.h"
#include "run_reporting.h"
#include "window.h"

// Automatic loop closure on the finished odometry (before the consistency pass and the export):
//  1. the windows are grouped into segments of loop_closure_segment_windows windows,
//  2. revisits are segment pairs far apart in time but close in space,
//  3. each revisit is registered against the map around the earlier visit (coarse voxel search,
//     then point-to-distribution refinement) and accepted only if the overlap is high,
//  4. a pose graph over the segment poses (odometry edges + accepted loop edges) is optimized,
//  5. the correction is interpolated onto every trajectory node of every window.
// Returns the number of accepted loop edges.
int run_loop_closure(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);

// Step 4 on its own, for tests: the segment poses, the accepted loop edges, and the check of the result.
struct LoopClosureEdge
{
    int i = 0;                                                // earlier segment
    int j = 0;                                                // later segment
    Eigen::Affine3d correction = Eigen::Affine3d::Identity(); // world transform moving segment j onto i
    double fitness = 0.0;
    bool cross_floor = false; // between floors: ties the translation, the rotation only weakly (loop_closure_cross_floor_*)
};

// Levenberg-Marquardt on the segment poses: odometry edges between consecutive segments, loop edges, first pose fixed.
// A step is kept only if the cost falls (a plain Gauss-Newton step first, damping only when it fails). With
// loop_closure_kernel_scale > 0 the loop edges cost c^2 log(1 + s^2 / c^2) instead of s^2 (s: the edge's residual in sigmas),
// solved by reweighting each step with 1 / (1 + s^2 / c^2) at the current poses, starting from the plain least-squares
// solution (from the odometry's poses every true loop would look like an outlier); odometry edges stay quadratic.
std::vector<Eigen::Affine3d> optimize_loop_pose_graph(const std::vector<Eigen::Affine3d> &initial, const std::vector<LoopClosureEdge> &loops,
                                                      const OdometryConfig &params);

// How far the optimized poses are from each loop edge's measurement: (initial_i^-1 correction initial_j)^-1 (optimized_i^-1 optimized_j).
struct LoopEdgeResidual
{
    double translation_m = 0.0;
    double angle_deg = 0.0;
};
std::vector<LoopEdgeResidual> loop_edge_residuals(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                                                  const std::vector<LoopClosureEdge> &loops);

// How far a loop edge's residual is beyond the drop limits (loop_closure_max_residual_m / _deg); above 1 it is a candidate
// for dropping. A cross-floor edge is judged by its translation only (its rotation is weak by design).
double loop_edge_drop_ratio(const LoopClosureEdge &edge, const LoopEdgeResidual &residual, const OdometryConfig &params);

// The optimized poses may be applied: all finite, and no segment moved further than loop_closure_max_correction_m plus
// the largest accepted loop correction (a pose graph that ran away). max_shift_m receives the largest segment correction.
bool loop_pose_graph_plausible(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                               const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params, double &max_shift_m);
