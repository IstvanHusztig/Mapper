#pragma once

#include <Eigen/Dense>
#include <vector>
#include "input_ports.h"
#include "odometry_config.h"
#include "run_reporting.h"
#include "window.h"

// Automatic loop closure on the finished odometry (before the consistency pass and the export):
//  1. the windows are grouped into segments of loop_closure_segment_windows windows,
//  2. revisits are segment pairs with a long path walked between them but close in space,
//  3. each revisit is registered against the map around the earlier visit (coarse voxel search,
//     then point-to-distribution refinement) and accepted only if the overlap is high,
//  4. a pose graph over the segment poses (odometry edges + accepted loop edges) is optimized,
//  5. the correction is interpolated onto every trajectory node of every window.
// Returns the number of accepted loop edges.
int run_loop_closure(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);

// Step 2, the path walked: for each window, the path [m] from the first window to its first node, summed over the
// windows' first nodes; a step counts only once the position has moved at least 0.1 m from the last counted one, so that
// the pose jitter of a person standing still adds nothing
std::vector<double> path_at_windows(const std::vector<Window> &windows);

// A segment as step 2 sees it: its mean node position and the path walked up to its begin and its end
struct RevisitSegment
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double path_begin = 0.0;
    double path_end = 0.0;
};

// Step 2 on the same floor: the earlier segments that qualify as revisits of segment j, longest path between first (so
// that a near revisit does not take the place of a far one). A segment i qualifies when at least loop_closure_min_path_m
// was walked from its end to j's begin, its centre lies within min(loop_closure_search_radius_m,
// loop_closure_radius_per_path * that path) horizontally and within loop_closure_max_vertical_offset_m vertically.
std::vector<int> same_floor_revisits(const std::vector<RevisitSegment> &segments, int j, const OdometryConfig &params);

// G2b, the same-floor revisits registered for segment j: of the qualifying ones (same_floor_revisits), the nearest, the
// nearest at least 3 segments away from it, the longest-path one, and the longest-path one at least 3 segments away from
// it; distinct, at most loop_closure_candidates. near / far: chosen as one of the two nearest / longest-path ones.
struct RevisitTry
{
    int i = 0;
    bool near = false;
    bool far = false;
};
std::vector<RevisitTry> revisit_tries(const std::vector<RevisitSegment> &segments, int j, const OdometryConfig &params);

// Step 4 on its own, for tests: the segment poses, the accepted loop edges, and the check of the result.
struct LoopClosureEdge
{
    int i = 0;                                                // earlier segment
    int j = 0;                                                // later segment
    Eigen::Affine3d correction = Eigen::Affine3d::Identity(); // world transform moving segment j onto i
    double fitness = 0.0;
    bool cross_floor = false; // between floors (loop_closure_cross_floor_*); such an edge is information-weighted
    // an information-weighted edge: omega weights its relative-pose residual in place of the fixed sigmas
    // (see loop_edge_information)
    bool information = false;
    Eigen::Matrix<double, 6, 6> omega = Eigen::Matrix<double, 6, 6>::Zero();
};

// The information matrix of a loop edge's relative-pose residual from its registration's information H (the normal matrix
// over the Tait-Bryan parameters of correction, the world transform moving segment j onto i): H mapped through the
// Jacobian G of the edge's measurement rel(pose_i, correction * pose_j) with respect to those parameters (G^-T H G^-1),
// with free_height the world-vertical shift eliminated as a free parameter, and scaled so that the best-determined
// translation direction has the sigma sigma_m. ok is false (and the result zero) if G or H can't be used.
Eigen::Matrix<double, 6, 6> loop_edge_information(const Eigen::Affine3d &pose_i, const Eigen::Affine3d &pose_j, const Eigen::Affine3d &correction,
                                                  const Eigen::Matrix<double, 6, 6> &H, bool free_height, double sigma_m, bool &ok);

// An information-weighted edge's sigmas in the frame of its shared region: the move of the region's centre (x, y, z, in
// metres) and the rotation about it (roll, pitch, yaw about world axes, in degrees); a direction the edge leaves free
// comes out very large.
Eigen::Matrix<double, 6, 1> loop_edge_region_sigmas(const Eigen::Affine3d &pose_i, const Eigen::Affine3d &pose_j, const LoopClosureEdge &edge,
                                                    const Eigen::Vector3d &centre);

// G2b, the same-floor edges kept for one segment from its accepted registrations: with edges_per_segment >= 2 the best near
// one and the best far one by overlap after the refinement (fitness), both only when their earlier segments are at least 3
// apart (otherwise the better one); with 1 the best one.
struct AcceptedRevisit
{
    LoopClosureEdge edge;
    bool near = false;
    bool far = false;
};
std::vector<LoopClosureEdge> revisit_slots(const std::vector<AcceptedRevisit> &accepted, int edges_per_segment);

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
    double weighted = -1.0; // information-weighted edges: sqrt(r^T omega r), the residual in edge sigmas; -1 otherwise
};
std::vector<LoopEdgeResidual> loop_edge_residuals(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                                                  const std::vector<LoopClosureEdge> &loops);

// How far a loop edge's residual is beyond the drop limits (loop_closure_max_residual_m / _deg); above 1 it is a candidate
// for dropping. An information-weighted edge is judged by its weighted residual in metres of loop_closure_edge_sigma_m
// (free directions don't count).
double loop_edge_drop_ratio(const LoopClosureEdge &edge, const LoopEdgeResidual &residual, const OdometryConfig &params);

// The least-squares cost of a pose-graph solution (odometry edges with the fixed sigmas, loop edges with their fixed
// sigmas or information; no robust kernel)
double loop_pose_graph_cost(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                            const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params);

// Which loop edge to drop after a solve (change loop-closure-fixes, G2a): of the edges beyond the drop limits
// (loop_edge_drop_ratio > 1), the one whose removal lowers the cost of the re-solved graph the most (leave one out); a
// false edge strains the others, so removing it frees the most cost, while a right edge near the limit leaves the
// strain in place. Returns loops.size() when no edge is beyond the limits.
size_t loop_edge_to_drop(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                         const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params);

// The optimized poses may be applied: all finite, and no segment moved further than loop_closure_max_correction_m plus
// the largest accepted loop correction (a pose graph that ran away). max_shift_m receives the largest segment correction.
bool loop_pose_graph_plausible(const std::vector<Eigen::Affine3d> &initial, const std::vector<Eigen::Affine3d> &optimized,
                               const std::vector<LoopClosureEdge> &loops, const OdometryConfig &params, double &max_shift_m);
