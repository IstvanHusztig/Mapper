#pragma once

// The NDT point terms of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp): every point against its cell
// in the indoor grid and (use_hierarchical_rgd) the outdoor grid, the degeneracy projection of the node blocks, and the
// window's translation information.

#include "ndt_map.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "scan_to_map.h"
#include "window.h"
#include "window_equations.h"
#include <structures.h>

#include <cmath>
#include <vector>

// Huber kernel on an NDT point term with squared Mahalanobis distance s = r^T Sigma^-1 r: weight 1 and cost s up to
// the cutoff c, beyond it weight c / sqrt(s) (IRLS) and cost 2 c sqrt(s) - c^2. Same formula as Codex's diagnostic
// (cbca36d, ndt_huber_evaluate).
struct HuberTerm
{
    double weight, cost;
};
inline HuberTerm ndt_huber(double s, double c)
{
    if (c <= 0.0 || s <= c * c)
        return {1.0, s};
    const double d = std::sqrt(s);
    return {c / d, 2.0 * c * d - c * c};
}

// what the point loops leave for the rest of the call, per node
struct NdtSums
{
    // the node blocks and right-hand sides, only with the degeneracy projection (which adds them to the matrix afterwards);
    // without it the points are added to the matrix directly
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> blocks;
    std::vector<Eigen::Matrix<double, 6, 1>> rhs;
    // sum of n * n^T over the surface normals the points are matched to (indoor grid): geometric constraint of the
    // translation, independent of the in-plane pull of the NDT cells; with the projection or when asked for
    std::vector<Eigen::Matrix3d> normal_information;
    // with the cost (lm_step_check): the NDT cost and the matched and unmatched point terms
    std::vector<double> node_cost;
    std::vector<int> node_matched, node_unmatched;
};

// the point terms of both grids, per node in parallel (a node's points in point order, so the result is the same with or
// without threads); planarity / range weights, the Huber kernel and the information scale as set. Throws
// std::domain_error on a non-finite distance with the Huber kernel on. timing: the indoor and outdoor phases.
NdtSums sum_ndt_point_terms(WindowEquations &equations, const Window &window, const std::vector<Point3Di> &intermediate_points,
                            NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor, const ScanToMapOptions &scan_to_map,
                            bool normal_information_wanted, bool with_cost, ScanToMapTiming *timing);

// degeneracy-aware NDT terms (degenerate_projection_ratio_full > 0): along a translation direction whose share of the
// window's geometric information is small the node blocks carry no information (M B M, M g with M = diag(P, I)); every
// node block goes into the matrix here
void add_projected_ndt_blocks(WindowEquations &equations, const NdtSums &sums, const ScanToMapOptions &scan_to_map);

// the window's translation information: the sum of the nodes' normal information (world frame)
Eigen::Matrix3d window_translation_information(const NdtSums &sums);
