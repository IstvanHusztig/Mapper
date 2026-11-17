#pragma once

// The solver part of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp), after every term has been added:
// the origin fix, the covariance of the extra unknowns, the damped solve and the step applied to the window.

#include "normal_equations.h"
#include "optimizer_options.h"
#include "run_reporting.h"
#include "scan_to_map.h"
#include "window.h"
#include "window_equations.h"

#include <Eigen/Sparse>
#include <vector>

// weight 1 / sigma^2 on the first node's six parameters (its right-hand side is zero): node 0 stays at the previous
// window's end
void fix_origin(WindowEquations &equations, const MotionModelOptions &motion_model);

// marginal covariance of the extra unknowns (gyro bias, acceleration offset, end velocity): their block of the inverse of the
// undamped normal matrix. Only meaningful when solved; each block only when its unknowns exist.
struct ExtraCovariances
{
    bool solved = false;
    Eigen::Matrix3d gyro_bias = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d acceleration_offset = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d end_velocity = Eigen::Matrix3d::Zero();
};
ExtraCovariances extra_unknowns_covariance(const Eigen::SparseMatrix<double> &AtPA, const WindowUnknowns &unknowns, PatternCachedCholesky &solver);

// the damped solve: lm_factor on the pose parameters only (AtPA gets the damping added); the step, one entry per unknown
struct WindowStep
{
    bool solved = false;
    std::vector<double> h_x;
};
WindowStep solve_window_step(Eigen::SparseMatrix<double> &AtPA, const Eigen::SparseMatrix<double> &AtPB, const WindowUnknowns &unknowns,
                             double lm_factor, PatternCachedCholesky &solver);

// applies a step: the nodes' new poses, the bias (clamped), the acceleration offset and the velocities, and delta (the sum of
// |updates|; 1e6 when not applied). A node that would end 1 m or more from its guess ("big jump") rejects the whole step.
// Counts big-jump nodes and failed solves in the workspace. Returns whether the step was applied.
bool apply_window_step(Window &window, const WindowStep &step, const WindowUnknowns &unknowns, const MotionModelRotationOptions &rotation_options,
                       Eigen::Vector3d &acc_offset, double &delta, ScanToMapWorkspace &workspace, Logger &logger);
