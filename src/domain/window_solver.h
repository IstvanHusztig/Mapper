#pragma once

// The window's side of the solver (window_problem.h): the origin fix, and a step applied to the window. The covariance and
// the damped solve are the solver's (lm_solver.h).

#include "normal_equations.h"
#include "optimizer_options.h"
#include "run_reporting.h"
#include "scan_to_map.h"
#include "window.h"
#include "window_equations.h"
#include "window_noise_model.h"

#include <Eigen/Sparse>
#include <vector>

// the noise model's origin weights on the first node's six parameters (its right-hand side is zero): node 0 stays at the
// previous window's end
void fix_origin(WindowEquations &equations, const WindowNoiseModel &noise);

// a solver step (lm_solver.h): one entry per unknown; solved false: the solve failed
struct WindowStep
{
    bool solved = false;
    std::vector<double> h_x;
};

// applies a step: the nodes' new poses, the bias (clamped), the acceleration offset and the velocities, and delta (the sum of
// |updates|; 1e6 when not applied). A node that would end 1 m or more from its guess ("big jump") rejects the whole step.
// Counts big-jump nodes and failed solves in the workspace. Returns whether the step was applied.
bool apply_window_step(Window &window, const WindowStep &step, const WindowUnknowns &unknowns, const MotionModelRotationOptions &rotation_options,
                       Eigen::Vector3d &acc_offset, double &delta, ScanToMapWorkspace &workspace, Logger &logger);
