#pragma once

// The motion-model terms of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp): the gyro-bias prior and the
// velocity continuity (and, from stage 2 commit 5, the edges between consecutive nodes). Each adds to the normal equations
// and to the running motion cost of WindowEquations.

#include "optimizer_options.h"
#include "window.h"
#include "window_equations.h"

// the gyro bias stays near the previous window's estimate: prior mean and covariance from rotation_options (only with
// bias estimation)
void add_gyro_bias_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelRotationOptions &rotation_options);

// velocity continuity on the world-frame translation (white-noise acceleration), only in a window without the
// accelerometer term and with timestamps: residual = change of velocity between consecutive steps [m/s], sigma =
// acc_sigma_mps2 * dt; with velocity_prior.use the first step is tied to the previous window's final velocity
void add_velocity_continuity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                             const MotionModelOptions &motion_model, const VelocityPrior &velocity_prior);
