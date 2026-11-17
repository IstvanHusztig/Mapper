#pragma once

// The motion-model terms of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp): the edges between
// consecutive nodes, the gyro-bias prior and the velocity continuity. Each adds to the normal equations and to the running
// motion cost of WindowEquations, in that order.

#include "optimizer_options.h"
#include "window.h"
#include "window_equations.h"

// the edges between consecutive nodes: each node pair's relative pose should match the motion model's (the IMU's), rotation
// and position in one block (rotation sigmas time-scaled from the gyro's noise when set, position weights scaled by
// imu_position_information_scale while the accelerometer term is active); with bias estimation the rotation increment is
// corrected by the bias update, which couples each edge to the bias unknowns. The edges' cost is added only when
// equations.with_cost is set.
void add_motion_edge_terms(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                           const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options);

// the gyro bias stays near the previous window's estimate: prior mean and covariance from rotation_options (only with
// bias estimation)
void add_gyro_bias_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelRotationOptions &rotation_options);

// velocity continuity on the world-frame translation (white-noise acceleration), only in a window without the
// accelerometer term and with timestamps: residual = change of velocity between consecutive steps [m/s], sigma =
// acc_sigma_mps2 * dt; with velocity_prior.use the first step is tied to the previous window's final velocity
void add_velocity_continuity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                             const MotionModelOptions &motion_model, const VelocityPrior &velocity_prior);
