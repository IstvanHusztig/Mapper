#pragma once

// The motion-model terms of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp): the edges between
// consecutive nodes, the gyro-bias prior and the velocity continuity. Each adds to the normal equations and to the running
// motion cost of WindowEquations, in that order.

#include "optimizer_options.h"
#include "window.h"
#include "window_equations.h"
#include "window_noise_model.h"

#include <vector>

// the edges' weights (MotionEdgeWeights, motion_edge_weights) are in window_noise_model.h

// the edges between consecutive nodes: each node pair's relative pose should match the motion model's (the IMU's), rotation
// and position in one block; with bias estimation the rotation increment is corrected by the bias update, which couples
// each edge to the bias unknowns. With translation_imu_options.edges_follow_state, in a window whose guess was integrated
// from the accelerometer (guess_integrated, not at rest), the position measurement follows the start velocity
// (window.velocity.start against velocity_guess) and the acceleration offset acc_offset as the integrated guess does, which
// couples each edge to those unknowns; otherwise the position measurement is the guess's. The edges' cost is added only
// when equations.with_cost is set.
void add_motion_edges(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                      const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options,
                      const Eigen::Vector3d &acc_offset, const MotionEdgeWeights &weights);

// add_motion_edges with the noise model's edge weights
void add_motion_edge_terms(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window, const MotionModelOptions &motion_model,
                           const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options,
                           const Eigen::Vector3d &acc_offset);

// the gyro bias stays near the previous window's estimate: the noise model's prior (only with bias estimation)
void add_gyro_bias_prior(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window);

// velocity continuity on the world-frame translation (white-noise acceleration), only in a window without the
// accelerometer term and with timestamps: residual = change of velocity between consecutive steps [m/s], weighted as the
// noise model says; with its first-step prior the first step is tied to the previous window's final velocity
void add_velocity_continuity(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window);
