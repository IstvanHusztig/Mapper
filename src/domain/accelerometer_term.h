#pragma once

// The accelerometer term of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp; imu_always_on): the
// acceleration terms with the node rotations and the start and end velocity unknowns, the velocity priors, and the
// acceleration-offset prior. Each adds to the normal equations and to the running motion cost of WindowEquations, in this
// order, and only in a window with the term (WindowUnknowns::acc_term).

#include "optimizer_options.h"
#include "window.h"
#include "window_equations.h"

// d(world rotation increment) / d(om, fi, ka) at a node's attitude, by central differences (independent of the angle
// convention): column j is the rotation vector of R(angles + h e_j) R(angles - h e_j)^T / 2h. No inverse at fi = +-90 deg.
Eigen::Matrix3d tait_bryan_rotation_jacobian(const Eigen::Affine3d &m);

// the acceleration terms over the anchors (nodes about span_s apart), the start velocity over [node 0, first anchor] and
// the end velocity over the last two anchors to the last node; a term whose stretch has an IMU gap (longer than 2 span_s)
// is dropped. Returns whether the end velocity was tied by its term.
bool add_acceleration_terms(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                            const MotionModelTranslationImuOptions &translation_imu_options, const Eigen::Vector3d &acc_offset);

// the start velocity's prior: the previous window's end velocity
void add_start_velocity_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                              const MotionModelTranslationImuOptions &translation_imu_options);

// at rest (translation_imu_options.at_rest): both velocities zero
void add_zero_velocity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                       const MotionModelTranslationImuOptions &translation_imu_options);

// when the end velocity has no term (end_tied false: a gap at the end), a weak tie of it to the start velocity (sigma 1 m/s)
void add_weak_end_velocity_tie(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, bool end_tied);

// the three above, in this order
void add_velocity_priors(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelTranslationImuOptions &translation_imu_options, bool end_tied);

// the acceleration offset stays near the prior carried from the previous window
void add_acceleration_offset_prior(WindowEquations &equations, const WindowUnknowns &unknowns,
                                   const MotionModelTranslationImuOptions &translation_imu_options, const Eigen::Vector3d &acc_offset);
