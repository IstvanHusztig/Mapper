#pragma once

// What the terms of a window's optimization (optimize_lidar_odometry, scan_to_map.cpp) share: the layout of the unknowns,
// and the normal equations with the cost they add to. Each term function writes only through WindowEquations, in a fixed
// order, so every entry receives its additions in the same order as before the split (docs/code-split-and-region-agents-plan.md,
// stage 2).

#include "normal_equations.h"
#include "optimizer_options.h"
#include "window.h"

#include <vector>

// The unknowns of one window: [pose_0 .. pose_n-1 | b_om b_fi b_ka (gyro bias) | e_x e_y e_z (acceleration offset) |
// v_start | v_end (accelerometer term)], and the band of the normal matrix
struct WindowUnknowns
{
    int n_nodes = 0;
    int n_pose_params = 0;      // 6 per node
    int n_params = 0;           // poses and the extra unknowns
    bool have_dt = false;       // a timestamp per node
    bool time_scaled_sigma = false;
    bool estimate_bias = false;
    bool acc_term = false;      // the accelerometer term with its offset and velocity unknowns
    int ib = 0;                 // first gyro-bias column
    int io = 0;                 // first acceleration-offset column
    int iv = 0;                 // start velocity
    int ie = 0;                 // end velocity
    // the accelerometer term's anchors: nodes about span_s apart (accelerometer term only)
    std::vector<int> anchors;
    // the pose-pose coupling reaches 17 parameters (two nodes) without the accelerometer term, and (last - first node) * 6 + 5
    // for its widest term; both depend only on the timestamps, so the band is known before anything is added
    int pose_band_offset = 17;
};

WindowUnknowns layout_window_unknowns(const Window &window, const MotionModelRotationOptions &rotation_options,
                                      const MotionModelTranslationImuOptions &translation_imu_options);

// the normal equations of one window and the cost of its motion and IMU terms
struct WindowEquations
{
    explicit WindowEquations(const WindowUnknowns &u)
        : matrix(u.n_pose_params, u.n_params - u.n_pose_params, u.pose_band_offset), rhs(Eigen::MatrixXd::Zero(u.n_params, 1))
    {
    }
    BandedNormalMatrix matrix; // stored by band (only the band is ever used)
    Eigen::MatrixXd rhs;       // n_params x 1
    // the motion and IMU terms' cost at the state the call starts from, one running sum in the terms' order (only summed,
    // never used in the solve)
    double motion_cost = 0.0;
    // whether the cost is wanted (lm_step_check): the motion-model edges compute theirs only then; the other terms always add
    // theirs (cheap)
    bool with_cost = false;
    // the scalar residuals the motion and IMU terms and the origin fix added (the NDT points are counted in NdtSums); only
    // counted, never used in the solve (the per-term table's cost per residual)
    int residuals = 0;
};
