#include "window_solver.h"
#include <transformations.h>

#include <algorithm>
#include <cmath>

void fix_origin(WindowEquations &equations, const WindowNoiseModel &noise)
{
    for (int k = 0; k < 6; k++)
        equations.matrix.add(k, k, noise.origin[size_t(k)]);
    equations.residuals += 6;
}

bool apply_window_step(Window &window, const WindowStep &step, const WindowUnknowns &unknowns, const MotionModelRotationOptions &rotation_options,
                       Eigen::Vector3d &acc_offset, double &delta, ScanToMapWorkspace &workspace, Logger &logger)
{
    const std::vector<double> &h_x = step.h_x;
    delta = 1000000.0;
    // the nodes' new poses; a node that would end 1 m or more from its guess ("big jump") rejects the whole step
    SteppedPoses stepped;
    if (step.solved)
    {
        stepped = stepped_poses(window, h_x, 1.0, logger);
        workspace.big_jump_nodes += stepped.big_jump_nodes;
    }
    if (step.solved && stepped.big_jump_nodes == 0)
    {
        if (unknowns.acc_term)
            for (int r = 0; r < 3; r++)
                acc_offset(r) += h_x[unknowns.io + r];
        if (unknowns.acc_term)
            for (int r = 0; r < 3; r++)
            {
                window.velocity.start(r) += h_x[unknowns.iv + r];
                window.velocity.end(r) += h_x[unknowns.ie + r];
            }

        if (unknowns.estimate_bias)
        {
            for (int r = 0; r < 3; r++)
            {
                window.gyro_bias.mean(r) += h_x[unknowns.ib + r];
                if (rotation_options.bias_max_rad_s > 0.0)
                    window.gyro_bias.mean(r) = std::clamp(window.gyro_bias.mean(r), -rotation_options.bias_max_rad_s, rotation_options.bias_max_rad_s);
            }
        }

        for (size_t i = 0; i < window.poses.size(); i++)
            window.poses[i] = affine_matrix_from_pose_tait_bryan(stepped.poses[i]);
        delta = 0.0;
        for (int i = 0; i < h_x.size(); i++)
            delta += sqrt(h_x[i] * h_x[i]);
    }
    else if (!step.solved)
        workspace.steps_skipped++;
    return step.solved && stepped.big_jump_nodes == 0;
}
