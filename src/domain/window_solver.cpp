#include "window_solver.h"
#include <transformations.h>

#include <algorithm>
#include <cmath>

void fix_origin(WindowEquations &equations, const MotionModelOptions &motion_model)
{
    const double fix_origin_weights[6] = {1.0 / (motion_model.fix_origin_x_sigma_m * motion_model.fix_origin_x_sigma_m),
                                          1.0 / (motion_model.fix_origin_y_sigma_m * motion_model.fix_origin_y_sigma_m),
                                          1.0 / (motion_model.fix_origin_z_sigma_m * motion_model.fix_origin_z_sigma_m),
                                          1.0 / (motion_model.fix_origin_om_sigma_deg * motion_model.fix_origin_om_sigma_deg),
                                          1.0 / (motion_model.fix_origin_fi_sigma_deg * motion_model.fix_origin_fi_sigma_deg),
                                          1.0 / (motion_model.fix_origin_ka_sigma_deg * motion_model.fix_origin_ka_sigma_deg)};
    for (int k = 0; k < 6; k++)
        equations.matrix.add(k, k, fix_origin_weights[k]);
    equations.residuals += 6;
}

ExtraCovariances extra_unknowns_covariance(const Eigen::SparseMatrix<double> &AtPA, const WindowUnknowns &unknowns, PatternCachedCholesky &solver)
{
    ExtraCovariances c;
    const int n_params = unknowns.n_params, n_pose_params = unknowns.n_pose_params;
    const int n_extra = n_params - n_pose_params;
    if (n_extra <= 0)
        return c;
    solver.compute(AtPA);
    const PatternCachedCholesky::Solver &solver_cov = solver.solver();
    if (solver_cov.info() != Eigen::Success)
        return c;
    Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(n_params, n_extra);
    for (int r = 0; r < n_extra; r++)
        unit(n_pose_params + r, r) = 1.0;
    const Eigen::MatrixXd cols = solver_cov.solve(unit);
    if (solver_cov.info() != Eigen::Success)
        return c;
    c.solved = true;
    if (unknowns.estimate_bias)
        c.gyro_bias = cols.block<3, 3>(unknowns.ib, unknowns.ib - n_pose_params);
    if (unknowns.acc_term)
    {
        c.acceleration_offset = cols.block<3, 3>(unknowns.io, unknowns.io - n_pose_params);
        c.end_velocity = cols.block<3, 3>(unknowns.ie, unknowns.ie - n_pose_params);
    }
    return c;
}

WindowStep solve_window_step(Eigen::SparseMatrix<double> &AtPA, const Eigen::SparseMatrix<double> &AtPB, const WindowUnknowns &unknowns,
                             double lm_factor, PatternCachedCholesky &solver)
{
    const int n_params = unknowns.n_params, n_pose_params = unknowns.n_pose_params;
    Eigen::SparseMatrix<double> AtPA_I(n_params, n_params);
    if (n_params > n_pose_params)
    {
        // LM damping on the pose parameters only: lm_factor grows to ~1e10, which is negligible against the pose
        // curvature (~1e8+) but would freeze the bias (curvature ~1e3); the bias is kept well-posed by its prior
        std::vector<Eigen::Triplet<double>> damping;
        damping.reserve(n_pose_params);
        for (int k = 0; k < n_pose_params; k++)
            damping.emplace_back(k, k, 1.0);
        AtPA_I.setFromTriplets(damping.begin(), damping.end());
    }
    else
        AtPA_I.setIdentity();
    AtPA_I *= lm_factor;
    AtPA += (AtPA_I);

    solver.compute(AtPA);
    Eigen::SparseMatrix<double> x = solver.solver().solve(AtPB);
    WindowStep step;
    step.solved = solver.solver().info() == Eigen::Success;
    step.h_x = solution_by_row(x, n_params);
    return step;
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
