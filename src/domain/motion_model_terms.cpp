#include "motion_model_terms.h"

#include <algorithm>

void add_gyro_bias_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelRotationOptions &rotation_options)
{
    if (!unknowns.estimate_bias)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    const int ib = unknowns.ib;
    // bias prior carried over from the previous window
    const Eigen::Matrix3d prior_inf = rotation_options.bias_prior_cov.inverse();
    const Eigen::Vector3d prior_delta = rotation_options.bias_prior_mean - window.gyro_bias.mean;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            AtPAndt.add(ib + r, ib + c, prior_inf(r, c));
    AtPBndt.block<3, 1>(ib, 0) += prior_inf * prior_delta;
    equations.motion_cost += prior_delta.dot(prior_inf * prior_delta);
}

void add_velocity_continuity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                             const MotionModelOptions &motion_model, const VelocityPrior &velocity_prior)
{
    if (!(motion_model.acc_sigma_mps2 > 0.0 && !unknowns.acc_term && window.timestamps.size() == window.poses.size()))
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    double &motion_cost = equations.motion_cost;
    constexpr double min_dt = 1e-4;

    for (size_t i = 1; i + 1 < window.poses.size(); i++)
    {
        const double dt1 = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, min_dt);
        const double dt2 = std::max(window.timestamps[i + 1].first - window.timestamps[i].first, min_dt);
        const double sigma_dv = motion_model.acc_sigma_mps2 * 0.5 * (dt1 + dt2);
        const double w = 1.0 / (sigma_dv * sigma_dv);

        const Eigen::Vector3d r = (window.poses[i + 1].translation() - window.poses[i].translation()) / dt2 -
                                  (window.poses[i].translation() - window.poses[i - 1].translation()) / dt1;

        const int nodes[3] = {int(i) - 1, int(i), int(i) + 1};
        const double c[3] = {1.0 / dt1, -(1.0 / dt1 + 1.0 / dt2), 1.0 / dt2};
        motion_cost += w * r.squaredNorm();

        for (int k = 0; k < 3; k++)
        {
            for (int a = 0; a < 3; a++)
            {
                for (int b = 0; b < 3; b++)
                    AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
            }
        }
    }

    // tie the first step of this window to the final velocity of the previous window
    if (velocity_prior.use && window.poses.size() > 1)
    {
        const double dt = std::max(window.timestamps[1].first - window.timestamps[0].first, min_dt);
        const double sigma_dv = motion_model.acc_sigma_mps2 * dt;
        const double w = 1.0 / (sigma_dv * sigma_dv);

        const Eigen::Vector3d r = (window.poses[1].translation() - window.poses[0].translation()) / dt - velocity_prior.velocity;

        const int nodes[2] = {0, 1};
        const double c[2] = {-1.0 / dt, 1.0 / dt};
        motion_cost += w * r.squaredNorm();

        for (int k = 0; k < 3; k++)
        {
            for (int a = 0; a < 2; a++)
            {
                for (int b = 0; b < 2; b++)
                    AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
            }
        }
    }
}
