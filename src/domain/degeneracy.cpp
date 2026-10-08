#include "degeneracy.h"

#include <cmath>

namespace
{
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    Eigen::Matrix3d skew(const Eigen::Vector3d &a)
    {
        Eigen::Matrix3d m;
        m << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
        return m;
    }

    // one observation's information vector v and the covariance of its noise (paper, Eqs. 13 and 14)
    void observation_terms(const PlaneObservation &o, const Eigen::Vector3d &centre, double point_var, Vector6d &v, Matrix6d &cov)
    {
        const Eigen::Vector3d r = o.point - centre;
        v.head<3>() = o.normal;
        v.tail<3>() = r.cross(o.normal);
        // B maps (point noise e, normal rotation eta) to the change of v
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        const Eigen::Matrix3d N = skew(o.normal);
        B.block<3, 3>(0, 3) = N;
        B.block<3, 3>(3, 0) = -N;
        B.block<3, 3>(3, 3) = skew(r) * N;
        Matrix6d noise = Matrix6d::Zero();
        noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * point_var;
        noise.block<3, 3>(3, 3) = o.normal_cov;
        cov = B * noise * B.transpose();
    }

    // standard normal cumulative distribution
    double phi(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }

    // p for the full-space directions u6 (columns) of one block (paper, Eq. 20)
    Eigen::Vector3d probabilities(const Eigen::Matrix<double, 6, 3> &u6, const Matrix6d &H, const Matrix6d &Sigma,
                                  const std::vector<PlaneObservation> &observations, const Eigen::Vector3d &centre, double point_var,
                                  double snr_factor)
    {
        Eigen::Vector3d variance = Eigen::Vector3d::Zero();
        Vector6d v;
        Matrix6d cov;
        for (const PlaneObservation &o : observations)
        {
            observation_terms(o, centre, point_var, v, cov);
            for (int k = 0; k < 3; k++)
            {
                const double s = u6.col(k).dot(cov * u6.col(k));
                const double d = u6.col(k).dot(v);
                variance(k) += 2.0 * s * s + 4.0 * s * d * d;
            }
        }
        Eigen::Vector3d p;
        for (int k = 0; k < 3; k++)
        {
            const double mu = u6.col(k).dot(Sigma * u6.col(k));
            const double signal = u6.col(k).dot(H * u6.col(k)) - mu; // H_est = H + Sigma on average
            const double sigma = std::sqrt(std::max(variance(k), 0.0));
            double pk = 0.0;
            if (sigma > 0.0)
                pk = phi((signal / snr_factor - mu) / sigma);
            else
                pk = signal > 0.0 ? 1.0 : 0.0;
            if (pk > 1.0 - 1e-6)
                pk = 1.0; // a clearly constrained direction (z above about 4.75) is exactly 1, so nothing is multiplied
            p(k) = pk;
        }
        return p;
    }
}

Eigen::Matrix3d normal_rotation_covariance(const Eigen::Vector3d &spreads, const Eigen::Matrix3d &vectors, double count, double point_sigma_m)
{
    // tilting the normal about e1 moves it towards e2 (spread lambda2), about e2 towards e1 (lambda1); none about the normal
    const double var = point_sigma_m * point_sigma_m / std::max(count, 1.0);
    const Eigen::Vector3d e1 = vectors.col(1), e2 = vectors.col(2);
    return var * (e1 * e1.transpose() / std::max(spreads(2), 1e-12) + e2 * e2.transpose() / std::max(spreads(1), 1e-12));
}

bool is_planar_cell(const Eigen::Vector3d &spreads, double count)
{
    return count >= 3.0 && spreads(0) > 0.0 && spreads(1) >= 3.0 * spreads(0);
}

DegeneracyVerdict detect_degeneracy(const std::vector<PlaneObservation> &observations, const Eigen::Vector3d &centre, double point_sigma_m,
                                    double snr_factor)
{
    DegeneracyVerdict verdict;
    if (observations.size() < 6)
        return verdict;
    const double point_var = point_sigma_m * point_sigma_m;

    Matrix6d H = Matrix6d::Zero(), Sigma = Matrix6d::Zero();
    Vector6d v;
    Matrix6d cov;
    for (const PlaneObservation &o : observations)
    {
        observation_terms(o, centre, point_var, v, cov);
        H += v * v.transpose();
        Sigma += cov;
    }

    const Eigen::Matrix3d Htt = H.block<3, 3>(0, 0), Htr = H.block<3, 3>(0, 3), Hrr = H.block<3, 3>(3, 3);
    const Eigen::Matrix3d Hrt = Htr.transpose();
    // a pseudo-inverse keeps the Schur complements finite when one block is (nearly) singular
    const auto pinv = [](const Eigen::Matrix3d &m)
    {
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(m);
        const double largest = e.eigenvalues().cwiseAbs().maxCoeff();
        Eigen::Vector3d inv = Eigen::Vector3d::Zero();
        for (int k = 0; k < 3; k++)
            if (e.eigenvalues()(k) > 1e-12 * largest && e.eigenvalues()(k) > 0.0)
                inv(k) = 1.0 / e.eigenvalues()(k);
        return Eigen::Matrix3d(e.eigenvectors() * inv.asDiagonal() * e.eigenvectors().transpose());
    };
    const Eigen::Matrix3d Hrr_inv = pinv(Hrr), Htt_inv = pinv(Htt);
    const Eigen::Matrix3d Ht = Htt - Htr * Hrr_inv * Hrt;
    const Eigen::Matrix3d Hr = Hrr - Hrt * Htt_inv * Htr;

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> et(0.5 * (Ht + Ht.transpose()));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> er(0.5 * (Hr + Hr.transpose()));
    verdict.translation_directions = et.eigenvectors();
    verdict.translation_information = et.eigenvalues();
    verdict.rotation_directions = er.eigenvectors();
    verdict.rotation_information = er.eigenvalues();

    // full-space directions: moving along u with the other block re-optimized
    Eigen::Matrix<double, 6, 3> ut, ur;
    ut.topRows<3>() = et.eigenvectors();
    ut.bottomRows<3>() = -Hrr_inv * Hrt * et.eigenvectors();
    ur.topRows<3>() = -Htt_inv * Htr * er.eigenvectors();
    ur.bottomRows<3>() = er.eigenvectors();

    verdict.translation_p = probabilities(ut, H, Sigma, observations, centre, point_var, snr_factor);
    verdict.rotation_p = probabilities(ur, H, Sigma, observations, centre, point_var, snr_factor);
    verdict.valid = true;
    return verdict;
}
