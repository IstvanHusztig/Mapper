#include <doctest/doctest.h>

#include "truth_statistics.h"

#include <cmath>
#include <cstdio>
#include <vector>

// The exact control of openspec change consistency-harness (design D2, D6; spec "The statistics are tested on a problem
// that is honest by construction"): the instruments that later fixtures use on the estimator are tested here, on a
// linear-Gaussian chain written in this file, whose claimed covariance is correct by construction. No production code.
//
// The chain: 6 nodes, each a 2-vector (position, velocity), dt = 0.5 s.
//   prior       x0 ~ N(m0, P0)                          P0 = diag(0.25, 0.04)
//   increments  x_{k+1} = F x_k + w_k, w_k ~ N(0, Q)    F = [1 dt; 0 1], Q = q [dt^3/3 dt^2/2; dt^2/2 dt], q = 0.5
//   absolutes   z_k = x_k(position) + v_k at nodes 2 and 5, v_k ~ N(0, R), R = 0.01
// The tested state is the last node (it carries an absolute measurement and all increments). Every random quantity,
// including the prior's error, is redrawn per run.

using truth_stats::Rng;

namespace
{
    constexpr int kNodes = 6;
    constexpr int kDim = 2;

    struct Chain
    {
        double dt = 0.5;
        Eigen::Matrix2d F;
        Eigen::Matrix2d Q;
        Eigen::Matrix2d P0;
        Eigen::Vector2d m0{1.0, -0.5};
        Eigen::RowVector2d H{1.0, 0.0};
        double R = 0.01;
        std::vector<int> measured{2, 5};

        Chain()
        {
            F << 1.0, dt, 0.0, 1.0;
            const double q = 0.5;
            Q << q * dt * dt * dt / 3.0, q * dt * dt / 2.0, q * dt * dt / 2.0, q * dt;
            P0 = Eigen::Vector2d(0.25, 0.04).asDiagonal();
        }
        bool is_measured(int k) const
        {
            for (int m : measured) if (m == k) return true;
            return false;
        }
    };

    struct Draw
    {
        Eigen::VectorXd truth;     // stacked nodes, kNodes * kDim
        std::vector<double> z;     // one value per measured node, in the order of Chain::measured
    };

    Draw draw_run(const Chain& c, Rng& rng)
    {
        Draw d;
        d.truth.resize(kNodes * kDim);
        Eigen::Vector2d x = c.m0 + rng.gaussian(c.P0);              // the prior's error is redrawn too
        d.truth.segment<2>(0) = x;
        for (int k = 1; k < kNodes; ++k)
        {
            x = c.F * x + rng.gaussian(c.Q);
            d.truth.segment<2>(kDim * k) = x;
        }
        for (int k : c.measured) d.z.push_back(d.truth[kDim * k] + std::sqrt(c.R) * rng.normal());
        return d;
    }

    struct Posterior
    {
        Eigen::VectorXd mean;       // the last node
        Eigen::MatrixXd cov;        // the last node's marginal
    };

    // All factors in one information matrix, solved once.
    Posterior solve_batch(const Chain& c, const std::vector<double>& z)
    {
        const int n = kNodes * kDim;
        Eigen::MatrixXd info = Eigen::MatrixXd::Zero(n, n);
        Eigen::VectorXd eta = Eigen::VectorXd::Zero(n);
        Eigen::LLT<Eigen::Matrix2d> p0(c.P0), q(c.Q);
        // prior factor on node 0: residual x0 - m0, weight P0^-1
        const Eigen::Matrix2d w0 = p0.solve(Eigen::Matrix2d::Identity());
        info.block<2, 2>(0, 0) += w0;
        eta.segment<2>(0) += w0 * c.m0;
        // increment factors: residual x_{k+1} - F x_k, weight Q^-1; Jacobian [-F, I]
        const Eigen::Matrix2d wq = q.solve(Eigen::Matrix2d::Identity());
        for (int k = 0; k + 1 < kNodes; ++k)
        {
            Eigen::Matrix<double, 2, 4> A;
            A << -c.F, Eigen::Matrix2d::Identity();
            info.block<4, 4>(kDim * k, kDim * k) += A.transpose() * wq * A;
        }
        // absolute measurements of the position
        for (std::size_t i = 0; i < c.measured.size(); ++i)
        {
            const int k = c.measured[i];
            info.block<2, 2>(kDim * k, kDim * k) += c.H.transpose() * (1.0 / c.R) * c.H;
            eta.segment<2>(kDim * k) += c.H.transpose() * (z[i] / c.R);
        }
        Eigen::LLT<Eigen::MatrixXd> llt(info);
        REQUIRE(llt.info() == Eigen::Success);
        const Eigen::VectorXd x = llt.solve(eta);
        const Eigen::MatrixXd cov = llt.solve(Eigen::MatrixXd::Identity(n, n));
        Posterior p;
        p.mean = x.tail<kDim>();
        p.cov = cov.bottomRightCorner<kDim, kDim>();
        return p;
    }

    // Exact sequential elimination of the same factors (Kalman filter, Joseph form), ending at the last node.
    Posterior solve_sequential(const Chain& c, const std::vector<double>& z)
    {
        Eigen::Vector2d x = c.m0;
        Eigen::Matrix2d P = c.P0;
        for (int k = 0; k < kNodes; ++k)
        {
            if (k > 0)
            {
                x = c.F * x;
                P = c.F * P * c.F.transpose() + c.Q;
            }
            for (std::size_t i = 0; i < c.measured.size(); ++i)
            {
                if (c.measured[i] != k) continue;
                const double S = (c.H * P * c.H.transpose())(0, 0) + c.R;
                const Eigen::Vector2d K = P * c.H.transpose() / S;
                x += K * (z[i] - (c.H * x)(0, 0));
                const Eigen::Matrix2d IKH = Eigen::Matrix2d::Identity() - K * c.H;
                P = IKH * P * IKH.transpose() + K * c.R * K.transpose();
            }
        }
        Posterior p;
        p.mean = x;
        p.cov = P;
        return p;
    }

    // Sum over `runs` independent runs of e^T (scale P)^-1 e for the last node, the claimed covariance being `scale`
    // times the batch covariance (scale = 1 is the honest claim).
    double summed_normalised_error(const Chain& c, std::uint64_t seed, int runs, double scale)
    {
        Rng rng(seed);
        double sum = 0.0;
        for (int r = 0; r < runs; ++r)
        {
            const Draw d = draw_run(c, rng);
            const Posterior p = solve_batch(c, d.z);
            const Eigen::VectorXd e = p.mean - d.truth.tail<kDim>();
            sum += truth_stats::normalised_error(e, scale * p.cov);
        }
        return sum;
    }

    bool inside(double s, const std::pair<double, double>& interval) { return s >= interval.first && s <= interval.second; }
}  // namespace

TEST_CASE("exact control 1: batch solve and exact sequential elimination of the same factors agree")
{
    // 20 draws of the chain (seed 11); the same factors in both solvers. Rounding bound 1e-10 (absolute and relative to
    // the entry's size): the information matrix has a condition number of about 1e3, so 1e-10 is far above rounding
    // and far below any structural difference (a dropped factor moves the covariance by 1e-3 or more).
    const Chain c;
    Rng rng(11);
    double worst_mean = 0.0, worst_cov = 0.0;
    for (int run = 0; run < 20; ++run)
    {
        const Draw d = draw_run(c, rng);
        const Posterior b = solve_batch(c, d.z);
        const Posterior s = solve_sequential(c, d.z);
        worst_mean = std::max(worst_mean, (b.mean - s.mean).cwiseAbs().maxCoeff() / (1.0 + s.mean.cwiseAbs().maxCoeff()));
        worst_cov = std::max(worst_cov, (b.cov - s.cov).cwiseAbs().maxCoeff() / s.cov.cwiseAbs().maxCoeff());
    }
    MESSAGE("largest relative difference: mean " << worst_mean << ", covariance " << worst_cov);
    CHECK(worst_mean < 1e-10);
    CHECK(worst_cov < 1e-10);

    // sanity: the factors are not trivial (the posterior is tighter than the prior's propagation alone)
    const Draw d = draw_run(c, rng);
    const Posterior b = solve_batch(c, d.z);
    CHECK(b.cov(0, 0) < 0.01);
}

TEST_CASE("exact control 2: the statistic and the chi-square quantiles on inputs with known values")
{
    // e^T P^-1 e by hand: diagonal P: 1/4 + 4/9; correlated P = [2 1; 1 2], e = (1, 1): (1/3)(2 - 1 - 1 + 2) = 2/3.
    Eigen::Vector2d e(1.0, 2.0);
    Eigen::Matrix2d P = Eigen::Vector2d(4.0, 9.0).asDiagonal();
    CHECK(truth_stats::normalised_error(e, P) == doctest::Approx(0.25 + 4.0 / 9.0).epsilon(1e-14));
    Eigen::Matrix2d P2;
    P2 << 2.0, 1.0, 1.0, 2.0;
    CHECK(truth_stats::normalised_error(Eigen::Vector2d(1.0, 1.0), P2) == doctest::Approx(2.0 / 3.0).epsilon(1e-14));
    // a covariance scaled by c scales the statistic by 1/c
    CHECK(truth_stats::normalised_error(e, 4.0 * P) == doctest::Approx(0.25 * (0.25 + 4.0 / 9.0)).epsilon(1e-14));

    // closed forms: k = 2: 1 - exp(-x/2); k = 4: 1 - exp(-x/2)(1 + x/2); k = 1: erf(sqrt(x/2))
    for (double x : {0.1, 1.0, 3.0, 7.5, 20.0})
    {
        CHECK(truth_stats::chi2_cdf(2.0, x) == doctest::Approx(1.0 - std::exp(-x / 2)).epsilon(1e-12));
        CHECK(truth_stats::chi2_cdf(4.0, x) == doctest::Approx(1.0 - std::exp(-x / 2) * (1.0 + x / 2)).epsilon(1e-12));
        CHECK(truth_stats::chi2_cdf(1.0, x) == doctest::Approx(std::erf(std::sqrt(x / 2))).epsilon(1e-12));
    }
    CHECK(truth_stats::chi2_quantile(2.0, 0.5) == doctest::Approx(2.0 * std::log(2.0)).epsilon(1e-12));

    // table values (standard chi-square tables, 6 significant digits; tolerance one unit in the last digit)
    struct Row { double k, p, value, tol; };
    const Row table[] = {
        {1, 0.95, 3.841459, 5e-6},    {2, 0.95, 5.991465, 5e-6},   {5, 0.95, 11.07050, 5e-5},
        {10, 0.95, 18.30704, 5e-5},   {10, 0.05, 3.940299, 5e-6},  {30, 0.95, 43.77298, 5e-5},
        {100, 0.975, 129.5612, 5e-4}, {100, 0.025, 74.22193, 5e-4}, {200, 0.99, 249.4451, 5e-4},
        {200, 0.01, 156.4320, 5e-4},  {1, 0.5, 0.4549364, 5e-7},
    };
    for (const Row& r : table)
    {
        INFO("k = " << r.k << ", p = " << r.p);
        CHECK(std::abs(truth_stats::chi2_quantile(r.k, r.p) - r.value) < r.tol);
        CHECK(truth_stats::chi2_cdf(r.k, truth_stats::chi2_quantile(r.k, r.p)) == doctest::Approx(r.p).epsilon(1e-10));
    }

    // the interval holds 1 - alpha
    const auto iv = truth_stats::chi2_interval(50.0, 0.01);
    CHECK(truth_stats::chi2_cdf(50.0, iv.second) - truth_stats::chi2_cdf(50.0, iv.first) == doctest::Approx(0.99).epsilon(1e-10));

    // the power function against the design's figures: 200 dof, 1 % two-sided, 1.5 and its reciprocal "about 94 % and 92 %"
    const double low = truth_stats::power_of_variance_ratio(200.0, 1.5, 0.01);
    const double high = truth_stats::power_of_variance_ratio(200.0, 1.0 / 1.5, 0.01);
    MESSAGE("power at 200 dof, 1 %, 1.5: claimed x1.5 " << low << ", claimed /1.5 " << high);
    // (the design's 94 % is the true variance 1.5 times the claim, i.e. claimed /1.5 here; its 92 % is the other side)
    CHECK(low == doctest::Approx(0.92).epsilon(0.01));
    CHECK(high == doctest::Approx(0.94).epsilon(0.01));
    // ratio 1 gives the intended tail's probability
    CHECK(truth_stats::power_of_variance_ratio(200.0, 1.0, 0.01) == doctest::Approx(0.005).epsilon(1e-6));

    // the Rng: same seed, same numbers; moments of its normals
    Rng a(5), b(5);
    CHECK(a.normal() == b.normal());
    Rng g(6);
    double s = 0, s2 = 0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) { const double v = g.normal(); s += v; s2 += v * v; }
    CHECK(std::abs(s / n) < 4.0 / std::sqrt(double(n)));
    CHECK(std::abs(s2 / n - 1.0) < 4.0 * std::sqrt(2.0 / n));
}

TEST_CASE("exact control 3: the power plan, and the honest chain lies inside its interval")
{
    // The tested state is the last node (r = 2). Design target: two-sided 1 %, 1.5 and its reciprocal, 90 % power,
    // one primary comparison (Bonferroni: alpha / number of comparisons).
    const auto plan = truth_stats::runs_for_power(2, 1.5, 0.01, 1, 0.9);
    MESSAGE("M = " << plan.runs << " (" << plan.degrees_of_freedom << " dof), power " << plan.power_low_side << " / "
                   << plan.power_high_side);
    REQUIRE(plan.runs > 0);
    CHECK(plan.power_low_side >= 0.9);
    CHECK(plan.power_high_side >= 0.9);
    // the smallest: one run fewer misses the target in at least one direction
    const double k1 = (plan.runs - 1) * 2.0;
    CHECK(std::min(truth_stats::power_of_variance_ratio(k1, 1.5, 0.01), truth_stats::power_of_variance_ratio(k1, 1.0 / 1.5, 0.01)) < 0.9);
    // more primary comparisons need more runs
    CHECK(truth_stats::runs_for_power(2, 1.5, 0.01, 3, 0.9).runs > plan.runs);
    // a smaller multiplier needs more runs
    CHECK(truth_stats::runs_for_power(2, 1.25, 0.01, 1, 0.9).runs > plan.runs);

    const Chain c;
    const auto interval = truth_stats::chi2_interval(plan.degrees_of_freedom, 0.01);
    const double s = summed_normalised_error(c, 1001, plan.runs, 1.0);
    MESSAGE("honest chain, seed 1001: sum " << s << " in [" << interval.first << ", " << interval.second << "], expected "
                                            << plan.degrees_of_freedom);
    CHECK(inside(s, interval));

    // the size: over 200 seeds the honest chain is rejected about 1 % of the time (binomial: 2 of 200 expected;
    // bound 5 % = 10, about 5 standard deviations above)
    int rejected = 0;
    for (std::uint64_t seed = 2000; seed < 2200; ++seed)
        if (!inside(summed_normalised_error(c, seed, plan.runs, 1.0), interval)) ++rejected;
    MESSAGE("honest chain rejected in " << rejected << " of 200 seeds");
    CHECK(rejected <= 10);
}

TEST_CASE("exact control 4: the reported covariance x1.5 and /1.5 are rejected, low and high")
{
    // The multiplier scales the whole reported covariance of the tested state (the last node, 2 dof). That scales the
    // statistic exactly by 1/1.5 (claimed too large: the statistic falls) or by 1.5 (claimed too small: it rises, D6),
    // so the scaled quantity is the whole of the claim and dominates the state. Over M = runs_for_power(2, 1.5, 1 %,
    // 1 comparison, 90 %) runs.
    const auto plan = truth_stats::runs_for_power(2, 1.5, 0.01, 1, 0.9);
    REQUIRE(plan.runs > 0);
    const Chain c;
    const auto interval = truth_stats::chi2_interval(plan.degrees_of_freedom, 0.01);

    const double s_large = summed_normalised_error(c, 3001, plan.runs, 1.5);        // claim x1.5
    const double s_small = summed_normalised_error(c, 3001, plan.runs, 1.0 / 1.5);  // claim /1.5
    MESSAGE("seed 3001, M = " << plan.runs << ": claim x1.5 gives " << s_large << ", claim /1.5 gives " << s_small
                              << ", interval [" << interval.first << ", " << interval.second << "]");
    CHECK(s_large < interval.first);      // rejected on the low side
    CHECK(s_small > interval.second);     // a too-small claimed sigma drives the value UP
    CHECK_FALSE(inside(s_large, interval));
    CHECK_FALSE(inside(s_small, interval));

    // the stated power, measured: over 200 seeds the fraction rejected on the intended side is about 90 % or more
    // (binomial standard deviation 2 %; bound 80 %)
    int low_hits = 0, high_hits = 0;
    for (std::uint64_t seed = 4000; seed < 4200; ++seed)
    {
        if (summed_normalised_error(c, seed, plan.runs, 1.5) < interval.first) ++low_hits;
        if (summed_normalised_error(c, seed, plan.runs, 1.0 / 1.5) > interval.second) ++high_hits;
    }
    MESSAGE("power measured over 200 seeds: low side " << low_hits / 200.0 << ", high side " << high_hits / 200.0
                                                       << " (plan: " << plan.power_low_side << " / " << plan.power_high_side << ")");
    CHECK(low_hits >= 160);
    CHECK(high_hits >= 160);
}

TEST_CASE("exact control 5: the normalised move averages 1, the old form 0.5")
{
    // Scalar: prior N(m, s2), truth drawn from it, one independent measurement of variance s2 (s2 = 0.04, m = 2).
    // Posterior variance s2/2; the move d = x_post - m has variance s2/2, so d^2 / s2 averages 0.5 (the old form), and
    // the trace term n - tr(P^-1 P_post) = 1 - 1/2 = 1/2 makes q = d^2 / s2 / (1/2) average 1.
    // 20000 draws (seed 77); tolerance 4 standard errors of the sample mean.
    {
        const double m = 2.0, s2 = 0.04, s = std::sqrt(s2);
        Rng rng(77);
        const int N = 20000;
        double sum_q = 0, sum_q2 = 0, sum_old = 0, sum_old2 = 0;
        for (int i = 0; i < N; ++i)
        {
            const double x = m + s * rng.normal();
            const double z = x + s * rng.normal();
            const double post = (m + z) / 2.0;
            const double d = post - m;
            const double old_form = d * d / s2;
            const double q = old_form / (1.0 - (s2 / 2.0) / s2);
            sum_q += q; sum_q2 += q * q; sum_old += old_form; sum_old2 += old_form * old_form;
        }
        const double mean_q = sum_q / N, mean_old = sum_old / N;
        const double se_q = std::sqrt((sum_q2 / N - mean_q * mean_q) / N);
        const double se_old = std::sqrt((sum_old2 / N - mean_old * mean_old) / N);
        MESSAGE("scalar: q averages " << mean_q << " (se " << se_q << "), d^T P^-1 d averages " << mean_old << " (se " << se_old << ")");
        CHECK(std::abs(mean_q - 1.0) < 4.0 * se_q);
        CHECK(std::abs(mean_old - 0.5) < 4.0 * se_old);
    }
    // Three unknowns, a full prior covariance and a two-row measurement with its own covariance: the same law, q has
    // expected value 1 whatever P, H, R (the sum form n - tr(P^-1 P_post)). 20000 draws (seed 78).
    {
        Eigen::Matrix3d P;
        P << 0.50, 0.20, -0.10, 0.20, 0.30, 0.05, -0.10, 0.05, 0.40;
        Eigen::Matrix<double, 2, 3> H;
        H << 1.0, 0.5, 0.0, 0.0, 1.0, -1.0;
        Eigen::Matrix2d R;
        R << 0.10, 0.03, 0.03, 0.20;
        const Eigen::Vector3d m(1.0, -1.0, 0.5);
        const Eigen::Matrix2d S = H * P * H.transpose() + R;
        const Eigen::Matrix<double, 3, 2> K = P * H.transpose() * S.inverse();
        const Eigen::Matrix3d Ppost = (Eigen::Matrix3d::Identity() - K * H) * P;
        const double trace_reduction = 3.0 - (P.llt().solve(Ppost)).trace();
        Rng rng(78);
        const int N = 20000;
        double sum = 0, sum2 = 0;
        for (int i = 0; i < N; ++i)
        {
            const Eigen::Vector3d x = m + rng.gaussian(P);
            const Eigen::Vector2d z = H * x + rng.gaussian(R);
            const Eigen::Vector3d d = K * (z - H * m);
            const double q = truth_stats::normalised_error(d, P) / trace_reduction;
            sum += q; sum2 += q * q;
        }
        const double mean = sum / N;
        const double se = std::sqrt((sum2 / N - mean * mean) / N);
        MESSAGE("3 unknowns: trace reduction " << trace_reduction << ", q averages " << mean << " (se " << se << ")");
        CHECK(std::abs(mean - 1.0) < 4.0 * se);
    }
}
