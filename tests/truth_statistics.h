#pragma once

// Statistics for tests that ask "is the claimed covariance as good as it says?" (openspec change consistency-harness,
// design D2 and D6). Test helpers only, no production code. They are themselves tested in truth_statistics_test.cpp on
// a linear-Gaussian problem that is honest by construction, before any fixture uses them on the estimator.
//
// Direction (D6): a claimed covariance that is too SMALL makes the normalised error too LARGE; one that is too large
// makes it too small.

#include <Eigen/Dense>

#include <cstdint>
#include <utility>

namespace truth_stats
{
    // e^T P^-1 e through a Cholesky solve (no explicit inverse). P must be symmetric positive definite; a failed
    // factorisation throws std::runtime_error.
    double normalised_error(const Eigen::VectorXd& e, const Eigen::MatrixXd& P);

    // Regularised lower incomplete gamma P(a, x), a > 0, x >= 0 (series for x < a + 1, continued fraction above).
    double regularised_gamma_p(double a, double x);

    // Chi-square distribution function with k degrees of freedom (k > 0, not necessarily an integer).
    double chi2_cdf(double k, double x);

    // Chi-square quantile: the x with chi2_cdf(k, x) = p, 0 < p < 1.
    double chi2_quantile(double k, double p);

    // Two-sided interval [lo, hi] that holds a chi-square variable with k degrees of freedom with probability
    // 1 - alpha (alpha / 2 in each tail).
    std::pair<double, double> chi2_interval(double k, double alpha);

    // Power of the two-sided test (interval of chi2_interval(k, alpha)) on one side, when the claimed covariance is
    // `ratio` times the true one (ratio = claimed / true). The statistic is then chi-square(k) / ratio.
    //   ratio > 1: claimed too large, the statistic falls; power = P(statistic below the interval's lower end).
    //   ratio < 1: claimed too small, the statistic rises; power = P(statistic above the interval's upper end).
    // ratio == 1 gives the probability of the intended side's tail, alpha / 2.
    double power_of_variance_ratio(double k, double ratio, double alpha);

    struct PowerPlan
    {
        int runs = 0;                // smallest M that meets the target in both directions (0: none up to max_runs)
        double degrees_of_freedom = 0.0;  // M * dof_per_run
        double alpha_per_comparison = 0.0;
        double power_low_side = 0.0;   // claimed covariance x multiplier  (statistic falls)
        double power_high_side = 0.0;  // claimed covariance / multiplier  (statistic rises)
    };

    // The smallest number of runs M for which a two-sided test of the summed normalised error (M * dof_per_run degrees
    // of freedom) detects BOTH a claimed covariance multiplied by `multiplier` (> 1) and divided by it, each with
    // probability >= power. The level is `alpha` divided by `primary_comparisons` (Bonferroni, the simplest correction
    // that needs no assumption about how the comparisons depend on each other), then split into two tails.
    PowerPlan runs_for_power(int dof_per_run, double multiplier = 1.5, double alpha = 0.01,
                             int primary_comparisons = 1, double power = 0.9, int max_runs = 100000);

    // Deterministic random numbers that do not depend on the standard library's distribution classes (the same
    // sequence on every platform): splitmix64 bits, Box-Muller normals.
    class Rng
    {
    public:
        explicit Rng(std::uint64_t seed) : state_(seed) {}
        double uniform();   // in (0, 1)
        double normal();    // standard normal
        Eigen::VectorXd normal_vector(int n);
        // a draw from N(0, P) through the Cholesky factor of P
        Eigen::VectorXd gaussian(const Eigen::MatrixXd& P);

    private:
        std::uint64_t next_bits();
        std::uint64_t state_;
        bool has_spare_ = false;
        double spare_ = 0.0;
    };
}  // namespace truth_stats
