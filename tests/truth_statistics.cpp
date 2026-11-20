#include "truth_statistics.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace truth_stats
{
    double normalised_error(const Eigen::VectorXd& e, const Eigen::MatrixXd& P)
    {
        Eigen::LLT<Eigen::MatrixXd> llt(P);
        if (llt.info() != Eigen::Success) throw std::runtime_error("normalised_error: covariance not positive definite");
        // e^T P^-1 e = |L^-1 e|^2
        const Eigen::VectorXd w = llt.matrixL().solve(e);
        return w.squaredNorm();
    }

    double regularised_gamma_p(double a, double x)
    {
        if (!(a > 0.0) || x < 0.0) throw std::invalid_argument("regularised_gamma_p: a > 0 and x >= 0 required");
        if (x == 0.0) return 0.0;
        const double eps = 1e-16;
        const double log_prefix = -x + a * std::log(x) - std::lgamma(a);
        if (x < a + 1.0)
        {
            double term = 1.0 / a;
            double sum = term;
            for (int n = 1; n < 100000; ++n)
            {
                term *= x / (a + n);
                sum += term;
                if (std::abs(term) < std::abs(sum) * eps) break;
            }
            return sum * std::exp(log_prefix);
        }
        // continued fraction for Q(a, x) (modified Lentz)
        const double tiny = 1e-300;
        double b = x + 1.0 - a;
        double c = 1.0 / tiny;
        double d = 1.0 / b;
        double h = d;
        for (int i = 1; i < 100000; ++i)
        {
            const double an = -i * (i - a);
            b += 2.0;
            d = an * d + b;
            if (std::abs(d) < tiny) d = tiny;
            c = b + an / c;
            if (std::abs(c) < tiny) c = tiny;
            d = 1.0 / d;
            const double delta = d * c;
            h *= delta;
            if (std::abs(delta - 1.0) < eps) break;
        }
        return 1.0 - std::exp(log_prefix) * h;
    }

    double chi2_cdf(double k, double x)
    {
        if (!(k > 0.0)) throw std::invalid_argument("chi2_cdf: k > 0 required");
        if (x <= 0.0) return 0.0;
        return regularised_gamma_p(0.5 * k, 0.5 * x);
    }

    double chi2_quantile(double k, double p)
    {
        if (!(p > 0.0 && p < 1.0)) throw std::invalid_argument("chi2_quantile: 0 < p < 1 required");
        double lo = 0.0;
        double hi = k + 10.0 * std::sqrt(2.0 * k) + 10.0;
        while (chi2_cdf(k, hi) < p) hi *= 2.0;
        for (int i = 0; i < 300; ++i)
        {
            const double mid = 0.5 * (lo + hi);
            if (chi2_cdf(k, mid) < p) lo = mid; else hi = mid;
            if (hi - lo <= 1e-14 * hi) break;
        }
        return 0.5 * (lo + hi);
    }

    std::pair<double, double> chi2_interval(double k, double alpha)
    {
        return {chi2_quantile(k, 0.5 * alpha), chi2_quantile(k, 1.0 - 0.5 * alpha)};
    }

    double power_of_variance_ratio(double k, double ratio, double alpha)
    {
        const auto [lo, hi] = chi2_interval(k, alpha);
        // statistic S = X / ratio with X ~ chi2(k)
        if (ratio >= 1.0) return chi2_cdf(k, ratio * lo);           // P(S < lo)
        return 1.0 - chi2_cdf(k, ratio * hi);                       // P(S > hi)
    }

    PowerPlan runs_for_power(int dof_per_run, double multiplier, double alpha, int primary_comparisons, double power,
                             int max_runs)
    {
        if (dof_per_run < 1 || !(multiplier > 1.0) || primary_comparisons < 1)
            throw std::invalid_argument("runs_for_power: bad arguments");
        PowerPlan plan;
        plan.alpha_per_comparison = alpha / primary_comparisons;
        for (int m = 1; m <= max_runs; ++m)
        {
            const double k = static_cast<double>(m) * dof_per_run;
            const double low = power_of_variance_ratio(k, multiplier, plan.alpha_per_comparison);
            const double high = power_of_variance_ratio(k, 1.0 / multiplier, plan.alpha_per_comparison);
            if (low >= power && high >= power)
            {
                plan.runs = m;
                plan.degrees_of_freedom = k;
                plan.power_low_side = low;
                plan.power_high_side = high;
                return plan;
            }
        }
        return plan;
    }

    std::uint64_t Rng::next_bits()
    {
        state_ += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    double Rng::uniform()
    {
        // 53 random bits, shifted off zero
        return (static_cast<double>(next_bits() >> 11) + 0.5) * (1.0 / 9007199254740992.0);
    }

    double Rng::normal()
    {
        if (has_spare_)
        {
            has_spare_ = false;
            return spare_;
        }
        const double u1 = uniform();
        const double u2 = uniform();
        const double r = std::sqrt(-2.0 * std::log(u1));
        const double a = 6.283185307179586476925286766559 * u2;
        spare_ = r * std::sin(a);
        has_spare_ = true;
        return r * std::cos(a);
    }

    Eigen::VectorXd Rng::normal_vector(int n)
    {
        Eigen::VectorXd v(n);
        for (int i = 0; i < n; ++i) v[i] = normal();
        return v;
    }

    Eigen::VectorXd Rng::gaussian(const Eigen::MatrixXd& P)
    {
        Eigen::LLT<Eigen::MatrixXd> llt(P);
        if (llt.info() != Eigen::Success) throw std::runtime_error("Rng::gaussian: covariance not positive definite");
        return llt.matrixL() * normal_vector(static_cast<int>(P.rows()));
    }
}  // namespace truth_stats
