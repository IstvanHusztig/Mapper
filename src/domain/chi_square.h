#pragma once

// The chi-square distribution's quantile (the loop closure's match check, loop_closure_match_check_quantile): the x with
// P(chi^2_dof <= x) = p, from the regularized lower incomplete gamma function P(dof / 2, x / 2).

#include <cmath>
#include <limits>

// P(a, x) = gamma(a, x) / Gamma(a), a > 0, x >= 0: the series below a + 1, the continued fraction above (Lentz)
inline double regularized_lower_gamma(double a, double x)
{
    if (!(a > 0.0) || !(x >= 0.0))
        return std::numeric_limits<double>::quiet_NaN();
    if (x == 0.0)
        return 0.0;
    const double front = std::exp(-x + a * std::log(x) - std::lgamma(a));
    if (x < a + 1.0)
    {
        double ap = a, term = 1.0 / a, sum = term;
        for (int n = 0; n < 10000; n++)
        {
            ap += 1.0;
            term *= x / ap;
            sum += term;
            if (std::abs(term) < std::abs(sum) * 1e-16)
                break;
        }
        return sum * front;
    }
    constexpr double tiny = 1e-300;
    double b = x + 1.0 - a, c = 1.0 / tiny, d = 1.0 / b, h = d;
    for (int i = 1; i < 10000; i++)
    {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b;
        if (std::abs(d) < tiny)
            d = tiny;
        c = b + an / c;
        if (std::abs(c) < tiny)
            c = tiny;
        d = 1.0 / d;
        const double step = d * c;
        h *= step;
        if (std::abs(step - 1.0) < 1e-16)
            break;
    }
    return 1.0 - front * h;
}

// the p quantile of chi-square with dof degrees of freedom (0 < p < 1, dof >= 1; NaN otherwise), by bisection
inline double chi_square_quantile(double p, int dof)
{
    if (!(p > 0.0 && p < 1.0) || dof < 1)
        return std::numeric_limits<double>::quiet_NaN();
    const double a = dof / 2.0;
    double lo = 0.0, hi = 1.0;
    while (regularized_lower_gamma(a, hi / 2.0) < p)
        hi *= 2.0;
    for (int i = 0; i < 200; i++)
    {
        const double mid = 0.5 * (lo + hi);
        if (regularized_lower_gamma(a, mid / 2.0) < p)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}
