#include <doctest/doctest.h>

#include "window_verdict.h"
#include "truth_statistics.h"

#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

// The window verdict (window_verdict.h, stage 4) on made-up evidence: the flags at their limits, missing readings, and the
// run summary's counts, coverage and triggers.

namespace
{
    WindowEvidence evidence(int window, double bend_deg)
    {
        WindowEvidence e;
        e.window = window;
        e.time_s = 0.5 * window;
        e.heading_bend_deg = bend_deg;
        e.speed_mps = 1.0;
        e.max_bias_deg_s = 0.1;
        return e;
    }
}

TEST_CASE("window verdict: a flag only above its limit, none at it")
{
    CHECK(judge_window(evidence(1, 3.0)).bent == Check::clear);
    CHECK(judge_window(evidence(1, 3.01)).bent == Check::trigger);
    CHECK(judge_window(evidence(1, -3.01)).bent == Check::trigger); // the size counts, not the sign
    CHECK(judge_window(evidence(1, 10.0)).bent_severely == Check::clear);
    CHECK(judge_window(evidence(1, 10.5)).bent_severely == Check::trigger);
    WindowEvidence e = evidence(1, 0.0);
    e.speed_mps = 2.5;
    CHECK(judge_window(e).fast == Check::clear);
    e.speed_mps = 2.51;
    CHECK(judge_window(e).fast == Check::trigger);
    e.max_bias_deg_s = 0.51;
    CHECK(judge_window(e).bias_high == Check::trigger);
    CHECK(judge_window(e).any());
    CHECK_FALSE(judge_window(evidence(1, 1.0)).any());
}

TEST_CASE("window verdict: a missing reading is NOT MEASURED, and the summary counts it as not measured")
{
    WindowEvidence e = evidence(1, 0.5);
    e.speed_mps.reset();
    e.max_bias_deg_s.reset();
    const WindowVerdict v = judge_window(e);
    CHECK(v.fast == Check::not_measured);
    CHECK(v.bias_high == Check::not_measured);
    CHECK(v.bent == Check::clear);

    std::vector<WindowVerdict> windows{v, judge_window(evidence(2, 0.5))};
    const RunVerdict r = summarize_run(windows, true);
    REQUIRE(r.flags.size() == 4);
    CHECK(r.flags[2].measured == 1); // fast: one of two windows has a speed
    CHECK(r.flags[2].run == Check::clear);
    std::vector<WindowVerdict> none{v};
    CHECK(summarize_run(none, true).flags[2].run == Check::not_measured);
}

TEST_CASE("window verdict: the run summary's counts, shares, first and worst window, coverage and triggers")
{
    // 200 windows: two bent (exactly 1 %: clear), then a third (1.5 %: trigger); one bent severely
    std::vector<WindowVerdict> windows;
    for (int k = 1; k <= 200; k++)
        windows.push_back(judge_window(evidence(k, k == 50 ? 4.0 : (k == 120 ? -6.0 : 0.2))));
    RunVerdict r = summarize_run(windows, true);
    CHECK(r.windows == 200);
    CHECK(r.flags[0].flagged == 2);
    CHECK(r.flags[0].run == Check::clear);
    CHECK(r.flags[0].first_window == 50);
    CHECK(r.flags[0].worst_window == 120);
    CHECK(r.flags[0].worst_value == 6.0);
    CHECK(r.flags[1].run == Check::clear);
    windows[150] = judge_window(evidence(151, 12.0));
    r = summarize_run(windows, false);
    CHECK(r.flags[0].flagged == 3);
    CHECK(r.flags[0].run == Check::trigger);
    CHECK(r.flags[1].flagged == 1);
    CHECK(r.flags[1].run == Check::trigger);
    CHECK_FALSE(r.complete);
}

TEST_CASE("window verdict: the velocity chi2 over the weak windows and over all; percentiles as numpy")
{
    CHECK(percentile_linear({1.0, 2.0, 3.0, 4.0}, 50.0) == 2.5);
    CHECK(percentile_linear({1.0, 2.0, 3.0, 4.0, 5.0}, 95.0) == doctest::Approx(4.8));
    std::vector<WindowVerdict> windows;
    for (int k = 1; k <= 10; k++)
    {
        WindowEvidence e = evidence(k, 0.0);
        e.information_ratio = k <= 4 ? 0.2 : 0.8; // windows 1-4 are weak
        e.velocity_chi2 = k <= 4 ? 8.0 : 1.0;
        windows.push_back(judge_window(e));
    }
    const RunVerdict r = summarize_run(windows, true);
    CHECK(r.chi2_weak.measured == 4);
    CHECK(r.chi2_weak.median == 8.0);
    CHECK(r.chi2_run == Check::trigger); // median above 5
    CHECK(r.chi2_all.measured == 10);
    CHECK(r.chi2_all.median == 1.0);
    WindowEvidence without = evidence(11, 0.0);
    std::vector<WindowVerdict> unmeasured{judge_window(without)};
    CHECK(summarize_run(unmeasured, true).chi2_run == Check::not_measured);
}

// The normalised move (design D10 of consistency-harness): q = d^T P^-1 d / (n - tr(P^-1 P_post)), and the reasons a
// reading is NOT MEASURED. K5: on the exact linear-Gaussian control q averages 1; the old form d^T P^-1 d / n averages
// 0.5 when the measurement is as precise as the prior.
namespace
{
    // a prior with very different scales per component and correlations (the whitening must not depend on the units)
    Eigen::MatrixXd scaled_prior(int n)
    {
        truth_stats::Rng rng(11);
        Eigen::MatrixXd a(n, n);
        for (int r = 0; r < n; r++)
            for (int c = 0; c < n; c++)
                a(r, c) = rng.normal();
        const Eigen::MatrixXd correlation = a * a.transpose() + Eigen::MatrixXd::Identity(n, n) * n;
        Eigen::VectorXd sigma(n);
        for (int k = 0; k < n; k++)
            sigma(k) = std::pow(10.0, k % 2 == 0 ? -3.0 + k : 1.0 - k); // 1e-3 .. 1e+1
        return sigma.asDiagonal() * correlation * sigma.asDiagonal();
    }

    struct MoveDraws
    {
        double q_mean = 0.0, old_mean = 0.0;
        int measured = 0;
    };

    // draws: truth from the prior, a measurement z = truth + noise with covariance R, the linear-Gaussian update
    MoveDraws draw_moves(const Eigen::MatrixXd &P, const Eigen::MatrixXd &R, int draws)
    {
        const int n = static_cast<int>(P.rows());
        truth_stats::Rng rng(5);
        const Eigen::VectorXd m = Eigen::VectorXd::LinSpaced(n, 1.0, 2.0);
        const Eigen::MatrixXd K = P * (P + R).inverse();
        const Eigen::MatrixXd P_post = P - K * P;
        MoveDraws out;
        for (int k = 0; k < draws; k++)
        {
            const Eigen::VectorXd truth = m + rng.gaussian(P);
            const Eigen::VectorXd z = truth + rng.gaussian(R);
            const Eigen::VectorXd x = m + K * (z - m);
            const NormalisedMove q = normalised_move(m, P, x, P_post, Eigen::VectorXd::Zero(n));
            if (!q.value)
                continue;
            out.measured++;
            out.q_mean += *q.value;
            out.old_mean += truth_stats::normalised_error(x - m, P) / n;
        }
        out.q_mean /= out.measured;
        out.old_mean /= out.measured;
        return out;
    }
}

TEST_CASE("window verdict, normalised move K5: q averages 1 on the exact control; the old form 0.5")
{
    const int n = 6, draws = 4000;
    const Eigen::MatrixXd P = scaled_prior(n);
    // a measurement as precise as the prior: P_post = P / 2
    const MoveDraws equal = draw_moves(P, P, draws);
    CHECK(equal.measured == draws);
    MESSAGE("equal precision: q mean ", equal.q_mean, ", old form mean ", equal.old_mean);
    CHECK(std::abs(equal.q_mean - 1.0) < 0.05);    // standard error about 0.009
    CHECK(std::abs(equal.old_mean - 0.5) < 0.025); // about 0.0045
    // a measurement of other precision per component: q still averages 1
    Eigen::MatrixXd R = P.diagonal().asDiagonal();
    R(0, 0) *= 0.1;
    R(3, 3) *= 10.0;
    const MoveDraws other = draw_moves(P, R, draws);
    MESSAGE("other precision: q mean ", other.q_mean);
    CHECK(other.measured == draws);
    CHECK(std::abs(other.q_mean - 1.0) < 0.05);
}

TEST_CASE("window verdict, normalised move: the same value in any units")
{
    const Eigen::Matrix3d P = scaled_prior(3);
    const Eigen::Matrix3d P_post = P / 3.0;
    const Eigen::Vector3d m(0.1, 0.2, 0.3);
    const Eigen::Vector3d x = m + Eigen::Vector3d(std::sqrt(P(0, 0)), -std::sqrt(P(1, 1)), 0.5 * std::sqrt(P(2, 2)));
    const NormalisedMove a = normalised_move(m, P, x, P_post, Eigen::Vector3d::Zero());
    REQUIRE(a.value);
    // the same state in other units: component k scaled by s_k
    const Eigen::Vector3d s(1e3, 1e-2, 1e5);
    const NormalisedMove b = normalised_move(s.asDiagonal() * m, s.asDiagonal() * P * s.asDiagonal(), s.asDiagonal() * x,
                                             s.asDiagonal() * P_post * s.asDiagonal(), Eigen::Vector3d::Zero());
    REQUIRE(b.value);
    CHECK(*b.value == doctest::Approx(*a.value).epsilon(1e-9));
    CHECK(*a.reduction == doctest::Approx(2.0).epsilon(1e-9)); // 3 - tr(I / 3)
}

TEST_CASE("window verdict, normalised move: each reason it is NOT MEASURED")
{
    const Eigen::Matrix3d P = Eigen::Vector3d(1e-2, 4e-2, 1e-4).asDiagonal();
    const Eigen::Matrix3d P_post = P / 4.0;
    const Eigen::Vector3d m(1.0, 0.0, 0.0), x(1.05, -0.1, 0.002), zero = Eigen::Vector3d::Zero();
    const NormalisedMove ok = normalised_move(m, P, x, P_post, zero);
    REQUIRE(ok.value);
    CHECK(ok.reason == MoveNotMeasured::measured);
    const auto reason = [](const NormalisedMove &q) { return q.value ? MoveNotMeasured::measured : q.reason; };

    // absent: no state, or sizes that don't match
    CHECK(reason(normalised_move(Eigen::VectorXd(), Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::MatrixXd(), Eigen::VectorXd())) ==
          MoveNotMeasured::absent);
    CHECK(reason(normalised_move(m, P, x, P_post, Eigen::Vector2d::Zero())) == MoveNotMeasured::absent);
    // not finite
    Eigen::Matrix3d nan_post = P_post;
    nan_post(1, 1) = std::numeric_limits<double>::quiet_NaN();
    CHECK(reason(normalised_move(m, P, x, nan_post, zero)) == MoveNotMeasured::not_finite);
    // a failed factorisation: a prior that is not positive definite
    Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
    indefinite(0, 1) = indefinite(1, 0) = 2.0;
    CHECK(reason(normalised_move(m, indefinite, x, P_post, zero)) == MoveNotMeasured::factorisation);
    // ill-conditioned: a zero variance, and two components that are practically one
    Eigen::Matrix3d zero_variance = P;
    zero_variance(2, 2) = 0.0;
    CHECK(reason(normalised_move(m, zero_variance, x, P_post, zero)) == MoveNotMeasured::ill_conditioned);
    Eigen::Matrix3d nearly_singular = Eigen::Matrix3d::Identity();
    nearly_singular(0, 1) = nearly_singular(1, 0) = 1.0 - 1e-14;
    const NormalisedMove singular = normalised_move(m, nearly_singular, x, Eigen::Matrix3d::Identity() * 0.1, zero);
    CHECK((reason(singular) == MoveNotMeasured::ill_conditioned || reason(singular) == MoveNotMeasured::factorisation));
    // a "reduction" that is not physical: a posterior larger than the prior, or a negative one
    CHECK(reason(normalised_move(m, P, x, P * 1.5, zero)) == MoveNotMeasured::non_physical);
    CHECK(reason(normalised_move(m, P, x, -P_post, zero)) == MoveNotMeasured::non_physical);
    // almost no information: n - tr(P^-1 P_post) = 3 * 0.1, below 0.5
    const NormalisedMove small = normalised_move(m, P, x, P * 0.9, zero);
    CHECK(reason(small) == MoveNotMeasured::small_reduction);
    CHECK(*small.reduction == doctest::Approx(0.3));
    // the last step: at 1 % of the prior sigma still measured, above it not (the third's sigma is 0.01)
    CHECK(reason(normalised_move(m, P, x, P_post, Eigen::Vector3d(0.0, 0.0, 0.99e-4))) == MoveNotMeasured::measured);
    CHECK(reason(normalised_move(m, P, x, P_post, Eigen::Vector3d(0.0, 0.0, 1.01e-4))) == MoveNotMeasured::last_step);
    CHECK(reason(normalised_move(m, P, x, P_post, Eigen::Vector3d(-0.0011, 0.0, 0.0))) == MoveNotMeasured::last_step);
}

TEST_CASE("window verdict, honesty readings: which states are read, a clamped bias, no covariance")
{
    HonestyInputs in;
    // no accelerometer term, no bias estimation: every reading absent
    HonestyReadings r = honesty_readings(in);
    CHECK(r.start_velocity.reason == MoveNotMeasured::absent);
    CHECK(r.joint.reason == MoveNotMeasured::absent);
    CHECK(r.gyro_bias.reason == MoveNotMeasured::absent);

    in.covariance_valid = true;
    in.velocity = true;
    in.velocity_prior_mean = Eigen::Vector3d(1.0, 0.0, 0.0);
    in.velocity_prior_cov = Eigen::Matrix3d::Identity() * 1e-2;
    in.start_velocity = Eigen::Vector3d(1.02, -0.01, 0.0);
    in.start_velocity_posterior = Eigen::Matrix3d::Identity() * 2e-3;
    r = honesty_readings(in);
    REQUIRE(r.start_velocity.value);
    // d^T P^-1 d = (0.02^2 + 0.01^2) / 1e-2 = 0.05, reduction 3 - 3 * 0.2 = 2.4
    CHECK(*r.start_velocity.value == doctest::Approx(0.05 / 2.4));
    CHECK(r.joint.reason == MoveNotMeasured::absent); // never measured (honest-inertial-carry layer 2 removed)

    in.bias = true;
    in.bias_prior_cov = Eigen::Matrix3d::Identity() * 1e-4;
    in.bias_mean = Eigen::Vector3d(0.003, 0.0, -0.001);
    in.bias_posterior = Eigen::Matrix3d::Identity() * 1e-6;
    CHECK(honesty_readings(in).gyro_bias.value);
    in.bias_clamped = true;
    CHECK(honesty_readings(in).gyro_bias.reason == MoveNotMeasured::clamped);
    CHECK(honesty_readings(in).start_velocity.value); // the clamp concerns the bias's reading
    // no covariance taken for the present state: nothing measured
    in.covariance_valid = false;
    r = honesty_readings(in);
    CHECK(r.start_velocity.reason == MoveNotMeasured::absent);
    CHECK(r.joint.reason == MoveNotMeasured::absent);
    CHECK(r.gyro_bias.reason == MoveNotMeasured::absent);
}

namespace
{
    struct LinesLogger : Logger
    {
        std::vector<std::string> lines;
        void write(LogLevel, std::string_view message) override { lines.emplace_back(message); }
        std::string find(const std::string &start) const
        {
            for (const std::string &l : lines)
                if (l.rfind(start, 0) == 0)
                    return l;
            return {};
        }
    };
}

TEST_CASE("window verdict, normalised moves in the summary: largest, median, p95, coverage, also when none is measured; on a flagged line")
{
    std::vector<WindowVerdict> windows;
    for (int k = 1; k <= 5; k++)
    {
        WindowEvidence e = evidence(k, k == 5 ? 4.0 : 0.0);
        if (k <= 4)
        {
            e.moves.start_velocity.value = 0.5 * k; // 0.5 .. 2
            e.moves.start_velocity.reason = MoveNotMeasured::measured;
        }
        else
            e.moves.start_velocity.reason = MoveNotMeasured::last_step;
        windows.push_back(judge_window(e));
    }
    const RunVerdict r = summarize_run(windows, true);
    REQUIRE(r.moves.size() == 3);
    CHECK(r.moves[0].reading.measured == 4);
    CHECK(r.moves[0].reading.median == doctest::Approx(1.25));
    CHECK(r.moves[0].reading.p95 == doctest::Approx(1.925));
    CHECK(r.moves[0].reading.largest == 2.0);
    CHECK(r.moves[0].not_measured[size_t(MoveNotMeasured::last_step)] == 1);
    CHECK(r.moves[1].reading.measured == 0);
    CHECK(r.moves[1].not_measured[size_t(MoveNotMeasured::absent)] == 5);
    // the velocity chi2 and the other readings don't see the moves
    CHECK(r.chi2_all.measured == 0);
    CHECK(r.readings.size() == 9);

    LinesLogger log;
    log_run_verdict(r, VerdictLimits{}, log);
    const std::string velocity = log.find("window verdict summary: normalised move, start velocity (");
    const std::string joint = log.find("window verdict summary: normalised move, start velocity and offset");
    const std::string bias = log.find("window verdict summary: normalised move, gyro bias");
    MESSAGE(velocity);
    MESSAGE(joint);
    CHECK(velocity.find("coverage 4/5, median 1.250, p95 1.925, largest 2.000; not measured: last step above 1 % of prior sigma 1") !=
          std::string::npos);
    CHECK(joint.find("coverage 0/5; not measured: absent 5") != std::string::npos); // zero coverage is listed
    CHECK(bias.find("coverage 0/5") != std::string::npos);

    LinesLogger flagged;
    REQUIRE(windows[4].any());
    log_window_verdict(windows[4], flagged);
    REQUIRE(flagged.lines.size() == 1);
    CHECK(flagged.lines[0].find("normalised move start velocity NOT MEASURED (last step above 1 % of prior sigma), start velocity and offset "
                                "NOT MEASURED (absent), gyro bias NOT MEASURED (absent)") != std::string::npos);
    LinesLogger measured;
    log_window_verdict(windows[0], measured);
    REQUIRE(measured.lines.size() == 1);
    CHECK(measured.lines[0].find("normalised move start velocity 0.500,") != std::string::npos);
}

TEST_CASE("window verdict: the limits line comes first and reads back exactly, for the defaults and for other limits")
{
    const RunVerdict r = summarize_run({judge_window(evidence(1, 0.0))}, true);

    LinesLogger defaults;
    log_run_verdict(r, VerdictLimits{}, defaults);
    REQUIRE_FALSE(defaults.lines.empty());
    CHECK(defaults.lines[0] == "window verdict limits: bend 3 deg, severe bend 10 deg, speed 2.5 m/s, gyro bias 0.5 deg/s, bent share 0.01, "
                               "chi2 median 5, chi2 p95 20, weak ratio 0.45");
    CHECK(defaults.lines[1].rfind("window verdict summary: ", 0) == 0);

    VerdictLimits other;
    other.bend_deg = 2.75;
    other.severe_bend_deg = 12.0;
    other.speed_mps = 1.8;
    other.bias_deg_s = 0.125;
    other.bent_share = 0.015;
    other.chi2_median = 4.5;
    other.chi2_p95 = 30.0;
    other.weak_ratio = 0.3;
    LinesLogger changed;
    log_run_verdict(r, other, changed);
    REQUIRE_FALSE(changed.lines.empty());
    CHECK(changed.lines[0] == "window verdict limits: bend 2.75 deg, severe bend 12 deg, speed 1.8 m/s, gyro bias 0.125 deg/s, bent share 0.015, "
                              "chi2 median 4.5, chi2 p95 30, weak ratio 0.3");
}
