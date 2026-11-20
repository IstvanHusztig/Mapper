#include "window_verdict.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>

const char *check_name(Check c)
{
    switch (c)
    {
    case Check::clear:
        return "CLEAR";
    case Check::trigger:
        return "TRIGGER";
    case Check::not_measured:
        return "NOT MEASURED";
    }
    return "?";
}

namespace
{
    Check above(const std::optional<double> &value, double limit)
    {
        if (!value)
            return Check::not_measured;
        return std::abs(*value) > limit ? Check::trigger : Check::clear;
    }
}

const char *move_not_measured_name(MoveNotMeasured reason)
{
    switch (reason)
    {
    case MoveNotMeasured::measured:
        return "measured";
    case MoveNotMeasured::absent:
        return "absent";
    case MoveNotMeasured::not_finite:
        return "not finite";
    case MoveNotMeasured::factorisation:
        return "failed factorisation";
    case MoveNotMeasured::ill_conditioned:
        return "ill-conditioned";
    case MoveNotMeasured::non_physical:
        return "non-physical reduction";
    case MoveNotMeasured::small_reduction:
        return "reduction below 0.5";
    case MoveNotMeasured::last_step:
        return "last step above 1 % of prior sigma";
    case MoveNotMeasured::clamped:
        return "clamped bias";
    }
    return "?";
}

NormalisedMove normalised_move(const Eigen::VectorXd &prior_mean, const Eigen::MatrixXd &prior_cov, const Eigen::VectorXd &result,
                               const Eigen::MatrixXd &posterior_cov, const Eigen::VectorXd &last_step, const NormalisedMoveLimits &limits)
{
    NormalisedMove r;
    const Eigen::Index n = prior_mean.size();
    if (n == 0 || result.size() != n || last_step.size() != n || prior_cov.rows() != n || prior_cov.cols() != n || posterior_cov.rows() != n ||
        posterior_cov.cols() != n)
        return r; // absent
    if (!prior_mean.allFinite() || !prior_cov.allFinite() || !result.allFinite() || !posterior_cov.allFinite() || !last_step.allFinite())
    {
        r.reason = MoveNotMeasured::not_finite;
        return r;
    }
    const Eigen::VectorXd variances = prior_cov.diagonal();
    if ((variances.array() <= 0.0).any())
    {
        r.reason = MoveNotMeasured::ill_conditioned;
        return r;
    }
    // the prior in correlation form, S^-1 P S^-1 = L L^T; then P^-1 = W^T W with W = L^-1 S^-1
    const Eigen::VectorXd inv_sigma = variances.cwiseSqrt().cwiseInverse();
    const Eigen::MatrixXd correlation = inv_sigma.asDiagonal() * prior_cov * inv_sigma.asDiagonal();
    const Eigen::LLT<Eigen::MatrixXd> llt(correlation);
    if (llt.info() != Eigen::Success)
    {
        r.reason = MoveNotMeasured::factorisation;
        return r;
    }
    if (!(llt.rcond() >= limits.min_rcond))
    {
        r.reason = MoveNotMeasured::ill_conditioned;
        return r;
    }
    const Eigen::VectorXd whitened_move = llt.matrixL().solve(inv_sigma.asDiagonal() * (result - prior_mean));
    const Eigen::MatrixXd scaled_posterior = inv_sigma.asDiagonal() * (0.5 * (posterior_cov + posterior_cov.transpose())) * inv_sigma.asDiagonal();
    const Eigen::MatrixXd half = llt.matrixL().solve(scaled_posterior);            // L^-1 M
    const Eigen::MatrixXd whitened = llt.matrixL().solve(half.transpose().eval()); // L^-1 M L^-T (M symmetric)
    const Eigen::MatrixXd whitened_sym = 0.5 * (whitened + whitened.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(whitened_sym, Eigen::EigenvaluesOnly);
    if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -limits.eigen_tolerance ||
        eig.eigenvalues().maxCoeff() > 1.0 + limits.eigen_tolerance)
    {
        r.reason = MoveNotMeasured::non_physical;
        return r;
    }
    const double reduction = static_cast<double>(n) - whitened_sym.trace();
    r.reduction = reduction;
    if (reduction < limits.min_reduction)
    {
        r.reason = MoveNotMeasured::small_reduction;
        return r;
    }
    if ((last_step.cwiseAbs().array() > limits.step_share * variances.cwiseSqrt().array()).any())
    {
        r.reason = MoveNotMeasured::last_step;
        return r;
    }
    r.value = whitened_move.squaredNorm() / reduction;
    r.reason = MoveNotMeasured::measured;
    return r;
}

namespace
{
    NormalisedMove not_measured(MoveNotMeasured reason)
    {
        NormalisedMove r;
        r.reason = reason;
        return r;
    }
}

HonestyReadings honesty_readings(const HonestyInputs &in, const NormalisedMoveLimits &limits)
{
    HonestyReadings r;
    if (in.velocity)
    {
        if (!in.covariance_valid)
            r.start_velocity = not_measured(MoveNotMeasured::absent);
        else
            r.start_velocity = normalised_move(in.velocity_prior_mean, in.velocity_prior_cov, in.start_velocity,
                                               in.start_posterior.topLeftCorner<3, 3>(), in.start_velocity_step, limits);
        if (in.joint_prior && in.covariance_valid)
        {
            Eigen::Matrix<double, 6, 1> mean, result, step;
            mean << in.velocity_prior_mean, in.offset_prior_mean;
            result << in.start_velocity, in.acc_offset;
            step << in.start_velocity_step, in.acc_offset_step;
            r.joint = normalised_move(mean, in.joint_prior_cov, result, in.start_posterior, step, limits);
        }
    }
    if (in.bias)
    {
        if (!in.covariance_valid)
            r.gyro_bias = not_measured(MoveNotMeasured::absent);
        else if (in.bias_clamped)
            r.gyro_bias = not_measured(MoveNotMeasured::clamped);
        else
            r.gyro_bias = normalised_move(in.bias_prior_mean, in.bias_prior_cov, in.bias_mean, in.bias_posterior, in.bias_step, limits);
    }
    return r;
}

WindowVerdict judge_window(const WindowEvidence &e, const VerdictLimits &limits)
{
    WindowVerdict v;
    v.evidence = e;
    v.bent = above(e.heading_bend_deg, limits.bend_deg);
    v.bent_severely = above(e.heading_bend_deg, limits.severe_bend_deg);
    v.fast = above(e.speed_mps, limits.speed_mps);
    v.bias_high = above(e.max_bias_deg_s, limits.bias_deg_s);
    return v;
}

double percentile_linear(std::vector<double> values, double p)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const double pos = p / 100.0 * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(lo + 1, values.size() - 1);
    return values[lo] + (pos - static_cast<double>(lo)) * (values[hi] - values[lo]);
}

namespace
{
    ReadingSummary reading(const char *name, const std::vector<double> &values)
    {
        ReadingSummary r;
        r.name = name;
        r.measured = static_cast<int>(values.size());
        if (!values.empty())
        {
            r.median = percentile_linear(values, 50.0);
            r.p95 = percentile_linear(values, 95.0);
            r.largest = *std::max_element(values.begin(), values.end());
        }
        return r;
    }

    FlagSummary flag(const char *name, const std::vector<WindowVerdict> &windows, Check WindowVerdict::*check,
                     const std::function<std::optional<double>(const WindowEvidence &)> &value)
    {
        FlagSummary f;
        f.name = name;
        for (const WindowVerdict &w : windows)
        {
            if (w.*check == Check::not_measured)
                continue;
            f.measured++;
            if (w.*check != Check::trigger)
                continue;
            f.flagged++;
            if (!f.first_window)
            {
                f.first_window = w.evidence.window;
                f.first_time_s = w.evidence.time_s;
            }
            const double v = std::abs(*value(w.evidence));
            if (!f.worst_window || v > f.worst_value)
            {
                f.worst_window = w.evidence.window;
                f.worst_time_s = w.evidence.time_s;
                f.worst_value = v;
            }
        }
        return f;
    }

    std::vector<double> values_of(const std::vector<WindowVerdict> &windows, const std::function<std::optional<double>(const WindowEvidence &)> &value,
                                  const std::function<bool(const WindowEvidence &)> &keep = {})
    {
        std::vector<double> v;
        for (const WindowVerdict &w : windows)
            if (const std::optional<double> x = value(w.evidence); x && (!keep || keep(w.evidence)))
                v.push_back(*x);
        return v;
    }
}

RunVerdict summarize_run(const std::vector<WindowVerdict> &windows, bool complete, const VerdictLimits &limits)
{
    RunVerdict r;
    r.windows = static_cast<int>(windows.size());
    r.complete = complete;
    const auto bend = [](const WindowEvidence &e) { return e.heading_bend_deg; };
    r.flags.push_back(flag("bent", windows, &WindowVerdict::bent, bend));
    r.flags.push_back(flag("bent severely", windows, &WindowVerdict::bent_severely, bend));
    r.flags.push_back(flag("fast", windows, &WindowVerdict::fast, [](const WindowEvidence &e) { return e.speed_mps; }));
    r.flags.push_back(flag("bias high", windows, &WindowVerdict::bias_high, [](const WindowEvidence &e) { return e.max_bias_deg_s; }));
    // the run's triggers: more than bent_share of the measured windows bent; any window with one of the other flags
    for (size_t k = 0; k < r.flags.size(); k++)
    {
        FlagSummary &f = r.flags[k];
        if (!f.measured)
            continue;
        const bool trigger = k == 0 ? static_cast<double>(f.flagged) > limits.bent_share * static_cast<double>(f.measured) : f.flagged > 0;
        f.run = trigger ? Check::trigger : Check::clear;
    }

    const auto chi2 = [](const WindowEvidence &e) { return e.velocity_chi2; };
    r.chi2_weak = reading("velocity chi2, weak windows", values_of(windows, chi2, [&](const WindowEvidence &e)
                                                                   { return e.information_ratio && *e.information_ratio < limits.weak_ratio; }));
    r.chi2_all = reading("velocity chi2, all", values_of(windows, chi2));
    if (r.chi2_weak.measured)
        r.chi2_run = r.chi2_weak.median > limits.chi2_median || r.chi2_weak.p95 > limits.chi2_p95 ? Check::trigger : Check::clear;

    const auto abs_of = [](std::optional<double> x) { return x ? std::optional<double>(std::abs(*x)) : std::nullopt; };
    r.readings.push_back(reading("heading bend [deg]", values_of(windows, [&](const WindowEvidence &e) { return abs_of(e.heading_bend_deg); })));
    r.readings.push_back(reading("roll bend [deg]", values_of(windows, [&](const WindowEvidence &e) { return abs_of(e.roll_bend_deg); })));
    r.readings.push_back(reading("pitch bend [deg]", values_of(windows, [&](const WindowEvidence &e) { return abs_of(e.pitch_bend_deg); })));
    r.readings.push_back(reading("speed [m/s]", values_of(windows, [](const WindowEvidence &e) { return e.speed_mps; })));
    r.readings.push_back(reading("gyro bias [deg/s]", values_of(windows, [](const WindowEvidence &e) { return e.max_bias_deg_s; })));
    r.readings.push_back(reading("information ratio", values_of(windows, [](const WindowEvidence &e) { return e.information_ratio; })));
    r.readings.push_back(reading("iterations", values_of(windows, [](const WindowEvidence &e) { return std::optional<double>(e.iterations); })));
    r.readings.push_back(reading("remaining step [m]", values_of(windows, [](const WindowEvidence &e) { return e.remaining_step_m; })));
    r.readings.push_back(reading("LiDAR weight share along travel", values_of(windows, [](const WindowEvidence &e) { return e.lidar_share_along; })));
    const auto move = [&](const char *name, NormalisedMove HonestyReadings::*m)
    {
        MoveSummary s;
        s.reading = reading(name, values_of(windows, [&](const WindowEvidence &e) { return (e.moves.*m).value; }));
        for (const WindowVerdict &w : windows)
            if (!(w.evidence.moves.*m).value)
                s.not_measured[static_cast<int>((w.evidence.moves.*m).reason)]++;
        return s;
    };
    r.moves.push_back(move("start velocity", &HonestyReadings::start_velocity));
    r.moves.push_back(move("start velocity and offset", &HonestyReadings::joint));
    r.moves.push_back(move("gyro bias", &HonestyReadings::gyro_bias));
    for (const WindowVerdict &w : windows)
    {
        r.stopped[static_cast<int>(w.evidence.stop)]++;
        r.rejected_steps += w.evidence.rejected_steps;
        r.steps_not_applied += w.evidence.steps_not_applied;
        r.big_jump_nodes += w.evidence.big_jump_nodes;
    }
    return r;
}

void log_run_verdict(const RunVerdict &r, const VerdictLimits &limits, Logger &logger)
{
    log_info(logger) << "window verdict summary: " << r.windows << " windows" << (r.complete ? "" : " (run cancelled: incomplete)")
                     << "; investigation triggers, not a statement about the map's accuracy";
    for (const FlagSummary &f : r.flags)
    {
        auto line = log_info(logger);
        line << "window verdict summary: " << f.name << " " << check_name(f.run) << ": " << f.flagged << " of " << f.measured << " measured";
        if (f.measured)
            line << " (" << std::fixed << std::setprecision(1) << 100.0 * f.flagged / f.measured << " %)";
        line << ", coverage " << f.measured << "/" << r.windows;
        if (f.flagged)
            line << ", first window " << f.first_window << " at " << std::fixed << std::setprecision(1) << f.first_time_s << " s, worst window "
                 << f.worst_window << " at " << f.worst_time_s << " s (" << std::setprecision(3) << f.worst_value << ")";
    }
    log_info(logger) << "window verdict summary: velocity chi2 " << check_name(r.chi2_run) << " (median > " << limits.chi2_median << " or p95 > "
                     << limits.chi2_p95 << " over the windows with an information ratio below " << limits.weak_ratio << "): weak windows "
                     << r.chi2_weak.measured << ", median " << std::fixed << std::setprecision(2) << r.chi2_weak.median << ", p95 " << r.chi2_weak.p95
                     << "; all " << r.chi2_all.measured << ", median " << r.chi2_all.median << ", p95 " << r.chi2_all.p95;
    for (const ReadingSummary &x : r.readings)
        if (x.measured)
            log_info(logger) << "window verdict summary: " << x.name << ": " << x.measured << " windows, median " << std::setprecision(4) << x.median
                             << ", p95 " << x.p95 << ", largest " << x.largest;
    for (const MoveSummary &m : r.moves)
    {
        auto line = log_info(logger);
        line << "window verdict summary: normalised move, " << m.reading.name << " (descriptive, 1 expected under the window's own model): coverage "
             << m.reading.measured << "/" << r.windows;
        if (m.reading.measured)
            line << ", median " << std::fixed << std::setprecision(3) << m.reading.median << ", p95 " << m.reading.p95 << ", largest "
                 << m.reading.largest;
        bool first = true;
        for (int k = 0; k < kMoveNotMeasuredReasons; k++)
            if (m.not_measured[size_t(k)])
            {
                line << (first ? "; not measured: " : ", ") << move_not_measured_name(static_cast<MoveNotMeasured>(k)) << " "
                     << m.not_measured[size_t(k)];
                first = false;
            }
    }
    log_info(logger) << "window verdict summary: iterations stopped: converged " << r.stopped[0] << ", iteration limit " << r.stopped[1]
                     << ", time limit " << r.stopped[2] << ", forced " << r.stopped[3] << "; rejected steps " << r.rejected_steps
                     << ", steps not applied " << r.steps_not_applied << ", big-jump nodes " << r.big_jump_nodes;
}

void log_window_verdict(const WindowVerdict &v, Logger &logger)
{
    const WindowEvidence &e = v.evidence;
    const auto opt = [](const std::optional<double> &x, int precision)
    {
        if (!x)
            return std::string("NOT MEASURED");
        std::ostringstream o;
        o << std::fixed << std::setprecision(precision) << *x;
        return o.str();
    };
    log_info(logger) << "window verdict " << e.window << ": t " << std::fixed << std::setprecision(2) << e.time_s << " s; bent " << check_name(v.bent)
                     << ", bent severely " << check_name(v.bent_severely) << ", fast " << check_name(v.fast) << ", bias high " << check_name(v.bias_high)
                     << "; heading bend " << opt(e.heading_bend_deg, 3) << " deg, roll " << opt(e.roll_bend_deg, 3) << ", pitch "
                     << opt(e.pitch_bend_deg, 3) << " deg, speed " << opt(e.speed_mps, 3) << " m/s, bias " << opt(e.max_bias_deg_s, 3)
                     << " deg/s, chi2 " << opt(e.velocity_chi2, 1) << ", information ratio " << opt(e.information_ratio, 3) << ", stopped "
                     << lm_stop_name(e.stop) << " after " << e.iterations << " iterations, damping " << std::scientific << std::setprecision(1)
                     << e.lm_factor << std::fixed << ", rejected " << e.rejected_steps << ", not applied " << e.steps_not_applied << ", big jump nodes "
                     << e.big_jump_nodes << ", remaining step " << opt(e.remaining_step_m, 4) << " m, LiDAR share " << opt(e.lidar_share_along, 3)
                     << "; normalised move start velocity " << move_text(e.moves.start_velocity) << ", start velocity and offset "
                     << move_text(e.moves.joint) << ", gyro bias " << move_text(e.moves.gyro_bias);
}

std::string move_text(const NormalisedMove &m)
{
    if (!m.value)
        return std::string("NOT MEASURED (") + move_not_measured_name(m.reason) + ")";
    std::ostringstream o;
    o << std::fixed << std::setprecision(3) << *m.value;
    return o.str();
}
