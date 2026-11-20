#include <doctest/doctest.h>

#include "terms_at_truth.h"
#include "truth_statistics.h"

#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>

// Task 5.2 of consistency-harness (design D5, F-b): the accelerometer terms' noise. Signed, whitened residuals at the TRUE
// state (true rotations, gravity, offset zero, unit factor 1 g) over model-world recordings: the only random input is the
// accelerometer's white noise, drawn by the generator with density D (ImuNoise::accel_density_mps2_sqrt_hz).
//
// What the model world makes true and what it does not (design D4, row 1). A sample k holds the mean acceleration over the
// interval [t_k, t_k+1) with noise of variance D^2 / dt_k. A term's mean acceleration weighs sample k by its overlap l_k with
// the term's interval [t0, t1], normalised by L = sum l_k, and its residual is  r = -sign * tau * mean (+ the state's part), so
//     var(r) = tau^2 sum_k (l_k / L)^2 D^2 / dt_k              cov(r_a, r_b) = sign_a sign_b tau_a tau_b sum_k (l_a,k / L_a)(l_b,k / L_b) D^2 / dt_k
// The production weight is w = 1 / ((density^2 / tau + floor^2) tau^2). The whitened variance w var(r) is therefore 1 only
// when the interval is whole samples (l_k = dt, L = tau: var(r) = D^2 tau) AND D^2 = density^2 + floor^2 tau, i.e. at one
// tau (tau0), where the floor, which the estimator treats as an error of the mean independent of tau, equals the white
// noise that the model world draws. Terms of another tau (the start term over half an anchor span, the end term over the
// rest of the window) carry the exact analytic ratio, asserted below rather than assumed. Two terms are independent only
// where their sample supports are disjoint; a sample split by a term boundary feeds both neighbours.
//
// (a) the sample-aligned fixture: 500 Hz, span 11.5 ms, so the anchors are 6 nodes (12 ms) apart and every term boundary
//     falls on a sample boundary; D chosen for tau0 = 12 ms. Disjoint supports (asserted), unit variance for the triples,
//     the analytic ratio for the start and end term, no covariance.
// (b) the Livox timing: 500 Hz, the shipped span 10 ms (anchors 5 nodes = 10 ms apart, or 6 where the timestamps' rounding
//     tips the >= test), D 0.01056 as the generator's default (design D4, tau0 = 11 ms): the measured variance and covariance
//     per term kind, reported as the size of the approximation, beside the analytic values for the actual supports.

namespace
{
    using namespace synthetic;
    using namespace terms_truth;

    // walking from the first window on: rest 1 s, speed up, walk (the motion's rotation is what the terms see)
    Motion walking(double seconds)
    {
        Motion m(Eigen::Vector3d(0.0, 0.0, 1.5), 0.0);
        m.rest(1.0).change_speed(1.2, 2.0, 0.5, true).cruise(seconds, true);
        return m;
    }

    struct NoiseWindows
    {
        ImuRecording rec;
        std::vector<std::unique_ptr<TermWindow>> windows;
        double D = 0.0;
        double seconds = 0.0;
    };

    std::unique_ptr<TermWindow> imu_window(const ImuRecording &rec, int g0, int T, const Options &opt)
    {
        auto tw = std::make_unique<TermWindow>();
        tw->options = opt;
        tw->motion = &rec.motion;
        tw->time_offset_s = rec.options.t0_ns * 1e-9;
        tw->dt_s = rec.dt_s();
        for (int i = 0; i < T; i++)
        {
            const MotionState st = rec.motion.state(rec.node_time(g0 + i));
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = st.rotation;
            pose.translation() = st.position;
            tw->window.poses.push_back(pose);
            tw->window.timestamps.push_back({rec.t_s[g0 + i], rec.t_s[g0 + i]});
            const Eigen::Vector3d &a = rec.acc_g[g0 + i];
            tw->window.imu_acc.push_back(Eigen::Vector3d(static_cast<float>(a.x()), static_cast<float>(a.y()), static_cast<float>(a.z()))); // as the source narrows it
        }
        tw->window.motion_model_poses = tw->window.poses;
        tw->window.velocity.start = rec.motion.state(rec.node_time(g0)).velocity;
        tw->window.velocity.end = rec.motion.state(rec.node_time(g0 + T - 1)).velocity;
        tw->build_noise();
        return tw;
    }

    // the model-world recording of `seconds` of walking with accelerometer white noise D and nothing else; windows of 180 nodes
    // from node `first_node` on
    std::unique_ptr<NoiseWindows> make_noise_windows(double D, double span_s, double seconds, std::uint64_t seed, int first_node = 2000)
    {
        const auto t0 = std::chrono::steady_clock::now();
        ImuOptions io;
        io.noise = ImuNoise::none();
        io.noise.accel_density_mps2_sqrt_hz = D;
        io.seed = seed;
        const Motion m = walking(seconds);
        auto r = std::make_unique<NoiseWindows>(NoiseWindows{generate_imu(m, io), {}, D, 0.0});
        const Options opt = make_options(span_s, Eigen::Vector3d(0.028, 0.042, 0.057));
        const int T = 180;
        for (int g0 = first_node; g0 + T <= r->rec.n_nodes(); g0 += T)
            r->windows.push_back(imu_window(r->rec, g0, T, opt));
        r->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return r;
    }

    // the noise-part covariance of the whitened residuals of terms a and b of one window (a == b: the variance)
    double noise_covariance(const TermWindow &tw, size_t a, size_t b, double D)
    {
        const AccelerometerTermSpan &sa = tw.noise.accelerometer_terms[a], &sb = tw.noise.accelerometer_terms[b];
        const Support ua = support_of(tw.window, sa), ub = support_of(tw.window, sb);
        const std::vector<double> la = sample_overlaps(tw.window, ua.t0, ua.t1), lb = sample_overlaps(tw.window, ub.t0, ub.t1);
        double La = 0.0, Lb = 0.0;
        for (double l : la)
            La += l;
        for (double l : lb)
            Lb += l;
        double sum = 0.0;
        for (size_t k = 0; k + 1 < la.size(); k++)
            sum += (la[k] / La) * (lb[k] / Lb) * D * D / (tw.window.timestamps[k + 1].first - tw.window.timestamps[k].first);
        return term_sign(sa) * term_sign(sb) * sa.tau * sb.tau * std::sqrt(sa.weight * sb.weight) * sum;
    }

    // do the sample supports of two terms share a sample (a sample with any overlap)
    bool supports_overlap(const TermWindow &tw, size_t a, size_t b)
    {
        const Support ua = support_of(tw.window, tw.noise.accelerometer_terms[a]), ub = support_of(tw.window, tw.noise.accelerometer_terms[b]);
        const std::vector<double> la = sample_overlaps(tw.window, ua.t0, ua.t1), lb = sample_overlaps(tw.window, ub.t0, ub.t1);
        for (size_t k = 0; k < la.size(); k++)
            if (la[k] > 1e-12 && lb[k] > 1e-12)
                return true;
        return false;
    }

    struct KindStats
    {
        int terms = 0;
        double sum_z2 = 0.0;          // sum of z^2 (raw whitened)
        double sum_z2_over_e = 0.0;   // sum of z^2 / (the analytic variance of that term)
        double sum_z = 0.0;
        double analytic = 0.0;        // the mean analytic whitened variance
    };

    struct PairStats
    {
        int n = 0;
        double sum = 0.0, analytic = 0.0; // sum of z_a z_b / sqrt(e_a e_b); the analytic correlation summed
        double rho() const { return n ? sum / n : 0.0; }
        double rho_analytic() const { return n ? analytic / n : 0.0; }
    };

    struct NoiseResult
    {
        std::map<std::string, KindStats> kinds;
        std::map<std::string, PairStats> pairs;
        int windows = 0;
        std::map<int, int> spacings; // anchor spacing in nodes -> count
    };

    NoiseResult measure(const NoiseWindows &nw)
    {
        NoiseResult res;
        res.windows = static_cast<int>(nw.windows.size());
        for (const auto &twp : nw.windows)
        {
            const TermWindow &tw = *twp;
            const auto &terms = tw.noise.accelerometer_terms;
            const size_t n = terms.size();
            for (size_t j = 1; j < tw.noise.unknowns.anchors.size(); j++)
                res.spacings[tw.noise.unknowns.anchors[j] - tw.noise.unknowns.anchors[j - 1]]++;
            std::vector<Eigen::Vector3d> z(n);
            std::vector<double> e(n);
            for (size_t j = 0; j < n; j++)
            {
                z[j] = std::sqrt(terms[j].weight) * observe_span(tw, j); // signed, whitened
                e[j] = noise_covariance(tw, j, j, nw.D);
                KindStats &k = res.kinds[terms[j].kind == AccelerometerTermKind::anchor_triple ? "triple" : terms[j].kind == AccelerometerTermKind::start_velocity ? "start" : "end"];
                k.terms++;
                k.sum_z2 += z[j].squaredNorm();
                k.sum_z2_over_e += z[j].squaredNorm() / e[j];
                k.sum_z += z[j].sum();
                k.analytic += e[j];
            }
            const auto pair = [&](const char *name, size_t a, size_t b)
            {
                PairStats &p = res.pairs[name];
                p.n += 3;
                p.sum += z[a].dot(z[b]) / std::sqrt(e[a] * e[b]);
                p.analytic += 3.0 * noise_covariance(tw, a, b, nw.D) / std::sqrt(e[a] * e[b]);
            };
            // the production order: the triples, then the start term, then the end term
            size_t triples = 0;
            while (triples < n && terms[triples].kind == AccelerometerTermKind::anchor_triple)
                triples++;
            for (size_t j = 0; j + 1 < triples; j++)
                pair("adjacent triples", j, j + 1);
            for (size_t j = 0; j + 2 < triples; j++)
                pair("triples two apart", j, j + 2);
            if (triples >= 1 && triples < n)
            {
                pair("start and first triple", triples, 0);
                pair("last triple and end", triples - 1, n - 1);
                pair("start and end", triples, n - 1);
            }
        }
        return res;
    }

    // chi-square interval for a statistic with df degrees of freedom, level alpha
    bool inside(double statistic, double df, double alpha)
    {
        const auto [lo, hi] = truth_stats::chi2_interval(df, alpha);
        return statistic >= lo && statistic <= hi;
    }

    std::string report(const char *label, const NoiseWindows &nw, const NoiseResult &r)
    {
        std::ostringstream o;
        o << std::fixed << std::setprecision(4);
        o << "F-b " << label << ": " << r.windows << " windows of 180 nodes (" << nw.rec.n_nodes() << " samples, generated and measured in " << std::setprecision(1) << nw.seconds
          << " s), D " << std::setprecision(5) << nw.D << " m/s2/sqrt(Hz); anchor spacings (nodes:count)";
        for (const auto &[s, c] : r.spacings)
            o << " " << s << ":" << c;
        o << std::setprecision(4);
        for (const auto &[kind, k] : r.kinds)
        {
            const double df = 3.0 * k.terms;
            o << "\n    " << std::setw(7) << kind << "  terms " << std::setw(5) << k.terms << "  measured whitened variance " << k.sum_z2 / df << "  analytic " << 3.0 * k.analytic / df
              << "  measured/analytic " << k.sum_z2_over_e / df << "  mean z " << k.sum_z / df << "  (a chi-square interval at 1%/3: ["
              << truth_stats::chi2_interval(df, 0.01 / 3.0).first / df << ", " << truth_stats::chi2_interval(df, 0.01 / 3.0).second / df << "] per degree of freedom; power to see a claimed variance 1.5 times / 1/1.5 of the true one "
              << std::setprecision(3) << truth_stats::power_of_variance_ratio(df, 1.5, 0.01 / 3.0) << " / " << truth_stats::power_of_variance_ratio(df, 1.0 / 1.5, 0.01 / 3.0) << std::setprecision(4) << ")";
        }
        for (const auto &[name, p] : r.pairs)
            o << "\n    correlation of whitened residuals, " << std::setw(24) << name << "  measured " << std::setw(8) << p.rho() << "  analytic " << std::setw(8) << p.rho_analytic() << "  (n "
              << p.n << ", one standard error " << 1.0 / std::sqrt(static_cast<double>(p.n)) << ")";
        return o.str();
    }
}

TEST_CASE("accelerometer noise: (a) the sample-aligned fixture, disjoint supports, unit variance and no covariance")
{
    constexpr double span = 0.0115;                                   // anchors 6 nodes (12 ms) apart
    const double tau0 = 0.012;
    const double D = std::sqrt(0.0013 * 0.0013 + 0.1 * 0.1 * tau0);   // the white noise whose weight is the shipped one at tau0
    const auto nw = make_noise_windows(D, span, 60.0, 11);
    REQUIRE(nw->windows.size() > 100);

    // validity: the supports are whole samples and disjoint, the anchors are 6 nodes apart
    {
        const TermWindow &tw = *nw->windows[0];
        const auto &terms = tw.noise.accelerometer_terms;
        const size_t n = terms.size();
        for (size_t j = 1; j < tw.noise.unknowns.anchors.size(); j++)
            REQUIRE(tw.noise.unknowns.anchors[j] - tw.noise.unknowns.anchors[j - 1] == 6);
        int overlaps = 0, fractional = 0;
        for (size_t a = 0; a < n; a++)
        {
            const Support u = support_of(tw.window, terms[a]);
            for (const double l : sample_overlaps(tw.window, u.t0, u.t1))
                fractional += (l > 1e-12 && std::abs(l - tw.dt_s) > 1e-9);
            for (size_t b = a + 1; b < n; b++)
                overlaps += supports_overlap(tw, a, b);
        }
        MESSAGE("aligned fixture: " << n << " terms in a window, " << overlaps << " pairs of terms share a sample, " << fractional << " samples are split by a boundary");
        CHECK(overlaps == 0);
        CHECK(fractional == 0);
        // the weight is the white noise's variance at tau0: unit analytic variance for a triple (tau = 12 ms)
        CHECK(noise_covariance(tw, 0, 0, D) == doctest::Approx(1.0).epsilon(1e-9));
    }

    const NoiseResult r = measure(*nw);
    MESSAGE(report("(a) aligned", *nw, r));
    const double alpha = 0.01 / 3.0;
    for (const auto &[kind, k] : r.kinds)
    {
        CAPTURE(kind);
        const double df = 3.0 * k.terms;
        CHECK(inside(k.sum_z2_over_e, df, alpha)); // measured == analytic for the actual support and weight
        CHECK(std::abs(k.sum_z / df) <= 3.3 * std::sqrt(k.analytic / k.terms / 3.0) / std::sqrt(df)); // the mean is zero
    }
    // the triples: analytic variance 1 (asserted above), measured within its interval of 1
    const KindStats &triple = r.kinds.at("triple");
    CHECK(inside(triple.sum_z2, 3.0 * triple.terms, alpha));
    // no covariance between disjoint supports
    for (const auto &[name, p] : r.pairs)
    {
        CAPTURE(name);
        CHECK(std::abs(p.rho_analytic()) < 1e-12);
        CHECK(std::abs(p.rho()) <= 3.3 / std::sqrt(static_cast<double>(p.n)));
    }
}

TEST_CASE("accelerometer noise: (b) the Livox timing, the size of the approximation (measurement)")
{
    const auto nw = make_noise_windows(0.01056, 0.01, 60.0, 12);
    REQUIRE(nw->windows.size() > 100);
    const NoiseResult r = measure(*nw);
    MESSAGE(report("(b) Livox timing, span 10 ms", *nw, r));
    // asserted: the measured variance and covariance are what the supports predict (the generator draws what it says)
    const double alpha = 0.01 / 3.0;
    for (const auto &[kind, k] : r.kinds)
    {
        CAPTURE(kind);
        CHECK(inside(k.sum_z2_over_e, 3.0 * k.terms, alpha));
    }
    for (const auto &[name, p] : r.pairs)
    {
        CAPTURE(name);
        CHECK(std::abs(p.rho() - p.rho_analytic()) <= 3.3 / std::sqrt(static_cast<double>(p.n)));
    }
}
