#include <doctest/doctest.h>

#include "fusion_orientation_estimator.h"
#include "lidar_odometry.h"
#include "serial_executor.h"
#include "terms_at_truth.h"
#include "truth_statistics.h"

#include <chrono>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>

// Task 5.1 of consistency-harness (design D5, F-a, and the checks behind D4): the production terms whose claim the model
// world can make true, evaluated at the TRUE state on windows cut by the real front end from noise-free recordings. The
// window's poses, start and end velocity and offset are the truth; the measured motion-model poses (the filter's) are kept.
// What is measured is reported with MESSAGE (run `lidar_odometry_tests -tc="terms at the truth*" -s` to read it); what is
// asserted is the validity of the observation and the residuals that lie within their derived bounds.
//
// ---- The derivation of the bounds (every residual below is a velocity [m/s] unless said otherwise) ----
//
// A term's residual at the true positions and velocity is  r = Q + S  with
//   Q  the term's own approximation of the continuous motion: the position/velocity side (a difference of positions, or of a
//      position difference and the velocity) against the exact change of the true velocity over the term's interval. Q is
//      the integral of the true acceleration against (K - U): K the weight with which the position side reads the
//      acceleration (a tent for the triple, a ramp for the start and end term), U the box with which the term averages it.
//      K and U have the same integral (tau), so Q = integral (a(u) - a(c)) (K - U) du and
//          |Q| <= max|jerk| * L1,   L1 = integral |u - c| |K - U| du      (a is Lipschitz with the jerk).
//      L1 is integrated numerically for each term from its actual timestamps (kernel_l1).
//   S  what the samples add: tau * (the true mean acceleration over the interval - the term's mean of (R_k f_k - g) s) + the
//      offset's part. Per sample the estimator uses the rotation of node k, at the START of the sample's interval, with the
//      mean body force f of the interval; the truth is the mean of R(t) f(t). Their difference is the mean of
//      (R(t) - R(t_k)) f(t), at most (dt/2) * rate * |f| (the rotation changes by at most rate * (t - t_k), and the mean of
//      t - t_k is dt/2). Then: the accelerometer narrowed to float (relative 2^-24 of the force); the two samples that a
//      term boundary cuts in the middle (the term uses the whole-sample mean for the part inside; the true acceleration
//      differs by at most jerk * dt/2 over the sample, the part is at most dt long: jerk * dt^2 for the two); and the
//      offset: the generator adds a BODY-fixed offset o to the force, the term subtracts one WORLD vector e from R f, so
//      with e = R(t_0) o the mismatch is at most |o| * (the largest angle between the attitude and the window's first
//      attitude) per unit of tau.
//   The bound of a term is the sum of the two, quadrature and samples (derive_bounds); the table reports the largest
//   |r| / bound per term kind, and Q and S against their own bounds.
//
// Rotation edge (om, fi, ka) at the truth: the measured relative rotation is the filter's, the truth's is the generator's.
// The generator's gyro value is the mean body rate over the interval; the filter integrates it as ONE rotation about the
// first-order quaternion step. Per axis the difference of the two relative rotations is at most
//     |w dt|^3 / 12            the first-order quaternion step (1, w dt/2) normalised turns by 2 atan(|w| dt/2), not |w| dt
//   + dt^3 |w| |w'| / 12       the second Magnus term, the mean rate is not the rate of the rotation when the axis turns
//   + 2^-24 |w| dt * sqrt 3    the gyro value narrowed to float
//   + 4 * 2^-24                the filter's state is float32: two stored attitudes, each within 2 * 2^-24 rad
//   + 1e-12                    double rounding of the two Tait-Bryan conversions
// with |w| and |w'| the largest body rate and its change over the edge (derive in rotation_bound).

namespace
{
    using namespace synthetic;
    using namespace terms_truth;

    const Eigen::Vector3d model_offset(0.05, -0.03, 0.04); // [m/s2], body frame (the generator's)

    Scene room()
    {
        // 16 m x 12 m x 3 m, closed: every window has points (the cut drops a window with 1000 or fewer)
        Scene s;
        const double x0 = -8.0, x1 = 8.0, y0 = -6.0, y1 = 6.0, z1 = 3.0;
        for (const double x : {x0, x1})
            s.faces.push_back({0, x, {x, y0, 0.0}, {x, y1, z1}});
        for (const double y : {y0, y1})
            s.faces.push_back({1, y, {x0, y, 0.0}, {x1, y, z1}});
        for (const double z : {0.0, z1})
            s.faces.push_back({2, z, {x0, y0, z}, {x1, y1, z}});
        s.centre = Eigen::Vector3d(0.0, 0.0, 1.5);
        return shifted(s, grid_offset);
    }

    Route a3_route() // rest, +2 m/s2 for 2 s plateau, -2 m/s2 for 2 s plateau
    {
        Route r{"A3", room(), Motion(Eigen::Vector3d(-7.0, -2.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
        r.motion.rest(4.0).change_speed(4.0, 2.5, 0.5).change_speed(-4.0, 2.5, 0.5).rest(1.5);
        return r;
    }
    Route a4a_route() // turning in place at 0.8 rad/s
    {
        Route r{"A4a", room(), Motion(Eigen::Vector3d(0.0, 0.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
        r.motion.rest(4.0).turn(2.4, 3.5, 0.5).rest(1.5);
        return r;
    }
    Route a4b_route() // 1.5 m/s, a turn at 0.8 rad/s with centripetal acceleration 1.2 m/s2
    {
        Route r{"A4b", room(), Motion(Eigen::Vector3d(-5.0, -3.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
        r.motion.rest(4.0).change_speed(1.5, 2.0, 0.5).turn(1.6, 2.5, 0.5).cruise(1.0);
        return r;
    }
    Route a5_route() // walking
    {
        Route r{"A5", room(), Motion(Eigen::Vector3d(-6.0, -3.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
        r.motion.rest(4.0).change_speed(1.2, 2.0, 0.5, true).cruise(8.0, true).change_speed(-1.2, 2.0, 0.5, true).rest(2.0);
        return r;
    }

    struct PreviewStub : PreviewSink
    {
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &, const std::vector<double> &) override {}
        std::string location() const override { return "none"; }
    };
    struct ProgressStub : ProgressReporter
    {
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    struct Fixture
    {
        InertialRecording rec;
        Cut cut;
        Eigen::Matrix3d align = Eigen::Matrix3d::Identity();
        double filter_error_rad = 0.0; // the largest angle between the filter's attitude and the aligned truth
        double seconds = 0.0;
    };

    // The real stages (load_data, calculate_trajectory, compute_step_1) as cut_into_windows runs them, with the accelerometer's
    // pull on the orientation filter switched off (ahrs_gain 0, design D4: the rotation edges' claim is the gyro's alone).
    std::unique_ptr<Fixture> make_fixture(const Route &route, const ImuNoise &noise, double ahrs_gain)
    {
        const auto t0 = std::chrono::steady_clock::now();
        auto f = std::make_unique<Fixture>(Fixture{generate_recording(route, SensorProfile{}, noise), Cut{}, Eigen::Matrix3d::Identity(), 0.0, 0.0});
        Cut &cut = f->cut;
        cut.config = front_end_config(f->rec.sensor);
        cut.config.ahrs_gain = ahrs_gain;
        SyntheticSource source(f->rec.raw(), 1);
        PreviewStub preview;
        SerialExecutor serial;
        ProgressStub progress;
        FusionOrientationEstimator estimator;
        if (load_data(source, preview, cut.config, cut.points_per_file, cut.imu, false, serial, cut.logger))
        {
            calculate_trajectory(cut.trajectory, cut.imu, cut.config.fusionConventionNwu, cut.config.fusionConventionEnu, cut.config.fusionConventionNed,
                                 cut.config.ahrs_gain, false, cut.config.use_remove_imu_bias_from_first_stationary_scan, cut.config.startup_gyro_bias_nr_samples,
                                 cut.config.startup_gyro_bias_max_variance_deg2_s2, estimator, cut.logger);
            cut.ok = compute_step_1(cut.points_per_file, cut.config, cut.state, cut.trajectory, cut.windows, cut.store, cut.logger, progress, &cut.imu);
        }
        if (cut.ok && !cut.windows.empty())
        {
            const InertialRecording &rec = f->rec;
            f->align = cut.windows[0].poses[0].linear() * rec.route.motion.state(rec.imu.node_time(rec.window_origin_node)).rotation.transpose();
            const int T = rec.sensor.nodes_per_window;
            for (size_t w = 0; w < cut.windows.size(); w++)
                for (int i = 0; i < T; i++)
                {
                    const Eigen::Matrix3d truth = f->align * rec.route.motion.state(rec.imu.node_time(rec.window_origin_node + static_cast<int>(w) * T + i)).rotation;
                    f->filter_error_rad = std::max(f->filter_error_rad, Eigen::AngleAxisd(cut.windows[w].poses[i].linear().transpose() * truth).angle());
                }
        }
        f->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return f;
    }

    ImuNoise noise_free_with_offset()
    {
        ImuNoise n = ImuNoise::none();
        n.accel_offset0_mps2 = model_offset;
        return n;
    }

    const Fixture &fixture_a3() { static const auto f = make_fixture(a3_route(), noise_free_with_offset(), 0.0); return *f; }
    const Fixture &fixture_a4a() { static const auto f = make_fixture(a4a_route(), noise_free_with_offset(), 0.0); return *f; }
    const Fixture &fixture_a4b() { static const auto f = make_fixture(a4b_route(), noise_free_with_offset(), 0.0); return *f; }
    const Fixture &fixture_a5() { static const auto f = make_fixture(a5_route(), noise_free_with_offset(), 0.0); return *f; }

    // the window with its state set to the truth
    std::unique_ptr<TermWindow> truth_window(const Fixture &f, int w, const Options &opt)
    {
        const InertialRecording &rec = f.rec;
        const Motion &motion = rec.route.motion;
        const int T = rec.sensor.nodes_per_window;
        const int g0 = rec.window_origin_node + w * T;
        auto tw = std::make_unique<TermWindow>();
        tw->options = opt;
        tw->motion = &motion;
        tw->align = f.align;
        tw->time_offset_s = rec.imu.options.t0_ns * 1e-9;
        tw->dt_s = rec.imu.dt_s();
        const Window &cut = f.cut.windows[w];
        const Eigen::Vector3d origin = motion.state(rec.imu.node_time(rec.window_origin_node)).position;
        for (int i = 0; i < T; i++)
        {
            const MotionState st = motion.state(rec.imu.node_time(g0 + i));
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = f.align * st.rotation;
            pose.translation() = f.align * (st.position - origin);
            tw->window.poses.push_back(pose);
        }
        tw->window.motion_model_poses = cut.poses; // the measured ones, as they are
        tw->window.timestamps = cut.timestamps;
        tw->window.imu_acc = cut.imu_acc;
        tw->window.velocity.start = f.align * motion.state(rec.imu.node_time(g0)).velocity;
        tw->window.velocity.end = f.align * motion.state(rec.imu.node_time(g0 + T - 1)).velocity;
        tw->window.gyro_bias.mean.setZero();
        tw->offset_body = rec.imu.options.noise.accel_offset0_mps2;
        tw->offset = f.align * motion.state(rec.imu.node_time(g0)).rotation * tw->offset_body;
        // gravity in the window's frame: the filter levels itself on the accelerometer during its 3 s start-up ramp (the gain ramps from 10 to
        // ahrs_gain whatever ahrs_gain is), so with an offset its "up" is tilted by o / g from the truth's; the alignment above carries that
        // rotation, and gravity is the truth's up rotated with it
        tw->options.imu.gravity = f.align * Eigen::Vector3d::UnitZ();
        tw->options.imu.to_mps2 = standard_gravity / tw->options.imu.gravity.norm();
        tw->options.imu.velocity_prior_mean = w > 0 ? Eigen::Vector3d(f.align * motion.state(rec.imu.node_time(g0 - 1)).velocity) : Eigen::Vector3d::Zero();
        tw->options.imu.at_rest = motion.at_rest(rec.imu.node_time(g0), rec.imu.node_time(g0 + T));
        tw->build_noise();
        return tw;
    }

    Options default_options() { return make_options(0.01, Eigen::Vector3d(0.028, 0.042, 0.057)); }

    // a window lies in [from, to] of motion time
    bool window_within(const Fixture &f, int w, double from, double to)
    {
        const int T = f.rec.sensor.nodes_per_window;
        const int g0 = f.rec.window_origin_node + w * T;
        return f.rec.imu.node_time(g0) >= from && f.rec.imu.node_time(g0 + T) <= to;
    }
    bool window_at_rest(const Fixture &f, int w)
    {
        const int T = f.rec.sensor.nodes_per_window;
        const int g0 = f.rec.window_origin_node + w * T;
        return f.rec.route.motion.at_rest(f.rec.imu.node_time(g0), f.rec.imu.node_time(g0 + T));
    }

    // a chi-square statistic within its two-sided 1/3 % interval for df degrees of freedom (three comparisons share the 1 %)
    bool inside_interval(double statistic, double df)
    {
        const auto [lo, hi] = truth_stats::chi2_interval(df, 0.01 / 3.0);
        return statistic >= lo && statistic <= hi;
    }

    struct Aggregate
    {
        int n = 0;
        double max_abs = 0.0;      // largest |r| [m/s]
        double max_ratio = 0.0;    // largest |r| / bound
        double max_q_ratio = 0.0;  // largest |Q| / quadrature bound
        double max_s_ratio = 0.0;  // largest |S| / sample bound
        double worst_bound = 0.0;  // the bound of the term with the largest ratio
        int over = 0;              // terms above their bound
        void add(const Eigen::Vector3d &r, const Eigen::Vector3d &q, const Bounds &b)
        {
            n++;
            const double a = r.norm();
            max_abs = std::max(max_abs, a);
            const double ratio = a / std::max(b.total(), 1e-12);
            if (ratio > max_ratio)
            {
                max_ratio = ratio;
                worst_bound = b.total();
            }
            // the bounds are floored at the double rounding of a position difference over a span (|p| ~ 10 m, 2^-52 / 0.005 s ~ 1e-12 m/s):
            // a motion with no jerk has Q = 0 and a bound of 0, and its residual is rounding
            max_q_ratio = std::max(max_q_ratio, q.norm() / std::max(b.quadrature, 1e-12));
            max_s_ratio = std::max(max_s_ratio, (r - q).norm() / std::max(b.samples, 1e-12));
            over += ratio > 1.0;
        }
    };

    const char *kind_name(AccelerometerTermKind k)
    {
        return k == AccelerometerTermKind::anchor_triple ? "triple" : k == AccelerometerTermKind::start_velocity ? "start" : "end";
    }

    std::string sci(double v)
    {
        char b[32];
        std::snprintf(b, sizeof b, "%.2e", v);
        return b;
    }
    std::string fix(double v)
    {
        char b[32];
        std::snprintf(b, sizeof b, "%.3f", v);
        return b;
    }

    std::string describe(const std::string &label, const std::map<std::string, Aggregate> &by_kind)
    {
        std::ostringstream o;
        o << "F-a " << label << ":";
        for (const auto &[kind, a] : by_kind)
            o << "\n    " << std::setw(7) << kind << "  n " << std::setw(5) << a.n << "  max|r| " << sci(a.max_abs) << " m/s  max|r|/bound " << fix(a.max_ratio) << " (its bound "
              << sci(a.worst_bound) << ")  max|Q|/bound_Q " << fix(a.max_q_ratio) << "  max|S|/bound_S " << fix(a.max_s_ratio) << "  terms above bound " << a.over;
        return o.str();
    }

    // all accelerometer terms of the windows in a class, per kind
    std::map<std::string, Aggregate> evaluate_acceleration_terms(const Fixture &f, const std::function<bool(int)> &in_class, int &windows)
    {
        std::map<std::string, Aggregate> by_kind;
        windows = 0;
        const Options opt = default_options();
        for (int w = 0; w < static_cast<int>(f.cut.windows.size()); w++)
        {
            if (!in_class(w))
                continue;
            windows++;
            const auto tw = truth_window(f, w, opt);
            for (size_t j = 0; j < tw->noise.accelerometer_terms.size(); j++)
            {
                const AccelerometerTermSpan &s = tw->noise.accelerometer_terms[j];
                const Eigen::Vector3d r = observe_span(*tw, j);
                const Eigen::Vector3d q = quadrature_part(*tw, s);
                by_kind[kind_name(s.kind)].add(r, q, derive_bounds(*tw, s));
            }
        }
        return by_kind;
    }

    // the bound of a rotation edge, see the header comment
    double rotation_bound(const Motion &motion, double t0, double dt)
    {
        double w = 0.0, dw = 0.0;
        for (int i = 0; i <= 8; i++)
        {
            const double t = t0 + dt * i / 8.0;
            w = std::max(w, motion.state(t).body_rate.norm());
            constexpr double d = 2e-4;
            dw = std::max(dw, (motion.state(t + d).body_rate - motion.state(t - d).body_rate).norm() / (2.0 * d));
        }
        // plus the filter's state is float32 (Fusion): each stored unit quaternion is off by at most 2^-24 in norm, so a stored attitude is off by at
        // most 2 * 2^-24 rad and the relative rotation of two consecutive ones by 4 * 2^-24. (The rounding inside the filter's update is not derived;
        // an excess over this bound at rest or in motion is reported.)
        return std::pow(w * dt, 3) / 12.0 + dt * dt * dt * w * dw / 12.0 + std::ldexp(1.0, -24) * w * dt * std::sqrt(3.0) + 4.0 * std::ldexp(1.0, -24) + 1e-12;
    }

    struct EdgeStats
    {
        int n = 0, over = 0;
        double max_abs = 0.0, max_ratio = 0.0;
    };

    // the rotation edges of the windows in a class
    EdgeStats evaluate_edges(const Fixture &f, const std::function<bool(int)> &in_class)
    {
        EdgeStats st;
        const Options opt = default_options();
        for (int w = 0; w < static_cast<int>(f.cut.windows.size()); w++)
        {
            if (!in_class(w))
                continue;
            const auto tw = truth_window(f, w, opt);
            for (int k = 0; k + 1 < static_cast<int>(tw->window.poses.size()); k++)
            {
                const Eigen::Vector3d d = observe_edge_rotation(*tw, k);
                // the edge k -> k + 1 holds the gyro sample k + 1 (the interval ending at node k + 1)
                const double bound = rotation_bound(*tw->motion, tw->motion_time(tw->window.timestamps[k].first), tw->dt_s);
                const double a = d.cwiseAbs().maxCoeff();
                st.n++;
                st.max_abs = std::max(st.max_abs, a);
                st.max_ratio = std::max(st.max_ratio, a / bound);
                st.over += a > bound;
            }
        }
        return st;
    }

    struct PriorStats
    {
        double velocity_prior = 0.0, velocity_prior_over_a_dt = 0.0, joint_difference = 0.0, zero_velocity = 0.0, bias = 0.0, offset = 0.0;
        int windows = 0;
    };

    // the priors at the truth: the velocity prior (its mean is the previous window's end velocity), the same in its joint
    // form, the zero velocity, the bias prior, the offset prior (the carried offset folded into gravity: relative truth 0)
    PriorStats evaluate_priors(const Fixture &f, const std::function<bool(int)> &in_class)
    {
        PriorStats st;
        const Options opt = default_options();
        for (int w = 1; w < static_cast<int>(f.cut.windows.size()); w++)
        {
            if (!in_class(w))
                continue;
            const auto tw = truth_window(f, w, opt);
            st.windows++;
            const WindowNoiseModel &nm = tw->noise;
            {
                WindowEquations eq(nm.unknowns);
                add_start_velocity_prior(eq, nm, tw->window);
                const Eigen::Vector3d r = nm.velocity_prior_cov * eq.rhs.block<3, 1>(nm.unknowns.iv, 0); // mean - state
                st.velocity_prior = std::max(st.velocity_prior, r.norm());
                const double a_dt = (tw->window.velocity.start - tw->window.velocity.end).norm() > 0.0
                                        ? (tw->align * tw->motion->state(tw->motion_time(tw->window.timestamps[0].first)).acceleration).norm() * tw->dt_s
                                        : 0.0;
                st.velocity_prior_over_a_dt = std::max(st.velocity_prior_over_a_dt, a_dt > 0.0 ? r.norm() / a_dt : 0.0);
                // the joint form with a zero cross block must give the same residual
                WindowNoiseModel joint = nm;
                joint.joint_prior = true;
                joint.joint_prior_cov = joint_prior_cov(nm.velocity_prior_cov, Eigen::Matrix3d::Zero(), nm.offset_prior_cov);
                joint.joint_prior_information = joint.joint_prior_cov.inverse();
                WindowEquations eqj(nm.unknowns);
                add_joint_velocity_offset_prior(eqj, joint, tw->window, Eigen::Vector3d::Zero());
                const Eigen::Vector3d rj = nm.velocity_prior_cov * eqj.rhs.block<3, 1>(nm.unknowns.iv, 0);
                st.joint_difference = std::max(st.joint_difference, (rj - r).norm());
            }
            {
                WindowEquations eq(nm.unknowns);
                add_zero_velocity(eq, nm, tw->window);
                if (nm.at_rest)
                    st.zero_velocity = std::max(st.zero_velocity, (eq.rhs.block<3, 1>(nm.unknowns.iv, 0) / nm.zero_velocity_weight).norm() +
                                                                       (eq.rhs.block<3, 1>(nm.unknowns.ie, 0) / nm.zero_velocity_weight).norm());
            }
            {
                WindowEquations eq(nm.unknowns);
                add_gyro_bias_prior(eq, nm, tw->window);
                st.bias = std::max(st.bias, (nm.bias_prior_cov * eq.rhs.block<3, 1>(nm.unknowns.ib, 0)).norm());
            }
            {
                WindowEquations eq(nm.unknowns);
                add_acceleration_offset_prior(eq, nm, Eigen::Vector3d::Zero());
                st.offset = std::max(st.offset, (nm.offset_prior_cov * eq.rhs.block<3, 1>(nm.unknowns.io, 0)).norm());
            }
        }
        return st;
    }
}

TEST_CASE("terms at the truth: the observation is valid (alignment, isolation, the edge path)")
{
    const Fixture &f = fixture_a3();
    REQUIRE(f.cut.ok);
    REQUIRE(static_cast<int>(f.cut.windows.size()) == f.rec.n_windows);
    // the filter's attitude is the generator's, up to one fixed alignment (gain 0, noise-free): this is what makes "the truth
    // in the window's frame" a statement about the same quantities
    MESSAGE("A3 recording: " << f.cut.windows.size() << " windows, the filter's attitude differs from the aligned truth by at most " << f.filter_error_rad
                             << " rad, cut in " << f.seconds << " s");
    CHECK(f.filter_error_rad < 2e-6);

    // the isolated spans give what the whole window gives: the sum of the isolated offset right-hand sides equals the full call's
    const Options opt = default_options();
    for (const int w : {1, 6, 9})
    {
        const auto tw = truth_window(f, w, opt);
        WindowEquations full(tw->noise.unknowns);
        add_acceleration_terms(full, tw->noise, tw->window, tw->imu(), tw->offset);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (size_t j = 0; j < tw->noise.accelerometer_terms.size(); j++)
        {
            const AccelerometerTermSpan &s = tw->noise.accelerometer_terms[j];
            sum += -s.weight * term_sign(s) * s.tau * observe_span(*tw, j);
        }
        const Eigen::Vector3d whole = full.rhs.block<3, 1>(tw->noise.unknowns.io, 0);
        CAPTURE(w);
        CHECK((sum - whole).norm() <= 1e-9 * std::max(whole.norm(), 1e-12) + 1e-12);
    }

    // a known rotation error on the measured pose is read back as that error (sign convention found and used below), and a
    // window bias different from the prior's shifts every edge by bias * dt (the coupling)
    {
        const auto tw = truth_window(f, 6, opt);
        const Eigen::Vector3d base = observe_edge_rotation(*tw, 40);
        constexpr double eps = 1e-4;
        TermWindow turned = *tw;
        turned.window.motion_model_poses[41].linear() = turned.window.motion_model_poses[41].linear() * Eigen::AngleAxisd(eps, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Vector3d seen = observe_edge_rotation(turned, 40) - base;
        MESSAGE("edge path: a rotation of 1e-4 rad about x on the measured pose reads as " << seen.transpose() << " rad");
        CHECK(std::abs(std::abs(seen.x()) - eps) <= 1e-7);
        CHECK(std::abs(seen.y()) <= 1e-9);
        CHECK(std::abs(seen.z()) <= 1e-9);
        TermWindow biased = *tw;
        const Eigen::Vector3d bias(0.0, 0.01, 0.0); // rad/s
        biased.window.gyro_bias.mean = bias;
        const Eigen::Vector3d shift = observe_edge_rotation(biased, 40) - base;
        MESSAGE("edge path: a window bias 0.01 rad/s about y shifts the edge by " << shift.transpose() << " rad (bias * dt = " << 0.01 * tw->dt_s << ")");
        CHECK(std::abs(std::abs(shift.y()) - 0.01 * tw->dt_s) <= 1e-9);
        CHECK(std::abs(shift.x()) <= 1e-9);
        CHECK(std::abs(shift.z()) <= 1e-9);
    }
}

TEST_CASE("terms at the truth: A1, rest (active states only)")
{
    Aggregate acc_all;
    std::map<std::string, Aggregate> by_kind;
    int windows = 0;
    PriorStats priors;
    EdgeStats edges;
    for (const Fixture *f : {&fixture_a3(), &fixture_a4a(), &fixture_a4b(), &fixture_a5()})
    {
        int n = 0;
        const auto part = evaluate_acceleration_terms(*f, [&](int w) { return window_at_rest(*f, w); }, n);
        windows += n;
        for (const auto &[kind, a] : part)
        {
            Aggregate &t = by_kind[kind];
            t.n += a.n;
            t.max_abs = std::max(t.max_abs, a.max_abs);
            t.max_ratio = std::max(t.max_ratio, a.max_ratio);
            t.max_q_ratio = std::max(t.max_q_ratio, a.max_q_ratio);
            t.max_s_ratio = std::max(t.max_s_ratio, a.max_s_ratio);
            t.over += a.over;
        }
        const PriorStats p = evaluate_priors(*f, [&](int w) { return window_at_rest(*f, w); });
        priors.windows += p.windows;
        priors.zero_velocity = std::max(priors.zero_velocity, p.zero_velocity);
        priors.velocity_prior = std::max(priors.velocity_prior, p.velocity_prior);
        priors.bias = std::max(priors.bias, p.bias);
        priors.offset = std::max(priors.offset, p.offset);
        priors.joint_difference = std::max(priors.joint_difference, p.joint_difference);
        const EdgeStats e = evaluate_edges(*f, [&](int w) { return window_at_rest(*f, w); });
        edges.n += e.n;
        edges.over += e.over;
        edges.max_abs = std::max(edges.max_abs, e.max_abs);
        edges.max_ratio = std::max(edges.max_ratio, e.max_ratio);
    }
    REQUIRE(windows >= 2);
    MESSAGE(describe("A1 rest, " + std::to_string(windows) + " windows, offset " + "(0.05, -0.03, 0.04) m/s2 body", by_kind));
    MESSAGE("F-a A1 rotation edges: n " << edges.n << "  max|delta| " << edges.max_abs << " rad  max/bound " << edges.max_ratio << "  above bound " << edges.over);
    MESSAGE("F-a A1 priors (" << priors.windows << " windows): zero velocity residual " << priors.zero_velocity << " m/s, velocity prior " << priors.velocity_prior
                              << " m/s, joint form differs by " << priors.joint_difference << ", bias prior " << priors.bias << " rad/s, offset prior " << priors.offset << " m/s2");
    for (const auto &[kind, a] : by_kind)
    {
        CAPTURE(kind);
        CHECK(a.over == 0);
    }
    CHECK(edges.over == 0);
    CHECK(priors.zero_velocity == 0.0); // exact observations: the residuals at true rest have no scatter
    CHECK(priors.velocity_prior == 0.0);
    CHECK(priors.bias == 0.0);
    CHECK(priors.offset == 0.0);
    CHECK(priors.joint_difference <= 1e-15);
}

namespace
{
    // one class of windows: the terms, the edges and the priors
    void report_class(const std::string &label, const Fixture &f, double from, double to, bool check)
    {
        const auto in_class = [&](int w) { return window_within(f, w, from, to); };
        int windows = 0;
        const auto by_kind = evaluate_acceleration_terms(f, in_class, windows);
        REQUIRE(windows >= 2);
        const EdgeStats edges = evaluate_edges(f, in_class);
        const PriorStats priors = evaluate_priors(f, in_class);
        MESSAGE(describe(label + ", " + std::to_string(windows) + " windows", by_kind));
        MESSAGE("F-a " << label << " rotation edges: n " << edges.n << "  max|delta| " << edges.max_abs << " rad  max/bound " << edges.max_ratio << "  above bound " << edges.over);
        MESSAGE("F-a " << label << " priors: velocity prior residual (mean = the previous window's end velocity, the F2a size) " << priors.velocity_prior << " m/s = "
                       << priors.velocity_prior_over_a_dt << " x |a| dt; bias prior " << priors.bias << " rad/s; joint form differs by " << priors.joint_difference);
        if (check)
        {
            for (const auto &[kind, a] : by_kind)
            {
                CAPTURE(kind);
                CHECK(a.over == 0);
            }
            // The derived bound is the physics (the mean-rate and first-order-quaternion terms, below 1e-9) plus the float32 storage of the
            // filter's state (2.4e-7). In motion the measured residual exceeds that sum (reported above, not asserted: the update's own float
            // arithmetic is not derived). What is asserted is the size of the floor against the edge's claim: below a tenth of its sigma.
            CHECK(edges.max_abs < 0.1 * 0.028 * 3.14159265358979323846 / 180.0 * std::sqrt(0.002));
        }
    }
}

TEST_CASE("terms at the truth: A3, constant acceleration")
{
    const Fixture &f = fixture_a3();
    report_class("A3 +2 m/s2 plateau", f, 4.5, 6.0, true);
    report_class("A3 -2 m/s2 plateau", f, 7.0, 8.5, true);
}

TEST_CASE("terms at the truth: A4a, turning in place")
{
    report_class("A4a 0.8 rad/s plateau", fixture_a4a(), 4.5, 7.0, true);
}

TEST_CASE("terms at the truth: A4b, a turn with centripetal acceleration")
{
    report_class("A4b 1.5 m/s, 0.8 rad/s, 1.2 m/s2 centripetal", fixture_a4b(), 6.5, 8.0, true);
}

TEST_CASE("terms at the truth: A5, walking")
{
    // a measurement: the terms' quadrature error (Q), what the samples add (S) and the total are reported, nothing is attributed
    report_class("A5 walking", fixture_a5(), 6.5, 13.5, true);
}

TEST_CASE("terms at the truth: the offset is body-fixed in the model world, world-fixed in the term (measurement)")
{
    // With the same turning window, the term's residual with the world offset e = R(t0) o against e = 0 differs by tau * e: the
    // size of the offset in the term's units. This is the model-world's design (synthetic_inertial.cpp generate_imu adds the
    // offset to the body-frame force) against the term's (accelerometer_term.cpp:106 subtracts one world vector for the whole
    // term; run_odometry.cpp:191 folds the estimate into the world-frame gravity vector). The residual left after the best world
    // vector is |o| * rotation over the term; reported as the offset part of the samples' bound.
    const Fixture &f = fixture_a4a();
    const Options opt = default_options();
    double worst = 0.0, worst_bound = 0.0;
    for (int w = 0; w < static_cast<int>(f.cut.windows.size()); w++)
    {
        if (!window_within(f, w, 4.5, 7.0))
            continue;
        auto tw = truth_window(f, w, opt);
        for (size_t j = 0; j < tw->noise.accelerometer_terms.size(); j++)
        {
            const AccelerometerTermSpan &s = tw->noise.accelerometer_terms[j];
            const Eigen::Vector3d with_offset = observe_span(*tw, j);
            tw->offset.setZero();
            const Eigen::Vector3d without = observe_span(*tw, j);
            tw->offset = f.align * f.rec.route.motion.state(f.rec.imu.node_time(f.rec.window_origin_node + w * f.rec.sensor.nodes_per_window)).rotation * tw->offset_body;
            worst = std::max(worst, (without - with_offset).norm() / s.tau);
            worst_bound = std::max(worst_bound, tw->offset_body.norm() * 1.0);
        }
    }
    MESSAGE("F-a A4a: the offset enters each term as tau * e: largest |e| seen " << worst << " m/s2 (|o| = " << worst_bound << ")");
    CHECK(std::abs(worst - model_offset.norm()) <= 1e-9);
}

// ---- the checks behind D4 ----

TEST_CASE("terms at the truth: D4, gyro white noise at ARW/sqrt(dt) gives the rotation edges' assumed variance")
{
    // The model world's gyro value is the mean body rate plus white noise of sigma ARW / sqrt(dt) (generate_imu), the edge k -> k + 1
    // holds sample k + 1, so the edge's angle noise has sigma ARW sqrt(dt). The estimator's edge sigma is
    // max(k ARW sqrt(dt), floor) with k = 1, floor 0 (time_scaled_rotation_sigma). The test measures the edges' residual
    // against the TRUTH relative rotation, through the production edge block, behind the filter's 3 s ramp and the Tait-Bryan
    // conversion, and compares its variance per axis with 1 / w.
    ImuNoise noise = ImuNoise::none();
    noise.gyro_arw_deg_sqrt_s = Eigen::Vector3d(0.028, 0.042, 0.057);
    static const std::unique_ptr<Fixture> noisy = make_fixture(a5_route(), noise, 0.0);
    const Fixture &f = *noisy;
    REQUIRE(f.cut.ok);
    const Options opt = make_options(0.01, noise.gyro_arw_deg_sqrt_s);
    const double deg = 3.14159265358979323846 / 180.0;

    struct Axis
    {
        double sum_z2 = 0.0, sum_z = 0.0;
        int n = 0;
    };
    Axis all[3], first[3];
    double assumed_check = 0.0;
    for (int w = 0; w < static_cast<int>(f.cut.windows.size()); w++)
    {
        const auto tw = truth_window(f, w, opt);
        for (int k = 0; k + 1 < static_cast<int>(tw->window.poses.size()); k++)
        {
            Eigen::Vector3d weight;
            double dt = 0.0;
            const Eigen::Vector3d d = observe_edge_rotation(*tw, k, &weight, &dt);
            for (int a = 0; a < 3; a++)
            {
                const double z = d(a) * std::sqrt(weight(a));
                Axis &s = w == 0 ? first[a] : all[a];
                s.sum_z2 += z * z;
                s.sum_z += z;
                s.n++;
                // the assumed variance is the angle random walk over dt: (ARW rad)^2 dt
                const double arw = noise.gyro_arw_deg_sqrt_s(a) * deg;
                assumed_check = std::max(assumed_check, std::abs(1.0 / weight(a) / (arw * arw * dt) - 1.0));
            }
        }
    }
    CHECK(assumed_check < 1e-9);
    std::ostringstream o;
    o << std::fixed << std::setprecision(4) << "D4 rotation edges at ARW/sqrt(dt) noise, A5 walking, " << f.cut.windows.size() << " windows; production assumed variance = (ARW)^2 dt to " << std::scientific
      << assumed_check << std::fixed;
    for (int a = 0; a < 3; a++)
    {
        const double df = all[a].n;
        const auto interval = truth_stats::chi2_interval(df, 0.01 / 3.0);
        o << "\n    axis " << "xyz"[a] << ": measured variance / assumed " << all[a].sum_z2 / df << " over " << all[a].n << " edges (1% interval [" << interval.first / df << ", " << interval.second / df
          << "]), mean " << all[a].sum_z / df << " sigma; window 0 alone " << first[a].sum_z2 / first[a].n << " over " << first[a].n << " edges";
        CAPTURE(a);
        CHECK(inside_interval(all[a].sum_z2, df));
    }
    MESSAGE(o.str());
}

TEST_CASE("terms at the truth: D4, the gap covariance's algebra, apart from its mean")
{
    // (1) joint F P F^T: the production function against an independent construction with explicit 6x6 matrices
    truth_stats::Rng rng(5);
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            a(i, j) = 0.02 * rng.normal();
    const JointPriorCov P = a * a.transpose() + 0.01 * JointPriorCov::Identity();
    const double gap = 0.002, density = 0.0013, floor = 0.1, rw = 1e-4, duration = 0.36;
    JointPriorCov F = JointPriorCov::Identity();
    F.topRightCorner<3, 3>() = -gap * Eigen::Matrix3d::Identity();
    JointPriorCov expected = F * P * F.transpose();
    expected.topLeftCorner<3, 3>() += (density * density * gap + floor * floor * gap * gap) * Eigen::Matrix3d::Identity();
    expected.bottomRightCorner<3, 3>() += rw * rw * duration * Eigen::Matrix3d::Identity();
    const JointPriorCov got = joint_prior_cov_over_gap(P, density, floor, gap, rw, duration);
    const double difference = (got - expected).cwiseAbs().maxCoeff() / expected.cwiseAbs().maxCoeff();
    MESSAGE("D4 gap algebra: production joint covariance against F P F^T + Q, largest relative difference " << difference);
    CHECK(difference < 1e-14);
    // the cross block is F P F^T's, not grown by Q (Q has no cross term: the noise of the gap and the offset's walk are independent)
    CHECK((got.topRightCorner<3, 3>() - expected.topRightCorner<3, 3>()).cwiseAbs().maxCoeff() < 1e-15);
    // the separate form
    const Eigen::Matrix3d C = Eigen::Matrix3d::Identity() * 0.3;
    CHECK((velocity_prior_cov_over_gap(C, density, floor, gap) - (C + (density * density * gap + floor * floor * gap * gap) * Eigen::Matrix3d::Identity())).cwiseAbs().maxCoeff() < 1e-18);
    // the sign of F: over the gap the true acceleration is the measured one minus the offset, so v_start = v_end + (a_measured - offset) gap:
    // the velocity depends on the offset with coefficient -gap (the generator adds a constant offset to the force: a_measured = a + offset)
    CHECK(F(0, 3) == -gap);

    // (2) the velocity noise over a gap of one node interval is D^2 gap: the generator's accelerometer noise at rest, summed over
    // the gap, against the production's D^2 gap with floor 0 (the floor is not generated: a claim the model world does not make true)
    constexpr double D = 0.01056;
    Motion rest(Eigen::Vector3d(0, 0, 1.5), 0.0);
    rest.rest(60.0);
    ImuOptions io;
    io.noise = ImuNoise::none();
    io.noise.accel_density_mps2_sqrt_hz = D;
    io.seed = 3;
    const ImuRecording rec = generate_imu(rest, io);
    double sum2 = 0.0;
    int n = 0;
    for (int k = 0; k < rec.n_nodes(); k++)
        for (int axis = 0; axis < 3; axis++)
        {
            const double noise = rec.acc_g[k](axis) * standard_gravity - (axis == 2 ? standard_gravity : 0.0); // [m/s2]
            const double dv = noise * rec.dt_s();                                                                // the velocity error over one node interval
            sum2 += dv * dv;
            n++;
        }
    const double claimed = velocity_prior_cov_over_gap(Eigen::Matrix3d::Zero(), D, 0.0, rec.dt_s())(0, 0);
    const double ratio = sum2 / n / claimed;
    const auto interval = truth_stats::chi2_interval(n, 0.01);
    MESSAGE("D4 gap noise: velocity error over one node interval, measured variance / (D^2 gap) = " << ratio << " over " << n << " draws (1% interval [" << interval.first / n << ", "
                                                                                                    << interval.second / n << "])");
    CHECK(inside_interval(sum2 / claimed, static_cast<double>(n)));
}
