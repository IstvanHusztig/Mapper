#include <doctest/doctest.h>

#include "angle_units.h"
#include "memory_point_store.h"
#include "run_odometry.h"
#include "synthetic_recording.h"
#include "window_loop.h"
#include "window_steps.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

// Unit-level cases of the consistency harness (openspec change consistency-harness, task 5.4; design D7): K3 (F2a), K4
// (F2b), A2 (the rest test on a noise-free constant-velocity window) and K6 (F3). Each MEASURES and reports its number
// beside the audit's hypothesis and names its oracle. Nothing here is marked doctest::should_fail (task 5.8, after the
// orchestrator's review): where an oracle exists the honest outcome is stated in a MESSAGE, and the CHECKs pin the
// MEASURED characterisation (the discrepancy as it is today), so that a change to it is seen.
//
// The windows are exact analytic ones: p(t) = x0 + v0 t + a t^2 / 2 along x at 500 Hz, level, the accelerometer sample of
// node k the exact specific force (a / g0, 0, 1) [g] over [t_k, t_k + 2 ms) (the convention of synthetic_inertial.h), no
// noise, timestamps t0 + k dt. No LiDAR point is given to the loop (the store holds none).

using synthetic::NullLogger;

namespace
{
    constexpr double dt = 0.002; // 500 Hz
    constexpr double g0 = 9.80665;
    constexpr int nodes = 180;

    Window motion_window(double t0, int n, double x0, double v0, double a)
    {
        Window w;
        for (int k = 0; k < n; k++)
        {
            const double tau = k * dt;
            Eigen::Affine3d p = Eigen::Affine3d::Identity();
            p.translation() = Eigen::Vector3d(x0 + v0 * tau + 0.5 * a * tau * tau, 0.0, 0.0);
            w.poses.push_back(p);
            w.timestamps.push_back({t0 + tau, t0 + tau});
            w.imu_acc.push_back(Eigen::Vector3d(a / g0, 0.0, 1.0));
            w.imu_angles.push_back(Eigen::Vector3d::Zero());
        }
        w.motion_model_poses = w.poses;
        return w;
    }

    class NoPreview : public PreviewSink
    {
    public:
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &,
                          const std::vector<double> &) override {}
        std::string location() const override { return "none"; }
    };

    class NoProgress : public ProgressReporter
    {
    public:
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    class KeepTerms : public WindowTermSink
    {
    public:
        void write(const WindowTermRecord &record) override { records.push_back(record); }
        std::vector<WindowTermRecord> records;
    };

    // three windows: [0] a rest window (the loop reads its accelerometer for gravity), [1] the preceding window under
    // constant acceleration a (starting at speed v1), [2] the next window, 2 ms after the last node of [1], the same motion
    struct Chain
    {
        std::vector<Window> windows;
        double a = 0.0;
        double t_previous_last = 0.0;   // the last node of window [1] [s]
        double v_previous_end = 0.0;    // its true velocity there [m/s]
        double v_start_true = 0.0;      // the true velocity at window [2]'s first node
        double gap_s = 0.0;
    };

    Chain make_chain(double a, double v1)
    {
        Chain c;
        c.a = a;
        const double t1 = 104.0;
        const double t2 = t1 + nodes * dt;
        c.windows.push_back(motion_window(100.0, nodes, 0.0, 0.0, 0.0));
        c.windows.push_back(motion_window(t1, nodes, 0.0, v1, a));
        const double tau2 = nodes * dt;
        c.windows.push_back(motion_window(t2, nodes, v1 * tau2 + 0.5 * a * tau2 * tau2, v1 + a * tau2, a));
        c.t_previous_last = t1 + (nodes - 1) * dt;
        c.v_previous_end = v1 + a * (nodes - 1) * dt;
        c.v_start_true = v1 + a * tau2;
        c.gap_s = t2 - c.t_previous_last;
        return c;
    }

    OdometryConfig loop_config()
    {
        OdometryConfig c;
        c.imu_always_on = true;
        c.log_window_terms = true;
        c.nr_iter = 20;
        c.real_time_threshold_seconds = 1000.0;
        return c;
    }

    // window [2] through the public loop (process_windows) with a set-up carry: the preceding window's end state is the
    // exact true one; the carry's covariance is small. Returns the window's record (log_window_terms).
    struct LoopRun
    {
        bool ok = false;
        WindowTermRecord record;
        Eigen::Vector3d carried_velocity = Eigen::Vector3d::Zero();
    };

    LoopRun run_third_window(Chain chain, const OdometryConfig &c)
    {
        LoopRun r;
        MemoryPointStore store;
        OdometryState state;
        NoPreview preview;
        NoProgress progress;
        NullLogger logger;
        double ts_failure = 0.0;
        WindowLoop loop;
        start_window_loop(loop, chain.windows[0], c, logger);
        r.carried_velocity = Eigen::Vector3d(chain.v_previous_end, 0.0, 0.0);
        loop.carry.velocity_valid = true;
        loop.carry.velocity_mean = r.carried_velocity;
        loop.carry.velocity_cov = Eigen::Matrix3d::Identity() * 1e-4;
        loop.carry.velocity_time_s = chain.t_previous_last;
        KeepTerms sink;
        WindowRange range;
        range.terms = &sink;
        r.ok = process_windows(loop, chain.windows, 2, 2, c, state, ts_failure, false, store, preview, logger, progress, range);
        if (r.ok && sink.records.size() == 1)
            r.record = sink.records[0];
        else
            r.ok = false;
        return r;
    }

    std::string mm(double metres) { std::ostringstream o; o << std::fixed << std::setprecision(4) << metres * 1000.0 << " mm"; return o.str(); }
    std::string mms(double mps) { std::ostringstream o; o << std::fixed << std::setprecision(4) << mps * 1000.0 << " mm/s"; return o.str(); }
}

// K3 (F2a): over the gap between windows the start velocity prior's mean is not moved by the measured acceleration.
// Reached through process_windows (public) on window [2] with log_window_terms: record.prepared.velocity_prior_mean is
// what prepare_velocity_state (file-local) set. Oracle: the true start velocity v_end + a * gap.
TEST_CASE("unit case K3: the start velocity prior's mean over a 2 ms gap, at constant acceleration")
{
    for (double a : {0.0, 1.0, 2.0})
    {
        CAPTURE(a);
        const Chain chain = make_chain(a, 0.5);
        // validity of the fixture: the gap is one sample interval, the window's own motion is the constant acceleration
        CHECK(chain.gap_s == doctest::Approx(dt).epsilon(1e-9));
        const OdometryConfig c = loop_config();
        const LoopRun run = run_third_window(chain, c);
        REQUIRE(run.ok);
        CHECK(run.record.window == 3);
        REQUIRE(run.record.prepared.velocity_prior_mean.has_value());
        const Eigen::Vector3d prior = *run.record.prepared.velocity_prior_mean;
        const double error = prior.x() - chain.v_start_true;

        MESSAGE("K3 a = ", a, " m/s2: prior mean ", prior.x(), " m/s, true start velocity ", chain.v_start_true, " m/s, error ",
                mms(error), " (audit's hypothesis: -a * 2 ms = ", mms(-a * dt), "; oracle: the true start velocity, error 0)");

        // the measured characterisation: the prior is the carried end velocity as it was, unmoved over the gap
        CHECK(prior.x() == run.carried_velocity.x());
        CHECK(prior.y() == 0.0);
        CHECK(prior.z() == 0.0);
        CHECK(error == doctest::Approx(-a * chain.gap_s).epsilon(1e-6).scale(1.0));
    }
}

// K4 (F2b): the tail re-propagation alone, and the next window's first pose as actually predicted.
// repropagate_window_tail, place_window_after / predict_window_poses and estimate_previous_velocity are public
// (window_steps.h). p = a t^2 / 2, so the finite-difference velocity over `span` nodes is the velocity at the middle of
// the span; the oracle is the truth.
namespace
{
    struct TailMeasurement
    {
        double last_node_error = 0.0;  // x of the re-propagated last node minus the truth [m]
        double next_first_error = 0.0; // x of the next window's first pose as predicted minus the truth [m]
    };

    TailMeasurement measure_tail(double a, int tail, int span)
    {
        const double t0 = 100.0;
        Window window = motion_window(t0, nodes, 0.0, 0.0, a);
        // the tail's own poses are not constrained by points: give them wrong values, which the step must replace
        for (int k = nodes - tail; k < nodes; k++)
            window.poses[k].translation() = Eigen::Vector3d(-1.0, 2.0, 3.0);
        repropagate_window_tail(window, tail, span, Eigen::Vector3d::Zero());

        const auto truth = [&](int k) { const double tau = k * dt; return 0.5 * a * tau * tau; };
        TailMeasurement m;
        m.last_node_error = window.poses.back().translation().x() - truth(nodes - 1);

        // the next window: first node one sample interval after, identity IMU motion, constant-velocity prediction from the
        // previous window's final velocity measured over `span` nodes (the always-on loop's prediction)
        Window next = motion_window(t0 + nodes * dt, 2, 0.0, 0.0, 0.0);
        next.poses.assign(2, Eigen::Affine3d::Identity());
        const PreviousVelocity previous = estimate_previous_velocity(window, span, 10.0);
        PosePredictionOptions options;
        options.constant_velocity = true;
        predict_window_poses(next, window, nullptr, previous.velocity, options);
        m.next_first_error = next.poses[0].translation().x() - truth(nodes);
        return m;
    }
}

TEST_CASE("unit case K4: the tail re-propagation's last node and the next window's first pose against the truth")
{
    // the audit's numbers: tail 15, span 10
    const TailMeasurement one = measure_tail(1.0, 15, 10);
    const TailMeasurement two = measure_tail(2.0, 15, 10);
    MESSAGE("K4 tail 15, span 10, a = 1 m/s2: last node ", mm(one.last_node_error), " (hypothesis -0.75 mm); next first pose ",
            mm(one.next_first_error));
    MESSAGE("K4 tail 15, span 10, a = 2 m/s2: last node ", mm(two.last_node_error), "; next first pose ", mm(two.next_first_error),
            " (hypothesis -1.66 mm); oracle: the truth, error 0");

    // the measured characterisation
    CHECK(one.last_node_error == doctest::Approx(-0.75e-3).epsilon(0.01));
    CHECK(two.next_first_error == doctest::Approx(-1.66e-3).epsilon(0.01));
    // the error is linear in the acceleration (no other source): doubling a doubles it
    CHECK(two.last_node_error == doctest::Approx(2.0 * one.last_node_error).epsilon(1e-9));
    // tail 0 does nothing (the control): the last node is the solved one, exact here
    const TailMeasurement none = measure_tail(2.0, 0, 10);
    CHECK(none.last_node_error == 0.0);

    // the settings of the shipped files (handoff_tail_nodes 30, span 20) for the same motion
    const TailMeasurement shipped = measure_tail(2.0, 30, 20);
    MESSAGE("K4 for context, tail 30, span 20, a = 2 m/s2: last node ", mm(shipped.last_node_error), "; next first pose ",
            mm(shipped.next_first_error));
}

// A2: a noise-free constant-velocity window through the rest test. window_rests (public) reads the world acceleration and
// the node rotations, never the positions: a window moving at 2 m/s at no acceleration is "at rest". Through the loop
// (process_windows, record.at_rest), with imu_always_on and use_zero_velocity_detection OFF, and what the zero-velocity
// term then does: the same window with the rest test forced to fail (zero_velocity_acc_threshold_mps2 < 0).
TEST_CASE("unit case A2: a noise-free constant-velocity window is taken as at rest, and the zero-velocity term then holds it")
{
    const double v = 2.0;
    const Chain chain = make_chain(0.0, v);
    const Window &w = chain.windows[2];
    CHECK(w.poses.back().translation().x() - w.poses.front().translation().x() > 0.7); // it moves, 0.716 m

    // the public rest test on the fixture
    const bool rests = window_rests(w.poses, w.timestamps, w.imu_acc, w.imu_angles, 1.0, 0.3, 3.0 * DEG_TO_RAD);
    MESSAGE("A2 window_rests on a window moving at ", v, " m/s without acceleration: ", std::string(rests ? "rest" : "not rest"),
            " (hypothesis: taken as rest)");
    CHECK(rests);

    // through the loop
    OdometryConfig rest_c = loop_config();
    REQUIRE_FALSE(rest_c.use_zero_velocity_detection);
    OdometryConfig moving_c = rest_c;
    moving_c.zero_velocity_acc_threshold_mps2 = -1.0; // |a| > threshold for every window: never at rest
    const LoopRun as_rest = run_third_window(chain, rest_c);
    const LoopRun as_moving = run_third_window(chain, moving_c);
    REQUIRE(as_rest.ok);
    REQUIRE(as_moving.ok);

    const double true_move = w.poses.back().translation().x() - w.poses.front().translation().x();
    const auto line = [&](const std::string &name, const LoopRun &r)
    {
        MESSAGE("A2 ", name, ": at_rest ", r.record.at_rest, ", velocity start ", r.record.velocity_start.x(), " end ", r.record.velocity_end.x(),
                " m/s (true ", v, "), guess move ", r.record.guess_move.x(), " m, result move ", r.record.result_move.x(), " m (true ", true_move, ")");
    };
    line("rest test as shipped (use_zero_velocity_detection off, imu_always_on on)", as_rest);
    line("rest test forced to fail (the paired control)", as_moving);

    CHECK(as_rest.record.at_rest);
    CHECK_FALSE(as_moving.record.at_rest);
    // the paired control keeps the true velocity (the prior, the accelerometer terms and the edges agree) ...
    CHECK(as_moving.record.velocity_start.x() == doctest::Approx(v).epsilon(1e-3));
    CHECK(as_moving.record.velocity_end.x() == doctest::Approx(v).epsilon(1e-3));
    // ... and at rest the zero-velocity observations (sigma 0.01 m/s at both ends) take the end velocity to about zero and the
    // start velocity (held by its prior of 2 m/s, sigma 0.01 m/s, against the same weight at zero) to a compromise far from 2
    CHECK(std::abs(as_rest.record.velocity_end.x()) < 0.05);
    CHECK(as_rest.record.velocity_start.x() < 1.0);
    CHECK(as_rest.record.velocity_start.x() > 0.1);
    CHECK(as_rest.record.result_move.x() < 0.1 * true_move); // the window is solved as (nearly) motionless
    // oracle (not asserted): the device moves at 2 m/s, so the honest outcome is not rest
}

// K6 (F3): with ahrs_gain = 0 the rest test removes gravity with an open-loop filter tilt. Level true poses, a constant
// tilted filter pitch in imu_angles, the true force (0, 0, 1) g, threshold 0.3 m/s2: |a| = 2 g sin(tilt / 2) = 0.3 at
// 1.754 deg. Reached through the public window_rests. Oracle: the device rests (every tilt).
namespace
{
    bool rests_at_tilt(double tilt_deg, int axis, double threshold = 0.3)
    {
        Window w = motion_window(100.0, nodes, 0.0, 0.0, 0.0);
        for (Eigen::Vector3d &angle : w.imu_angles)
            angle[axis] = tilt_deg * DEG_TO_RAD;
        return window_rests(w.poses, w.timestamps, w.imu_acc, w.imu_angles, 1.0, threshold, 3.0 * DEG_TO_RAD);
    }

    // the smallest tilt that is not recognised, by bisection on [0, 10] deg
    double tilt_threshold_deg(int axis)
    {
        double lo = 0.0, hi = 10.0;
        for (int i = 0; i < 60; i++)
        {
            const double mid = 0.5 * (lo + hi);
            (rests_at_tilt(mid, axis) ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    }
}

TEST_CASE("unit case K6: the rest test with a tilted open-loop filter attitude")
{
    // validity: level filter, the device rests: recognised; a window that really accelerates at 0.5 m/s2 is not
    CHECK(rests_at_tilt(0.0, 1));
    {
        Window moving = motion_window(100.0, nodes, 0.0, 0.0, 0.5);
        CHECK_FALSE(window_rests(moving.poses, moving.timestamps, moving.imu_acc, moving.imu_angles, 1.0, 0.3, 3.0 * DEG_TO_RAD));
    }

    std::ostringstream sweep;
    for (double tilt : {0.5, 1.0, 1.5, 1.7, 1.75, 1.8, 2.0, 3.0})
        sweep << tilt << " deg: " << (rests_at_tilt(tilt, 1) ? "rest" : "NOT rest") << ";  ";
    MESSAGE("K6 pitch sweep (threshold 0.3 m/s2): ", sweep.str());

    const double pitch = tilt_threshold_deg(1);
    const double roll = tilt_threshold_deg(0);
    const double expected = 2.0 * std::asin(0.3 / (2.0 * g0)) / DEG_TO_RAD;
    MESSAGE("K6 first tilt not recognised as rest: pitch ", pitch, " deg, roll ", roll, " deg (hypothesis about 1.75 deg; 2 asin(0.3 / 2 g) = ",
            expected, " deg); oracle: the device rests at every tilt, so the honest threshold would be none");
    // the measured characterisation
    CHECK(pitch == doctest::Approx(expected).epsilon(1e-3));
    CHECK(roll == doctest::Approx(expected).epsilon(1e-3));
    CHECK(rests_at_tilt(1.7, 1));
    CHECK_FALSE(rests_at_tilt(1.8, 1));
    // a looser threshold moves it, as the sensitivity: 0.5 m/s2 holds up to 2.92 deg
    CHECK(rests_at_tilt(2.8, 1, 0.5));
    CHECK_FALSE(rests_at_tilt(3.0, 1, 0.5));
}
