#include <doctest/doctest.h>

#include "angle_units.h"
#include "memory_point_store.h"
#include "ndt_map.h"
#include "run_odometry.h"
#include "synthetic_recording.h"
#include "synthetic_routes.h"
#include "toml_config_repository.h"
#include "transformations.h"
#include "truth_statistics.h"
#include "window_loop.h"
#include "window_noise_model.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// F-c of the consistency harness (openspec change consistency-harness, task 5.3; design D4, D5 "F-c" and "Error vectors",
// D6): ONE window solved by the real loop steps (process_windows, window 2) from a carried state whose prior error is drawn
// from the prior's claimed covariance, with model-world inertial data, and the error of what the window hands on compared
// with the covariance it claims. MEASURED AND REPORTED; nothing is marked doctest::should_fail. The CHECKs are the fixture's
// validity (the claimed prior as computed here, not at rest, blindness, the exact boundary) and the three controls of the
// error definitions, never the honesty outcome.
//
// The fixture (hand-built windows, as loop_chains_test.cpp, not the generator's front-end cut):
//  - window 0: a rest window (start_window_loop takes the gravity vector and the gravity norm from it); window 1: the
//    predecessor history, moving at the prior mean velocity so that the predictor puts window 2's first node exactly on the
//    true start (the boundary is exact: the origin fix is a conditional observation that holds); window 2 is solved;
//  - 500 Hz, 181 nodes per window (180 intervals), consecutive windows one interval apart (the gap of the prior);
//  - truth: constant attitude, constant world acceleration a = (0.8, 0, 0) m/s2, start velocity = prior mean - drawn prior
//    error (prior mean (1.2, 0, 0) m/s); constant acceleration makes the accelerometer terms' quadrature exact (task 5.1);
//  - accelerometer sample k = the true specific force over [t_k, t_k+1) plus white noise of density D in every body axis
//    (sigma D / sqrt(dt) per sample), plus the true offset, in the sensor's units (reading of 1 g = `scale_g`);
//  - gyro sample k (for the interval ENDING at node k) = the true bias + white noise of ARW / sqrt(dt) per body axis (the
//    settings' ARW); the prepared attitudes are its double-precision integration (no AHRS, so ahrs_gain plays no part; the
//    start-up bias subtracted is 0, so the gyro-bias error is estimate - generated bias);
//  - case (b): 20 rays per node cast from the true pose on a closed corridor 24 m x 2 m x 2.5 m (synthetic::cast_ray, range
//    6 m, both end walls out of range), each hit moved by N(0, 1 cm) along its face normal; the grids seeded with update_rgd
//    (exact cell statistics) from the faces sampled at uniform random positions, one per 25 cm2, same jitter; map and scan
//    redrawn in every run.
//
// The model world's settings (D4): settings/livox-vertical.toml with the stage 8 candidate's differences (as
// loop_chains_test.cpp), and: accelerometer density D = sqrt(density^2 + floor^2 tau0) of the shipped values (tau0 = 11 ms),
// floor 0; rotation k = 1, floor 0, turn k off; imu_acceleration_span_s 0.0039 (anchors every second node: every term's
// samples are its own, the supports disjoint: triples on samples 2j+1, 2j+2, the start term on sample 0, the end term on
// sample 179, so the weight 1 / (D^2 / tau) tau^2 is exactly the variance of each term's noise and the terms are
// independent); log_window_terms true; real_time_threshold_seconds 1000 (no time limit).
//
// Active factors and their kinds (D4), primary set:
//  - accelerometer terms (triples, start, end)          generated raw noise (disjoint supports, as above)
//  - rotation edges                                      generated raw noise (k = 1, floor 0, gyro at ARW / sqrt(dt))
//  - gyro-bias prior, velocity prior over the gap, offset prior (joint prior F P F^T with the joint carry)
//                                                        propagated claim; the truth is drawn from exactly that claim
//  - origin fix (node 0)                                 conditional observation: the boundary is exact (verified)
//  - zero velocity                                       not active: the window is not at rest (verified; constant
//                                                        acceleration 0.8 m/s2, so A2's constant-velocity case is avoided)
//  - edge position rows (C34)                            approximate factor, made inactive: imu_position_information_scale
//                                                        1e-6 (sigma 0.1 m per 2 ms edge; their velocity information is
//                                                        below 1e-4 of the prior's). With the shipped 0.1 they are reported
//                                                        descriptively (a separate set).
//  - NDT term (case b only)                              approximate factor (it fixes the attitude, which the
//                                                        accelerometer couples into the along-axis acceleration)
//
// Error vectors (D5): velocity = handed-on end velocity - true velocity at the last node; offset = s_before (g_after -
// g_true,raw) with g_true,raw = prepared gravity + true offset / s_before (raw units), against the handed-on offset
// covariance (SI); gyro bias = handed-on estimate - generated bias (sensor axes, rad/s); joint = (velocity, offset) along the
// axis with the handed-on cross block (joint carry only: the cross block is handed on only then). The axis u is world x
// (the travel direction, the corridor's axis), turned with the frame in the rotated control.
//
// Primary comparisons (fixed before the runs), each the summed normalised error over M runs against the two-sided
// chi-square interval at 1 % / 12 (Bonferroni):
//  case (a), switches off and on: velocity along u (r = 1), offset along u (r = 1), gyro bias (r = 3); switches on also
//  velocity + offset along u jointly (r = 2)                                                   -> 7
//  case (b), switches off and on: velocity along u (r = 1), offset along u (r = 1); switches on the joint (r = 2) -> 5
// The window is the only one solved, so no windows are pooled. Numerical tolerances: the prepared prior equals the claim
// computed here within 1e-9 relative; the boundary within 1e-9 m / 1e-9 rad; the controls' per-run normalised errors within
// 1e-3 of the reference run's (absolute).

using synthetic::NullLogger;

namespace
{
    constexpr double dt = 0.002;
    constexpr double g0 = 9.80665;
    constexpr int n_nodes = 181;
    constexpr double t_origin = 100.0;
    constexpr int primary_comparisons = 12;
    constexpr double alpha = 0.01;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    double node_time(int w, int k) { return t_origin + (w * n_nodes + k) * dt; }

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

    std::string f(double v, int precision = 3)
    {
        std::ostringstream o;
        o << std::fixed << std::setprecision(precision) << v;
        return o.str();
    }

    // what one run is set up with
    struct Spec
    {
        bool lidar = false;      // case (b): the corridor
        bool switches = false;   // imu_edges_follow_state and imu_carry_joint_covariance
        double position_scale = 1e-6;
        double scale_g = 1.0;    // the sensor's reading of 1 g (control 1)
        Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity(); // control 2: the whole problem in a frame turned by R0 (gravity
                                                          // included), the body's readings unchanged
        double carry_cross_rho = 0.0; // control 3: the carried velocity-offset correlation (joint carry)
        std::uint64_t seed = 1;
    };

    // the carried state's claims (before window 2)
    constexpr double carry_velocity_sigma = 0.02;  // m/s
    constexpr double carry_offset_sigma = 0.05;    // m/s2
    constexpr double carry_bias_sigma_deg_s = 0.05;
    const Eigen::Vector3d prior_velocity_mean_base(1.2, 0.0, 0.0);
    const Eigen::Vector3d acceleration_base(0.8, 0.0, 0.0);

    struct ModelConfig
    {
        OdometryConfig c;
        double density = 0.0; // D [m/s2/sqrt(Hz)]
    };

    ModelConfig model_config(const Spec &s)
    {
        ModelConfig m;
        OdometryConfig &c = m.c;
        RunPaths paths;
        GuiPreferences gui;
        TomlConfigRepository repo;
        const bool loaded = repo.load(std::string(LO_REPO_DIR) + "/settings/livox-vertical.toml", c, paths, gui);
        REQUIRE(loaded);
        // the stage 8 candidate (loop_chains_test.cpp)
        c.use_exact_ndt_bucket_statistics = true;
        c.degenerate_projection_ratio = 0.02;
        c.degenerate_projection_ratio_full = 0.04;
        c.imu_acceleration_offset_rw_mps2_sqrt_s = 0.01;
        c.log_window_terms = true;
        c.real_time_threshold_seconds = 1000.0;
        // the model world (D4)
        constexpr double tau0 = 0.011;
        m.density = std::sqrt(c.imu_accel_noise_density_mps2_sqrt_hz * c.imu_accel_noise_density_mps2_sqrt_hz +
                              c.imu_accel_floor_sigma_mps2 * c.imu_accel_floor_sigma_mps2 * tau0);
        c.imu_accel_noise_density_mps2_sqrt_hz = m.density;
        c.imu_accel_floor_sigma_mps2 = 0.0;
        c.imu_accel_window_sigma_mps2 = 0.0;
        c.use_time_scaled_rotation_sigma = true;
        c.rotation_sigma_inflation_k = 1.0;
        c.rotation_sigma_floor_deg = 0.0;
        c.imu_acceleration_span_s = 0.0039;
        c.imu_position_information_scale = s.position_scale;
        c.imu_edges_follow_state = s.switches;
        c.imu_carry_joint_covariance = s.switches;
        return m;
    }

    // ---- the corridor of case (b) ----
    synthetic::Scene corridor_scene()
    {
        return synthetic::shifted(synthetic::box_scene({-12.0, -1.0, 0.0}, {12.0, 1.0, 2.5}, false), synthetic::grid_offset);
    }
    constexpr double corridor_range = 6.0;
    constexpr double jitter_m = 0.01;
    constexpr int rays_per_node = 20;

    void seed_grids(OdometryState &state, const synthetic::Scene &scene, const OdometryConfig &c, truth_stats::Rng &rng)
    {
        std::vector<Point3Di> points;
        for (const synthetic::Face &face : scene.faces)
        {
            const int a = (face.axis + 1) % 3, b = (face.axis + 2) % 3;
            const int nu = std::max(1, int(std::ceil((face.hi[a] - face.lo[a]) / 0.05)));
            const int nv = std::max(1, int(std::ceil((face.hi[b] - face.lo[b]) / 0.05)));
            for (int i = 0; i < nu * nv; i++)
            {
                Point3Di p{};
                const double u = rng.uniform(), v = rng.uniform();
                p.point = synthetic::face_point(face, u, v, jitter_m * rng.normal());
                p.index_point = int(points.size());
                points.push_back(p);
            }
        }
        state.buckets_indoor.clear();
        state.buckets_outdoor.clear();
        update_rgd(c.in_out_params_indoor, state.buckets_indoor, points, scene.centre, c.use_exact_ndt_bucket_statistics);
        if (c.ablation_study_use_hierarchical_rgd)
            update_rgd(c.in_out_params_outdoor, state.buckets_outdoor, points, scene.centre, c.use_exact_ndt_bucket_statistics);
    }

    // what one run gives
    struct RunResult
    {
        bool valid = false;
        std::string why; // why not valid
        double seconds = 0.0;
        // normalised errors along u (signed z = e / sigma) and the full ones
        double z_v = nan, z_o = nan, joint = nan, bias = nan, velocity3 = nan, offset3 = nan;
        double z_v_y = nan, z_v_z = nan;
        double e_v = nan, e_o = nan;          // errors along u
        double claim_cross = nan;             // claimed cov(v_end, offset) along u, as a correlation
        double joint_without_cross = nan;     // the joint statistic with the cross block dropped
        double z_b[3] = {nan, nan, nan};
        // validity readings
        bool at_rest = false;
        double prior_mismatch = nan, boundary_position = nan, boundary_rotation = nan;
        double ndt_along_ratio = nan, ndt_sideways = nan, raw_along_share = nan;
        int iterations = 0, points = 0;
    };

    RunResult run_window(const Spec &s)
    {
        RunResult out;
        const ModelConfig mc = model_config(s);
        const OdometryConfig &c = mc.c;
        truth_stats::Rng rng(s.seed);
        const Eigen::Matrix3d &R0 = s.R0;
        const double scale = s.scale_g;
        const double to_mps2 = g0 / scale;
        const double sigma_sample = mc.density / std::sqrt(dt);
        const Eigen::Vector3d arw(c.gyro_arw_x_deg_sqrt_s * DEG_TO_RAD, c.gyro_arw_y_deg_sqrt_s * DEG_TO_RAD, c.gyro_arw_z_deg_sqrt_s * DEG_TO_RAD);
        const double duration = (n_nodes - 1) * dt;
        const double gap = node_time(2, 0) - node_time(1, n_nodes - 1);

        // the claimed prior of window 2, computed from the carry with the production's own covariance helpers (and checked
        // against what the window was actually prepared with)
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d Pv = I * (carry_velocity_sigma * carry_velocity_sigma);
        const Eigen::Matrix3d Po = I * (carry_offset_sigma * carry_offset_sigma);
        const Eigen::Matrix3d Pvo = s.switches ? Eigen::Matrix3d(I * (s.carry_cross_rho * carry_velocity_sigma * carry_offset_sigma))
                                               : Eigen::Matrix3d(Eigen::Matrix3d::Zero());
        const double sb = carry_bias_sigma_deg_s * DEG_TO_RAD;
        const Eigen::Matrix3d Pb = I * (sb * sb);
        JointPriorCov P6 = JointPriorCov::Zero();
        if (s.switches)
            P6 = joint_prior_cov_over_gap(joint_prior_cov(Pv, Pvo, Po), mc.density, 0.0, gap, c.imu_acceleration_offset_rw_mps2_sqrt_s, duration);
        else
            P6 = joint_prior_cov(velocity_prior_cov_over_gap(Pv, mc.density, 0.0, gap), Eigen::Matrix3d::Zero(),
                                 random_walk_prior_cov(Po, c.imu_acceleration_offset_rw_mps2_sqrt_s, duration));
        const Eigen::Matrix3d Pb_prior = random_walk_prior_cov(Pb, c.gyro_bias_rw_deg_s_sqrt_s * DEG_TO_RAD, duration);

        // the draws: prior errors (prior mean - truth) of start velocity and offset, jointly; the gyro bias; in the base frame
        // (the claims are multiples of I per block, so a draw turned by R0 has the same law)
        const Eigen::VectorXd prior_error = rng.gaussian(P6);
        const Eigen::Vector3d bias_true = rng.gaussian(Pb_prior);
        const Eigen::Vector3d v0_base = prior_velocity_mean_base - prior_error.head<3>();
        const Eigen::Vector3d offset_base = -prior_error.tail<3>(); // prior mean 0
        const Eigen::Vector3d p0 = s.lidar ? Eigen::Vector3d(0.0, 0.0, 1.2) + synthetic::grid_offset : Eigen::Vector3d(0.3, -0.2, 1.0);

        // ---- the windows ----
        std::vector<Window> windows(3);
        const Eigen::Vector3d prior_mean_world = R0 * prior_velocity_mean_base;
        for (int w = 0; w <= 1; w++)
        {
            Window &win = windows[w];
            for (int k = 0; k < n_nodes; k++)
            {
                const double t = node_time(w, k);
                win.timestamps.push_back({t, t});
                Eigen::Affine3d pose = Eigen::Affine3d::Identity();
                pose.linear() = R0;
                // window 1: the predecessor history, at the prior mean velocity, ending one gap before the true start
                pose.translation() = R0 * p0 - prior_mean_world * (node_time(2, 0) - (w == 1 ? t : node_time(1, 0)));
                win.poses.push_back(pose);
                win.imu_acc.push_back(Eigen::Vector3d(0.0, 0.0, scale));
                win.imu_angles.push_back(Eigen::Vector3d::Zero());
            }
            win.motion_model_poses = win.poses;
        }
        // window 2: the front end's form (prepared attitudes from the gyro, zero translations: the loop predicts them)
        Window &win = windows[2];
        Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity(); // the body's attitude as the gyro integrates it (base frame)
        const Eigen::Vector3d g_carry_base(0.0, 0.0, scale);
        for (int k = 0; k < n_nodes; k++)
        {
            const double t = node_time(2, k);
            win.timestamps.push_back({t, t});
            if (k > 0)
            {
                Eigen::Vector3d omega = bias_true;
                for (int a = 0; a < 3; a++)
                    omega[a] += arw[a] / std::sqrt(dt) * rng.normal();
                const double angle = omega.norm() * dt;
                if (angle > 0.0)
                    R_meas = R_meas * Eigen::AngleAxisd(angle, omega.normalized()).toRotationMatrix();
            }
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = R0 * R_meas;
            win.poses.push_back(pose);
            // the AHRS's own view of the body (gravity vertical in its frame), as the front end stores it
            Eigen::Affine3d body = Eigen::Affine3d::Identity();
            body.linear() = R_meas;
            const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(body);
            win.imu_angles.push_back(Eigen::Vector3d(tb.om, tb.fi, tb.ka));
            // the true attitude is constant (the body level in the base frame): the specific force in the body is the world's
            Eigen::Vector3d noise;
            for (int a = 0; a < 3; a++)
                noise[a] = sigma_sample * rng.normal();
            win.imu_acc.push_back(g_carry_base + (acceleration_base + offset_base + noise) / to_mps2);
        }
        win.motion_model_poses = win.poses;

        // ---- points and map ----
        MemoryPointStore store;
        OdometryState state;
        std::vector<Point3Di> points;
        if (s.lidar)
        {
            const synthetic::Scene scene = corridor_scene();
            seed_grids(state, scene, c, rng);
            for (int k = 0; k < n_nodes; k++)
            {
                const double tau = node_time(2, k) - node_time(2, 0);
                const Eigen::Vector3d origin = p0 + v0_base * tau + 0.5 * acceleration_base * tau * tau;
                for (int r = 0; r < rays_per_node; r++)
                {
                    Eigen::Vector3d dir(rng.normal(), rng.normal(), rng.normal());
                    dir.normalize();
                    double distance = 0.0;
                    int face = -1;
                    if (!synthetic::cast_ray(scene, origin, dir, corridor_range, distance, face))
                        continue;
                    Eigen::Vector3d hit = origin + distance * dir;
                    hit[scene.faces[face].axis] += jitter_m * rng.normal();
                    Point3Di p{};
                    p.point = hit - origin; // the attitude is the identity in case (b)
                    p.timestamp = node_time(2, k);
                    p.index_pose = k;
                    p.index_point = int(points.size());
                    points.push_back(p);
                }
            }
        }
        out.points = int(points.size());
        store.save(PointSet::WindowIntermediate, 2, points); // case (a): an explicitly empty point set

        // ---- the loop ----
        NoPreview preview;
        NoProgress progress;
        NullLogger logger;
        double ts_failure = 0.0;
        WindowLoop loop;
        start_window_loop(loop, windows[0], c, logger);
        const Eigen::Vector3d g_carry = loop.carry.acc_gravity; // R0 (0, 0, scale) from window 0
        loop.carry.velocity_valid = true;
        loop.carry.velocity_mean = prior_mean_world;
        loop.carry.velocity_cov = Pv;
        loop.carry.velocity_time_s = node_time(1, n_nodes - 1);
        loop.carry.velocity_offset_cov = Pvo;
        loop.carry.acc_offset_cov = Po;
        loop.carry.bias.mean.setZero();
        loop.carry.bias.covariance = Pb;
        KeepTerms sink;
        WindowRange range;
        range.terms = &sink;
        const auto start = std::chrono::steady_clock::now();
        const bool ok = process_windows(loop, windows, 2, 2, c, state, ts_failure, false, store, preview, logger, progress, range);
        out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (!ok || sink.records.size() != 1)
        {
            out.why = "the loop did not finish the window";
            return out;
        }
        const WindowTermRecord &r = sink.records[0];
        const WindowHandedOnState &h = r.handed_on;
        const WindowPreparedState &p = r.prepared;
        if (r.handoff != WindowHandOff::done || !h.velocity_end || !h.velocity_end_cov || !h.gravity || !h.to_mps2_before_fold ||
            !h.offset_cov || !h.bias_mean_rad_s || !h.bias_cov || !p.velocity_prior_cov || !p.offset_prior_cov || !p.bias_prior_cov ||
            !p.first_pose || !p.gravity || !p.velocity_prior_mean || (s.switches && (!h.velocity_offset_cov || !p.velocity_offset_prior_cov)))
        {
            out.why = "a record field is missing";
            return out;
        }
        out.at_rest = r.at_rest;
        out.iterations = r.iterations;

        // validity: the prepared prior is the claim the truth was drawn from; the boundary is exact
        const auto rel = [](const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) { return (a - b).norm() / std::max(b.norm(), 1e-300); };
        out.prior_mismatch = std::max({rel(*p.velocity_prior_cov, P6.topLeftCorner<3, 3>()), rel(*p.offset_prior_cov, P6.bottomRightCorner<3, 3>()),
                                       rel(*p.bias_prior_cov, Pb_prior), rel(*p.velocity_prior_mean, prior_mean_world),
                                       s.switches ? (*p.velocity_offset_prior_cov - Eigen::Matrix3d(P6.topRightCorner<3, 3>())).norm() /
                                                        P6.topLeftCorner<3, 3>().norm()
                                                  : 0.0});
        out.boundary_position = (p.first_pose->translation() - R0 * p0).norm();
        out.boundary_rotation = Eigen::AngleAxisd(p.first_pose->linear() * R0.transpose()).angle();

        // the error vectors (D5), in the run's world frame
        const Eigen::Vector3d u = R0 * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d v_end_true = R0 * (v0_base + acceleration_base * (node_time(2, n_nodes - 1) - node_time(2, 0)));
        const Eigen::Vector3d e_v = *h.velocity_end - v_end_true;
        const double s_before = *h.to_mps2_before_fold;
        const Eigen::Vector3d g_true_raw = *p.gravity + R0 * offset_base / s_before;
        const Eigen::Vector3d e_o = s_before * (*h.gravity - g_true_raw);
        const Eigen::Vector3d e_b = *h.bias_mean_rad_s - bias_true;
        const Eigen::Matrix3d &Cv = *h.velocity_end_cov, &Co = *h.offset_cov, &Cb = *h.bias_cov;
        (void)g_carry;

        const auto along = [&](const Eigen::Vector3d &e, const Eigen::Matrix3d &C, const Eigen::Vector3d &axis)
        { return axis.dot(e) / std::sqrt(axis.dot(C * axis)); };
        out.e_v = u.dot(e_v);
        out.e_o = u.dot(e_o);
        out.z_v = along(e_v, Cv, u);
        out.z_o = along(e_o, Co, u);
        out.z_v_y = along(e_v, Cv, R0 * Eigen::Vector3d::UnitY());
        out.z_v_z = along(e_v, Cv, R0 * Eigen::Vector3d::UnitZ());
        out.velocity3 = truth_stats::normalised_error(e_v, Cv);
        out.offset3 = truth_stats::normalised_error(e_o, Co);
        out.bias = truth_stats::normalised_error(e_b, Cb);
        for (int a = 0; a < 3; a++)
            out.z_b[a] = e_b[a] / std::sqrt(Cb(a, a));
        if (s.switches)
        {
            const Eigen::Matrix3d &Cvo = *h.velocity_offset_cov;
            Eigen::Matrix2d C2;
            C2 << u.dot(Cv * u), u.dot(Cvo * u), u.dot(Cvo * u), u.dot(Co * u);
            const Eigen::Vector2d e2(out.e_v, out.e_o);
            out.joint = truth_stats::normalised_error(e2, C2);
            out.claim_cross = C2(0, 1) / std::sqrt(C2(0, 0) * C2(1, 1));
            Eigen::Matrix2d D2 = C2;
            D2(0, 1) = D2(1, 0) = 0.0;
            out.joint_without_cross = truth_stats::normalised_error(e2, D2);
        }

        // validity of case (b): no information along the corridor's axis, information across kept
        if (s.lidar)
        {
            double along_w = 0.0, sideways_w = 0.0;
            for (const WindowTermRow &row : r.result.rows)
                if (row.name == "ndt_indoor" || row.name == "ndt_outdoor")
                {
                    along_w += row.weight[0][0];
                    sideways_w += row.weight[0][1];
                }
            out.ndt_along_ratio = sideways_w > 0.0 ? along_w / sideways_w : nan;
            out.ndt_sideways = sideways_w;
            const Eigen::Matrix3d &T = r.result.translation_information;
            out.raw_along_share = T.trace() > 0.0 ? r.result.directions[0].dot(T * r.result.directions[0]) / T.trace() : nan;
        }

        // the run's validity, as decided before the runs
        std::ostringstream why;
        if (out.at_rest)
            why << "taken as at rest; ";
        if (!(out.prior_mismatch <= 1e-9))
            why << "prepared prior differs from the claim by " << out.prior_mismatch << "; ";
        if (!(out.boundary_position <= 1e-9) || !(out.boundary_rotation <= 1e-9))
            why << "boundary not exact (" << out.boundary_position << " m, " << out.boundary_rotation << " rad); ";
        if (!std::isfinite(out.z_v) || !std::isfinite(out.z_o) || !std::isfinite(out.bias) || (s.switches && !std::isfinite(out.joint)))
            why << "a normalised error is not finite; ";
        if (s.lidar && !(out.ndt_along_ratio < 1e-3 && out.ndt_sideways > 0.0 && out.raw_along_share < 0.02))
            why << "not blind along the axis (along/sideways " << out.ndt_along_ratio << ", sideways " << out.ndt_sideways
                << ", raw share " << out.raw_along_share << "); ";
        out.why = why.str();
        out.valid = out.why.empty();
        return out;
    }

    // the smallest variance ratio rho > 1 detected with >= 90 % power both ways (claimed x rho and / rho) by the two-sided
    // test with k degrees of freedom at level a
    double detectable_ratio(double k, double a)
    {
        double lo = 1.0, hi = 100.0;
        for (int it = 0; it < 100; it++)
        {
            const double mid = 0.5 * (lo + hi);
            const bool ok = truth_stats::power_of_variance_ratio(k, mid, a) >= 0.9 && truth_stats::power_of_variance_ratio(k, 1.0 / mid, a) >= 0.9;
            (ok ? hi : lo) = mid;
        }
        return hi;
    }

    struct Summary
    {
        std::string name;
        int dof_per_run = 1;
        bool primary = true;
        std::vector<double> values; // per valid run: the normalised error (r dof)
    };

    std::string summary_line(const Summary &s)
    {
        std::ostringstream o;
        const int M = int(s.values.size());
        double sum = 0.0;
        for (double v : s.values)
            sum += v;
        const double k = double(M) * s.dof_per_run;
        o << "  " << s.name << " (r = " << s.dof_per_run << (s.primary ? ", primary" : ", descriptive") << "): M = " << M;
        if (M == 0)
            return o.str() + " (no valid run)";
        const double a = s.primary ? alpha / primary_comparisons : alpha;
        const auto interval = truth_stats::chi2_interval(k, a);
        o << ", sum " << f(sum, 2) << " [" << f(interval.first, 2) << ", " << f(interval.second, 2) << "], per dof " << f(sum / k)
          << " [" << f(interval.first / k) << ", " << f(interval.second / k) << "], P(chi2 <= sum) = " << f(truth_stats::chi2_cdf(k, sum), 4)
          << ", detectable variance ratio " << f(detectable_ratio(k, a), 2);
        return o.str();
    }

    struct SetResult
    {
        std::vector<RunResult> runs;
        int invalid = 0;
        std::string invalid_reasons;
        double seconds = 0.0;
        double mean_iterations = 0.0;
        int min_points = 0, max_points = 0;
    };

    SetResult run_set(Spec spec, int M, std::uint64_t first_seed)
    {
        SetResult set;
        double iterations = 0.0;
        set.min_points = std::numeric_limits<int>::max();
        for (int j = 0; j < M; j++)
        {
            spec.seed = first_seed + std::uint64_t(j);
            RunResult r = run_window(spec);
            set.seconds += r.seconds;
            iterations += r.iterations;
            set.min_points = std::min(set.min_points, r.points);
            set.max_points = std::max(set.max_points, r.points);
            if (!r.valid)
            {
                set.invalid++;
                if (set.invalid <= 3)
                    set.invalid_reasons += "seed " + std::to_string(spec.seed) + ": " + r.why + " ";
            }
            set.runs.push_back(r);
        }
        set.mean_iterations = M > 0 ? iterations / M : 0.0;
        return set;
    }

    std::string set_report(const std::string &tag, const Spec &spec, const SetResult &set, bool primary)
    {
        std::vector<Summary> rows;
        rows.push_back({"velocity along u", 1, primary, {}});
        rows.push_back({"offset along u", 1, primary, {}});
        if (!spec.lidar)
            rows.push_back({"gyro bias", 3, primary, {}});
        if (spec.switches)
            rows.push_back({"velocity + offset along u, joint", 2, primary, {}});
        rows.push_back({"velocity, 3 axes", 3, false, {}});
        rows.push_back({"offset, 3 axes", 3, false, {}});
        if (spec.lidar)
            rows.push_back({"gyro bias (NDT active: descriptive)", 3, false, {}});
        double mean_zv = 0.0, mean_zo = 0.0, ss_vy = 0.0, ss_vz = 0.0;
        double bx = 0.0, by = 0.0, bz = 0.0;
        int n = 0;
        for (const RunResult &r : set.runs)
        {
            if (!r.valid)
                continue;
            n++;
            size_t i = 0;
            rows[i++].values.push_back(r.z_v * r.z_v);
            rows[i++].values.push_back(r.z_o * r.z_o);
            if (!spec.lidar)
                rows[i++].values.push_back(r.bias);
            if (spec.switches)
                rows[i++].values.push_back(r.joint);
            rows[i++].values.push_back(r.velocity3);
            rows[i++].values.push_back(r.offset3);
            if (spec.lidar)
                rows[i++].values.push_back(r.bias);
            mean_zv += r.z_v;
            mean_zo += r.z_o;
            ss_vy += r.z_v_y * r.z_v_y;
            ss_vz += r.z_v_z * r.z_v_z;
            bx += r.z_b[0] * r.z_b[0];
            by += r.z_b[1] * r.z_b[1];
            bz += r.z_b[2] * r.z_b[2];
        }
        std::ostringstream o;
        o << tag << ": " << set.runs.size() << " runs, " << set.invalid << " invalid" << (set.invalid ? " (" + set.invalid_reasons + ")" : "")
          << ", " << f(set.seconds, 1) << " s (" << f(set.seconds / std::max<size_t>(1, set.runs.size()), 2) << " s per window, "
          << f(set.mean_iterations, 1) << " iterations)";
        if (spec.lidar)
            o << ", points per window " << set.min_points << "-" << set.max_points;
        o << "\n";
        for (const Summary &s : rows)
            o << summary_line(s) << "\n";
        if (n > 0)
            o << "  descriptive: mean signed z velocity along u " << f(mean_zv / n) << ", offset along u " << f(mean_zo / n)
              << " (+-" << f(1.0 / std::sqrt(double(n))) << " for 1 sigma); velocity y, z per run " << f(ss_vy / n) << ", " << f(ss_vz / n)
              << "; gyro bias x, y, z per run " << f(bx / n) << ", " << f(by / n) << ", " << f(bz / n);
        return o.str();
    }

    Eigen::Matrix3d rotated_start()
    {
        return (Eigen::AngleAxisd(40.0 * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-20.0 * DEG_TO_RAD, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(15.0 * DEG_TO_RAD, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    }

    // control 1 or 2: the same seeds with one definition-relevant change; every per-run normalised error must agree with
    // the reference run's within 1e-3
    struct ControlResult
    {
        double largest = 0.0;
        int runs = 0;
        bool all_valid = true;
    };

    ControlResult paired_control(const Spec &reference, const Spec &changed, int K, std::uint64_t first_seed)
    {
        ControlResult c;
        const SetResult a = run_set(reference, K, first_seed), b = run_set(changed, K, first_seed);
        for (int j = 0; j < K; j++)
        {
            const RunResult &x = a.runs[j], &y = b.runs[j];
            c.all_valid = c.all_valid && x.valid && y.valid;
            const double d[] = {std::abs(x.z_v - y.z_v), std::abs(x.z_o - y.z_o), std::abs(x.bias - y.bias),
                                reference.switches ? std::abs(x.joint - y.joint) : 0.0};
            for (double v : d)
                c.largest = std::max(c.largest, std::isfinite(v) ? v : 1e300);
            c.runs++;
        }
        return c;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The smoke case (in every ctest): 6 runs (case a, switches off and on, two each; case b, off and on, one each). It asserts
// only that the runs complete with finite values and that their preconditions hold; no honesty result (spec: "A small set
// claims nothing it cannot show").
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("window covariance smoke: one window per run completes with finite values and valid preconditions")
{
    double seconds = 0.0;
    std::ostringstream o;
    for (const bool lidar : {false, true})
        for (const bool switches : {false, true})
        {
            Spec s;
            s.lidar = lidar;
            s.switches = switches;
            const int runs = lidar ? 1 : 2;
            for (int j = 0; j < runs; j++)
            {
                s.seed = 9000 + std::uint64_t(j);
                const RunResult r = run_window(s);
                seconds += r.seconds;
                INFO("case ", std::string(lidar ? "b" : "a"), ", switches ", std::string(switches ? "on" : "off"), ", seed ", s.seed, ": ", r.why);
                CHECK(r.valid);
                CHECK_FALSE(r.at_rest);
                CHECK(r.prior_mismatch <= 1e-9);
                CHECK(r.boundary_position <= 1e-9);
                CHECK(r.boundary_rotation <= 1e-9);
                CHECK(std::isfinite(r.z_v));
                CHECK(std::isfinite(r.z_o));
                CHECK(std::isfinite(r.bias));
                if (lidar)
                {
                    CHECK(r.points > 1000);
                    CHECK(r.ndt_along_ratio < 1e-3);
                    CHECK(r.ndt_sideways > 0.0);
                }
                else
                    CHECK(r.points == 0);
                o << (lidar ? "b" : "a") << (switches ? " on" : " off") << ": z_v " << f(r.z_v, 2) << ", z_o " << f(r.z_o, 2) << "; ";
            }
        }
    MESSAGE("window covariance smoke (6 runs, a smoke check, not an honesty test): ", o.str(), f(seconds, 1), " s");
}

// ---------------------------------------------------------------------------------------------------------------------
// The full set, on demand:
//   lidar_odometry_tests -tc="window covariance:*" --no-skip -s
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("window covariance: the handed-on state against its claim, cases (a) and (b), switches off and on" * doctest::skip())
{
    const truth_stats::PowerPlan plan = truth_stats::runs_for_power(1, 1.5, alpha, primary_comparisons);
    const int M = plan.runs;
    MESSAGE("design (D6): r = 1 is the smallest state; ", primary_comparisons, " primary comparisons (Bonferroni), 1 %, 90 % power for x1.5 "
            "and /1.5: M = ", M, " runs (power ", f(plan.power_low_side), " / ", f(plan.power_high_side), "); the same runs serve every "
            "comparison of a set");
    double total = 0.0;
    std::uint64_t seed = 1;
    for (const bool lidar : {false, true})
        for (const bool switches : {false, true})
        {
            Spec s;
            s.lidar = lidar;
            s.switches = switches;
            const SetResult set = run_set(s, M, seed);
            seed += 100000;
            total += set.seconds;
            // case (a) has no data-dependent precondition: every run must be valid. In case (b) a run whose window is not
            // blind by the predeclared limit (projected NDT shift weight along / sideways < 1e-3) is excluded and reported (D8)
            if (!lidar)
                CHECK(set.invalid == 0);
            CHECK(set.invalid < M / 5);
            MESSAGE(set_report(std::string("case (") + (lidar ? "b), corridor, exact cells" : "a), no LiDAR") + ", switches " +
                                   (switches ? "ON" : "OFF"),
                               s, set, true));
        }
    MESSAGE("window covariance, primary sets: ", f(total, 1), " s");
}

TEST_CASE("window covariance: descriptive, the edge position rows at the shipped scale (approximate factor active)" * doctest::skip())
{
    const int M = 100;
    double total = 0.0;
    for (const bool switches : {false, true})
    {
        Spec s;
        s.switches = switches;
        s.position_scale = 0.1; // settings/livox-vertical.toml
        const SetResult set = run_set(s, M, 500001);
        total += set.seconds;
        MESSAGE(set_report(std::string("case (a), position rows at scale 0.1 (C34 active, descriptive), switches ") + (switches ? "ON" : "OFF"), s,
                           set, false));
    }
    MESSAGE("descriptive set: ", f(total, 1), " s");
}

TEST_CASE("window covariance: the three controls of the error definitions" * doctest::skip())
{
    const int K = 10;
    double total = 0.0;
    std::ostringstream o;
    for (const bool switches : {false, true})
    {
        Spec reference;
        reference.switches = switches;
        // control 1: a sensor scale other than 1 g (the offset error uses s_before; the readings scaled, the physics unchanged)
        Spec scaled = reference;
        scaled.scale_g = 1.02;
        const auto start = std::chrono::steady_clock::now();
        const ControlResult c1 = paired_control(reference, scaled, K, 700001);
        // control 2: a rotated start (the frame turned by yaw 40, pitch -20, roll 15 deg, gravity with it; the body's readings
        // unchanged): the error vectors and the axis are in the run's world frame
        Spec rotated = reference;
        rotated.R0 = rotated_start();
        const ControlResult c2 = paired_control(reference, rotated, K, 700001);
        total += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        CHECK(c1.all_valid);
        CHECK(c2.all_valid);
        CHECK(c1.largest <= 1e-3);
        CHECK(c2.largest <= 1e-3);
        o << "switches " << (switches ? "ON" : "OFF") << ": control 1 (scale 1.02 g) largest per-run difference " << std::scientific
          << std::setprecision(2) << c1.largest << ", control 2 (rotated start) " << c2.largest << std::fixed << " over " << K
          << " paired runs (tolerance 1e-3)\n";
    }

    // control 3: a cross covariance of known sign. Joint carry with the carried correlation -0.8: the claimed handed-on
    // cov(v_end, offset) along u must be negative (v_end = v_start - offset T + ..., and the carried block is negative), the
    // errors' sample cross moment negative too, and their sample correlation within 3 standard errors (Fisher z) of the
    // claimed one
    {
        const int M3 = 100;
        Spec s;
        s.switches = true;
        s.carry_cross_rho = -0.8;
        const SetResult set = run_set(s, M3, 800001);
        total += set.seconds;
        CHECK(set.invalid == 0);
        double claim = 0.0, sxy = 0.0, sxx = 0.0, syy = 0.0, j_with = 0.0, j_without = 0.0;
        bool all_negative = true;
        int n = 0;
        for (const RunResult &r : set.runs)
        {
            if (!r.valid)
                continue;
            n++;
            claim += r.claim_cross;
            all_negative = all_negative && r.claim_cross < 0.0;
            sxy += r.e_v * r.e_o;
            sxx += r.e_v * r.e_v;
            syy += r.e_o * r.e_o;
            j_with += r.joint;
            j_without += r.joint_without_cross;
        }
        REQUIRE(n > 3);
        claim /= n;
        const double empirical = sxy / std::sqrt(sxx * syy); // centred at the truth (the errors' mean is 0 if honest)
        const double fisher = std::abs(std::atanh(empirical) - std::atanh(claim)) * std::sqrt(double(n - 3));
        CHECK(all_negative);
        CHECK(sxy < 0.0);
        CHECK(fisher <= 3.0);
        const auto interval = truth_stats::chi2_interval(2.0 * n, alpha);
        o << "control 3 (carried correlation -0.8, joint carry, " << n << " runs): claimed handed-on correlation along u, mean " << f(claim)
          << (all_negative ? " (negative in every run)" : " (NOT negative in every run)") << "; the errors' sample correlation " << f(empirical)
          << " (Fisher distance " << f(fisher, 2) << " standard errors); joint sum with the cross block " << f(j_with, 1)
          << ", without it " << f(j_without, 1) << ", 1 % interval for " << 2 * n << " dof [" << f(interval.first, 1) << ", "
          << f(interval.second, 1) << "]\n";
    }
    MESSAGE(o.str(), "controls: ", f(total, 1), " s");
}
