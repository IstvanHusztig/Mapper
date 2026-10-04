#include <doctest/doctest.h>

#include "angle_units.h"
#include "scan_to_map.h"
#include "synthetic_recording.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>

// optimize_lidar_odometry on synthetic scenes with a known true trajectory: a closed box room (and an open corridor),
// both NDT maps built with the app's update_rgd (exact cell statistics), and a window of nodes 10 ms apart moving through it. The options are
// the values of settings/lidar-vertical.toml, set here so that tuning the shipped file doesn't change these tests.

namespace
{
    using namespace synthetic;

    const double deg = DEG_TO_RAD;

    Scene room() { return box_scene({-4.0, -2.5, 0.0}, {4.0, 2.5, 3.0}, false); }
    Scene corridor() { return box_scene({-15.0, -2.5, 0.0}, {15.0, 2.5, 3.0}, true); }

    struct Maps
    {
        NDTBucketMapType indoor;
        NDTBucketMapType outdoor;
    };

    // both reference maps, as seed_map builds them: update_rgd with the viewport at the scene centre and the exact cell
    // statistics (use_exact_ndt_bucket_statistics)
    Maps build_maps(const Scene &s, const ScanToMapOptions &options)
    {
        std::vector<Point3Di> points = sample_scene(s);
        Maps maps;
        update_rgd(options.grid_indoor, maps.indoor, points, s.centre, true);
        update_rgd(options.grid_outdoor, maps.outdoor, points, s.centre, true);
        return maps;
    }

    constexpr int n_nodes = 50;
    constexpr double node_dt = 0.01;
    constexpr int points_per_node = 60;

    // the true trajectory: from (-1, 0, 1.5) along x at 1 m/s, yawing at 20 deg/s
    std::vector<Eigen::Affine3d> true_poses()
    {
        std::vector<Eigen::Affine3d> poses;
        for (int k = 0; k < n_nodes; k++)
        {
            const double t = k * node_dt;
            poses.push_back(pose(Eigen::Vector3d(-1.0 + 1.0 * t, 0.0, 1.5), 20.0 * deg * t));
        }
        return poses;
    }

    // what each node sees: points_per_node points spread over the faces by area (not the map's points, own seed),
    // with a +-5 mm jitter, given in the node's frame
    std::vector<Point3Di> observe(const Scene &s, const std::vector<Eigen::Affine3d> &truth)
    {
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> jitter(-0.005, 0.005);
        std::vector<double> areas;
        for (const Face &f : s.faces)
            areas.push_back(face_area(f));
        std::discrete_distribution<int> pick_face(areas.begin(), areas.end());

        std::vector<Point3Di> points;
        for (int k = 0; k < static_cast<int>(truth.size()); k++)
            for (int j = 0; j < points_per_node; j++)
            {
                const Face &f = s.faces[pick_face(rng)];
                const double u = unit(rng), v = unit(rng);
                Point3Di p{};
                p.point = truth[k].inverse() * face_point(f, u, v, jitter(rng));
                p.timestamp = k * node_dt;
                p.index_pose = k;
                p.index_point = static_cast<int>(points.size());
                points.push_back(p);
            }
        return points;
    }

    // a window whose guess (poses and motion_model_poses) is the given trajectory
    Window make_window(const std::vector<Eigen::Affine3d> &guess)
    {
        Window w;
        w.poses = guess;
        w.motion_model_poses = guess;
        for (int k = 0; k < static_cast<int>(guess.size()); k++)
            w.timestamps.push_back({k * node_dt, k * node_dt});
        return w;
    }

    // the settings of settings/lidar-vertical.toml that the optimizer reads
    OdometryConfig test_config()
    {
        OdometryConfig c;
        c.in_out_params_indoor.resolution_X = c.in_out_params_indoor.resolution_Y = c.in_out_params_indoor.resolution_Z = 0.15;
        c.in_out_params_outdoor.resolution_X = c.in_out_params_outdoor.resolution_Y = c.in_out_params_outdoor.resolution_Z = 0.4;
        c.useMultithread = false;
        c.min_distance_lidar = 0.15;
        c.max_distance_lidar = 25.0;
        c.min_distance_lidar_outdoor_grid = 1.0;
        c.ablation_study_use_planarity = false;
        c.ablation_study_use_norm = false;
        c.ablation_study_use_hierarchical_rgd = true;
        c.ablation_study_use_view_point_and_normal_vectors = true;
        c.motion_model_correction = TaitBryanPose{};
        c.lidar_odometry_motion_model_x_1_sigma_m = c.lidar_odometry_motion_model_y_1_sigma_m = c.lidar_odometry_motion_model_z_1_sigma_m = 1e-4;
        c.lidar_odometry_motion_model_om_1_sigma_deg = 0.1;
        c.lidar_odometry_motion_model_fi_1_sigma_deg = 0.1;
        c.lidar_odometry_motion_model_ka_1_sigma_deg = 0.15;
        c.lidar_odometry_motion_model_fix_origin_x_1_sigma_m = c.lidar_odometry_motion_model_fix_origin_y_1_sigma_m =
            c.lidar_odometry_motion_model_fix_origin_z_1_sigma_m = 1e-6;
        c.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = c.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg =
            c.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = 1e-6;
        c.lidar_odometry_motion_model_acc_1_sigma_mps2 = 1.0;
        c.use_time_scaled_rotation_sigma = true;
        c.gyro_arw_x_deg_sqrt_s = 0.028;
        c.gyro_arw_y_deg_sqrt_s = 0.042;
        c.gyro_arw_z_deg_sqrt_s = 0.057;
        c.rotation_sigma_inflation_k = 4.0;
        c.rotation_sigma_floor_deg = 0.002;
        c.use_gyro_bias_estimation = false; // the bias test switches it on
        c.gyro_bias_max_deg_s = 20.0;
        c.lm_factor_initial = 5.0;
        return c;
    }

    struct Options
    {
        ScanToMapOptions scan_to_map;
        MotionModelOptions motion_model;
        MotionModelRotationOptions rotation;
        MotionModelTranslationImuOptions translation_imu; // off
        VelocityPrior velocity_prior;                     // off
    };

    Options options_from(const OdometryConfig &c)
    {
        Options o;
        o.scan_to_map = get_scan_to_map_options(c);
        o.motion_model = get_motion_model_options(c);
        o.rotation = get_motion_model_rotation_options(c);
        const double bias_sigma0 = 10.0 * deg; // gyro_bias_initial_sigma_deg_s
        o.rotation.bias_prior_cov = Eigen::Matrix3d::Identity() * bias_sigma0 * bias_sigma0;
        return o;
    }

    // what one Levenberg-Marquardt iteration leaves behind
    struct IterationResult
    {
        int iteration = 0;
        double delta = 0.0;
    };

    // the iteration loop of run_lm_iterations (app/run_odometry.cpp): lm_factor starts at lm_factor_initial and grows x10
    // after iterations 10, 20, ...; after_iteration (optional) sees the window after each iteration; returns the last delta
    double run_iterations(std::vector<Point3Di> &points, Window &window, Maps &maps, const Options &o, double lm_factor_initial,
                          int iterations, Logger &logger, Eigen::Matrix3d *translation_information = nullptr,
                          const std::function<void(const IterationResult &, const Window &)> &after_iteration = {})
    {
        double lm_factor = lm_factor_initial;
        double delta = 0.0;
        Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
        Eigen::Matrix3d acc_offset_cov = Eigen::Matrix3d::Identity();
        for (int iter = 0; iter < iterations; iter++)
        {
            delta = 100000.0;
            optimize_lidar_odometry(points, window, maps.indoor, maps.outdoor, o.scan_to_map, o.motion_model, o.rotation,
                                    o.translation_imu, o.velocity_prior, lm_factor, delta, acc_offset, acc_offset_cov, logger,
                                    translation_information);
            if (after_iteration)
                after_iteration({iter, delta}, window);
            if (iter % 10 == 0 && iter > 0)
                lm_factor *= 10.0;
        }
        return delta;
    }

    // signed heading (yaw) of a's last node relative to b's last node [rad]
    double end_heading_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
    {
        const Eigen::Matrix3d r = a.back().linear() * b.back().linear().transpose();
        return std::atan2(r(1, 0), r(0, 0));
    }
}

TEST_CASE("scan-to-map fixtures: scene points on the faces, maps filled, points per node in range")
{
    const OdometryConfig config = test_config();
    const Options o = options_from(config);
    const Scene s = room();

    double farthest = 0.0; // largest distance of a scene point from its nearest face
    for (const Point3Di &p : sample_scene(s))
    {
        double nearest = 1e9;
        for (const Face &f : s.faces)
            nearest = std::min(nearest, std::abs(p.point[f.axis] - f.value));
        farthest = std::max(farthest, nearest);
    }
    CHECK(farthest <= 0.01);

    const Maps maps = build_maps(s, o.scan_to_map);
    CHECK(maps.indoor.size() > 1000);
    CHECK(maps.outdoor.size() > 100);

    const std::vector<Eigen::Affine3d> truth = true_poses();
    const std::vector<Point3Di> points = observe(s, truth);
    std::vector<int> in_range(n_nodes, 0);
    for (const Point3Di &p : points)
        if (p.point.norm() >= config.min_distance_lidar && p.point.norm() <= config.max_distance_lidar)
            in_range[p.index_pose]++;
    CHECK(*std::min_element(in_range.begin(), in_range.end()) >= 40);
    CHECK(*std::max_element(in_range.begin(), in_range.end()) <= 80);
}

namespace
{
    // the drifting guess of a gyro bias: true translations, yaw off by bias * t (node 0 is exact, as the fix origin holds it)
    std::vector<Eigen::Affine3d> drifting_guess(const std::vector<Eigen::Affine3d> &truth, double bias_rad_s)
    {
        std::vector<Eigen::Affine3d> guess = truth;
        for (int k = 0; k < static_cast<int>(guess.size()); k++)
            guess[k].linear() = Eigen::AngleAxisd(bias_rad_s * k * node_dt, Eigen::Vector3d::UnitZ()).toRotationMatrix() * truth[k].linear();
        return guess;
    }
}

TEST_CASE("scan-to-map: a perfect guess stays at the truth")
{
    const Options o = options_from(test_config());
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    Window w = make_window(truth);
    NullLogger logger;

    // Measured 2026-10-04 with exact cell statistics: 0.37 mm, 0.046 deg, delta 9e-8 (with the previous statistics, whose
    // cell mean is the cell's first point: 1.5 mm, 0.046 deg, delta 1.5e-4). The rotation error scales with the point noise
    // (0.022 deg at +-1 mm). The limits record this with margin.
    const double delta = run_iterations(points, w, maps, o, 5.0, 30, logger);
    const PoseError e = max_error(w.poses, truth);
    MESSAGE("perfect guess after 30 iterations: position ", e.position * 1000.0, " mm, rotation ", e.rotation / deg, " deg, delta ", delta);
    CHECK(e.position < 0.001);
    CHECK(e.rotation < 0.1 * deg);
    CHECK(delta < 1e-6);
}

TEST_CASE("scan-to-map: a yaw drift is corrected")
{
    const Options o = options_from(test_config());
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    const double bias = 0.5 * deg;
    const std::vector<Eigen::Affine3d> guess = drifting_guess(truth, bias);
    Window w = make_window(guess);
    NullLogger logger;

    run_iterations(points, w, maps, o, 5.0, 30, logger);
    const PoseError before = max_error(guess, truth);
    const PoseError after = max_error(w.poses, truth);
    const double heading_before = end_heading_error(guess, truth);
    const double heading_after = end_heading_error(w.poses, truth);
    MESSAGE("yaw drift: end heading error ", heading_before / deg, " -> ", heading_after / deg, " deg; largest position error ",
            before.position * 1000.0, " -> ", after.position * 1000.0, " mm; largest rotation error ", before.rotation / deg, " -> ",
            after.rotation / deg, " deg");
    // Measured 2026-10-04: 0.245 -> 0.014 deg at the end; the positions (exact in the guess) pick up 0.69 mm (1.8 mm with
    // the previous cell statistics)
    CHECK(std::abs(heading_after) <= 0.1 * std::abs(heading_before));
    CHECK(after.position < 0.0015);
}

TEST_CASE("scan-to-map: the gyro bias is estimated")
{
    OdometryConfig c = test_config();
    c.use_gyro_bias_estimation = true;
    const Options o = options_from(c);
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    const double bias = 0.5 * deg;
    const std::vector<Eigen::Affine3d> guess = drifting_guess(truth, bias);
    Window w = make_window(guess);
    NullLogger logger;

    run_iterations(points, w, maps, o, 5.0, 30, logger);
    const Eigen::Vector3d b = w.gyro_bias.mean / deg;
    const PoseError after = max_error(w.poses, truth);
    MESSAGE("estimated bias ", b.x(), " ", b.y(), " ", b.z(), " deg/s (injected 0 0 0.5); end heading error ",
            end_heading_error(w.poses, truth) / deg, " deg; largest position error ", after.position * 1000.0, " mm");
    // Measured 2026-10-04: 0.038 -0.042 0.473 deg/s (0.045 -0.031 0.469 with the previous cell statistics)
    CHECK(b.z() == doctest::Approx(0.5).epsilon(0.1));
    CHECK(std::abs(b.x()) < 0.05);
    CHECK(std::abs(b.y()) < 0.05);
}

// How many Levenberg-Marquardt iterations the optimizer needs (design decision 8): the two drift scenarios run the 100
// iterations of a real run (nr_iter), and the test reports when the poses stop changing (N_final) and when the update first
// falls below the fast-mode convergence_delta_threshold of 1e-2 (N_fast), with the error against the truth at both.
// Only N_final < 100 is asserted; the numbers are a measurement (see docs/PERFORMANCE.md).
TEST_CASE("scan-to-map: iterations needed")
{
    for (const bool estimate_bias : {false, true})
    {
        OdometryConfig c = test_config();
        c.use_gyro_bias_estimation = estimate_bias;
        const Options o = options_from(c);
        const Scene s = room();
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(drifting_guess(truth, 0.5 * deg));
        NullLogger logger;

        constexpr int iterations = 100;
        std::vector<std::vector<Eigen::Affine3d>> poses_after;
        std::vector<double> deltas;
        const auto start = std::chrono::steady_clock::now();
        run_iterations(points, w, maps, o, 5.0, iterations, logger, nullptr,
                       [&](const IterationResult &r, const Window &window)
                       {
                           poses_after.push_back(window.poses);
                           deltas.push_back(r.delta);
                       });
        const double ms_per_iteration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / iterations;

        // N_final: the number of iterations after which the poses stay within 0.1 mm and 0.001 deg of the result after 100
        const std::vector<Eigen::Affine3d> &final_poses = poses_after.back();
        int n_final = iterations;
        for (int i = iterations - 1; i >= 0; i--)
        {
            const PoseError e = max_error(poses_after[i], final_poses);
            if (e.position > 1e-4 || e.rotation > 0.001 * deg)
                break;
            n_final = i + 1;
        }
        // N_fast: the number of iterations run when delta first falls below 1e-2 (run_lm_iterations stops there)
        int n_fast = 0;
        for (int i = 0; i < iterations; i++)
            if (deltas[i] < 1e-2)
            {
                n_fast = i + 1;
                break;
            }

        const PoseError at_final = max_error(final_poses, truth);
        const PoseError at_fast = n_fast > 0 ? max_error(poses_after[n_fast - 1], truth) : PoseError{};
        MESSAGE(std::string(estimate_bias ? "with" : "without"), " bias estimation: N_final ", n_final, ", N_fast ", n_fast,
                "; error vs truth at N_fast ", at_fast.position * 1000.0, " mm / ", at_fast.rotation / deg, " deg, after ", iterations,
                " ", at_final.position * 1000.0, " mm / ", at_final.rotation / deg, " deg; ", ms_per_iteration, " ms per iteration");
        CHECK(n_final < iterations);
    }
}

namespace
{
    // the translation information the optimizer reports for a window on the true trajectory (one iteration): the sum of
    // n * n^T over the matched surface normals, which optimize_windows uses to detect degenerate windows
    Eigen::Matrix3d translation_information(const Scene &s, const Options &o)
    {
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(truth);
        NullLogger logger;
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        run_iterations(points, w, maps, o, 5.0, 1, logger, &info);
        return info;
    }
}

TEST_CASE("scan-to-map: the translation information is strong in every direction in a closed room")
{
    const Eigen::Vector3d eigenvalues =
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(translation_information(room(), options_from(test_config()))).eigenvalues();
    MESSAGE("room: information eigenvalues ", eigenvalues.x(), " ", eigenvalues.y(), " ", eigenvalues.z());
    // Measured 2026-10-04: 477 807 1226 (677 740 1419 with the previous cell statistics)
    CHECK(eigenvalues.minCoeff() > 0.1 * eigenvalues.maxCoeff());
}

// The previous cell statistics fail this (found 2026-10-04): they fix a cell's normal once, from its first 3 points and the
// initial 3 cm isotropic covariance, so on this corridor 60 % of the indoor cells got a normal more than 30 deg off the
// faces, and the information was 925 along x against 155 across. With the exact statistics and no normal for cells whose
// points lie on one line, it is 1.7 along against 987 across (measured 2026-10-04).
TEST_CASE("scan-to-map: the translation information shows a degenerate corridor")
{
    const Eigen::Matrix3d info = translation_information(corridor(), options_from(test_config()));
    const double along = info(0, 0);
    const double across = std::min(info(1, 1), info(2, 2));
    MESSAGE("corridor: information along x ", along, ", across (smaller of y and z) ", across);
    CHECK(along <= 0.01 * across);
}
