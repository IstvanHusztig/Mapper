#include <doctest/doctest.h>

#include "lidar_odometry.h"
#include "ndt_map.h"
#include "synthetic_routes.h"

#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <unordered_set>

// The routes R1, R2 and R3, their points, the recording source and the cut into windows by the real front end
// (synthetic_routes.h), tested before any fixture relies on them: every eligible generated point is in its window's
// original point set at the node it was generated for, the decimated sets cover every node, each window has more than
// 1 000 assigned points, the first window is at rest, the motion starts after the orientation filter's ramp, and the
// windows and their timestamps are the expected ones.

namespace
{
    using namespace synthetic;

    struct Fixture
    {
        InertialRecording rec;
        Cut cut;
    };

    std::unique_ptr<Fixture> make(const Route &route, const SensorProfile &sensor)
    {
        auto f = std::make_unique<Fixture>(Fixture{generate_recording(route, sensor), Cut{}});
        cut_into_windows(f->cut, f->rec);
        return f;
    }

    const Fixture &r1()
    {
        static const std::unique_ptr<Fixture> f = make(route_r1(), SensorProfile{});
        return *f;
    }
    const Fixture &r2()
    {
        static const std::unique_ptr<Fixture> f = []
        {
            SensorProfile s;
            s.max_range_m = 6.0;
            return make(route_r2(), s);
        }();
        return *f;
    }
    const Fixture &r3()
    {
        static const std::unique_ptr<Fixture> f = make(route_r3(), SensorProfile{});
        return *f;
    }

    // the windows' original points, read from the store
    std::vector<Point3Di> original(const Fixture &f, size_t w)
    {
        std::vector<Point3Di> p;
        REQUIRE(const_cast<MemoryPointStore &>(f.cut.store).load(PointSet::WindowOriginal, w, p));
        return p;
    }
    std::vector<Point3Di> intermediate(const Fixture &f, size_t w)
    {
        std::vector<Point3Di> p;
        REQUIRE(const_cast<MemoryPointStore &>(f.cut.store).load(PointSet::WindowIntermediate, w, p));
        return p;
    }

    void check_cut(const Fixture &f)
    {
        const InertialRecording &rec = f.rec;
        const int T = rec.sensor.nodes_per_window;
        REQUIRE(f.cut.ok);
        // the window count and timestamps
        REQUIRE(static_cast<int>(f.cut.windows.size()) == rec.n_windows);
        CHECK(rec.window_origin_node > 1000);
        for (int w = 0; w < rec.n_windows; w++)
        {
            REQUIRE(static_cast<int>(f.cut.windows[w].timestamps.size()) == T);
            for (int i = 0; i < T; i++)
                REQUIRE(f.cut.windows[w].timestamps[i].first == rec.imu.t_s[rec.window_origin_node + w * T + i]);
        }
        CHECK(rec.imu.n_nodes() == rec.end_node + 1); // one IMU sample beyond the last window
        // the first window at rest, the motion after the filter's 3 s ramp
        const double t_begin = rec.imu.node_time(rec.window_origin_node), t_end0 = rec.imu.node_time(rec.window_origin_node + T);
        CHECK(rec.route.motion.at_rest(0.0, t_end0));
        CHECK(t_begin >= 3.0);
        CHECK(rec.route.motion.first_motion_time() >= 3.0);
        CHECK(rec.route.motion.first_motion_time() >= t_end0);

        // every eligible tagged hit is in its window's original set at its node; nothing else is
        std::map<int, std::pair<int, int>> found; // tag -> window, index_pose
        size_t min_points = ~size_t(0), boundary_first = 0, boundary_last = 0, total = 0;
        for (int w = 0; w < rec.n_windows; w++)
        {
            const std::vector<Point3Di> pts = original(f, w);
            min_points = std::min(min_points, pts.size());
            total += pts.size();
            for (const Point3Di &p : pts)
            {
                REQUIRE(found.emplace(p.index_point, std::make_pair(w, p.index_pose)).second); // each tag once
                CHECK(p.index_pose >= 0);
                CHECK(p.index_pose < T);
            }
        }
        size_t eligible = 0, wrong = 0, missing = 0;
        for (size_t tag = 0; tag < rec.hits.size(); tag++)
        {
            const Hit &h = rec.hits[tag];
            const bool eligible_hit = h.passes_filter && h.node >= rec.window_origin_node && h.node < rec.end_node;
            const auto it = found.find(static_cast<int>(tag));
            if (!eligible_hit)
            {
                CHECK(it == found.end()); // a filtered point, a seed-prefix point or one beyond the last window is nowhere
                continue;
            }
            eligible++;
            if (it == found.end())
            {
                missing++;
                continue;
            }
            const int local = (h.node - rec.window_origin_node) % T;
            if (it->second.first != (h.node - rec.window_origin_node) / T || it->second.second != local)
                wrong++;
            boundary_first += local == 0;
            boundary_last += local == T - 1;
        }
        MESSAGE(rec.route.name << ": " << rec.n_windows << " windows of " << T << ", origin node " << rec.window_origin_node << ", " << eligible << " eligible hits (" << missing
                               << " missing, " << wrong << " in the wrong place), points per window min " << min_points << " mean " << total / rec.n_windows
                               << ", at window boundaries " << boundary_first << " (first node) " << boundary_last << " (last node)");
        CHECK(missing == 0);
        CHECK(wrong == 0);
        CHECK(eligible == total);
        CHECK(boundary_first > 0);
        CHECK(boundary_last > 0);
        CHECK(min_points > 1000);

        // the decimated sets: one representative per 3 cm cell, every node covered
        size_t worst_cover = ~size_t(0), kept = 0, before = 0;
        for (int w = 0; w < rec.n_windows; w++)
        {
            const std::vector<Point3Di> pts = original(f, w), dec = intermediate(f, w);
            std::unordered_set<unsigned long long> cells;
            for (const Point3Di &p : pts)
                cells.insert(ndt_cell_index(p.point, Eigen::Vector3d(0.03, 0.03, 0.03)));
            CHECK(dec.size() == cells.size());
            std::set<int> covered;
            for (const Point3Di &p : dec)
            {
                covered.insert(p.index_pose);
                CHECK(found.at(p.index_point).first == w); // a representative is one of the window's own points
            }
            worst_cover = std::min(worst_cover, covered.size());
            kept += dec.size();
            before += pts.size();
        }
        MESSAGE(rec.route.name << ": decimation keeps " << kept << " of " << before << " points, fewest nodes covered in a window " << worst_cover << " of " << T);
        CHECK(worst_cover == static_cast<size_t>(T));
    }
}

TEST_CASE("synthetic routes: R1, the loop, is cut as generated")
{
    check_cut(r1());
}

TEST_CASE("synthetic routes: R2, out and back through a corridor, is cut as generated")
{
    check_cut(r2());
}

TEST_CASE("synthetic routes: R3, rest and walk and rest, is cut as generated")
{
    check_cut(r3());
}

TEST_CASE("synthetic routes: R2 refuses a sensor range other than 6 m")
{
    // the refusal comes before any ray is cast: another range is rejected at once, with the reason in the message
    for (const double range : {5.9, 8.0, 100.0})
    {
        SensorProfile s;
        s.max_range_m = range;
        CHECK_THROWS_WITH_AS(generate_recording(route_r2(), s), doctest::Contains("route R2 needs SensorProfile::max_range_m = 6"), std::invalid_argument);
    }
}

TEST_CASE("synthetic routes: R2 has a blind middle and structure at both ends")
{
    const Fixture &f = r2();
    const Route &route = f.rec.route;
    size_t blind_nodes = 0, blind_structure = 0, near_structure = 0, far_structure = 0, blind_hits = 0;
    std::set<int> blind_node_set;
    for (const Hit &h : f.rec.hits)
    {
        const double x = f.rec.route.motion.state(f.rec.imu.node_time(h.node)).position.x();
        const bool structure = h.face >= route.n_shell_faces;
        if (x >= route.blind_x_lo && x <= route.blind_x_hi)
        {
            blind_hits++;
            blind_structure += structure;
            blind_node_set.insert(h.node);
        }
        else if (x < route.blind_x_lo)
            near_structure += structure;
        else
            far_structure += structure;
    }
    blind_nodes = blind_node_set.size();
    MESSAGE("blind stretch x in [" << route.blind_x_lo << ", " << route.blind_x_hi << "]: " << blind_nodes << " nodes, " << blind_hits << " hits, " << blind_structure << " on structure; structure hits before it "
                                   << near_structure << ", after it " << far_structure);
    CHECK(blind_nodes > 1000);
    CHECK(blind_structure == 0);
    CHECK(near_structure > 1000);
    CHECK(far_structure > 1000);
    // the route goes out and comes back: x rises past the blind stretch and falls again
    double x_max = -1e9;
    for (int k = 0; k < f.rec.imu.n_nodes(); k += 50)
        x_max = std::max(x_max, route.motion.state(f.rec.imu.node_time(k)).position.x());
    CHECK(x_max > 22.0);
    CHECK(route.motion.end_position().x() < 0.0);
    // the emitted range is limited explicitly: no latent hit is farther than max_range from its sensor
    for (const Hit &h : f.rec.hits)
        CHECK(h.raw.norm() <= f.rec.sensor.max_range_m + 6.0 * f.rec.sensor.jitter_sigma_m); // the limit is on the latent hit, the jitter moves it along a normal
}

TEST_CASE("synthetic routes: the points are the latent hits plus the specified jitter, stamped inside their node's interval")
{
    const Fixture &f = r1();
    const InertialRecording &rec = f.rec;
    double sum = 0, sum2 = 0, off_axis = 0;
    size_t n = 0;
    bool stamps_ok = true;
    for (size_t i = 0; i < rec.hits.size(); i += 7)
    {
        const Hit &h = rec.hits[i];
        const MotionState s = rec.route.motion.state(rec.imu.node_time(h.node));
        const Eigen::Vector3d emitted = s.position + s.rotation * (h.raw + rec.sensor.lever_arm);
        const Eigen::Vector3d d = emitted - h.latent_world;
        const int axis = rec.route.scene.faces[h.face].axis;
        for (int a = 0; a < 3; a++)
            if (a != axis)
                off_axis = std::max(off_axis, std::abs(d[a]));
        sum += d[axis];
        sum2 += d[axis] * d[axis];
        n++;
        const double t_node = rec.imu.t_s[h.node];
        stamps_ok = stamps_ok && h.timestamp == std::nextafter(t_node, 1e300) && h.timestamp > t_node && (h.node + 1 >= rec.imu.n_nodes() || h.timestamp < rec.imu.t_s[h.node + 1]);
        const double range = h.raw.norm();
        stamps_ok = stamps_ok && range >= rec.sensor.min_range_m - 1e-9 && range <= rec.sensor.max_range_m + 1e-9 + 0.05;
    }
    const double mean = sum / n, sigma = std::sqrt(sum2 / n - mean * mean);
    MESSAGE(n << " hits: jitter sigma " << sigma << " (specified " << rec.sensor.jitter_sigma_m << "), mean " << mean << ", off-axis " << off_axis);
    CHECK(off_axis < 1e-9);
    CHECK(sigma == doctest::Approx(rec.sensor.jitter_sigma_m).epsilon(4.0 / std::sqrt(2.0 * n)));
    CHECK(std::abs(mean) < 4.5 * rec.sensor.jitter_sigma_m / std::sqrt(static_cast<double>(n)));
    CHECK(stamps_ok);
    // a latent hit lies on its face
    for (size_t i = 0; i < rec.hits.size(); i += 101)
    {
        const Face &face = rec.route.scene.faces[rec.hits[i].face];
        CHECK(std::abs(rec.hits[i].latent_world[face.axis] - face.value) < 1e-9);
    }
    // the same seed gives the same hits
    const InertialRecording again = generate_recording(route_r3(), SensorProfile{});
    const InertialRecording third = generate_recording(route_r3(), SensorProfile{});
    REQUIRE(again.hits.size() == third.hits.size());
    CHECK(again.hits.back().raw == third.hits.back().raw);
    CHECK(again.imu.gyro_rad_s == third.imu.gyro_rad_s);
}

TEST_CASE("synthetic source: does what the production source does")
{
    RawRecording raw;
    for (int k = 0; k < 6; k++)
    {
        raw.t_ns.push_back(1'000'000'000 + k * 2'000'000);
        raw.gyro_rad_s.emplace_back(0.1 * (k + 1), 0.2, -0.3);
        raw.acc_g.emplace_back(0.01, 0.02, 1.0 + 1e-9 * k);
    }
    // inside the inner radius (xy 0.30) but far in 3D; just outside it (0.36); outside the outer radius (41); zero time;
    // an ordinary one
    raw.points.push_back({{0.30, 0.0, 5.0}, 1.001, 0});
    raw.points.push_back({{0.36, 0.0, 1.0}, 1.002, 1});
    raw.points.push_back({{41.0, 0.0, 0.0}, 1.003, 2});
    raw.points.push_back({{3.0, 4.0, 1.0}, 0.0, 3});
    raw.points.push_back({{3.0, 4.0, 1.0}, 1.004, 4});
    raw.points.push_back({{0.20, 0.20, 1.0}, 1.005, 5}); // xy 0.283: filtered
    raw.points.push_back({{0.26, 0.26, 1.0}, 1.0055, 6}); // xy 0.368: kept
    SyntheticSource source(raw, 1);
    synthetic::NullLogger logger;

    ImuCalibration calibration;
    calibration.gyro_scale = Eigen::Vector3d(1.01, 1.0, 0.99);
    const Imu imu = source.read_imu(0, calibration, logger);
    REQUIRE(imu.size() == 6);
    CHECK(imu[2].timestamps.first == static_cast<double>(1'004'000'000) / 1e9);
    CHECK(imu[2].gyro.x() == static_cast<float>(raw.gyro_rad_s[2].x() * 1.01)); // scaled as a double, narrowed once
    CHECK(imu[2].gyro.y() == 0.2f);                            // an axis with factor 1 is not touched
    CHECK(imu[2].gyro.z() == static_cast<float>(-0.3 * 0.99));
    CHECK(imu[5].acc.z() == static_cast<float>(1.0 + 5e-9));

    PointFilter filter;
    filter.omit_zero_timestamp = true;
    filter.xy_inner = 0.35;
    filter.xy_outer = 40.0;
    filter.lever_arm = Eigen::Vector3d(0.1, -0.2, 0.3);
    const std::vector<Point3Di> pts = source.read_points(0, filter, logger);
    // kept: tags 1, 4 and 6; the XY filter acts on the raw point, the lever arm is added after it
    REQUIRE(pts.size() == 3);
    CHECK(pts[0].index_point == 1);
    CHECK(pts[1].index_point == 4);
    CHECK(pts[2].index_point == 6);
    CHECK((pts[0].point - Eigen::Vector3d(0.46, -0.2, 1.3)).norm() < 1e-15);
    CHECK(pts[1].timestamp == 1.004);
    filter.omit_zero_timestamp = false;
    CHECK(source.read_points(0, filter, logger).size() == 4); // the zero-time point is kept only when asked

    // chunks split the rows and the points by time and lose nothing
    SyntheticSource three(raw, 3);
    REQUIRE(three.chunk_count() == 3);
    size_t rows = 0, kept = 0;
    filter.omit_zero_timestamp = true;
    for (size_t c = 0; c < 3; c++)
    {
        rows += three.read_imu(c, ImuCalibration{}, logger).size();
        kept += three.read_points(c, filter, logger).size();
    }
    CHECK(rows == 6);
    CHECK(kept == 3);
}

TEST_CASE("synthetic routes: a cut in several chunks gives the same windows")
{
    const Fixture &f = r3();
    Cut again;
    cut_into_windows(again, f.rec, 3);
    REQUIRE(again.ok);
    REQUIRE(again.windows.size() == f.cut.windows.size());
    for (size_t w = 0; w < again.windows.size(); w += 7)
    {
        CHECK(again.windows[w].timestamps == f.cut.windows[w].timestamps);
        CHECK(again.windows[w].poses[0].matrix() == f.cut.windows[w].poses[0].matrix());
    }
}
