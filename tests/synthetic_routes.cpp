#include "synthetic_routes.h"

#include "fusion_orientation_estimator.h"
#include "lidar_odometry.h"
#include "serial_executor.h"
#include "truth_statistics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace synthetic
{
namespace
{
    constexpr double pi = 3.14159265358979323846;

    void add_box(Scene &s, const Eigen::Vector3d &lo, const Eigen::Vector3d &hi)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            s.faces.push_back({axis, lo[axis], lo, hi});
            s.faces.push_back({axis, hi[axis], lo, hi});
        }
    }

    // a wall piece: the face normal to `axis` at `value`, between lo and hi on the other two axes
    void add_wall(Scene &s, int axis, double value, const Eigen::Vector3d &lo, const Eigen::Vector3d &hi)
    {
        s.faces.push_back({axis, value, lo, hi});
    }

    // a straight leg of the current speed along the current heading until the position's coordinate `axis` reaches `target`
    void leg_to(Motion &m, int axis, double target)
    {
        const Eigen::Vector3d p = m.end_position();
        const double heading = m.end_heading();
        const Eigen::Vector2d direction(std::cos(heading), std::sin(heading));
        const double length = (target - p[axis]) / direction[axis];
        if (length < 0.5 || m.end_speed() <= 0.0)
            throw std::logic_error("synthetic route: a leg that is too short or at rest");
        m.cruise(length / m.end_speed());
    }

    // the time a stop from the current speed takes in distance, by trying it on a copy
    double stopping_distance(const Motion &m, double duration)
    {
        Motion copy = m;
        copy.change_speed(-m.end_speed(), duration);
        return (copy.end_position() - m.end_position()).norm();
    }

    std::int64_t node_ns(const SensorProfile &s, std::int64_t t0_ns, int k) { return t0_ns + static_cast<std::int64_t>(k) * s.dt_ns; }
}

Route route_r1()
{
    Route r{"R1", loop_scene(), Motion(Eigen::Vector3d(-6.0, -4.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
    const double v = 2.0, rate = 1.0, ramp = 0.5;
    const double turn_s = 0.5 * pi / rate + ramp;
    // a quarter turn's displacement in the heading's frame, found on a copy
    Motion probe(Eigen::Vector3d::Zero(), 0.0);
    probe.change_speed(v, 2.0, ramp);
    const Eigen::Vector3d before = probe.end_position();
    probe.turn(0.5 * pi, turn_s, ramp);
    const Eigen::Vector3d d = probe.end_position() - before;
    const double dx = d.x(), dy = d.y();
    (void)dy;

    Motion &m = r.motion;
    m.rest(4.0).change_speed(v, 2.0, ramp);
    // the corridors' centrelines (x = +-8, y = +-4) are where each quarter turn ends
    leg_to(m, 0, 8.0 + grid_offset.x() - dx);
    m.turn(0.5 * pi, turn_s, ramp);
    leg_to(m, 1, 4.0 + grid_offset.y() - dx);
    m.turn(0.5 * pi, turn_s, ramp);
    leg_to(m, 0, -8.0 + grid_offset.x() + dx);
    m.turn(0.5 * pi, turn_s, ramp);
    leg_to(m, 1, -4.0 + grid_offset.y() + dx);
    m.turn(0.5 * pi, turn_s, ramp);
    leg_to(m, 0, -2.0 + grid_offset.x() - stopping_distance(m, 2.0));
    m.change_speed(-v, 2.0, ramp).rest(1.5);
    return r;
}

Route route_r2()
{
    // x: room A [-6, 0], corridor [0, 20], room B [20, 26]; y: corridor [-1, 1], rooms [-3, 3]; z: [0, 2.5]
    Scene s;
    const double h = 2.5;
    // the plain shell: floors and ceilings, the corridor's walls, the rooms' walls
    for (const double z : {0.0, h})
    {
        add_wall(s, 2, z, {-6.0, -3.0, 0.0}, {0.0, 3.0, 0.0});
        add_wall(s, 2, z, {0.0, -1.0, 0.0}, {20.0, 1.0, 0.0});
        add_wall(s, 2, z, {20.0, -3.0, 0.0}, {26.0, 3.0, 0.0});
    }
    for (const double y : {-1.0, 1.0})
        add_wall(s, 1, y, {0.0, 0.0, 0.0}, {20.0, 0.0, h});
    for (const double y : {-3.0, 3.0})
    {
        add_wall(s, 1, y, {-6.0, 0.0, 0.0}, {0.0, 0.0, h});
        add_wall(s, 1, y, {20.0, 0.0, 0.0}, {26.0, 0.0, h});
    }
    add_wall(s, 0, -6.0, {0.0, -3.0, 0.0}, {0.0, 3.0, h});
    add_wall(s, 0, 26.0, {0.0, -3.0, 0.0}, {0.0, 3.0, h});
    for (const double x : {0.0, 20.0})
    {
        add_wall(s, 0, x, {0.0, -3.0, 0.0}, {0.0, -1.0, h});
        add_wall(s, 0, x, {0.0, 1.0, 0.0}, {0.0, 3.0, h});
    }
    const int shell = static_cast<int>(s.faces.size());
    // structure in the rooms: pillars and boxes at irregular places
    add_box(s, {-4.6, -2.3, 0.0}, {-4.0, -1.7, 1.2});
    add_box(s, {-2.0, 1.8, 0.0}, {-1.7, 2.1, h});
    add_box(s, {-5.4, 1.0, 0.0}, {-4.8, 1.9, 0.8});
    add_box(s, {21.0, 0.8, 0.0}, {21.6, 1.5, 1.0});
    add_box(s, {23.5, -2.4, 0.0}, {23.8, -2.1, h});
    add_box(s, {24.5, 1.5, 0.0}, {25.3, 2.3, 1.5});
    s.centre = Eigen::Vector3d(10.0, 0.0, 1.2);
    s = shifted(s, grid_offset);

    Route r{"R2", s, Motion(Eigen::Vector3d(-3.0, 0.0, 1.2) + grid_offset, 0.0), shell, 6.0 + grid_offset.x(), 14.0 + grid_offset.x()};
    const double v = 1.5;
    Motion &m = r.motion;
    m.rest(4.0).change_speed(v, 2.5);
    leg_to(m, 0, 23.0 + grid_offset.x() - stopping_distance(m, 2.5));
    m.change_speed(-v, 2.5).rest(0.5).turn(pi, 3.0).rest(0.5).change_speed(v, 2.5);
    leg_to(m, 0, -1.0 + grid_offset.x() + stopping_distance(m, 2.5));
    m.change_speed(-v, 2.5).rest(1.5);
    return r;
}

Route route_r3()
{
    Route r{"R3", loop_scene(), Motion(Eigen::Vector3d(-6.0, -4.0, 1.5) + grid_offset, 0.0), 0, 0.0, 0.0};
    const double v = 1.2;
    r.motion.rest(4.0).change_speed(v, 2.0, 0.5, true).cruise(6.0, true).change_speed(-v, 2.0, 0.5, true).rest(2.0);
    return r;
}

bool cast_ray(const Scene &scene, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction, double max_range, double &distance, int &face)
{
    double best = max_range;
    int best_face = -1;
    constexpr double eps = 1e-9;
    for (int i = 0; i < static_cast<int>(scene.faces.size()); i++)
    {
        const Face &f = scene.faces[i];
        const double d = direction[f.axis];
        if (std::abs(d) < 1e-12)
            continue;
        const double t = (f.value - origin[f.axis]) / d;
        if (t <= eps || t >= best)
            continue;
        const Eigen::Vector3d p = origin + t * direction;
        const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
        if (p[a] < f.lo[a] - eps || p[a] > f.hi[a] + eps || p[b] < f.lo[b] - eps || p[b] > f.hi[b] + eps)
            continue;
        best = t;
        best_face = i;
    }
    if (best_face < 0)
        return false;
    distance = best;
    face = best_face;
    return true;
}

InertialRecording generate_recording(const Route &route, const SensorProfile &sensor, const ImuNoise &noise)
{
    const Motion &motion = route.motion;
    const double dt = sensor.dt_ns * 1e-9;
    ImuOptions io;
    io.dt_ns = sensor.dt_ns;
    io.nodes_per_window = sensor.nodes_per_window;
    io.noise = noise;
    io.seed = sensor.seed;
    const int n_max = static_cast<int>(std::floor(motion.duration() / dt + 1e-9));

    truth_stats::Rng rng(sensor.seed * 0x9E3779B97F4A7C15ULL + 7);
    const double rest_until = motion.first_motion_time();
    std::vector<Hit> hits;
    int accepted = 0, origin = -1;
    for (int k = 0; k < n_max; k++)
    {
        const MotionState st = motion.state(k * dt);
        const int count = k * dt < rest_until ? sensor.prefix_points_per_node : sensor.points_per_node;
        const double stamp = std::nextafter(static_cast<double>(node_ns(sensor, io.t0_ns, k)) / 1e9, std::numeric_limits<double>::infinity());
        const Eigen::Vector3d sensor_origin = st.position + st.rotation * sensor.lever_arm;
        for (int j = 0; j < count; j++)
        {
            const double z = 2.0 * rng.uniform() - 1.0, phi = 2.0 * pi * rng.uniform(), r = std::sqrt(1.0 - z * z);
            const double jitter = sensor.jitter_sigma_m * rng.normal();
            const Eigen::Vector3d world_direction = st.rotation * Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
            double distance = 0.0;
            int face = -1;
            if (!cast_ray(route.scene, sensor_origin, world_direction, sensor.max_range_m, distance, face) || distance < sensor.min_range_m)
                continue;
            Hit h;
            h.node = k;
            h.face = face;
            h.latent_world = sensor_origin + distance * world_direction;
            Eigen::Vector3d emitted = h.latent_world;
            emitted[route.scene.faces[face].axis] += jitter;
            h.raw = st.rotation.transpose() * (emitted - sensor_origin);
            h.timestamp = stamp;
            const double xy = std::hypot(h.raw.x(), h.raw.y());
            h.passes_filter = xy > sensor.filter_xy_inner && xy < sensor.filter_xy_outer;
            hits.push_back(h);
            if (h.passes_filter && ++accepted == sensor.seed_points + 1)
                origin = k + 1; // the seed's last point is in node k; the first node at or after its stamp is k + 1
        }
    }
    if (origin < 0)
        throw std::runtime_error("generate_recording: the seed (threshold_initial_points + 1 accepted points) is never complete");
    const int n_windows = (n_max - origin - 1) / sensor.nodes_per_window;
    if (n_windows < 1)
        throw std::runtime_error("generate_recording: no full window after the seed");
    const int end_node = origin + n_windows * sensor.nodes_per_window;
    io.window_origin_node = origin;
    io.n_samples = end_node + 1; // one IMU sample beyond the last window
    while (!hits.empty() && hits.back().node >= io.n_samples)
        hits.pop_back();

    InertialRecording rec(route, sensor, generate_imu(motion, io));
    rec.hits = std::move(hits);
    rec.window_origin_node = origin;
    rec.n_windows = n_windows;
    rec.end_node = end_node;
    return rec;
}

RawRecording InertialRecording::raw() const
{
    RawRecording r;
    r.t_ns = imu.t_ns;
    r.gyro_rad_s = imu.gyro_rad_s;
    r.acc_g = imu.acc_g;
    r.points.reserve(hits.size());
    for (size_t i = 0; i < hits.size(); i++)
        r.points.push_back({hits[i].raw, hits[i].timestamp, static_cast<int>(i)});
    return r;
}

SyntheticSource::SyntheticSource(RawRecording raw, int chunks) : raw_(std::move(raw))
{
    const std::size_t n = raw_.t_ns.size();
    std::vector<double> start_time;
    for (int c = 0; c < chunks; c++)
    {
        Chunk ch;
        ch.imu_begin = n * c / chunks;
        ch.imu_end = n * (c + 1) / chunks;
        chunks_.push_back(ch);
        start_time.push_back(static_cast<double>(raw_.t_ns[ch.imu_begin]) / 1e9);
    }
    for (const RawPoint &p : raw_.points)
    {
        const auto it = std::upper_bound(start_time.begin(), start_time.end(), p.timestamp);
        const std::size_t c = it == start_time.begin() ? 0 : static_cast<std::size_t>(it - start_time.begin()) - 1;
        chunks_[c].points.push_back(p);
    }
}

Imu SyntheticSource::read_imu(std::size_t chunk, const ImuCalibration &calibration, Logger &)
{
    Imu imu;
    const Chunk &c = chunks_.at(chunk);
    for (std::size_t i = c.imu_begin; i < c.imu_end; i++)
    {
        const double timestamp = static_cast<double>(raw_.t_ns[i]);
        Eigen::Vector3f gyr, acc;
        for (int a = 0; a < 3; a++)
        {
            gyr[a] = calibration.gyro(a, raw_.gyro_rad_s[i][a]);
            acc[a] = static_cast<float>(raw_.acc_g[i][a]);
        }
        imu.push_back({std::pair(timestamp / 1e9, (timestamp + 1.7e18) / 1e9), gyr, acc});
    }
    return imu;
}

std::vector<Point3Di> SyntheticSource::read_points(std::size_t chunk, const PointFilter &filter, Logger &)
{
    std::vector<Point3Di> points;
    for (const RawPoint &rp : chunks_.at(chunk).points)
    {
        Point3Di p{};
        Eigen::Affine3d calibration = Eigen::Affine3d::Identity();
        calibration.translation() = filter.lever_arm;
        const Eigen::Vector3d pf = rp.raw;
        p.point = calibration * pf;
        p.lidarid = 0;
        p.timestamp = rp.timestamp;
        p.intensity = 0.0f;
        p.index_pose = -1;
        p.index_point = rp.tag;
        if (p.timestamp == 0 && filter.omit_zero_timestamp)
            continue;
        if (std::sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter.xy_inner && std::sqrt(pf.x() * pf.x() + pf.y() * pf.y()) < filter.xy_outer)
            points.emplace_back(p);
    }
    return points;
}

OdometryConfig front_end_config(const SensorProfile &sensor)
{
    OdometryConfig c;
    c.threshold_initial_points = sensor.seed_points;
    c.threshold_nr_poses = sensor.nodes_per_window;
    c.decimation = 0.03;
    c.filter_threshold_xy_inner = sensor.filter_xy_inner;
    c.filter_threshold_xy_outer = sensor.filter_xy_outer;
    c.lidar_imu_lever_arm_x_m = sensor.lever_arm.x();
    c.lidar_imu_lever_arm_y_m = sensor.lever_arm.y();
    c.lidar_imu_lever_arm_z_m = sensor.lever_arm.z();
    c.ahrs_gain = 0.05;
    c.use_remove_imu_bias_from_first_stationary_scan = true;
    c.startup_gyro_bias_nr_samples = 500;
    c.startup_gyro_bias_max_variance_deg2_s2 = 5.0;
    return c;
}

namespace
{
    struct NullPreview : PreviewSink
    {
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &, const std::vector<double> &) override {}
        std::string location() const override { return "none"; }
    };

    struct SilentProgress : ProgressReporter
    {
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };
}

void cut_into_windows(Cut &cut, const InertialRecording &recording, int chunks)
{
    cut.config = front_end_config(recording.sensor);
    SyntheticSource source(recording.raw(), chunks);
    NullPreview preview;
    SerialExecutor serial;
    SilentProgress progress;
    FusionOrientationEstimator estimator;
    if (!load_data(source, preview, cut.config, cut.points_per_file, cut.imu, false, serial, cut.logger))
        return;
    calculate_trajectory(cut.trajectory, cut.imu, cut.config.fusionConventionNwu, cut.config.fusionConventionEnu, cut.config.fusionConventionNed,
                         cut.config.ahrs_gain, false, cut.config.use_remove_imu_bias_from_first_stationary_scan, cut.config.startup_gyro_bias_nr_samples,
                         cut.config.startup_gyro_bias_max_variance_deg2_s2, estimator, cut.logger);
    cut.ok = compute_step_1(cut.points_per_file, cut.config, cut.state, cut.trajectory, cut.windows, cut.store, cut.logger, progress, &cut.imu);
}
}
