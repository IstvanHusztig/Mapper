#pragma once

// Scenes, routes, points and the recording source of the synthetic recordings with inertial data (openspec change
// consistency-harness, design D3 and D8, task 4.2), and the cut into windows by the real front end (`load_data`,
// `calculate_trajectory`, `compute_step_1`). Test helpers only.
//
// What the generator draws (D3): points are the hits of rays from the sensor at node k's true pose on the scene,
// stamped strictly inside node k's interval (the next representable time after the node, so that the front end's rule
// "a point belongs to the last node before its timestamp" gives node k); a stationary prefix long enough for the map
// seed (the first threshold_initial_points + 1 accepted points) and for the orientation filter's 3 s ramp; and the
// IMU samples of synthetic_inertial.h up to one sample beyond the last full window.

#include "memory_point_store.h"
#include "odometry_config.h"
#include "odometry_state.h"
#include "input_ports.h"
#include "synthetic_inertial.h"
#include "synthetic_recording.h"
#include "window.h"

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <vector>

namespace synthetic
{
// the sensor model; the timing is the Livox's (500 Hz IMU, 180-node windows), the points are far fewer than the real
// sensor's, and the emitted range is limited explicitly
struct SensorProfile
{
    std::int64_t dt_ns = 2'000'000;
    int nodes_per_window = 180;
    double min_range_m = 0.5;
    double max_range_m = 8.0;
    int points_per_node = 10;        // while the body moves
    int prefix_points_per_node = 40; // before the first motion (the seed prefix)
    double jitter_sigma_m = 0.003;   // along the face normal
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero(); // the IMU's position in the point frame (lidar_imu_lever_arm_*)
    double filter_xy_inner = 0.35;   // the production source's raw XY filter, [m]
    double filter_xy_outer = 40.0;
    int seed_points = 60000;         // threshold_initial_points
    std::uint64_t seed = 1;
};

// a scene and the motion through it
struct Route
{
    std::string name;
    Scene scene;
    Motion motion;
    int n_shell_faces = 0;       // the faces before this index are the plain shell (R2: corridor and room walls)
    double blind_x_lo = 0.0, blind_x_hi = 0.0; // R2: where no structure should be in range
};

// R1: the loop scene (seen everywhere), one lap at 2 m/s through its four corridors, with a 4 s rest before and 1.5 s after
Route route_r1();
// R2: out and back through a plain corridor (20 m, 2 m wide) between two structured rooms, 1.5 m/s, a 180 degree turn in
// place in the far room; max_range 6 m makes its middle blind. Use SensorProfile::max_range_m = 6.
Route route_r2();
// R3: rest, walk (the gait of Motion), rest, in the loop scene's south corridor
Route route_r3();

struct Hit
{
    int node = 0;                 // the node whose pose it was generated at
    int face = -1;                // the scene face it lies on
    Eigen::Vector3d latent_world; // the true surface point
    Eigen::Vector3d raw;          // as emitted, in the sensor frame, with the jitter
    double timestamp = 0.0;
    bool passes_filter = true;    // inner < |raw xy| < outer: the production source keeps it
};

// what a recording file holds, before any calibration or filtering
struct RawPoint
{
    Eigen::Vector3d raw;
    double timestamp;
    int tag; // the generator's hit index, kept in Point3Di::index_point (the production source leaves it unset)
};

struct RawRecording
{
    std::vector<std::int64_t> t_ns;
    std::vector<Eigen::Vector3d> gyro_rad_s;
    std::vector<Eigen::Vector3d> acc_g;
    std::vector<RawPoint> points;
};

struct InertialRecording
{
    Route route;
    SensorProfile sensor;
    ImuRecording imu;
    std::vector<Hit> hits; // in generation order, hit i has tag i
    // what the front end should make of it (predicted by the generator, verified against the cut by the tests)
    int window_origin_node = 0;    // the first node of window 0
    int n_windows = 0;             // the windows of nodes [origin + w T, origin + (w + 1) T)
    int end_node = 0;              // origin + n_windows T: the nodes from here on are beyond the last window

    InertialRecording(Route r, SensorProfile s, ImuRecording i) : route(std::move(r)), sensor(std::move(s)), imu(std::move(i)) {}
    RawRecording raw() const;
    double node_time(int k) const { return imu.t_s[k]; }
};

// The recording of a route. The hits come first (from the truth poses), then the window origin follows from the seed,
// then the IMU samples are generated for origin + n_windows T + 1 nodes. noise = ImuNoise{} is the model world;
// ImuNoise::none() the noise-free samples.
InertialRecording generate_recording(const Route &route, const SensorProfile &sensor = SensorProfile{}, const ImuNoise &noise = ImuNoise{});

// the nearest hit of a ray on the scene's faces (within max_range); false if there is none
bool cast_ray(const Scene &scene, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction, double max_range, double &distance, int &face);

// A RecordingSource over a RawRecording that does what the production source does (recording_folder_source.cpp): the
// gyro calibration narrowed to float, the accelerometer narrowed to float, timestamps in seconds as ns / 1e9, zero-time
// rejection, the XY filter on the RAW point, then the lever-arm translation. The chunks split the rows and the points
// by time.
class SyntheticSource : public RecordingSource
{
public:
    explicit SyntheticSource(RawRecording raw, int chunks = 1);
    bool open(Logger &) override { return true; }
    std::size_t chunk_count() const override { return chunks_.size(); }
    Imu read_imu(std::size_t chunk, const ImuCalibration &calibration, Logger &logger) override;
    std::vector<Point3Di> read_points(std::size_t chunk, const PointFilter &filter, Logger &logger) override;

private:
    struct Chunk
    {
        std::size_t imu_begin, imu_end;
        std::vector<RawPoint> points;
    };
    RawRecording raw_;
    std::vector<Chunk> chunks_;
};

// the front end's settings of the Livox sensor profile (settings/livox-vertical.toml where it matters to the cut)
OdometryConfig front_end_config(const SensorProfile &sensor = SensorProfile{});

// A recording cut by the real stages. The memory point store keeps what compute_step_1 saved.
struct Cut
{
    OdometryConfig config;
    Imu imu;
    ImuTrajectory trajectory;
    std::vector<std::vector<Point3Di>> points_per_file;
    std::vector<Window> windows;
    MemoryPointStore store;
    OdometryState state;
    NullLogger logger;
    bool ok = false;
};

// load_data (a serial executor) -> calculate_trajectory (the Fusion estimator) -> compute_step_1
void cut_into_windows(Cut &cut, const InertialRecording &recording, int chunks = 1);
// the same with other settings (their front end fields must be front_end_config's for the cut to be the generator's)
void cut_into_windows(Cut &cut, const InertialRecording &recording, const OdometryConfig &config, int chunks = 1);
}
