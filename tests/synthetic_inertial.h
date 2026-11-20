#pragma once

// Motion, truth and IMU samples of the synthetic recordings (openspec change consistency-harness, design D3, task 4.1).
// Test helpers only, no production code and none of the estimator's code: an independent strap-down integration in
// synthetic_inertial_test.cpp checks the samples against the truth.
//
// World frame: north-west-up as the orientation filter starts it (z up, yaw 0 at the first sample, no magnetometer).
// Body frame: x forward, y left, z up (the rotation maps body to world). Gravity g0 = 9.80665 m/s2 along -z; the
// accelerometer reads the specific force, +1 g up at rest.
//
// Time conventions of the estimator, generated as the estimator uses them (design D3):
//  - gyro sample k is the mean body rate over the interval ENDING at node k, (t[k-1], t[k]]; sample 0 covers the first
//    update's 5 ms step, (t[0] - 5 ms, t[0]];
//  - accelerometer sample k is the mean body-frame specific force over the interval STARTING at node k, [t[k], t[k+1]);
//  - a gyro bias and an acceleration offset are constant inside a window (a block of nodes_per_window nodes starting at
//    window_origin_node) and step from window to window by their random walks.

#include <Eigen/Dense>

#include <array>
#include <cstdint>
#include <vector>

namespace synthetic
{
constexpr double standard_gravity = 9.80665; // [m/s2]

// ---- motion ----

// What the model-world walking looks like: about 0.5 / 0.5 / 0.9 m/s2 rms per body axis, 11 / 11 / 32 deg/s rms, 2 Hz
// steps (the measured Livox 3 dynamics). Roll and sway go at half the step rate, pitch, surge and the vertical bob at
// the step rate, yaw twist at half. The sway and surge amplitudes are chosen so that, with the gravity coupling of the
// roll and pitch (g * angle), the total body-axis acceleration has the stated rms (the roll or pitch and the sway or
// surge are a quarter period apart, so their squares add).
struct Gait
{
    double step_hz = 2.0;
    Eigen::Vector3d accel_rms_mps2{0.5, 0.5, 0.9};
    Eigen::Vector3d rate_rms_deg_s{11.0, 11.0, 32.0};
};

struct MotionState
{
    Eigen::Vector3d position;        // [m]
    Eigen::Vector3d velocity;        // [m/s]
    Eigen::Vector3d acceleration;    // [m/s2], gravity not included
    Eigen::Matrix3d rotation;        // body to world
    Eigen::Vector3d body_rate;       // [rad/s], in the body frame
    Eigen::Vector3d specific_force;  // [m/s2], body frame: rotation^T (acceleration + g0 e_z)
};

// A trajectory built from segments that are joined smoothly. Within a segment the speed along the heading changes by
// a constant acceleration A and the heading by a constant yaw rate R, each with smooth ends: the envelope E(tau) is a
// quintic smoothstep up over `ramp`, one, and the same smoothstep down. So acceleration, yaw rate and their first
// derivatives are continuous at every join (position is C3, the angles C2), and the segment's speed and heading
// change are A (T - ramp) and R (T - ramp). The position is the integral of the velocity, summed with 8-point
// Gauss-Legendre panels of at most 20 ms (the integrand is smooth, so this is exact to about 1e-15 relative); every
// other quantity is a closed form. A walking segment (straight: R = 0) adds the gait as sinusoids in the heading's
// frame and as oscillating roll, pitch and yaw, under the same envelope.
class Motion
{
public:
    Motion(const Eigen::Vector3d &start_position, double start_heading_rad, const Gait &gait = Gait{});

    Motion &rest(double duration_s);
    Motion &cruise(double duration_s, bool walking = false);
    // speed change by dv [m/s] with constant acceleration over duration_s (less the two ramps)
    Motion &change_speed(double dv, double duration_s, double ramp_s = 0.5, bool walking = false);
    // heading change by dpsi [rad] at the current speed: a turn with centripetal acceleration, in place at rest
    Motion &turn(double dpsi, double duration_s, double ramp_s = 0.5);

    double duration() const;
    double end_speed() const;
    double end_heading() const;
    Eigen::Vector3d end_position() const;
    // the first instant at which anything moves (speed or yaw rate non-zero); duration() if nothing does
    double first_motion_time() const;
    // true if the body does not move at all between t0 and t1 (rest segments and nothing else)
    bool at_rest(double t0, double t1) const;

    MotionState state(double t) const;
    // the mean body rate over [t0, t1] and the mean body-frame specific force over [t0, t1] (8-point Gauss-Legendre
    // panels of at most 1 ms)
    Eigen::Vector3d mean_body_rate(double t0, double t1) const;
    Eigen::Vector3d mean_specific_force(double t0, double t1) const;

    const Gait &gait() const { return gait_; }

private:
    struct Seg
    {
        double t0, duration, ramp, accel, yaw_rate;
        bool walking, walk_in, walk_out;
        double v0, psi0;
        Eigen::Vector2d p0;
        double panel;
        std::vector<Eigen::Vector2d> cumulative; // integral of the planar velocity at the panel boundaries
    };
    void add(double duration, double ramp, double accel, double yaw_rate, bool walking);
    const Seg &segment_at(double t) const;
    Eigen::Vector2d planar_position(const Seg &s, double tau) const;

    Gait gait_;
    double z_;
    std::vector<Seg> segs_;
    double v_end_, psi_end_, t_end_;
    Eigen::Vector2d p_end_;
    // gait amplitudes: position [m] (surge, sway, bob), angle [rad] (roll, pitch, yaw), derived from gait_
    Eigen::Vector3d pos_amp_, ang_amp_;
};

// ---- the IMU ----

// The sensor errors the recording draws. The defaults are the estimator's stated weights, at the Livox timing (500 Hz):
//  - gyro white noise in body rate with sigma ARW / sqrt(dt) (the rotation edges' k = 1, floor 0 weight; defaults the
//    configuration's gyro_arw_*_deg_sqrt_s),
//  - accelerometer white noise density D = sqrt(density^2 + floor^2 tau0) with the configuration's density 0.0013 and
//    floor 0.1 at tau0 = 11 ms (design D4: 0.01057 m/s2/sqrt(Hz)); a term's weight is this only at tau0,
//  - a gyro bias and an acceleration offset constant inside a window, stepping between windows with the random walks
//    gyro_bias_rw (deg/s per sqrt(s)) and accel_offset_rw (m/s2 per sqrt(s)) over the window's duration.
struct ImuNoise
{
    Eigen::Vector3d gyro_arw_deg_sqrt_s{0.028, 0.042, 0.057};
    double accel_density_mps2_sqrt_hz = 0.01056;
    double gyro_bias_rw_deg_s_sqrt_s = 0.05;
    double accel_offset_rw_mps2_sqrt_s = 0.05;
    Eigen::Vector3d gyro_bias0_deg_s = Eigen::Vector3d::Zero();   // the bias before the first window and in it
    Eigen::Vector3d accel_offset0_mps2 = Eigen::Vector3d::Zero();

    // every error zero: the noise-free samples of the strap-down test
    static ImuNoise none();
};

struct ImuOptions
{
    std::int64_t t0_ns = 100'000'000'000; // the first sample's timestamp [ns]
    std::int64_t dt_ns = 2'000'000;       // 500 Hz
    int n_samples = 0;                    // 0: as many as the motion's duration holds (the last accelerometer interval inside)
    double first_step_s = 0.005;          // the first update's step in calculate_trajectory
    int nodes_per_window = 180;
    int window_origin_node = 0;           // the first node of window 0; before it the bias is the initial one
    std::uint64_t seed = 1;
    ImuNoise noise;
};

struct Truth
{
    MotionState motion;
    Eigen::Vector3d gyro_bias;    // [rad/s]
    Eigen::Vector3d accel_offset; // [m/s2]
};

struct ImuRecording
{
    Motion motion;
    ImuOptions options;
    std::vector<std::int64_t> t_ns;                // the nodes' timestamps
    std::vector<double> t_s;                       // t_ns / 1e9 as the source reads them (double division)
    std::vector<Eigen::Vector3d> gyro_rad_s;       // the values in the file: rad/s, as in a recorder's file
    std::vector<Eigen::Vector3d> acc_g;            // [g]
    std::vector<Eigen::Vector3d> gyro_bias_rad_s;  // see bias_index: [0] before window 0, [1 + w] window w ([1] == [0])
    std::vector<Eigen::Vector3d> accel_offset_mps2;

    ImuRecording(Motion m) : motion(std::move(m)) {}

    int n_nodes() const { return static_cast<int>(t_ns.size()); }
    double dt_s() const { return options.dt_ns * 1e-9; }
    // the motion's time of node k (the motion starts at node 0)
    double node_time(int k) const { return k * dt_s(); }
    // index into the bias and offset tables: 0 for nodes before window 0, then 1 + window number
    int bias_index(int node) const;
    Truth truth_at_node(int k) const;
    Truth truth_at(double motion_time) const; // the bias and offset of the node at or before it
};

ImuRecording generate_imu(const Motion &motion, const ImuOptions &options);
}
