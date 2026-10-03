#pragma once

// Output-side ports of a processing run (application layer):
//   ResultSink       - the result of one run (clouds, trajectories, poses, session, parameters, report)
//   ConfigRepository - loading and saving the settings

#include "lidar_odometry_utils.h"

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

// one trajectory node of an output chunk
struct TrajectoryRow
{
    std::pair<double, double> timestamps; // (timestamp, unix timestamp) [s]
    Eigen::Affine3d pose;                 // relative to the chunk's first node
    Eigen::Vector3d om_fi_ka;             // IMU orientation [rad]
};

struct SessionSummary
{
    double trajectory_length_m = 0.0;
    double elapsed_seconds = 0.0;
    double decimation = 0.0;
    int threshold_nr_poses = 0;
    bool index_poses = false; // index poses were written
    std::size_t chunk_count = 0;
};

// Receives a finished run in this order: begin, point_sizes, [open_merged_cloud], per chunk chunk_cloud,
// [merged_cloud], chunk_trajectory, then [close_merged_cloud], chunk_poses, [index_poses], session, parameters,
// processing_results.
class ResultSink
{
public:
    virtual ~ResultSink() = default;
    virtual bool begin() = 0;
    // number of points of each window
    virtual void point_sizes(const std::vector<int> &sizes) = 0;
    virtual bool open_merged_cloud() = 0;
    // a chunk's points relative to its first pose
    virtual void chunk_cloud(std::size_t chunk, const std::vector<Eigen::Vector3d> &points,
                             const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
    virtual void merged_cloud(const std::vector<Eigen::Vector3d> &world_points,
                              const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
    // false: the result can't be written, stop saving
    virtual bool chunk_trajectory(std::size_t chunk, const std::vector<TrajectoryRow> &rows) = 0;
    virtual void close_merged_cloud() = 0;
    // the first pose of each chunk
    virtual void chunk_poses(const std::vector<Eigen::Affine3d> &first_poses) = 0;
    virtual void index_poses(const std::vector<std::vector<int>> &index_poses) = 0;
    virtual void session(const SessionSummary &summary) = 0;
    virtual void parameters(const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) = 0;
    virtual void processing_results(const OdometryConfig &config, const OdometryState &state,
                                    const std::vector<Window> &windows, double elapsed_seconds) = 0;
};

class ConfigRepository
{
public:
    virtual ~ConfigRepository() = default;
    // overwrites only what the location contains; false if it can't be read
    virtual bool load(const std::string &location, OdometryConfig &config, RunPaths &paths, GuiPreferences &gui) = 0;
    virtual bool save(const std::string &location, const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) = 0;
};
