#pragma once

// RecordingSource adapter for a recording folder (infrastructure): LiDAR chunks (*.laz/*.las, LASzip)
// paired with IMU chunks (*.csv).

#include "input_ports.h"
#include "recording_files.h"

#include <string>
#include <vector>

class RecordingFolderSource : public RecordingSource
{
public:
    explicit RecordingFolderSource(RecordingFiles files) : files_(std::move(files)) {}

    bool open(Logger &logger) override;
    std::size_t chunk_count() const override { return files_.pair_count(); }
    Imu read_imu(std::size_t chunk, Logger &logger) override;
    std::vector<Point3Di> read_points(std::size_t chunk, const PointFilter &filter, Logger &logger) override;

    const RecordingFiles &files() const { return files_; }

private:
    RecordingFiles files_;
};

//! This function load inertial measurement unit data.
//! This function expects a file with the following format:
//! timestamp angular_velocity_x angular_velocity_y angular_velocity_z linear_acceleration_x linear_acceleration_y linear_acceleration_z imu_id
//! @note imu_id is an optional column, if not present, it is assumed that all data comes from the same IMU.
//! @param imu_file - path to file with IMU data
//! @param imuToUse - id number of IMU to use, the same index as in pointcloud return by @ref load_point_cloud
//! @return the samples ((timestamp, timestampUnix), angular_velocity, linear_acceleration)
Imu load_imu(const std::string &imu_file, int imuToUse, Logger &logger);

//! This function load point cloud from LAS/LAZ file.
//! Each point is shifted by the LiDAR-IMU lever arm.
//! @param lazFile - path to file with point cloud
//! @param ommit_points_with_timestamp_equals_zero - if true, points with timestamp == 0 will be omited
//! @param filter_threshold_xy - threshold for filtering points in xy plane
//! @param lever_arm - LiDAR-IMU lever arm added to each point
//! @return vector of points of @ref Point3Di type
std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy_inner, double filter_threshold_xy_outer,
                                       const Eigen::Vector3d &lever_arm, Logger &logger);
