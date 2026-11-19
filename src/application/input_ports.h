#pragma once

// Input-side ports of a processing run (application layer):
//   RecordingSource - the recording's LiDAR + IMU chunk pairs
//   PointStore      - point sets kept between stages (per window, per session chunk)
//   PreviewSink     - optional preview output while the run is going

#include "imu_sample.h"
#include "run_reporting.h"

#include <Eigen/Dense>
#include <structures.h>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// which LiDAR points of a chunk are kept, and the lever arm added to each
struct PointFilter
{
    bool omit_zero_timestamp = true;
    double xy_inner = 0.0;
    double xy_outer = 0.0;
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
};

class RecordingSource
{
public:
    virtual ~RecordingSource() = default;
    // checks the recording and prints why it can't be processed; call before the rest
    virtual bool open(Logger &logger) = 0;
    // complete LiDAR + IMU pairs
    virtual std::size_t chunk_count() const = 0;
    // may be called from several threads at once
    virtual Imu read_imu(std::size_t chunk, const ImuCalibration &calibration, Logger &logger) = 0;
    virtual std::vector<Point3Di> read_points(std::size_t chunk, const PointFilter &filter, Logger &logger) = 0;
};

enum class PointSet
{
    WindowOriginal,     // a window's points as loaded
    WindowIntermediate, // a window's (decimated) points used by the optimizers
    SessionChunk        // the points of one output chunk (several windows)
};

class PointStore
{
public:
    virtual ~PointStore() = default;
    virtual bool save(PointSet set, std::size_t index, const std::vector<Point3Di> &points) = 0;
    virtual bool load(PointSet set, std::size_t index, std::vector<Point3Di> &points) = 0;
    // removes everything stored
    virtual void clear() = 0;
    // where the points are kept (recorded in the outputs as working_directory_cache)
    virtual std::string location() const = 0;
};

class PreviewSink
{
public:
    virtual ~PreviewSink() = default;
    // the first max_points points of the first chunk, to check the LiDAR calibration
    virtual void calibration_validation(const std::vector<Point3Di> &points, std::size_t max_points, Logger &logger) = 0;
    // a window's points in world coordinates
    virtual void window_cloud(std::size_t window, const std::vector<Eigen::Vector3d> &points,
                              const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) = 0;
    // where the preview is written (recorded in the outputs as working_directory_preview)
    virtual std::string location() const = 0;
};

// the input side of one run; all empty if the recording could not be opened
struct RunInputs
{
    std::unique_ptr<RecordingSource> recording;
    std::unique_ptr<PointStore> points;
    std::unique_ptr<PreviewSink> preview;

    bool complete() const { return recording && points && preview; }
};
