#pragma once

// PreviewSink adapter: files in the recording's preview folder (infrastructure).

#include "input_ports.h"

#include <filesystem>

// calibrationValidation.asc and temp_point_cloud_<i>.laz in `folder`; the folder is created on the first write
class FolderPreviewSink : public PreviewSink
{
public:
    explicit FolderPreviewSink(std::filesystem::path folder) : folder_(std::move(folder)) {}

    void calibration_validation(const std::vector<Point3Di> &points, std::size_t max_points, Logger &logger) override;
    void window_cloud(std::size_t window, const std::vector<Eigen::Vector3d> &points,
                      const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) override;
    std::string location() const override { return folder_.string(); }

private:
    std::filesystem::path folder_;
};
