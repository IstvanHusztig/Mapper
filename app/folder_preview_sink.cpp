#include "folder_preview_sink.h"

#include <export_laz.h>

#include <algorithm>
#include <fstream>

namespace fs = std::filesystem;

void FolderPreviewSink::calibration_validation(const std::vector<Point3Di> &points, std::size_t max_points, Logger &logger)
{
    if (!fs::exists(folder_))
    {
        log_info(logger) << "Creating folder: " << folder_ << "\n";
        fs::create_directory(folder_);
    }

    fs::path outFile = folder_ / "calibrationValidation.asc";
    std::ofstream testPointcloud(outFile);
    if (!testPointcloud)
        log_error(logger) << "Failed to open file: " << outFile << "\n";
    else
        for (size_t j = 0; j < std::min(points.size(), max_points); ++j)
        {
            const auto& p = points[j];
            testPointcloud << p.point.x() << "\t" << p.point.y() << "\t" << p.point.z() << "\t" << p.intensity << "\t" << (int)p.lidarid << "\n";
        }
}

void FolderPreviewSink::window_cloud(std::size_t window, const std::vector<Eigen::Vector3d> &points,
                                     const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps)
{
    if (!fs::exists(folder_))
        fs::create_directory(folder_);
    std::string fn = folder_.string() + "/temp_point_cloud_" + std::to_string(window) + ".laz";
    exportLaz(fn.c_str(), points, intensity, timestamps);
}
