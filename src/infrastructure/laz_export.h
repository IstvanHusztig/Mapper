#pragma once

// Writes a LAZ file (infrastructure): taken over unchanged from HDMapping core's export_laz.h (retire-core).

#include <Eigen/Dense>

#include <string>
#include <vector>

// points (with offset_x/y/alt added), their intensity and timestamps, LASzip-compressed into filename
bool exportLaz(const std::string &filename,
               const std::vector<Eigen::Vector3d> &pointcloud,
               const std::vector<unsigned short> &intensity,
               const std::vector<double> &timestamps,
               double offset_x = 0.0, double offset_y = 0.0, double offset_alt = 0.0);
