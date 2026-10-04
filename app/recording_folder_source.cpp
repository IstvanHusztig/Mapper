#include "recording_folder_source.h"
#include <Fusion.h>

#include "csv.hpp"

#include <laszip/laszip_api.h>

#include <filesystem>
#include <set>

namespace fs = std::filesystem;

bool RecordingFolderSource::open(Logger &logger)
{
    for (std::size_t i = 0; i < chunk_count(); i++)
        for (const std::string *name : {&files_.laz[i], &files_.csv[i]})
            if (!fs::exists(*name))
            {
                log_error(logger) << "File '" << *name << "' does not exist.";
                return false;
            }
    return true;
}

Imu RecordingFolderSource::read_imu(std::size_t chunk, Logger &logger)
{
    return load_imu(files_.csv[chunk].c_str(), 0, logger);
}

std::vector<Point3Di> RecordingFolderSource::read_points(std::size_t chunk, const PointFilter &filter, Logger &logger)
{
    return load_point_cloud(files_.laz[chunk].c_str(), filter.omit_zero_timestamp, filter.xy_inner, filter.xy_outer, filter.lever_arm, logger);
}

std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string &imu_file, int imuToUse, Logger &logger)
{
    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> all_data;

    csv::CSVFormat format;
    format.delimiter({' ', ',', '\t'});

    try
    {
        csv::CSVReader reader(imu_file, format);

        const auto columns = reader.get_col_names();
        const std::set<std::string> columnsSet(columns.begin(), columns.end());

        // mandatory columns
        const bool hasTsColumn = columnsSet.contains("timestamp");
        const bool hasGyrosColumns = columnsSet.contains("gyroX") && columnsSet.contains("gyroY") && columnsSet.contains("gyroZ");
        const bool hasAccsColumns = columnsSet.contains("accX") && columnsSet.contains("accY") && columnsSet.contains("accZ");

        // optional
        const bool hasImuIdColumn = columnsSet.contains("imuId");
        const bool hasUnixTimestampColumn = columnsSet.contains("timestampUnix");

        // check if legacy
        bool is_legacy = true;
        if (hasTsColumn)
        {
            is_legacy = false;
            if (!hasAccsColumns && !hasGyrosColumns && !hasUnixTimestampColumn)
            {
                log_error(logger) << "Input csv file is missing one of the mandatory columns :\n"
                                  << "timestamp,timestampUnix,gyroX,gyroY,gyroZ,accX,accY,accZ";
                return all_data;
            }
        }

        if (!is_legacy)
        {
            // check if all needed columns are in csv
            for (auto row : reader)
            {
                int imu_id = -1;
                if (hasImuIdColumn)
                {
                    imu_id = row["imuId"].get<int>();
                }
                if (imu_id < 0 || imuToUse == imu_id)
                {
                    double timestamp = row["timestamp"].get<double>();
                    double timestampUnix = row["timestampUnix"].get<double>();
                    FusionVector gyr;
                    gyr.axis.x = row["gyroX"].get<double>();
                    gyr.axis.y = row["gyroY"].get<double>();
                    gyr.axis.z = row["gyroZ"].get<double>();
                    FusionVector acc;
                    acc.axis.x = row["accX"].get<double>();
                    acc.axis.y = row["accY"].get<double>();
                    acc.axis.z = row["accZ"].get<double>();
                    all_data.emplace_back(std::pair(timestamp / 1e9, timestampUnix / 1e9), gyr, acc);
                }
            }
        }
        if (is_legacy)
        {
            std::ifstream myfile(imu_file);
            if (myfile.is_open())
            {
                while (myfile)
                {
                    double data[7];
                    double timestampUnix = 0.0;
                    int imuId = 0;
                    std::string line;
                    std::getline(myfile, line);
                    std::istringstream iss(line);
                    iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                    if (!iss.eof())
                    {
                        iss >> imuId;
                    }
                    if (!iss.eof())
                    {
                        iss >> timestampUnix;
                    }
                    if (data[0] > 0 && imuId == imuToUse)
                    {
                        FusionVector gyr;
                        gyr.axis.x = data[1];
                        gyr.axis.y = data[2];
                        gyr.axis.z = data[3];

                        FusionVector acc;
                        acc.axis.x = data[4];
                        acc.axis.y = data[5];
                        acc.axis.z = data[6];

                        all_data.emplace_back(std::pair(data[0] / 1e9, timestampUnix / 1e9), gyr, acc);
                    }
                }
                myfile.close();
            }
        }
    }
    catch (...)
    {
        log_info(logger) << "load_imu error for file: '" << imu_file << "'";
    }

    return all_data;
}

std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy_inner, double filter_threshold_xy_outer,
                                       const Eigen::Vector3d &lever_arm, Logger &logger)
{
    std::vector<Point3Di> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        log_error(logger) << "DLL ERROR: creating laszip reader";
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
    {
        log_error(logger) << "DLL ERROR: opening laszip reader for '" << lazFile << "'";
        std::abort();
    }
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        log_error(logger) << "DLL ERROR: getting header pointer from laszip reader";
        std::abort();
    }
    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        log_error(logger) << "DLL ERROR: getting point pointer from laszip reader";
        std::abort();
    }

    int counter_ts0 = 0;
    int counter_filtered_points = 0;

    for (laszip_U32 j = 0; j < header->number_of_point_records; j++)
    {

        if (laszip_read_point(laszip_reader))
        {
            log_error(logger) << "DLL ERROR: reading point " << j;
            laszip_close_reader(laszip_reader);
            return points;
        }

        Point3Di p;
        int id = point->user_data;

        // Apply the mapped Unitree L2 IMU lever arm offset for standard NWU frame
        // Original Native: X=-0.007698 (Down), Y=-0.014655 (Left), Z=0.00667 (Front)
        // Mapped NWU: X=Front, Y=Left, Z=Up
        Eigen::Affine3d calibration = Eigen::Affine3d::Identity();
        calibration.translation() = lever_arm;

        const Eigen::Vector3d pf(
            header->x_offset + header->x_scale_factor * static_cast<double>(point->X),
            header->y_offset + header->y_scale_factor * static_cast<double>(point->Y),
            header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));

        p.point = calibration * (pf);
        p.lidarid = id;
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        if (p.timestamp == 0 && ommit_points_with_timestamp_equals_zero)
        {
            counter_ts0++;
        }
        else
        {
            if (sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter_threshold_xy_inner && sqrt(pf.x() * pf.x() + pf.y() * pf.y()) < filter_threshold_xy_outer)
            {
                points.emplace_back(p);
            }
            else
            {
                counter_filtered_points++;
            }
        }
    }

    {
        auto msg = log_info(logger);
        msg << header->number_of_point_records << " - " << counter_filtered_points << " = " << points.size();
        if (counter_ts0 > 0)
            msg << " (points with 0 timestamp: " << counter_ts0 << ")";
    }

    laszip_close_reader(laszip_reader);
    return points;
}
