#include "lidar_odometry_utils.h"
#include <filesystem>
#include "csv.hpp"
#include <algorithm>
#include <regex>

namespace fs = std::filesystem;

// this function provides unique index
unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}

std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z)
{
    Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
    std::vector<Point3Di> out;

    std::vector<PointCloud::PointBucketIndexPair> ip;
    ip.resize(points.size());
    out.reserve(points.size());

    for (int i = 0; i < points.size(); i++)
    {
        ip[i].index_of_point = i;
        ip[i].index_of_bucket = get_rgd_index(points[i].point, b);
    }
    std::sort(ip.begin(), ip.end(), [](const PointCloud::PointBucketIndexPair& a, const PointCloud::PointBucketIndexPair& b)
        { return a.index_of_bucket < b.index_of_bucket; });

    if (ip.size() != 0)
	    out.emplace_back(points[ip[0].index_of_point]);

    for (int i = 1; i < ip.size(); i++)
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
            out.emplace_back(points[ip[i].index_of_point]);

    return out;
}

void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }

            if (this_bucket.number_of_points == 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                // calculate normal vector
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();

                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                // flip towards viewport
                if (nv.dot(viewport - this_bucket.mean) < 0.0)
                {
                    nv *= -1.0;
                }
                this_bucket.normal_vector = nv;
            }

            if (this_bucket.number_of_points > 3)
            {
                Eigen::Vector3d &nv = this_bucket.normal_vector;

                if (nv.dot(viewport - this_bucket.mean) >= 0.0)
                {
                    this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                      cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                }
            }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03; //ToDo move to params
            bucket_to_add.number_of_points = 1;
            bucket_to_add.index_begin = 0;
            bucket_to_add.index_end = 0;
            // no normal until the 3rd point; must not be left uninitialized (it is read by the viewpoint check)
            bucket_to_add.normal_vector = Eigen::Vector3d::Zero();
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}

void update_rgd_spherical_coordinates(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global_spherical[i], b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }

            if (this_bucket.number_of_points >= 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
            bucket_to_add.number_of_points = 1;
            bucket_to_add.index_begin = 0;
            bucket_to_add.index_end = 0;
            // no normal until the 3rd point; must not be left uninitialized (it is read by the viewpoint check)
            bucket_to_add.normal_vector = Eigen::Vector3d::Zero();
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        std::cout << "if You can see only '' it means there is no filename assigned to poses, please read manual or contact me januszbedkowski@gmail.com" << std::endl;
        std::cout << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2" << std::endl;
        std::cout << "1: update initial poses from RESSO file" << std::endl;
        std::cout << "2: update poses from RESSO file" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string &imu_file, int imuToUse)
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
                std::cerr << "Input csv file is missing one of the mandatory columns :\n";
                std::cerr << "timestamp,timestampUnix,gyroX,gyroY,gyroZ,accX,accY,accZ";
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
        std::cout << "load_imu error for file: '" << imu_file << "'" << std::endl;
    }

    return all_data;
}

std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy_inner, double filter_threshold_xy_outer, const std::unordered_map<int, Eigen::Affine3d> &calibrations,
                                       const Eigen::Vector3d &lever_arm)
{
    std::vector<Point3Di> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", lazFile.c_str());
        std::abort();
    }
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    int counter_ts0 = 0;
    int counter_filtered_points = 0;

    for (laszip_U32 j = 0; j < header->number_of_point_records; j++)
    {

        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", j);
            laszip_close_reader(laszip_reader);
            return points;
        }

        Point3Di p;
        int id = point->user_data;

        if (!calibrations.empty())
        {
            if (!calibrations.contains(id))
            {
                continue;
            }
        }

        Eigen::Affine3d calibration = Eigen::Affine3d::Identity();
        if (calibrations.empty())
        {
            // Apply the mapped Unitree L2 IMU lever arm offset for standard NWU frame
            // Original Native: X=-0.007698 (Down), Y=-0.014655 (Left), Z=0.00667 (Front)
            // Mapped NWU: X=Front, Y=Left, Z=Up
            calibration.translation() = lever_arm;
        }
        else
        {
            calibration = calibrations.at(id);
        }

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

    std::cout << header->number_of_point_records << " - " << counter_filtered_points << " = " << points.size();
    if (counter_ts0 > 0)
    {
        std::cout << " (points with 0 timestamp: " << counter_ts0 << ")";
    }
    std::cout << std::endl;

    laszip_close_reader(laszip_reader);
    return points;
}

std::unordered_map<int, std::string> MLvxCalib::GetIdToSnMapping(const std::string &filename)
{
    if (!std::filesystem::exists(filename))
    {
        std::cout << "!std::filesystem::exists(filename) '" << filename << "'" << std::endl;
        return std::unordered_map<int, std::string>();
    }
    std::unordered_map<int, std::string> dataMap;
    std::ifstream fst(filename);
    std::string line;
    while (std::getline(fst, line))
    {
        std::istringstream iss(line);
        int key;
        std::string value;

        if (iss >> key >> value)
        {
            dataMap[key] = value;
        }
        else
        {
            std::cerr << "Failed to parse line: " << line << std::endl;
        }
    }
    return dataMap;
}

inline bool JsonGetBool(
    const nlohmann::json& obj,
    const char* key,
    bool defaultValue = false)
{
    if (!obj.contains(key))
        return defaultValue;

    const auto& v = obj.at(key);

    if (v.is_boolean())
        return v.get<bool>();

    if (v.is_string())
    {
        std::string s = v.get<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::toupper);
        return (s == "TRUE" || s == "1" || s == "YES");
    }

    return defaultValue;
}

std::unordered_map<std::string, Eigen::Affine3d> MLvxCalib::GetCalibrationFromFile(const std::string &filename)
{
    std::unordered_map<std::string, Eigen::Affine3d> dataMap;
    std::ifstream file;
    using json = nlohmann::json;
    json jsonData;

    file.open(filename);
    if (!file)
    {
        std::cerr << "Cannot open file '" << filename << "'" << std::endl;
        return {};
    }

    try
    {
        jsonData = json::parse(file);
    }
    catch (const json::exception& e)
    {
		std::cerr << "JSON parsing error in file '" << filename << "': " << e.what() << std::endl;
        return {};
    }

    if (!jsonData.contains("calibration"))
        return {};

    // Iterate through the JSON object and parse each value into Eigen::Affine3d
    for (auto &calibrationEntry : jsonData["calibration"].items())
    {
        const std::string &lidarSn = calibrationEntry.key();
        Eigen::Matrix4d value;

        bool identity = JsonGetBool(calibrationEntry.value(),"identity", false);
        if (identity)
        {
            dataMap[lidarSn] = Eigen::Matrix4d::Identity();
            continue;
        }

        assert(calibrationEntry.value().contains("data"));
        const auto matrixRawData = calibrationEntry.value()["data"];
        assert(matrixRawData.size() == 16);
        // Populate the Eigen::Affine3d matrix from the JSON array
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                value(i, j) = matrixRawData[i * 4 + j]; // default is row-major order
            }
        }

        if (calibrationEntry.value().contains("order"))
        {
            std::string order = calibrationEntry.value()["order"].get<std::string>();
            std::transform(order.begin(), order.end(), order.begin(), ::toupper);
            if (order == "COLUMN")
                value = value.transpose();
        }

        bool inverted = JsonGetBool(calibrationEntry.value(),"inverted", false);
        if (inverted)
            value = value.inverse();

        dataMap[lidarSn] = value;
    }

    // check for blacklisted
    if (jsonData.contains("blacklist"))
    {
        json blacklist = jsonData["blacklist"];
        for (const auto &item : blacklist)
        {
            std::string blacklistedSn = item.get<std::string>();
            // erase rather than store a zero matrix, to avoid dealing with zero matrices later on
            dataMap.erase(blacklistedSn);
        }
    }
    return dataMap;
}

std::string MLvxCalib::GetImuSnToUse(const std::string &filename)
{
    std::ifstream file;
    using json = nlohmann::json;
    json jsonData;

    file.open(filename);
    if (!file)
    {
        std::cerr << "Cannot open file '" << filename << "'" << std::endl;
        return {};
    }

    try
    {
        jsonData = json::parse(file);
    }
    catch (const json::exception& e)
    {
        std::cerr << "JSON parsing error in file '" << filename << "': " << e.what() << std::endl;
        return {};
    }

    if (!jsonData.contains("imuToUse"))
        return {};
    if (!jsonData["imuToUse"].is_string())
        return {};

    return jsonData["imuToUse"].get<std::string>();
}

std::unordered_map<int, Eigen::Affine3d> MLvxCalib::CombineIntoCalibration(const std::unordered_map<int, std::string> &idToSn, const std::unordered_map<std::string, Eigen::Affine3d> &calibration)
{
    if (calibration.empty())
    {
        return std::unordered_map<int, Eigen::Affine3d>();
    }
    std::unordered_map<int, Eigen::Affine3d> dataMap;
    for (const auto &[id, sn] : idToSn)
    {
        const auto &affine = calibration.at(sn);
        dataMap[id] = affine;
    }
    return dataMap;
}

int MLvxCalib::GetImuIdToUse(const std::unordered_map<int, std::string> &idToSn, const std::string &snToUse)
{
    if (snToUse.empty() || idToSn.empty())
    {
        std::cout << "snToUse.empty() || idToSn.empty()" << std::endl;
        std::cout << "(int)snToUse.empty()" << (int)snToUse.empty() << std::endl;
        std::cout << "(int)idToSn.empty()" << (int)idToSn.empty() << std::endl;
        std::cout << __FILE__ << " " << __LINE__ << std::endl;
        return 0;
    }
    for (const auto &[id, sn] : idToSn)
    {
        if (snToUse == sn)
        {
            return id;
        }
    }
    return 0;
}

fs::path get_next_result_path(const std::string working_directory)
{
    std::regex pattern(R"(lio_result_(\d+))");
    int max_number = -1;
    for (const auto &entry : fs::directory_iterator(working_directory))
    {
        if (entry.is_directory())
        {
            std::smatch match;
            std::string folder_name = entry.path().filename().string();

            if (std::regex_match(folder_name, match, pattern))
            {
                int folder_number = std::stoi(match[1].str());
                max_number = std::max(max_number, folder_number);
            }
        }
    }
    return (working_directory / fs::path("lio_result_" + std::to_string(max_number + 1)));
}
