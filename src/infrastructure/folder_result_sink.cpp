#include "folder_result_sink.h"
#include "odometry_state.h"

#include <dmapper_version.h>
#include <build_info.h>
#include "laz_export.h"
#include <laszip/laszip_api.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

namespace fs = std::filesystem;

namespace
{
const float RAD_TO_DEG = 180.0f / M_PI; // as in lidar_odometry.cpp: the results JSON must not change

std::string chunk_laz_name(std::size_t chunk) { return "scan_lio_" + std::to_string(chunk) + ".laz"; }
} // namespace

// one LAZ with all chunks in world coordinates, written chunk by chunk (memory stays flat on long recordings);
// the header's point count and bounding box are filled in from the written points when it is closed
class FolderResultSink::MergedLazWriter
{
public:
    explicit MergedLazWriter(Logger &logger) : logger_(&logger) {}

    bool open(const std::string &filename)
    {
        if (laszip_create(&writer_))
            return false;
        laszip_header *header;
        if (laszip_get_header_pointer(writer_, &header))
            return false;
        header->file_source_ID = 4711;
        header->global_encoding = (1 << 0);
        header->version_major = 1;
        header->version_minor = 2;
        header->point_data_format = 1;
        header->point_data_record_length = 28;
        header->x_scale_factor = header->y_scale_factor = header->z_scale_factor = 0.0001;
        header->x_offset = header->y_offset = header->z_offset = 0.0;
        if (laszip_open_writer(writer_, filename.c_str(), 1) || laszip_get_point_pointer(writer_, &point_))
            return false;
        open_ = true;
        return true;
    }

    void write(const std::vector<Eigen::Vector3d> &points, const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps)
    {
        if (!open_)
            return;
        for (size_t i = 0; i < points.size(); i++)
        {
            laszip_F64 coordinates[3] = {points[i].x(), points[i].y(), points[i].z()};
            point_->intensity = intensity[i];
            point_->gps_time = timestamps[i] * 1e9;
            if (laszip_set_coordinates(writer_, coordinates) || laszip_write_point(writer_) || laszip_update_inventory(writer_))
            {
                log_error(*logger_) << "merged LAZ: write error, file incomplete";
                close();
                return;
            }
            count_++;
        }
    }

    void close()
    {
        if (!open_)
            return;
        laszip_close_writer(writer_);
        laszip_destroy(writer_);
        open_ = false;
    }

    long long count() const { return count_; }

private:
    Logger *logger_;
    laszip_POINTER writer_ = nullptr;
    laszip_point *point_ = nullptr;
    bool open_ = false;
    long long count_ = 0;
};

static bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames, Logger &logger)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        log_info(logger) << "can not save file: '" << file_name << "'";
        log_info(logger) << "if You can see only '' it means there is no filename assigned to poses, please read manual or contact me januszbedkowski@gmail.com";
        log_info(logger) << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2";
        log_info(logger) << "1: update initial poses from RESSO file";
        log_info(logger) << "2: update poses from RESSO file";
        return false;
    }

    outfile << std::setprecision(17); // full double precision: chunk poses far from the origin keep sub-millimetre digits
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

FolderResultSink::FolderResultSink(fs::path folder, ConfigRepository &config_repository, Logger &logger)
    : outwd_(std::move(folder)), config_repository_(&config_repository), logger_(&logger)
{
}

FolderResultSink::~FolderResultSink() = default;

bool FolderResultSink::begin()
{
    std::filesystem::create_directory(outwd_);
    return true;
}

void FolderResultSink::point_sizes(const std::vector<int> &sizes)
{
    fs::path point_sizes_path = outwd_ / "point_sizes_per_chunk.json";
    nlohmann::json j_point_sizes = sizes;
    std::ofstream out_point_sizes(point_sizes_path);
    if (!out_point_sizes)
    {
        log_error(*logger_) << "Failed to open " << point_sizes_path << " for writing point sizes per chunk.\n";
    }
    else
    {
        out_point_sizes << j_point_sizes.dump(2);
        out_point_sizes.close();
    }
}

bool FolderResultSink::open_merged_cloud()
{
    merged_ = std::make_unique<MergedLazWriter>(*logger_);
    if (!merged_->open((outwd_ / "scan_lio_merged.laz").string()))
    {
        log_error(*logger_) << "can not open " << (outwd_ / "scan_lio_merged.laz") << " for writing";
        return false;
    }
    return true;
}

void FolderResultSink::chunk_cloud(std::size_t chunk, const std::vector<Eigen::Vector3d> &points,
                                   const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps)
{
    log_info(*logger_) << "------------------------";
    fs::path path(outwd_);
    path /= chunk_laz_name(chunk);
    log_info(*logger_) << "saving to: " << path;
    exportLaz(path.string(), points, intensity, timestamps);
}

void FolderResultSink::merged_cloud(const std::vector<Eigen::Vector3d> &world_points,
                                    const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps)
{
    if (merged_)
        merged_->write(world_points, intensity, timestamps);
}

bool FolderResultSink::chunk_trajectory(std::size_t chunk, const std::vector<TrajectoryRow> &rows)
{
    std::string trajectory_filename = ("trajectory_lio_" + std::to_string(chunk) + ".csv");
    fs::path pathtrj(outwd_);
    pathtrj /= trajectory_filename;
    log_info(*logger_) << "saving to: " << pathtrj;

    std::ofstream outfile;
    outfile.open(pathtrj);
    if (!outfile.good())
    {
        log_info(*logger_) << "can not save file: " << pathtrj;
        return false;
    }

    outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;
    for (const auto &row : rows)
    {
        const auto &pose = row.pose;

        outfile
            << std::setprecision(20) << row.timestamps.first * 1e9 << " " << std::setprecision(10)
            << pose(0, 0) << " "
            << pose(0, 1) << " "
            << pose(0, 2) << " "
            << pose(0, 3) << " "
            << pose(1, 0) << " "
            << pose(1, 1) << " "
            << pose(1, 2) << " "
            << pose(1, 3) << " "
            << pose(2, 0) << " "
            << pose(2, 1) << " "
            << pose(2, 2) << " "
            << pose(2, 3) << " "
            << std::setprecision(20) << row.timestamps.second * 1e9 << " "
            << row.om_fi_ka.x() << " "
            << row.om_fi_ka.y() << " "
            << row.om_fi_ka.z() << " "
            << std::endl;
    }
    outfile.close();
    return true;
}

void FolderResultSink::close_merged_cloud()
{
    if (!merged_)
        return;
    merged_->close();
    log_info(*logger_) << "saved " << merged_->count() << " points to " << (outwd_ / "scan_lio_merged.laz");
}

void FolderResultSink::chunk_poses(const std::vector<Eigen::Affine3d> &first_poses)
{
    std::vector<std::string> file_names;
    for (std::size_t i = 0; i < first_poses.size(); i++)
        file_names.push_back(chunk_laz_name(i));

    fs::path path(outwd_);
    path /= "session_ini_poses.mri";
    save_poses(path.string(), first_poses, file_names, *logger_);
    fs::path path2(outwd_);
    path2 /= "session_poses.mrp";
    save_poses(path2.string(), first_poses, file_names, *logger_);
}

void FolderResultSink::index_poses(const std::vector<std::vector<int>> &index_poses)
{
    fs::path index_poses_path = outwd_ / "index_poses.json";
    nlohmann::json j_index_poses = index_poses;
    std::ofstream out_index(index_poses_path);
    if (!out_index)
    {
        log_error(*logger_) << "Failed to open " << index_poses_path << " for writing index poses.\n";
    }
    else
    {
        out_index << j_index_poses.dump(2);
        out_index.close();
    }
}

void FolderResultSink::session(const SessionSummary &summary)
{
    const fs::path &outwd = outwd_;
    fs::path path(outwd);
    path /= "session_ini_poses.mri";
    fs::path path2(outwd);
    path2 /= "session_poses.mrp";
    fs::path index_poses_path = outwd / "index_poses.json";
    fs::path point_sizes_path = outwd / "point_sizes_per_chunk.json";

    fs::path path3(outwd);
    path3 /= "session.mjs";

    // save session file
    log_info(*logger_) << "saving file: '" << path3 << "'";

    nlohmann::json jj;
    nlohmann::json j;

    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd;
    j["out_folder_name"] = outwd;
    j["poses_file_name"] = path2.string();
    j["initial_poses_file_name"] = path.string();
    j["out_poses_file_name"] = path2.string();
    j["lidar_odometry_version"] = "v" + dmapper_version();
    j["length of trajectory[m]"] = summary.trajectory_length_m;
    j["elapsed time seconds"] = summary.elapsed_seconds;
    if (summary.index_poses){
        j["index_poses_path"] = index_poses_path.string();
    }
    j["point_sizes_path"] = point_sizes_path.string();
    j["decimation"] = summary.decimation;
    j["threshold_nr_poses"] = summary.threshold_nr_poses;

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (std::size_t i = 0; i < summary.chunk_count; i++)
    {
        fs::path path(outwd);
        std::string filename = chunk_laz_name(i);
        path /= filename;
        log_info(*logger_) << "adding file: " << path;

        nlohmann::json jfn{
            {"file_name", path.string()}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(path3.string());
    fs << jj.dump(2);
    fs.close();
}

void FolderResultSink::parameters(const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui)
{
    // Get current date and time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    // Format: YYYY-MM-DD_HH-MM
    std::ostringstream datetime_stream;
    datetime_stream << std::put_time(&tm, "%Y-%m-%d_%H-%M");
    std::string datetime_str = datetime_stream.str();

    // Create filename with version info and datetime for TOML parameters
    std::string toml_filename = "dMapper_params_" + config.software_version + "_" + datetime_str + ".toml";
    fs::path toml_path = outwd_ / toml_filename;

    try
    {
        bool success = config_repository_->save(toml_path.string(), config, paths, gui);

        if (success)
        {
            log_info(*logger_) << "Parameters saved to TOML file: " << toml_path;
        }
        else
        {
            log_error(*logger_) << "Failed to save parameters to TOML file: " << toml_path;
        }
    }
    catch (const std::exception &e)
    {
        log_error(*logger_) << "Error saving parameters to TOML file: " << e.what();
    }
}

void FolderResultSink::processing_results(const OdometryConfig &config, const OdometryState &state,
                                          const std::vector<Window> &windows, double elapsed_seconds)
{
    // Get current date and time
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    // Format: YYYY-MM-DD_HH-MM
    std::ostringstream datetime_stream;
    datetime_stream << std::put_time(&tm, "%Y-%m-%d_%H-%M");
    std::string datetime_str = datetime_stream.str();

    // Create filename for processing results and complex data
    std::string json_filename = "dMapper_results_" + config.software_version + "_" + datetime_str + ".json";
    fs::path json_path = outwd_ / json_filename;

    try
    {
        // Create JSON structure for processing results and complex data
        nlohmann::json results;

        // Processing metadata
        results["processing_info"]["software_version"] = config.software_version;
        results["processing_info"]["config_version"] = config.config_version;
        results["processing_info"]["build_date"] = config.build_date;
        results["processing_info"]["processing_date"] = datetime_str;
        results["processing_info"]["elapsed_time_seconds"] = elapsed_seconds;
        // the build that made this result (build_info.h, written at build time)
        results["build_info"]["git_commit"] = DMAPPER_GIT_COMMIT;
        results["build_info"]["git_uncommitted_changes"] = DMAPPER_GIT_UNCOMMITTED_CHANGES;
        results["build_info"]["build_type"] = DMAPPER_BUILD_TYPE;
        results["build_info"]["compiler"] = DMAPPER_COMPILER;
        results["build_info"]["cxx_flags"] = DMAPPER_CXX_FLAGS;
        results["build_info"]["compile_definitions"] = DMAPPER_COMPILE_DEFINITIONS;

        // Processing results and computed values
        results["trajectory_results"]["total_length_calculated"] = state.total_length_of_calculated_trajectory;
        results["trajectory_results"]["consecutive_distance"] = state.consecutive_distance;
        results["directory_info"]["current_output_dir"] = state.paths.current_output_dir;
        results["directory_info"]["working_directory_preview"] = state.paths.working_directory_preview;

        // Complex data structures that can't be easily loaded back as parameters
        // NDT grid parameters (read-only results)
        results["ndt_grid_indoor"]["bounding_box_min_X"] = config.in_out_params_indoor.bounding_box_min_X;
        results["ndt_grid_indoor"]["bounding_box_min_Y"] = config.in_out_params_indoor.bounding_box_min_Y;
        results["ndt_grid_indoor"]["bounding_box_min_Z"] = config.in_out_params_indoor.bounding_box_min_Z;
        results["ndt_grid_indoor"]["bounding_box_max_X"] = config.in_out_params_indoor.bounding_box_max_X;
        results["ndt_grid_indoor"]["bounding_box_max_Y"] = config.in_out_params_indoor.bounding_box_max_Y;
        results["ndt_grid_indoor"]["bounding_box_max_Z"] = config.in_out_params_indoor.bounding_box_max_Z;
        results["ndt_grid_indoor"]["resolution_X"] = config.in_out_params_indoor.resolution_X;
        results["ndt_grid_indoor"]["resolution_Y"] = config.in_out_params_indoor.resolution_Y;
        results["ndt_grid_indoor"]["resolution_Z"] = config.in_out_params_indoor.resolution_Z;
        results["ndt_grid_indoor"]["number_of_buckets"] = static_cast<long long>(config.in_out_params_indoor.number_of_buckets);

        results["ndt_grid_outdoor"]["bounding_box_min_X"] = config.in_out_params_outdoor.bounding_box_min_X;
        results["ndt_grid_outdoor"]["bounding_box_min_Y"] = config.in_out_params_outdoor.bounding_box_min_Y;
        results["ndt_grid_outdoor"]["bounding_box_min_Z"] = config.in_out_params_outdoor.bounding_box_min_Z;
        results["ndt_grid_outdoor"]["bounding_box_max_X"] = config.in_out_params_outdoor.bounding_box_max_X;
        results["ndt_grid_outdoor"]["bounding_box_max_Y"] = config.in_out_params_outdoor.bounding_box_max_Y;
        results["ndt_grid_outdoor"]["bounding_box_max_Z"] = config.in_out_params_outdoor.bounding_box_max_Z;
        results["ndt_grid_outdoor"]["resolution_X"] = config.in_out_params_outdoor.resolution_X;
        results["ndt_grid_outdoor"]["resolution_Y"] = config.in_out_params_outdoor.resolution_Y;
        results["ndt_grid_outdoor"]["resolution_Z"] = config.in_out_params_outdoor.resolution_Z;
        results["ndt_grid_outdoor"]["number_of_buckets"] = static_cast<long long>(config.in_out_params_outdoor.number_of_buckets);

        // Motion model correction (complex structure)
        results["motion_model_correction"]["om"] = config.motion_model_correction.om;
        results["motion_model_correction"]["fi"] = config.motion_model_correction.fi;
        results["motion_model_correction"]["ka"] = config.motion_model_correction.ka;

        // Estimated gyro bias per window [deg/s] and its 1-sigma [deg/s]
        if (config.use_gyro_bias_estimation)
        {
            nlohmann::json track = nlohmann::json::array();
            for (const auto &wd : windows)
            {
                if (wd.timestamps.empty())
                    continue;
                const Eigen::Vector3d b = wd.gyro_bias.mean * RAD_TO_DEG;
                const Eigen::Vector3d s = wd.gyro_bias.covariance.diagonal().cwiseMax(0.0).cwiseSqrt() * RAD_TO_DEG;
                track.push_back({{"t_start_s", wd.timestamps.front().first},
                                 {"bias_deg_s", {b.x(), b.y(), b.z()}},
                                 {"sigma_deg_s", {s.x(), s.y(), s.z()}}});
            }
            results["gyro_bias_per_window"] = track;
        }

        // Transformation matrix (if needed for debugging)
        auto &m_g = state.m_g;
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                results["transformation_matrix"][i][j] = m_g(i, j);
            }
        }

        // Save JSON file
        std::ofstream file(json_path);
        if (file.is_open())
        {
            file << results.dump(4); // Pretty print with 4-space indentation
            file.close();
            log_info(*logger_) << "Processing results saved to JSON file: " << json_path;
            log_info(*logger_) << "Processing time: " << std::fixed << std::setprecision(2) << elapsed_seconds << " [s]";
        }
        else
        {
            log_error(*logger_) << "Failed to create results JSON file: " << json_path;
        }
    }
    catch (const std::exception &e)
    {
        log_error(*logger_) << "Error saving processing results to JSON file: " << e.what();
    }
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
