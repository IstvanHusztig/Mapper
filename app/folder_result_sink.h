#pragma once

// ResultSink adapter: one output folder (infrastructure). File names and formats:
//   point_sizes_per_chunk.json, scan_lio_<i>.laz, scan_lio_merged.laz, trajectory_lio_<i>.csv,
//   session_ini_poses.mri, session_poses.mrp, index_poses.json, session.mjs,
//   HDMapping_params_<version>_<date-time>.toml (through a ConfigRepository), HDMapping_results_<version>_<date-time>.json

#include "output_ports.h"

#include <filesystem>
#include <memory>

class FolderResultSink : public ResultSink
{
public:
    FolderResultSink(std::filesystem::path folder, ConfigRepository &config_repository, Logger &logger);
    ~FolderResultSink() override;

    bool begin() override;
    void point_sizes(const std::vector<int> &sizes) override;
    bool open_merged_cloud() override;
    void chunk_cloud(std::size_t chunk, const std::vector<Eigen::Vector3d> &points,
                     const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) override;
    void merged_cloud(const std::vector<Eigen::Vector3d> &world_points,
                      const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps) override;
    bool chunk_trajectory(std::size_t chunk, const std::vector<TrajectoryRow> &rows) override;
    void close_merged_cloud() override;
    void chunk_poses(const std::vector<Eigen::Affine3d> &first_poses) override;
    void index_poses(const std::vector<std::vector<int>> &index_poses) override;
    void session(const SessionSummary &summary) override;
    void parameters(const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) override;
    void processing_results(const OdometryConfig &config, const OdometryState &state,
                            const std::vector<WorkerData> &windows, double elapsed_seconds) override;

private:
    class MergedLazWriter;

    std::filesystem::path outwd_;
    ConfigRepository *config_repository_;
    Logger *logger_;
    std::unique_ptr<MergedLazWriter> merged_;
};

// the next free <working_directory>/lio_result_<n> (the GUI saves each result to a new folder)
std::filesystem::path get_next_result_path(const std::string working_directory);
