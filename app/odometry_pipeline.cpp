#include "odometry_pipeline.h"

#include "loop_closure.h"

OdometryPipeline::OdometryPipeline(const OdometryConfig &config, RunPaths initial_paths, bool debug)
    : config_(&config), debug_(debug)
{
    state.paths = std::move(initial_paths);
}

bool OdometryPipeline::load(std::vector<std::string> input_file_names, const std::atomic<bool> &pause)
{
    const OdometryConfig &params = *config_;
    if (!load_data(input_file_names, params, state, points_per_file, imu, debug_))
        return false;
    calculate_trajectory(trajectory, imu, params.fusionConventionNwu, params.fusionConventionEnu, params.fusionConventionNed,
                         params.ahrs_gain, debug_, params.use_remove_imu_bias_from_first_stationary_scan);
    compute_step_1(points_per_file, params, state, trajectory, worker_data, pause, &imu);
    return true;
}

bool OdometryPipeline::run_odometry(std::atomic<float> &progress, const std::atomic<bool> &pause)
{
    double ts_failure = 0.0;
    return compute_step_2(worker_data, *config_, state, ts_failure, progress, pause, debug_);
}

void OdometryPipeline::post_process()
{
    const OdometryConfig &params = *config_;
    if (params.use_loop_closure)
        run_loop_closure(worker_data, params);

    if (params.use_consistency && params.num_constistency_iter > 0)
        run_consistency(worker_data, params);
}

bool OdometryPipeline::run_all(std::atomic<float> &progress, const std::atomic<bool> &pause)
{
    const bool ok = run_odometry(progress, pause);
    post_process();
    return ok;
}

void OdometryPipeline::save(const fs::path &output_folder, const GuiPreferences &gui, double elapsed_seconds)
{
    save_result(worker_data, *config_, state, gui, output_folder, elapsed_seconds);
}
