#include "odometry_pipeline.h"

#include "loop_closure.h"

OdometryPipeline::OdometryPipeline(const OdometryConfig &config, Logger &logger, ProgressReporter &progress, RunPaths initial_paths, bool debug)
    : config_(&config), logger_(&logger), progress_(&progress), debug_(debug)
{
    state.paths = std::move(initial_paths);
}

bool OdometryPipeline::load(std::vector<std::string> input_file_names)
{
    const OdometryConfig &params = *config_;
    progress_->stage_started(RunStage::Loading);
    progress_->checkpoint();
    if (!load_data(input_file_names, params, state, points_per_file, imu, debug_, *logger_))
        return false;
    calculate_trajectory(trajectory, imu, params.fusionConventionNwu, params.fusionConventionEnu, params.fusionConventionNed,
                         params.ahrs_gain, debug_, params.use_remove_imu_bias_from_first_stationary_scan, *logger_);
    compute_step_1(points_per_file, params, state, trajectory, worker_data, *logger_, *progress_, &imu);
    return true;
}

bool OdometryPipeline::run_odometry()
{
    progress_->stage_started(RunStage::Odometry);
    progress_->checkpoint();
    double ts_failure = 0.0;
    return compute_step_2(worker_data, *config_, state, ts_failure, debug_, *logger_, *progress_);
}

void OdometryPipeline::post_process()
{
    const OdometryConfig &params = *config_;
    if (params.use_loop_closure)
    {
        progress_->stage_started(RunStage::LoopClosure);
        progress_->checkpoint();
        run_loop_closure(worker_data, params, *logger_);
    }

    if (params.use_consistency && params.num_constistency_iter > 0)
    {
        progress_->stage_started(RunStage::Consistency);
        progress_->checkpoint();
        run_consistency(worker_data, params, *logger_);
    }
}

bool OdometryPipeline::run_all()
{
    const bool ok = run_odometry();
    post_process();
    return ok;
}

void OdometryPipeline::save(const fs::path &output_folder, const GuiPreferences &gui, double elapsed_seconds)
{
    progress_->stage_started(RunStage::Saving);
    save_result(worker_data, *config_, state, gui, output_folder, elapsed_seconds, *logger_);
}
