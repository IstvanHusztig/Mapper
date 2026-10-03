#include "odometry_pipeline.h"

#include "loop_closure.h"

OdometryPipeline::OdometryPipeline(const OdometryConfig &config, Logger &logger, ProgressReporter &progress, RunInputs inputs,
                                   RunPaths initial_paths, bool debug)
    : config_(&config), logger_(&logger), progress_(&progress), debug_(debug), inputs_(std::move(inputs))
{
    state.paths = std::move(initial_paths);
}

bool OdometryPipeline::load()
{
    const OdometryConfig &params = *config_;
    progress_->stage_started(RunStage::Loading);
    progress_->checkpoint();
    if (!inputs_.complete() || !inputs_.recording->open(*logger_))
        return false;
    state.paths.working_directory_preview = inputs_.preview->location();
    state.paths.working_directory_cache = inputs_.points->location();
    if (!load_data(*inputs_.recording, *inputs_.preview, params, points_per_file, imu, debug_, *logger_))
        return false;
    calculate_trajectory(imu_trajectory, imu, params.fusionConventionNwu, params.fusionConventionEnu, params.fusionConventionNed,
                         params.ahrs_gain, debug_, params.use_remove_imu_bias_from_first_stationary_scan, *logger_);
    compute_step_1(points_per_file, params, state, imu_trajectory, windows, *inputs_.points, *logger_, *progress_, &imu);
    return true;
}

bool OdometryPipeline::run_odometry()
{
    progress_->stage_started(RunStage::Odometry);
    progress_->checkpoint();
    double ts_failure = 0.0;
    return compute_step_2(windows, *config_, state, ts_failure, debug_, *inputs_.points, *inputs_.preview, *logger_, *progress_);
}

void OdometryPipeline::post_process()
{
    const OdometryConfig &params = *config_;
    if (params.use_loop_closure)
    {
        progress_->stage_started(RunStage::LoopClosure);
        progress_->checkpoint();
        run_loop_closure(windows, params, *inputs_.points, *logger_);
    }

    if (params.use_consistency && params.num_constistency_iter > 0)
    {
        progress_->stage_started(RunStage::Consistency);
        progress_->checkpoint();
        run_consistency(windows, params, *inputs_.points, *logger_);
    }
}

bool OdometryPipeline::run_all()
{
    const bool ok = run_odometry();
    post_process();
    return ok;
}

void OdometryPipeline::save(ResultSink &sink, const GuiPreferences &gui, double elapsed_seconds)
{
    progress_->stage_started(RunStage::Saving);
    save_result(windows, *config_, state, gui, elapsed_seconds, sink, *inputs_.points, *logger_);
}

void OdometryPipeline::clear_point_store()
{
    if (inputs_.points)
        inputs_.points->clear();
}

bool OdometryPipeline::load_window_points(std::size_t window, std::vector<Point3Di> &points) const
{
    return inputs_.points && inputs_.points->load(PointSet::WindowIntermediate, window, points);
}
