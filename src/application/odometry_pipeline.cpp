#include "odometry_pipeline.h"
#include "run_odometry.h"

#include "loop_closure.h"

#include <chrono>
#include <iomanip>

namespace
{
    // log_stage_timing: logs "timing <step> N s" when the step ends (stop() or the end of the scope)
    class StepTiming
    {
    public:
        StepTiming(const OdometryConfig &params, Logger &logger, const char *step)
            : logger_(params.log_stage_timing ? &logger : nullptr), step_(step), start_(std::chrono::steady_clock::now())
        {
        }
        ~StepTiming() { stop(); }
        StepTiming(const StepTiming &) = delete;
        StepTiming &operator=(const StepTiming &) = delete;

        void stop()
        {
            if (!logger_)
                return;
            const std::chrono::duration<double> seconds = std::chrono::steady_clock::now() - start_;
            log_info(*logger_) << "timing " << step_ << " " << std::fixed << std::setprecision(1) << seconds.count() << " s";
            logger_ = nullptr;
        }

    private:
        Logger *logger_;
        const char *step_;
        std::chrono::steady_clock::time_point start_;
    };
} // namespace

OdometryPipeline::OdometryPipeline(const OdometryConfig &config, Logger &logger, ProgressReporter &progress, RunInputs inputs,
                                   RunServices services, RunPaths initial_paths, bool debug)
    : config_(&config), logger_(&logger), progress_(&progress), debug_(debug), inputs_(std::move(inputs)), services_(std::move(services))
{
    state.paths = std::move(initial_paths);
}

bool OdometryPipeline::load()
{
    const OdometryConfig &params = *config_;
    progress_->stage_started(RunStage::Loading);
    progress_->checkpoint();
    if (!inputs_.complete() || !services_.complete() || !inputs_.recording->open(*logger_))
        return false;
    state.paths.working_directory_preview = inputs_.preview->location();
    state.paths.working_directory_cache = inputs_.points->location();
    {
        StepTiming timing(params, *logger_, "load_data");
        if (!load_data(*inputs_.recording, *inputs_.preview, params, points_per_file, imu, debug_, *services_.parallel, *logger_))
            return false;
    }
    {
        StepTiming timing(params, *logger_, "calculate_trajectory");
        calculate_trajectory(imu_trajectory, imu, params.fusionConventionNwu, params.fusionConventionEnu, params.fusionConventionNed,
                             params.ahrs_gain, debug_, params.use_remove_imu_bias_from_first_stationary_scan,
                             params.startup_gyro_bias_nr_samples, params.startup_gyro_bias_max_variance_deg2_s2, *services_.orientation, *logger_);
    }
    StepTiming timing(params, *logger_, "compute_step_1");
    compute_step_1(points_per_file, params, state, imu_trajectory, windows, *inputs_.points, *logger_, *progress_, &imu);
    return true;
}

bool OdometryPipeline::run_odometry()
{
    progress_->stage_started(RunStage::Odometry);
    progress_->checkpoint();
    double ts_failure = 0.0;
    StepTiming timing(*config_, *logger_, "odometry");
    return optimize_windows(windows, *config_, state, ts_failure, debug_, *inputs_.points, *inputs_.preview, *logger_, *progress_, capture_, terms_);
}

OdometrySnapshot OdometryPipeline::snapshot() const
{
    OdometrySnapshot s;
    s.windows = windows;
    s.m_g = state.m_g;
    s.total_length_of_calculated_trajectory = state.total_length_of_calculated_trajectory;
    s.consecutive_distance = state.consecutive_distance;
    return s;
}

bool OdometryPipeline::restore_odometry(const OdometrySnapshot &snapshot, std::string &why)
{
    progress_->stage_started(RunStage::Odometry);
    if (snapshot.windows.size() != windows.size())
    {
        why = "the snapshot has " + std::to_string(snapshot.windows.size()) + " windows, the recording " + std::to_string(windows.size());
        return false;
    }
    for (size_t i = 0; i < windows.size(); i++)
        if (snapshot.windows[i].timestamps != windows[i].timestamps)
        {
            why = "the timestamps of window " + std::to_string(i + 1) + " differ from the recording's";
            return false;
        }
    windows = snapshot.windows;
    state.m_g = snapshot.m_g;
    state.total_length_of_calculated_trajectory = snapshot.total_length_of_calculated_trajectory;
    state.consecutive_distance = snapshot.consecutive_distance;
    log_info(*logger_) << "odometry restored from the snapshot (" << windows.size() << " windows)";
    return true;
}

void OdometryPipeline::post_process()
{
    const OdometryConfig &params = *config_;
    if (params.use_loop_closure)
    {
        progress_->stage_started(RunStage::LoopClosure);
        progress_->checkpoint();
        StepTiming timing(params, *logger_, "loop_closure");
        run_loop_closure(windows, params, *inputs_.points, *logger_);
    }

    if (params.use_consistency && params.num_constistency_iter > 0)
    {
        progress_->stage_started(RunStage::Consistency);
        progress_->checkpoint();
        StepTiming timing(params, *logger_, "consistency");
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
    StepTiming timing(*config_, *logger_, "save");
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
