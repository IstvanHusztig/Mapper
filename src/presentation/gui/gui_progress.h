#pragma once

// The GUI's ProgressReporter (presentation layer).

#include "run_reporting.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>

// Progress of the current run for the progress window (stage, fraction done, stage start) and Pause/Resume:
// while paused, the run blocks at its next checkpoint. Stages and progress are also forwarded to the console.
class GuiProgress : public ProgressReporter
{
public:
    explicit GuiProgress(ProgressReporter &console) : console_(&console) {}

    // a new run starts with Loading, not paused
    void reset()
    {
        set_paused(false);
        windows_done_.store(0);
        start_stage(RunStage::Loading);
    }

    void stage_started(RunStage stage) override
    {
        {
            // waits for a view that is reading finished windows (read_finished_windows)
            std::lock_guard<std::mutex> lock(windows_mutex_);
            start_stage(stage);
        }
        console_->stage_started(stage);
    }
    void progress(std::size_t done, std::size_t total) override
    {
        fraction_.store(total > 0 ? static_cast<float>(done) / static_cast<float>(total) : 0.0f);
        if (stage_.load() == RunStage::Odometry)
            windows_done_.store(done);
        console_->progress(done, total);
    }
    void checkpoint() override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        resumed_.wait(lock, [this] { return !paused_; });
    }

    void set_paused(bool paused)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            paused_ = paused;
        }
        resumed_.notify_all();
    }
    bool paused()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return paused_;
    }

    RunStage stage() const { return stage_.load(); }
    float fraction() const { return fraction_.load(); }
    // windows the odometry has finished in the current run (their trajectory is final until post-processing)
    std::size_t windows_done() const { return windows_done_.load(); }
    // Calls read(windows_done()) if the odometry is the current stage, under a lock that a stage change takes too.
    // The run announces the next stage before it changes any pose again, so the windows below windows_done() don't
    // change while read runs. Keep read short: the run waits for it at its next stage change.
    template <class Read> void read_finished_windows(Read read)
    {
        std::lock_guard<std::mutex> lock(windows_mutex_);
        if (stage_.load() == RunStage::Odometry)
            read(windows_done_.load());
    }
    // seconds since the current stage started
    double stage_seconds() const
    {
        const auto start = std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(stage_start_.load()));
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    }

private:
    void start_stage(RunStage stage)
    {
        stage_start_.store(std::chrono::steady_clock::now().time_since_epoch().count());
        fraction_.store(0.0f);
        stage_.store(stage);
    }

    ProgressReporter *console_;
    std::atomic<RunStage> stage_{RunStage::Loading};
    std::atomic<float> fraction_{0.0f};
    std::atomic<std::size_t> windows_done_{0};
    std::atomic<std::chrono::steady_clock::rep> stage_start_{std::chrono::steady_clock::now().time_since_epoch().count()};
    std::mutex mutex_;
    std::mutex windows_mutex_; // read_finished_windows against stage changes
    std::condition_variable resumed_;
    bool paused_ = false;
};
