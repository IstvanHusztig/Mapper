#include "console_reporting.h"

void ConsoleReporter::close_counter()
{
    if (!counter_open_)
        return;
    *out_ << std::endl;
    counter_open_ = false;
}

void ConsoleReporter::write(LogLevel level, std::string_view message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_counter();
    std::ostream &os = (level == LogLevel::Warning || level == LogLevel::Error) ? *err_ : *out_;
    os << message << std::endl;
}

void ConsoleReporter::stage_started(RunStage stage)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_counter();
    stage_ = stage;
    *out_ << "[stage] " << run_stage_name(stage) << std::endl;
}

void ConsoleReporter::progress(std::size_t done, std::size_t total)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stage_ != RunStage::Loading) // the odometry stage prints one line per window itself
        return;
    if (done >= total)
    {
        *out_ << "\rrunning iterations: " << total << " (done)" << std::endl;
        counter_open_ = false;
        return;
    }
    *out_ << "\rrunning iterations: " << done << "/" << total << std::flush;
    counter_open_ = true;
}
