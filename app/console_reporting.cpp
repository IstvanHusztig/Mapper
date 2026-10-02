#include "console_reporting.h"

void ConsoleReporter::close_counter(bool done)
{
    if (!counter_open_)
        return;
    if (done)
        *out_ << "\rrunning iterations: " << counter_total_ << " (done)";
    *out_ << std::endl;
    counter_open_ = false;
}

void ConsoleReporter::write(LogLevel level, std::string_view message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_counter(false);
    std::ostream &os = (level == LogLevel::Warning || level == LogLevel::Error) ? *err_ : *out_;
    os << message << std::endl;
}

void ConsoleReporter::stage_started(RunStage stage)
{
    std::lock_guard<std::mutex> lock(mutex_);
    close_counter(true);
    stage_ = stage;
    *out_ << "[stage] " << run_stage_name(stage) << std::endl;
}

void ConsoleReporter::progress(std::size_t done, std::size_t total)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stage_ != RunStage::Loading) // the odometry stage prints one line per window itself
        return;
    *out_ << "\rrunning iterations: " << done << "/" << total << std::flush;
    counter_open_ = true;
    counter_total_ = total;
}
