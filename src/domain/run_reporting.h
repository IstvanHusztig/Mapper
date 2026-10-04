#pragma once

// Ports through which a processing run reports what it is doing (application layer, standard library only).
//   Logger           - whole console messages with a level
//   ProgressReporter - the current stage, progress within it, and pause checkpoints

#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>

enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

class Logger
{
public:
    virtual ~Logger() = default;
    // one whole message without a trailing newline (it may contain inner newlines); safe to call from any thread
    virtual void write(LogLevel level, std::string_view message) = 0;
};

enum class RunStage
{
    Loading,
    Odometry,
    LoopClosure,
    Consistency,
    Saving
};

inline const char *run_stage_name(RunStage stage)
{
    switch (stage)
    {
    case RunStage::Loading: return "Loading";
    case RunStage::Odometry: return "Odometry";
    case RunStage::LoopClosure: return "Loop closure";
    case RunStage::Consistency: return "Consistency";
    case RunStage::Saving: return "Saving";
    }
    return "";
}

class ProgressReporter
{
public:
    virtual ~ProgressReporter() = default;
    virtual void stage_started(RunStage stage) = 0;
    // done of total units of the current stage
    virtual void progress(std::size_t done, std::size_t total) = 0;
    // a safe point to pause: returns when the run may go on
    virtual void checkpoint() = 0;
};

// Builds one message with stream syntax and writes it to the logger when it goes out of scope:
//   log_info(logger) << "loaded " << n << " files";
// A trailing newline is dropped (the logger ends the line). Each message has its own stream, so
// manipulators such as std::setprecision never carry over to the next message.
class LogMessage
{
public:
    LogMessage(Logger &logger, LogLevel level) : logger_(&logger), level_(level) {}
    LogMessage(const LogMessage &) = delete;
    LogMessage &operator=(const LogMessage &) = delete;
    ~LogMessage()
    {
        std::string text = stream_.str();
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
            text.pop_back();
        logger_->write(level_, text);
    }

    template <typename T>
    LogMessage &operator<<(const T &value)
    {
        stream_ << value;
        return *this;
    }
    // manipulators (std::endl, std::fixed, ...)
    LogMessage &operator<<(std::ostream &(*manip)(std::ostream &))
    {
        if (manip == static_cast<std::ostream &(*)(std::ostream &)>(std::endl<char, std::char_traits<char>>))
            stream_ << '\n';
        else
            manip(stream_);
        return *this;
    }
    LogMessage &operator<<(std::ios_base &(*manip)(std::ios_base &))
    {
        manip(stream_);
        return *this;
    }

private:
    Logger *logger_;
    LogLevel level_;
    std::ostringstream stream_;
};

inline LogMessage log_debug(Logger &logger) { return LogMessage(logger, LogLevel::Debug); }
inline LogMessage log_info(Logger &logger) { return LogMessage(logger, LogLevel::Info); }
inline LogMessage log_warning(Logger &logger) { return LogMessage(logger, LogLevel::Warning); }
inline LogMessage log_error(Logger &logger) { return LogMessage(logger, LogLevel::Error); }
