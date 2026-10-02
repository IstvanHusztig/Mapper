#include <doctest/doctest.h>

#include "console_reporting.h"
#include "run_reporting.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
// records every write
struct RecordingLogger : Logger
{
    std::vector<std::pair<LogLevel, std::string>> messages;
    void write(LogLevel level, std::string_view message) override { messages.emplace_back(level, std::string(message)); }
};

std::vector<std::string> lines_of(const std::string &text)
{
    std::vector<std::string> lines;
    std::istringstream in(text);
    for (std::string line; std::getline(in, line);)
        lines.push_back(line);
    return lines;
}
} // namespace

TEST_CASE("LogMessage writes one whole message without the trailing newline")
{
    RecordingLogger logger;
    log_info(logger) << "loaded " << 3 << " files" << std::endl;
    {
        auto msg = log_warning(logger);
        msg << "part one, ";
        msg << "part two\n";
    }
    log_error(logger) << "a\nb";

    REQUIRE(logger.messages.size() == 3);
    CHECK(logger.messages[0] == std::make_pair(LogLevel::Info, std::string("loaded 3 files")));
    CHECK(logger.messages[1] == std::make_pair(LogLevel::Warning, std::string("part one, part two")));
    CHECK(logger.messages[2] == std::make_pair(LogLevel::Error, std::string("a\nb")));
}

TEST_CASE("LogMessage stream state does not carry over to the next message")
{
    RecordingLogger logger;
    log_info(logger) << std::fixed << std::setprecision(10) << 0.5;
    log_info(logger) << 0.5;
    REQUIRE(logger.messages.size() == 2);
    CHECK(logger.messages[0].second == "0.5000000000");
    CHECK(logger.messages[1].second == "0.5");
}

TEST_CASE("ConsoleReporter sends info and debug to out, warnings and errors to err")
{
    std::ostringstream out, err;
    ConsoleReporter console(out, err);
    log_debug(console) << "d";
    log_info(console) << "i";
    log_warning(console) << "w";
    log_error(console) << "e";
    CHECK(out.str() == "d\ni\n");
    CHECK(err.str() == "w\ne\n");
}

TEST_CASE("ConsoleReporter keeps lines whole when written from many threads")
{
    std::ostringstream out, err;
    ConsoleReporter console(out, err);
    constexpr int threads = 8, per_thread = 1000;
    std::vector<std::thread> workers;
    for (int t = 0; t < threads; t++)
        workers.emplace_back([&console, t]
                             {
                                 for (int i = 0; i < per_thread; i++)
                                 {
                                     // built from several pieces, like the per-file loader line
                                     auto msg = log_info(console);
                                     msg << "thread " << t;
                                     msg << " line " << i;
                                     msg << " end";
                                 } });
    for (auto &w : workers)
        w.join();

    const auto lines = lines_of(out.str());
    REQUIRE(lines.size() == threads * per_thread);
    for (const auto &line : lines)
    {
        REQUIRE(line.rfind("thread ", 0) == 0);
        REQUIRE(line.size() >= 4);
        REQUIRE(line.substr(line.size() - 4) == " end");
        REQUIRE(line.find("thread ", 1) == std::string::npos);
    }
}

TEST_CASE("ConsoleReporter stage lines and the loading counter")
{
    std::ostringstream out, err;
    ConsoleReporter console(out, err);
    console.stage_started(RunStage::Loading);
    console.progress(1, 3);
    console.progress(2, 3);
    log_info(console) << "message while counting"; // ends the open counter line first
    console.progress(3, 3);
    console.stage_started(RunStage::Odometry);
    console.progress(1, 10); // no counter outside Loading
    log_info(console) << "window 1";

    CHECK(out.str() == "[stage] Loading\n"
                       "\rrunning iterations: 1/3\rrunning iterations: 2/3\n"
                       "message while counting\n"
                       "\rrunning iterations: 3/3\rrunning iterations: 3 (done)\n"
                       "[stage] Odometry\n"
                       "window 1\n");
    CHECK(err.str().empty());
}
