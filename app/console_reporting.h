#pragma once

// Console adapter for the run-reporting ports (infrastructure).

#include "run_reporting.h"

#include <iostream>
#include <mutex>

// Writes whole messages to the console from any thread: Debug/Info to `out`, Warning/Error to `err`.
// Prints one "[stage] <name>" line per stage, and during Loading an in-place "running iterations: i/n"
// counter that ends with "(done)" when i reaches n, or with a line break before any other output.
// Never pauses.
class ConsoleReporter : public Logger, public ProgressReporter
{
public:
    explicit ConsoleReporter(std::ostream &out = std::cout, std::ostream &err = std::cerr) : out_(&out), err_(&err) {}

    void write(LogLevel level, std::string_view message) override;
    void stage_started(RunStage stage) override;
    void progress(std::size_t done, std::size_t total) override;
    void checkpoint() override {}

private:
    // ends an open counter line; caller holds the lock
    void close_counter();

    std::ostream *out_;
    std::ostream *err_;
    std::mutex mutex_;
    RunStage stage_ = RunStage::Loading;
    bool counter_open_ = false;
};
