#pragma once

// The per-term window tables on disk (log_window_terms): <prefix>window_terms.csv (one row per window, state and term:
// cost, residuals, and weight and pull for every motion and direction) and <prefix>window_features.csv (one row per
// window: what the loop knew of it, and its change from guess to result in the twelve motion numbers). Both are opened on
// the first window and flushed after each one, so a run that stops keeps what it had.

#include "window_term_sink.h"

#include <filesystem>
#include <fstream>
#include <string>

class FolderWindowTermSink : public WindowTermSink
{
public:
    FolderWindowTermSink(std::filesystem::path folder, std::string prefix = "") : folder_(std::move(folder)), prefix_(std::move(prefix)) {}
    void write(const WindowTermRecord &record) override;

    std::filesystem::path terms_file() const { return folder_ / (prefix_ + "window_terms.csv"); }
    std::filesystem::path features_file() const { return folder_ / (prefix_ + "window_features.csv"); }

private:
    std::filesystem::path folder_;
    std::string prefix_;
    std::ofstream terms_;
    std::ofstream features_;
};
