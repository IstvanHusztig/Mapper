#include "gui_view_model.h"

#include "folder_inputs.h"
#include "folder_result_sink.h"
#include "run_services.h"
#include "toml_config_repository.h"

#include <exception>
#include <iostream>
#include <vector>

namespace fs = std::filesystem;

GuiViewModel::GuiViewModel(RunNotifier &notifier) : notifier_(&notifier) {}

GuiViewModel::~GuiViewModel()
{
    // as before: closing the program doesn't wait for a run
    if (worker_.joinable())
        worker_.detach();
}

void GuiViewModel::load_default_parameters(const fs::path &file)
{
    if (!fs::is_regular_file(file))
    {
        std::cout << "Default settings not found: '" << file.string() << "' (using built-in defaults)" << std::endl;
        return;
    }
    TomlConfigRepository config_repository;
    if (config_repository.load(file.string(), config_, pipeline_.state.paths, preferences_))
    {
        active_parameters_ = file.string();
        std::cout << "Default settings loaded from: '" << file.string() << "'" << std::endl;
    }
}

bool GuiViewModel::load_parameters(const std::string &file)
{
    bool loaded = false;
    try
    {
        TomlConfigRepository config_repository;
        if (config_repository.load(file, config_, pipeline_.state.paths, preferences_))
        {
            active_parameters_ = file;
            loaded = true;
        }
        std::cout << "Parameters loaded from: " << file << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading TOML file: " << e.what() << std::endl;
    }
    return loaded;
}

bool GuiViewModel::start_run(const std::string &folder)
{
    std::cout << "Selected folder: '" << folder << std::endl;

    if (!fs::exists(folder))
        return false;

    std::vector<std::string> input_file_names;
    for (const auto &entry : fs::directory_iterator(folder))
        if (entry.is_regular_file())
            input_file_names.push_back(entry.path().string());
    if (!input_file_names.empty())
        working_directory_ = fs::path(input_file_names[0]).parent_path().string();

    progress_.reset();
    pipeline_ = OdometryPipeline(config_, console_, progress_, make_folder_inputs(input_file_names, console_), make_run_services(),
                                 pipeline_.state.paths);

    // the progress window is shown from here on; loading runs on the worker thread so it stays visible
    running_.store(true);
    start_time_ = std::chrono::system_clock::now();
    run_parameters_ = active_parameters_;

    // a finished run's thread may still show its message box: let it, as before
    if (worker_.joinable())
        worker_.detach();
    worker_ = std::thread([this, folder]() { process(folder); });
    return true;
}

void GuiViewModel::process(const std::string &folder)
{
    if (!pipeline_.load())
    {
        notifier_->load_failed(folder);
        running_ = false;
        return;
    }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // odometry + post-processing (loop closure, consistency), the same stages as the CLI. An error ends the run with a
    // message instead of std::terminate on this worker thread (the CLI reports it in main and exits 1)
    try
    {
        pipeline_.run_all();
    }
    catch (const std::exception &e)
    {
        log_error(console_) << "run failed: " << e.what();
        running_ = false;
        notifier_->run_failed(e.what());
        return;
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    // writes the run to the next free lio_result_<n> in the recording's folder
    {
        fs::path outwd = get_next_result_path(working_directory_);
        TomlConfigRepository config_repository;
        FolderResultSink sink(outwd, config_repository, console_);
        pipeline_.save(sink, preferences_, elapsed_seconds.count());
    }

    running_ = false;

    notifier_->run_saved(working_directory_, pipeline_.state.total_length_of_calculated_trajectory, elapsed_seconds.count());
}

void GuiViewModel::wait_for_run()
{
    if (worker_.joinable())
        worker_.join();
}

double GuiViewModel::elapsed_seconds() const
{
    return std::chrono::duration<double>(std::chrono::system_clock::now() - start_time_).count();
}

void GuiViewModel::clear_point_store()
{
    // remove cache
    std::cout << "remove cache: '" << pipeline_.state.paths.working_directory_cache << "' START" << std::endl;
    pipeline_.clear_point_store();
    std::cout << "remove cache: '" << pipeline_.state.paths.working_directory_cache << "' FINISHED" << std::endl;
}
