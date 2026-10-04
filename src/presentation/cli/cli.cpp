#include "cli.h"

#include "console_reporting.h"
#include "folder_inputs.h"
#include "folder_result_sink.h"
#include "format_time.h"
#include "odometry_config.h"
#include "odometry_pipeline.h"
#include "run_services.h"
#include "toml_config_repository.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

int run_cli(char *argv[])
{
    // CLI settings: built-in defaults + the given file only (independent of the working directory)
    OdometryConfig config{};
    RunPaths paths;
    GuiPreferences gui;
    TomlConfigRepository config_repository;
    if (!fs::is_regular_file(argv[2]) || !config_repository.load(argv[2], config, paths, gui))
    {
        std::cerr << "Cannot load parameters from: '" << argv[2] << "'" << std::endl;
        return 1;
    }
    std::cout << "Parameters loaded OK from: " << argv[2] << std::endl;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    std::vector<std::string> input_file_names;
    for (const auto &entry : fs::directory_iterator(argv[1]))
    {
        if (!entry.is_directory())
        {
            std::cout << entry.path() << std::endl;
            input_file_names.push_back(entry.path().string());
        }
    }

    ConsoleReporter reporter;
    OdometryPipeline pipeline(config, reporter, reporter, make_folder_inputs(input_file_names, reporter), make_run_services(), paths);
    if (pipeline.load())
        log_info(reporter) << "step_1_done";

    pipeline.run_all();

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "calculations finished computation at "
              << std::ctime(&end_time)
              << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

    FolderResultSink sink(argv[3], config_repository, reporter);
    pipeline.save(sink, gui, elapsed_seconds.count());
    return 0;
}
