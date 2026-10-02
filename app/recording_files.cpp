#include "recording_files.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace fs = std::filesystem;

bool pair_recording_files(std::vector<std::string> input_file_names, RecordingFiles &out, Logger &logger)
{
    std::sort(input_file_names.begin(), input_file_names.end());

    std::vector<std::string> csv_files, laz_files;

    // --- Separate files by type
    for (const auto& fileName : input_file_names)
    {
        std::string ext = fs::path(fileName).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".csv")
            csv_files.push_back(fileName);
        else if ((ext == ".laz") || (ext == ".las"))
            laz_files.push_back(fileName);
    }

    auto get4index = [](const std::string& path) -> std::string {
        auto s = fs::path(path).stem().string();
        return s.size() > 4 ? s.substr(s.size() - 4) : s;
        };

    for (size_t i = 0; i < laz_files.size(); i++)
    {
        std::string lfs = get4index(laz_files[i]);

        bool found = false;
        for (size_t j = 0; j < csv_files.size(); j++)
        {
            std::string cfs = get4index(csv_files[j]);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            log_info(logger) << "\n!!! there is no CSV file for: " << laz_files[i] << " !!! \n\n";

            if (i < (laz_files.size() - 1))
            {
                log_info(logger) << "!!! can't continue with missmatched middle file pairs.. !!!\n\n";
                return false;
            }
        }
    }

    for (size_t i = 0; i < csv_files.size(); i++)
    {
        std::string lfs = get4index(csv_files[i]);

        bool found = false;
        for (size_t j = 0; j < laz_files.size(); j++)
        {
            std::string cfs = get4index(laz_files[j]);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            log_info(logger) << "\n!!! there is no LAZ file for: " << csv_files[i] << " !!! \n\n";

            if (i < (csv_files.size() - 1))
            {
                log_info(logger) << "!!! can't continue with missmatched middle file pairs.. !!!\n\n";
                return false;
            }
        }
    }

    if (input_file_names.size() < 2)
    {
        log_info(logger) << "Not enough files to continue: " << input_file_names.size();
        return false;
    }

    std::string working_directory = fs::path(input_file_names.front()).parent_path().string();

    if (laz_files.size() != csv_files.size())
        log_info(logger) << "!!! file number issue. continuing but ignoring missmatched last file pair.. !!!\n";

    // check if folder exists!
    if (!fs::exists(working_directory))
    {
        log_error(logger) << "Folder '" << working_directory << "' does not exist.\n";
        return false;
    }

    out.laz = std::move(laz_files);
    out.csv = std::move(csv_files);
    out.folder = working_directory;
    return true;
}
