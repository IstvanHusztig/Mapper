#pragma once

// File layout of a recording folder: LiDAR chunks (lidar*.laz/*.las) paired with IMU chunks (imu*.csv).

#include "run_reporting.h"

#include <string>
#include <vector>

struct RecordingFiles
{
    // sorted by name; pair i is laz[i] + csv[i] (the shorter list sets the number of pairs)
    std::vector<std::string> laz;
    std::vector<std::string> csv;
    // folder of the recording: the parent of the first sorted file name
    std::string folder;

    std::size_t pair_count() const { return laz.size() < csv.size() ? laz.size() : csv.size(); }
};

// Sorts the names, keeps *.laz/*.las and *.csv (case-insensitive), and checks that every file has a partner
// with the same last four characters of the stem. A missing partner in the middle fails; on the last file
// it only prints a warning. Needs at least 2 names and an existing folder. Prints why it fails.
bool pair_recording_files(std::vector<std::string> input_file_names, RecordingFiles &out, Logger &logger);
