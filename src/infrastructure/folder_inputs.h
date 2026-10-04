#pragma once

// Builds the input adapters for a recording folder (composition helper used by the CLI and the GUI).

#include "input_ports.h"

#include <string>
#include <vector>

// Pairs the files (pair_recording_files) and returns a RecordingFolderSource on them, a DiskPointStore on
// <folder>/cache and a FolderPreviewSink on <folder>/preview, where <folder> is the parent of the first sorted
// file. Returns empty inputs, after printing why, if the files can't be paired.
RunInputs make_folder_inputs(std::vector<std::string> input_file_names, Logger &logger);
