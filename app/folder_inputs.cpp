#include "folder_inputs.h"

#include "disk_point_store.h"
#include "folder_preview_sink.h"
#include "mandeye_recording_source.h"

#include <filesystem>

namespace fs = std::filesystem;

RunInputs make_folder_inputs(std::vector<std::string> input_file_names, Logger &logger)
{
    RunInputs inputs;
    RecordingFiles files;
    if (!pair_recording_files(std::move(input_file_names), files, logger))
        return inputs;

    const fs::path folder = files.folder;
    inputs.points = std::make_unique<DiskPointStore>(folder / "cache");
    inputs.preview = std::make_unique<FolderPreviewSink>(folder / "preview");
    inputs.recording = std::make_unique<MandeyeFolderSource>(std::move(files));
    return inputs;
}
