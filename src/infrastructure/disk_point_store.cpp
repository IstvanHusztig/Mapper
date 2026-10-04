#include "disk_point_store.h"

namespace fs = std::filesystem;

fs::path DiskPointStore::file_name(PointSet set, std::size_t index) const
{
    const char *prefix = "";
    switch (set)
    {
    case PointSet::WindowOriginal: prefix = "original_points_"; break;
    case PointSet::WindowIntermediate: prefix = "intermediate_points_"; break;
    case PointSet::SessionChunk: prefix = "session_chunk_"; break;
    }
    return folder_ / (prefix + std::to_string(index) + ".cache");
}

bool DiskPointStore::save(PointSet set, std::size_t index, const std::vector<Point3Di> &points)
{
    if (!fs::exists(folder_))
        fs::create_directory(folder_);
    return save_vector_data(file_name(set, index).string(), points);
}

bool DiskPointStore::load(PointSet set, std::size_t index, std::vector<Point3Di> &points)
{
    return load_vector_data(file_name(set, index).string(), points);
}

void DiskPointStore::clear()
{
    std::error_code ec;
    fs::remove_all(folder_, ec);
}
