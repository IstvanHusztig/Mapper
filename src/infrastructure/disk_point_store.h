#pragma once

// PointStore adapter: binary .cache files in one folder (infrastructure).

#include "input_ports.h"

#include <filesystem>

// original_points_<i>.cache, intermediate_points_<i>.cache, session_chunk_<i>.cache in `folder`;
// the folder is created on the first save
class DiskPointStore : public PointStore
{
public:
    explicit DiskPointStore(std::filesystem::path folder) : folder_(std::move(folder)) {}

    bool save(PointSet set, std::size_t index, const std::vector<Point3Di> &points) override;
    bool load(PointSet set, std::size_t index, std::vector<Point3Di> &points) override;
    void clear() override;
    std::string location() const override { return folder_.string(); }

    std::filesystem::path file_name(PointSet set, std::size_t index) const;

private:
    std::filesystem::path folder_;
};
