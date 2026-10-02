#include <doctest/doctest.h>

#include "disk_point_store.h"

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace
{
std::vector<Point3Di> sample_points(int seed)
{
    std::vector<Point3Di> points;
    for (int i = 0; i < 5; i++)
    {
        Point3Di p{};
        p.point = Eigen::Vector3d(seed + i * 0.25, -i * 1.5, 0.125 * seed);
        p.timestamp = 1000.0 * seed + i;
        p.intensity = 10.0f * i;
        p.index_pose = i;
        p.lidarid = static_cast<uint8_t>(seed);
        p.index_point = 100 * seed + i;
        points.push_back(p);
    }
    return points;
}

bool same(const std::vector<Point3Di> &a, const std::vector<Point3Di> &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
        if (a[i].point != b[i].point || a[i].timestamp != b[i].timestamp || a[i].intensity != b[i].intensity ||
            a[i].index_pose != b[i].index_pose || a[i].lidarid != b[i].lidarid || a[i].index_point != b[i].index_point)
            return false;
    return true;
}
} // namespace

TEST_CASE("DiskPointStore keeps each point set under today's cache file names")
{
    const fs::path folder = fs::temp_directory_path() / "lidar_odometry_tests_cache";
    fs::remove_all(folder);
    DiskPointStore store(folder);
    CHECK(store.location() == folder.string());

    const PointSet sets[] = {PointSet::WindowOriginal, PointSet::WindowIntermediate, PointSet::SessionChunk};
    for (int k = 0; k < 3; k++)
        REQUIRE(store.save(sets[k], 7, sample_points(k + 1)));

    CHECK(fs::is_regular_file(folder / "original_points_7.cache"));
    CHECK(fs::is_regular_file(folder / "intermediate_points_7.cache"));
    CHECK(fs::is_regular_file(folder / "session_chunk_7.cache"));

    for (int k = 0; k < 3; k++)
    {
        std::vector<Point3Di> loaded;
        REQUIRE(store.load(sets[k], 7, loaded));
        CHECK(same(loaded, sample_points(k + 1)));
    }

    std::vector<Point3Di> missing;
    CHECK_FALSE(store.load(PointSet::WindowOriginal, 8, missing));

    store.clear();
    CHECK_FALSE(fs::exists(folder));
}
