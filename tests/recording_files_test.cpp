#include <doctest/doctest.h>

#include "laz_export.h"
#include "recording_files.h"
#include "recording_folder_source.h"

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
struct NullLogger : Logger
{
    std::vector<std::string> messages;
    void write(LogLevel, std::string_view message) override { messages.emplace_back(message); }
};

// a fresh folder; pairing only looks at the names, so the files are never created
fs::path test_folder()
{
    const fs::path folder = fs::temp_directory_path() / "lidar_odometry_tests_recording";
    fs::create_directories(folder);
    return folder;
}

std::vector<std::string> names(const fs::path &folder, std::vector<std::string> files)
{
    for (auto &f : files)
        f = (folder / f).string();
    return files;
}
} // namespace

TEST_CASE("pair_recording_files pairs LiDAR and IMU chunks in sorted order")
{
    const fs::path folder = test_folder();
    NullLogger logger;
    RecordingFiles files;
    // unsorted input, upper-case extension, an unrelated file
    REQUIRE(pair_recording_files(names(folder, {"lidar0002.laz", "imu0002.csv", "imu0001.csv", "lidar0001.LAZ", "notes.txt"}), files, logger));
    CHECK(files.pair_count() == 2);
    CHECK(fs::path(files.laz[0]).filename() == "lidar0001.LAZ");
    CHECK(fs::path(files.laz[1]).filename() == "lidar0002.laz");
    CHECK(fs::path(files.csv[0]).filename() == "imu0001.csv");
    CHECK(fs::path(files.csv[1]).filename() == "imu0002.csv");
    CHECK(files.folder == folder.string());
}

TEST_CASE("pair_recording_files fails on a missing partner in the middle")
{
    const fs::path folder = test_folder();
    NullLogger logger;
    RecordingFiles files;
    CHECK_FALSE(pair_recording_files(names(folder, {"lidar0001.laz", "lidar0002.laz", "lidar0003.laz", "imu0001.csv", "imu0003.csv"}), files, logger));
    CHECK_FALSE(logger.messages.empty());
}

TEST_CASE("pair_recording_files ignores a missing partner of the last file")
{
    const fs::path folder = test_folder();
    NullLogger logger;
    RecordingFiles files;
    REQUIRE(pair_recording_files(names(folder, {"lidar0001.laz", "lidar0002.laz", "lidar0003.laz", "imu0001.csv", "imu0002.csv"}), files, logger));
    CHECK(files.laz.size() == 3);
    CHECK(files.csv.size() == 2);
    CHECK(files.pair_count() == 2);
}

TEST_CASE("pair_recording_files needs at least two files and an existing folder")
{
    const fs::path folder = test_folder();
    NullLogger logger;
    RecordingFiles files;
    CHECK_FALSE(pair_recording_files(names(folder, {"lidar0001.laz"}), files, logger));
    CHECK_FALSE(pair_recording_files({}, files, logger));
    const fs::path missing = folder / "does_not_exist";
    CHECK_FALSE(pair_recording_files(names(missing, {"lidar0001.laz", "imu0001.csv"}), files, logger));
}

TEST_CASE("loading points: the xy filter drops points without a return, written as (0, 0, 0)")
{
    // some sensors write a missing return as a point at the origin; the inner xy filter must reject it
    const fs::path file = test_folder() / "origin_points.laz";
    const std::vector<Eigen::Vector3d> points = {{0.0, 0.0, 0.0}, {1.0, 0.5, 0.2}, {0.0, 0.0, 0.0}, {0.1, 0.1, 2.0}, {30.0, 0.0, 0.0}};
    REQUIRE(exportLaz(file.string(), points, {10, 20, 30, 40, 50}, {1.0, 1.1, 1.2, 1.3, 1.4}));
    NullLogger logger;
    const std::vector<Point3Di> loaded = load_point_cloud(file.string(), false, 0.35, 25.0, Eigen::Vector3d::Zero(), logger);
    REQUIRE(loaded.size() == 1); // only (1, 0.5, 0.2): the origin and (0.1, 0.1) are inside 0.35 m, (30, 0) is beyond 25 m
    CHECK(loaded[0].point.x() == doctest::Approx(1.0).epsilon(1e-3));
    CHECK(loaded[0].point.y() == doctest::Approx(0.5).epsilon(1e-3));
    fs::remove(file);
}
