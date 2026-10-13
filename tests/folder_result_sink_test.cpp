#include <doctest/doctest.h>

#include "folder_result_sink.h"
#include "synthetic_recording.h"
#include "toml_config_repository.h"

#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

TEST_CASE("FolderResultSink writes the chunk poses with full precision")
{
    // a chunk pose about a kilometre from the origin: with the stream's default 6 significant digits its translation
    // came back rounded to 1234.57 (5 mm off); it must round-trip exactly
    const fs::path folder = fs::temp_directory_path() / "lidar_odometry_tests_result_sink";
    fs::remove_all(folder);
    fs::create_directories(folder);
    TomlConfigRepository repository;
    synthetic::NullLogger logger;
    FolderResultSink sink(folder, repository, logger);

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = Eigen::AngleAxisd(0.123456789, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(1234.56789012, -987.654321098, 12.3456789);
    sink.chunk_poses({pose});

    for (const char *name : {"session_poses.mrp", "session_ini_poses.mri"})
    {
        std::ifstream in(folder / name);
        REQUIRE(in.good());
        std::string count, file_name;
        std::getline(in, count);
        std::getline(in, file_name);
        Eigen::Matrix<double, 3, 4> m;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 4; c++)
                in >> m(r, c);
        CHECK(m.block<3, 3>(0, 0) == pose.linear());
        CHECK(m.col(3) == pose.translation());
    }
    fs::remove_all(folder);
}
