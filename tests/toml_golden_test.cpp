#include <doctest/doctest.h>

#include "toml_io.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace fs = std::filesystem;

// The TOML written by the app must not change by accident: the saved settings are compared byte for
// byte with golden files in tests/data. Set LO_UPDATE_GOLDEN=1 to rewrite them (only for a deliberate
// format change, in its own commit).
namespace
{
const fs::path repo_dir = LO_REPO_DIR;

std::string read_bytes(const fs::path &p)
{
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void check_golden(OdometryConfig &params, const RunPaths &paths, const GuiPreferences &gui, const std::string &golden_name)
{
    params.build_date = "GOLDEN"; // __DATE__ of the build otherwise
    TomlIO toml_io;
    const fs::path out = fs::temp_directory_path() / ("lidar_odometry_tests_" + golden_name);
    REQUIRE(toml_io.SaveParametersToTomlFile(out.string(), params, paths, gui));
    const std::string written = read_bytes(out);
    fs::remove(out);

    const fs::path golden = repo_dir / "tests" / "data" / golden_name;
    if (const char *update = std::getenv("LO_UPDATE_GOLDEN"); update && std::string(update) == "1")
    {
        std::ofstream(golden, std::ios::binary) << written;
        MESSAGE("rewrote " << golden.string());
    }
    REQUIRE(fs::exists(golden));
    CHECK(written == read_bytes(golden));
}
} // namespace

TEST_CASE("default parameters are saved as the golden TOML")
{
    OdometryConfig params;
    check_golden(params, RunPaths{}, GuiPreferences{}, "default_params.golden.toml");
}

TEST_CASE("settings/l2-vertical.toml is saved as the golden TOML")
{
    OdometryConfig params;
    RunPaths paths;
    GuiPreferences gui;
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile((repo_dir / "settings" / "l2-vertical.toml").string(), params, paths, gui));
    check_golden(params, paths, gui, "l2_vertical.golden.toml");
}

TEST_CASE("settings/livox-vertical.toml is saved as the golden TOML")
{
    OdometryConfig params;
    RunPaths paths;
    GuiPreferences gui;
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile((repo_dir / "settings" / "livox-vertical.toml").string(), params, paths, gui));
    check_golden(params, paths, gui, "livox_vertical.golden.toml");
}

TEST_CASE("every shipped settings file and golden TOML loads without a refused key")
{
    for (const fs::path file : {repo_dir / "settings" / "default_settings.toml", repo_dir / "settings" / "l2-vertical.toml",
                                repo_dir / "settings" / "livox-vertical.toml", repo_dir / "tests" / "data" / "default_params.golden.toml",
                                repo_dir / "tests" / "data" / "l2_vertical.golden.toml", repo_dir / "tests" / "data" / "livox_vertical.golden.toml"})
    {
        CAPTURE(file.string());
        OdometryConfig params;
        RunPaths paths;
        GuiPreferences gui;
        CHECK(TomlIO().LoadParametersFromTomlFile(file.string(), params, paths, gui));
    }
}

TEST_CASE("a settings file with a key the loader would not read is refused")
{
    const auto refused = [](const std::string &body)
    {
        const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_refused.toml";
        std::ofstream(file, std::ios::binary) << body;
        OdometryConfig params;
        RunPaths paths;
        GuiPreferences gui;
        const bool loaded = TomlIO().LoadParametersFromTomlFile(file.string(), params, paths, gui);
        fs::remove(file);
        return !loaded;
    };
    CHECK_FALSE(refused("[imu_translation]\nndt_huber_cutoff = 3.0\n"));
    CHECK_FALSE(refused("[imu_translation]\nndt_huber_cutoff = 3\n")); // an integer is a valid double
    CHECK(refused("[lidar_odometry_control]\nndt_huber_cutoff = 3.0\n"));         // wrong section (run C, 2026-11-16)
    CHECK(refused("[imu_translation]\nndt_information_scale = \"0.1\"\n"));      // wrong type: was read as 0
    CHECK(refused("[imu_translation]\nno_such_setting = 1.0\n"));                // unknown key
    CHECK(refused("[no_such_section]\nnr_iter = 3\n"));                          // unknown section
    CHECK(refused("[lidar_odometry_control]\nnr_iter = 3.5\n"));                 // an int given as a double
    CHECK(refused("[in_out_params_indoor]\nresolution_W = 0.3\n"));              // unknown grid key
}

// settings removed from dMapper (RETIRED_KEYS in toml_io.cpp): a file written before the removal still loads when the
// setting asks for the behaviour that remains, and is refused when it asks for the removed one
TEST_CASE("a retired setting is ignored at the value the code kept and refused otherwise")
{
    const auto loads = [](const std::string &body)
    {
        const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_retired.toml";
        std::ofstream(file, std::ios::binary) << body;
        OdometryConfig params;
        RunPaths paths;
        GuiPreferences gui;
        const bool loaded = TomlIO().LoadParametersFromTomlFile(file.string(), params, paths, gui);
        fs::remove(file);
        return loaded;
    };
    CHECK(loads("[consistency]\nuse_mutliple_gaussian = false\nconsistency_min_distance = 0.4\n"));
    CHECK(loads("[imu_translation]\ndegenerate_projection_all_point_normals = false\nndt_unmatched_point_cost = 0.0\n"));
    CHECK(loads("[imu_translation]\nndt_unmatched_point_cost = 0\n")); // an integer 0 is 0
    CHECK(loads("[robust_lidar_odometry]\nuse_robust_and_accurate_lidar_odometry = false\ndistance_bucket = 0.2\n"));
    CHECK(loads("[imu_translation]\nimu_carry_joint_covariance = false\nimu_accel_window_sigma_mps2 = 0.0\n"));
    CHECK(loads("[loop_closure]\nloop_closure_kernel_scale = 0.0\n"));
    CHECK_FALSE(loads("[consistency]\nuse_mutliple_gaussian = true\n"));
    CHECK_FALSE(loads("[imu_translation]\ncarried_covariance_normal_information_only = true\n"));
    CHECK_FALSE(loads("[imu_translation]\nndt_unmatched_point_cost = 16.0\n"));
    CHECK_FALSE(loads("[robust_lidar_odometry]\nuse_robust_and_accurate_lidar_odometry = true\n"));
    CHECK_FALSE(loads("[imu_translation]\nimu_carry_joint_covariance = true\n"));
    CHECK_FALSE(loads("[imu_translation]\nimu_accel_window_sigma_mps2 = 0.05\n"));
    CHECK_FALSE(loads("[loop_closure]\nloop_closure_kernel_scale = 0.5\n"));
    CHECK_FALSE(loads("[robust_lidar_odometry]\nno_such_setting = 1.0\n")); // a retired section accepts only its retired keys
}
