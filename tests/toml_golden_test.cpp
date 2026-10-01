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

TEST_CASE("settings/lidar-vertical.toml is saved as the golden TOML")
{
    OdometryConfig params;
    RunPaths paths;
    GuiPreferences gui;
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile((repo_dir / "settings" / "lidar-vertical.toml").string(), params, paths, gui));
    check_golden(params, paths, gui, "lidar_vertical.golden.toml");
}
