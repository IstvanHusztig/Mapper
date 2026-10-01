#include <doctest/doctest.h>

#include "toml_io.h"

#include <filesystem>
#include <string>
#include <type_traits>
#include <variant>

namespace fs = std::filesystem;

// every field of TomlIO::POINTERS survives a save + load: the defaults are saved, every field of a
// second struct is changed, and loading the file must restore all of them
TEST_CASE("default parameters round-trip through TOML")
{
    TomlIO toml_io;
    LidarOdometryParams defaults;
    RunPaths paths;
    GuiPreferences gui;
    const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_roundtrip.toml";
    REQUIRE(toml_io.SaveParametersToTomlFile(file.string(), defaults, paths, gui));

    LidarOdometryParams loaded;
    for (const auto &[name, member] : toml_io.POINTERS)
        std::visit([&](auto ptr) {
            using T = std::remove_reference_t<decltype(loaded.*ptr)>;
            if constexpr (std::is_same_v<T, bool>)
                loaded.*ptr = !(loaded.*ptr);
            else if constexpr (std::is_same_v<T, std::string>)
                loaded.*ptr += "_changed";
            else
                loaded.*ptr += 3;
        }, member);

    REQUIRE(toml_io.LoadParametersFromTomlFile(file.string(), loaded, paths, gui));
    fs::remove(file);

    for (const auto &[name, member] : toml_io.POINTERS)
    {
        CAPTURE(name);
        std::visit([&](auto ptr) { CHECK(loaded.*ptr == defaults.*ptr); }, member);
    }
}
