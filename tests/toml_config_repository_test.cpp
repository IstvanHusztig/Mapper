#include <doctest/doctest.h>

#include "toml_config_repository.h"

#include <dmapper_version.h>

#include <filesystem>

namespace fs = std::filesystem;

TEST_CASE("TomlConfigRepository saves and loads the settings")
{
    TomlConfigRepository repo;
    const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_repository.toml";

    SUBCASE("defaults")
    {
        OdometryConfig config;
        RunPaths paths;
        GuiPreferences gui;
        REQUIRE(repo.save(file.string(), config, paths, gui));

        OdometryConfig loaded;
        loaded.decimation = 123.0;
        RunPaths loaded_paths;
        GuiPreferences loaded_gui;
        REQUIRE(repo.load(file.string(), loaded, loaded_paths, loaded_gui));
        CHECK(loaded.decimation == config.decimation);
        CHECK(loaded.nr_iter == config.nr_iter);
        CHECK(loaded.use_loop_closure == config.use_loop_closure);
    }

    SUBCASE("changed values of each kind")
    {
        OdometryConfig config;
        config.use_loop_closure = !config.use_loop_closure; // bool
        config.nr_iter = 77;                                // int
        config.decimation = 0.0625;                         // double
        config.config_version = "test-version";             // string
        config.software_version = "9.9.9";                  // replaced by the build's version on loading
        RunPaths paths;
        paths.current_output_dir = "out_dir";
        paths.working_directory_preview = "preview_dir";
        GuiPreferences gui;
        gui.clear_color[0] = 0.25f;
        REQUIRE(repo.save(file.string(), config, paths, gui));

        OdometryConfig loaded;
        RunPaths loaded_paths;
        GuiPreferences loaded_gui;
        REQUIRE(repo.load(file.string(), loaded, loaded_paths, loaded_gui));
        CHECK(loaded.use_loop_closure == config.use_loop_closure);
        CHECK(loaded.nr_iter == 77);
        CHECK(loaded.decimation == 0.0625);
        CHECK(loaded.config_version == "test-version");
        CHECK(loaded.software_version == dmapper_version());
        CHECK(loaded_paths.current_output_dir == "out_dir");
        CHECK(loaded_paths.working_directory_preview == "preview_dir");
        CHECK(loaded_gui.clear_color[0] == 0.25f);
    }

    SUBCASE("missing file")
    {
        OdometryConfig loaded;
        RunPaths loaded_paths;
        GuiPreferences loaded_gui;
        CHECK_FALSE(repo.load((fs::temp_directory_path() / "lidar_odometry_tests_no_such_file.toml").string(), loaded, loaded_paths, loaded_gui));
    }
}
