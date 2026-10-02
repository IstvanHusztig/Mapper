#include "toml_config_repository.h"

#include "toml_io.h"

bool TomlConfigRepository::load(const std::string &location, OdometryConfig &config, RunPaths &paths, GuiPreferences &gui)
{
    TomlIO toml_io;
    return toml_io.LoadParametersFromTomlFile(location, config, paths, gui);
}

bool TomlConfigRepository::save(const std::string &location, const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui)
{
    TomlIO toml_io;
    return toml_io.SaveParametersToTomlFile(location, config, paths, gui);
}
