#pragma once

// ConfigRepository adapter: settings as TOML files (infrastructure), through TomlIO.

#include "output_ports.h"

class TomlConfigRepository : public ConfigRepository
{
public:
    bool load(const std::string &location, OdometryConfig &config, RunPaths &paths, GuiPreferences &gui) override;
    bool save(const std::string &location, const OdometryConfig &config, const RunPaths &paths, const GuiPreferences &gui) override;
};
