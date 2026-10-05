#pragma once

// dMapper's version, set in the top-level CMakeLists.txt (DMAPPER_VERSION_MAJOR / _MINOR / _PATCH). The result files
// carry it in their names and contents; the GUI shows major.minor in its title.

#include <string>

#if !defined(DMAPPER_VERSION_MAJOR) || !defined(DMAPPER_VERSION_MINOR) || !defined(DMAPPER_VERSION_PATCH)
#error "dMapper's version is not defined: define DMAPPER_VERSION_MAJOR/MINOR/PATCH in the top-level CMakeLists.txt."
#endif

// "major.minor.patch", e.g. "0.1.0"
inline std::string dmapper_version()
{
    return std::to_string(DMAPPER_VERSION_MAJOR) + "." + std::to_string(DMAPPER_VERSION_MINOR) + "." +
           std::to_string(DMAPPER_VERSION_PATCH);
}

// "major.minor", e.g. "0.1"
inline std::string dmapper_version_short()
{
    return std::to_string(DMAPPER_VERSION_MAJOR) + "." + std::to_string(DMAPPER_VERSION_MINOR);
}
