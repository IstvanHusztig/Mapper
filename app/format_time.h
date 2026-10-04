#pragma once

#include <iomanip>
#include <sstream>
#include <string>

// Helper function to format time in human-readable format
inline std::string formatTime(double seconds)
{
    if (seconds < 0)
        return "Calculating...";

    int hours = static_cast<int>(seconds / 3600);
    int minutes = static_cast<int>((seconds - hours * 3600) / 60);
    int secs = static_cast<int>(seconds - hours * 3600 - minutes * 60);

    std::ostringstream oss;

    if (hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << hours << "h ";
    }
    if (minutes > 0 || hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << minutes << "m ";
    }

    oss << std::setw(2) << std::setfill('0') << secs << "s";

    return oss.str();
}
