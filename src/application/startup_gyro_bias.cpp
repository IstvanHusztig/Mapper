#include "startup_gyro_bias.h"

#include "angle_units.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace
{
    double median(std::vector<double> v)
    {
        const size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    }

    std::string axis_name(int a) { return a == 0 ? "x" : (a == 1 ? "y" : "z"); }
}

StartupBiasOptions get_startup_bias_options(const OdometryConfig &params)
{
    StartupBiasOptions o;
    o.window_s = params.imu_startup_bias_window_s;
    o.max_spread_deg_s = params.imu_startup_bias_max_spread_deg_s;
    o.max_tilt_deg = params.imu_startup_bias_max_tilt_deg;
    o.max_noise_ratio = params.imu_startup_bias_max_noise_ratio;
    o.arw_deg_sqrt_s = Eigen::Vector3d(params.gyro_arw_x_deg_sqrt_s, params.gyro_arw_y_deg_sqrt_s, params.gyro_arw_z_deg_sqrt_s);
    return o;
}

StartupBias startup_gyro_bias(const Imu &imu, const StartupBiasOptions &options)
{
    StartupBias out;
    if (imu.empty())
    {
        out.reason = "no IMU samples";
        return out;
    }
    const double t0 = imu.front().timestamps.first;
    size_t n = 0;
    while (n < imu.size() && imu[n].timestamps.first - t0 < options.window_s)
        n++;
    if (n < 40)
    {
        out.reason = "only " + std::to_string(n) + " samples in the first " + std::to_string(options.window_s) + " s";
        return out;
    }
    const double duration = imu[n - 1].timestamps.first - t0;
    const double rate = duration > 0.0 ? static_cast<double>(n - 1) / duration : 0.0;

    // the gyro in degrees as calculate_trajectory has always taken it (narrowed to float first)
    std::vector<double> g[3];
    for (int a = 0; a < 3; a++)
        g[a].reserve(n);
    for (size_t i = 0; i < n; i++)
        for (int a = 0; a < 3; a++)
            g[a].push_back(static_cast<float>(imu[i].gyro[a] * RAD_TO_DEG));

    Eigen::Vector3d med, spread, ratio;
    const size_t q = n / 4;
    for (int a = 0; a < 3; a++)
    {
        med[a] = median(g[a]);
        double lo = 1e300, hi = -1e300;
        for (int k = 0; k < 4; k++)
        {
            const auto first = g[a].begin() + k * q, last = k == 3 ? g[a].end() : g[a].begin() + (k + 1) * q;
            const double m = median(std::vector<double>(first, last));
            lo = std::min(lo, m);
            hi = std::max(hi, m);
        }
        spread[a] = hi - lo;
        double mean = 0.0, var = 0.0;
        for (double v : g[a])
            mean += v;
        mean /= static_cast<double>(n);
        for (double v : g[a])
            var += (v - mean) * (v - mean);
        const double noise = options.arw_deg_sqrt_s[a] * std::sqrt(rate);
        ratio[a] = noise > 0.0 ? std::sqrt(var / static_cast<double>(n)) / noise : 0.0;
    }
    Eigen::Vector3d acc_first = Eigen::Vector3d::Zero(), acc_last = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < q; i++)
    {
        acc_first += imu[i].acc.cast<double>();
        acc_last += imu[n - 1 - i].acc.cast<double>();
    }
    const double tilt = (acc_first.norm() > 0.0 && acc_last.norm() > 0.0)
                            ? std::acos(std::clamp(acc_first.normalized().dot(acc_last.normalized()), -1.0, 1.0)) * RAD_TO_DEG
                            : 0.0;

    std::ostringstream measured;
    measured.precision(3);
    measured << "first " << duration << " s, " << n << " samples: median " << med.x() << " " << med.y() << " " << med.z()
             << " deg/s, quarter spread " << spread.x() << " " << spread.y() << " " << spread.z() << " deg/s, tilt " << tilt
             << " deg, scatter " << ratio.x() << " " << ratio.y() << " " << ratio.z() << " x noise";
    std::string failed;
    for (int a = 0; a < 3 && failed.empty(); a++)
        if (spread[a] > options.max_spread_deg_s)
            failed = "turn: the quarter medians of axis " + axis_name(a) + " differ by more than " + std::to_string(options.max_spread_deg_s) + " deg/s";
    if (failed.empty() && tilt > options.max_tilt_deg)
        failed = "tilt: the gravity direction turns by more than " + std::to_string(options.max_tilt_deg) + " deg";
    for (int a = 0; a < 3 && failed.empty(); a++)
        if (ratio[a] > options.max_noise_ratio)
            failed = "shaking: the scatter of axis " + axis_name(a) + " is more than " + std::to_string(options.max_noise_ratio) + " x the sensor noise";
    out.accepted = failed.empty();
    if (out.accepted)
        out.bias_deg_s = med;
    out.reason = measured.str() + (failed.empty() ? "" : "; rejected (" + failed + ")");
    return out;
}
