#include "window_equations.h"

#include <algorithm>

WindowUnknowns layout_window_unknowns(const Window &window, const MotionModelRotationOptions &rotation_options,
                                      const MotionModelTranslationImuOptions &translation_imu_options)
{
    WindowUnknowns u;
    u.n_nodes = static_cast<int>(window.poses.size());
    u.have_dt = window.timestamps.size() == window.poses.size();
    u.time_scaled_sigma = rotation_options.time_scaled_sigma && u.have_dt;
    u.estimate_bias = rotation_options.estimate_bias && u.have_dt;
    u.n_pose_params = u.n_nodes * 6;
    u.acc_term = translation_imu_options.enabled && u.have_dt && translation_imu_options.acc &&
                 translation_imu_options.acc->size() == window.poses.size();
    u.n_params = u.n_pose_params + (u.estimate_bias ? 3 : 0) + (u.acc_term ? 9 : 0);
    u.ib = u.n_pose_params;
    u.io = u.n_pose_params + (u.estimate_bias ? 3 : 0);
    u.iv = u.io + 3;
    u.ie = u.iv + 3;

    if (u.acc_term)
    {
        const int n = u.n_nodes;
        u.anchors.push_back(0);
        for (int k = 1; k < n; k++)
            if (window.timestamps[k].first - window.timestamps[u.anchors.back()].first >= translation_imu_options.span_s)
                u.anchors.push_back(k);
        for (size_t j = 1; j + 1 < u.anchors.size(); j++)
            u.pose_band_offset = std::max(u.pose_band_offset, (u.anchors[j + 1] - u.anchors[j - 1]) * 6 + 5);
        if (u.anchors.size() > 1)
            u.pose_band_offset = std::max(u.pose_band_offset, u.anchors[1] * 6 + 5);
    }
    // the end velocity is tied to the last two anchors, with the accelerometer from their midpoint to the last node
    if (u.acc_term && u.anchors.size() > 1)
        u.pose_band_offset = std::max(u.pose_band_offset, (u.n_nodes - 1 - u.anchors[u.anchors.size() - 2]) * 6 + 5);
    return u;
}
