#include "window_steps.h"

#include <algorithm>

void place_window_after(Window &window, const Eigen::Affine3d &start)
{
    Eigen::Affine3d m_last = start;
    auto tmp = window.poses;

    window.poses[0] = m_last;
    for (int k = 1; k < tmp.size(); k++)
    {
        Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
        m_last = m_last * m_update;
        window.poses[k] = m_last;
    }
}

PreviousVelocity estimate_previous_velocity(const Window &previous, const Window &window, int span_nodes, double max_speed_mps)
{
    PreviousVelocity r;
    const auto &prev_tr = previous.poses;
    const auto &prev_ts = previous.timestamps;
    const int n = prev_tr.size();
    const int k = std::min(span_nodes, n - 1);
    if (k > 0 && prev_ts.size() == prev_tr.size())
    {
        const double dt = prev_ts[n - 1].first - prev_ts[n - 1 - k].first;
        if (dt > 0.0)
        {
            r.velocity = (prev_tr[n - 1].translation() - prev_tr[n - 1 - k].translation()) / dt;
            r.use = true;
            const auto &ts_i = window.timestamps;
            if (!ts_i.empty())
                r.age_s = std::max(ts_i[0].first - 0.5 * (prev_ts[n - 1].first + prev_ts[n - 1 - k].first), 0.0);
        }
    }

    if (r.velocity.norm() > max_speed_mps)
    {
        r.dropped = true;
        r.speed = r.velocity.norm();
        r.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        r.use = false;
    }
    return r;
}
