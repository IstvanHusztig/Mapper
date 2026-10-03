#include "window_steps.h"

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
