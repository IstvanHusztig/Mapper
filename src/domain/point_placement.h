#ifndef _POINT_PLACEMENT_H_
#define _POINT_PLACEMENT_H_

// points_to_vector without the rest of export_laz.h (which pulls in session.h, point_cloud.h, GL and LASzip);
// export_laz.h includes this header, so its users still see it

#include <Eigen/Eigen>
#include <vector>
#include <structures.h>

inline void points_to_vector(
    const std::vector<Point3Di> points, std::vector<Eigen::Affine3d> &trajectory, double threshold_output_filter, std::vector<int> *index_poses,
    std::vector<Eigen::Vector3d> &pointcloud, std::vector<unsigned short> &intensity, std::vector<double> &timestamps, bool use_first_pose, bool save_index_pose)
{
    Eigen::Affine3d m_pose = trajectory[0].inverse();
    for (const auto &org_p : points)
    {
        Point3Di p = org_p;
        if (p.point.norm() > threshold_output_filter)
        {
            if (use_first_pose)
            {
                p.point = m_pose * (trajectory[org_p.index_pose] * org_p.point);
            }
            else
            {
                p.point = trajectory[org_p.index_pose] * org_p.point;
            }
            pointcloud.push_back(p.point);
            intensity.push_back(p.intensity);
            timestamps.push_back(p.timestamp);
            if (save_index_pose)
            {
                if (index_poses)
                {
                    index_poses->push_back(org_p.index_pose);
                }
            }
        }
    }
}

#endif
