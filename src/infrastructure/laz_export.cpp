#include "laz_export.h"

#include <laszip/laszip_api.h>

#include <cstdio>
#include <iostream>
#include <limits>

bool exportLaz(const std::string &filename,
                      const std::vector<Eigen::Vector3d> &pointcloud,
                      const std::vector<unsigned short> &intensity,
                      const std::vector<double> &timestamps,
                      double offset_x, double offset_y, double offset_alt)
{
    double min_ts = std::numeric_limits<double>::max();
    double max_ts = std::numeric_limits<double>::lowest();
    int number_of_points_with_timestamp_eq_0 = 0;

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d _max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d _min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

    for (auto &p : pointcloud)
    {
        if (p.x() < _min.x())
        {
            _min.x() = p.x();
        }
        if (p.y() < _min.y())
        {
            _min.y() = p.y();
        }
        if (p.z() < _min.z())
        {
            _min.z() = p.z();
        }

        if (p.x() > _max.x())
        {
            _max.x() = p.x();
        }
        if (p.y() > _max.y())
        {
            _max.y() = p.y();
        }
        if (p.z() > _max.z())
        {
            _max.z() = p.z();
        }
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = _max.x() + offset_x;
    header->min_x = _min.x() + offset_x;
    header->max_y = _max.y() + offset_y;
    header->min_y = _min.y() + offset_y;
    header->max_z = _max.z() + offset_alt;
    header->min_z = _min.z() + offset_alt;

    header->x_offset = offset_x;
    header->y_offset = offset_y;
    header->z_offset = offset_alt;

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr, "writing %scompressed file '%s'\n", (compress ? "" : "un"), filename.c_str());

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < pointcloud.size(); i++)
    {
        const auto &p = pointcloud[i];
        point->intensity = intensity[i];
        point->gps_time = timestamps[i] * 1e9;

        if (point->gps_time < min_ts)
        {
            min_ts = point->gps_time;
        }

        if (point->gps_time > max_ts)
        {
            max_ts = point->gps_time;
        }

        if (point->gps_time == 0.0)
        {
            number_of_points_with_timestamp_eq_0++;
        }
        // std::setprecision(20);
        // std::cout << "point->gps_time " << point->gps_time << " " << timestamps[i] << " " << timestamps[i] * 1e9 << std::endl;

        p_count++;
        coordinates[0] = p.x() + offset_x;
        coordinates[1] = p.y() + offset_y;
        coordinates[2] = p.z() + offset_alt;
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "min_ts " << min_ts << std::endl;
    std::cout << "max_ts " << max_ts << std::endl;
    std::cout << "number_of_points_with_timestamp_eq_0: " << number_of_points_with_timestamp_eq_0 << std::endl;

    return true;
}
