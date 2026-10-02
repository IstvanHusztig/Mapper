// cut_recording: copies a time range of a recording folder (lidar*.laz + imu*.csv) into a new input folder.
//
//   cut_recording <in_folder> <out_folder> <start_s> <duration_s>
//
// The range [first IMU sample + start_s, + duration_s) is cut from every .laz/.las and .csv file of
// in_folder (non-recursive, like load_data). LAZ points keep their stored integer coordinates and
// every other attribute (the input header, point format, scale and offset are reused); IMU lines and
// the header line are copied byte for byte. IMU `timestamp` is in ns, LAZ gps_time in s.
// After writing, every kept point is re-read and compared with the input.

#include <laszip/laszip_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
std::string lower_ext(const fs::path &p)
{
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}

std::vector<std::string> split_fields(const std::string &line)
{
    std::vector<std::string> out;
    std::string cur;
    for (char c : line)
    {
        if (c == ' ' || c == ',' || c == '\t' || c == '\r')
        {
            if (!cur.empty())
                out.push_back(cur);
            cur.clear();
        }
        else
            cur += c;
    }
    if (!cur.empty())
        out.push_back(cur);
    return out;
}

// index of the `timestamp` column in an IMU CSV header, -1 if missing
int timestamp_column(const std::string &header)
{
    const auto f = split_fields(header);
    for (size_t i = 0; i < f.size(); i++)
        if (f[i] == "timestamp")
            return static_cast<int>(i);
    return -1;
}

// first IMU timestamp of a CSV [s]; NaN if the file has no data line
double first_imu_time_s(const fs::path &csv)
{
    std::ifstream in(csv, std::ios::binary);
    std::string line;
    if (!std::getline(in, line))
        return std::numeric_limits<double>::quiet_NaN();
    const int col = timestamp_column(line);
    if (col < 0)
        throw std::runtime_error("no 'timestamp' column in " + csv.string());
    while (std::getline(in, line))
    {
        const auto f = split_fields(line);
        if (static_cast<int>(f.size()) > col)
            return std::stod(f[col]) / 1e9;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

// copies the header and the lines with t0 <= timestamp < t1 verbatim; returns the number of kept lines
size_t cut_imu(const fs::path &in_path, const fs::path &out_path, double t0, double t1)
{
    std::ifstream in(in_path, std::ios::binary);
    std::ofstream out(out_path, std::ios::binary);
    std::string line;
    if (!std::getline(in, line))
        throw std::runtime_error("empty IMU file " + in_path.string());
    const int col = timestamp_column(line);
    out << line << '\n';
    size_t kept = 0;
    while (std::getline(in, line))
    {
        const auto f = split_fields(line);
        if (static_cast<int>(f.size()) <= col)
            continue; // partial last line
        const double t = std::stod(f[col]) / 1e9;
        if (t >= t0 && t < t1)
        {
            out << line << '\n';
            kept++;
        }
    }
    if (!out)
        throw std::runtime_error("write failed: " + out_path.string());
    return kept;
}

void check(laszip_POINTER h, laszip_I32 err, const std::string &what)
{
    if (err)
    {
        laszip_CHAR *msg = nullptr;
        laszip_get_error(h, &msg);
        throw std::runtime_error(what + ": " + (msg ? msg : "unknown LASzip error"));
    }
}

bool same_point(const laszip_point *a, const laszip_point *b)
{
    return a->X == b->X && a->Y == b->Y && a->Z == b->Z && a->intensity == b->intensity &&
           a->return_number == b->return_number && a->number_of_returns == b->number_of_returns &&
           a->scan_direction_flag == b->scan_direction_flag && a->edge_of_flight_line == b->edge_of_flight_line &&
           a->classification == b->classification && a->synthetic_flag == b->synthetic_flag &&
           a->keypoint_flag == b->keypoint_flag && a->withheld_flag == b->withheld_flag &&
           a->scan_angle_rank == b->scan_angle_rank && a->user_data == b->user_data &&
           a->point_source_ID == b->point_source_ID && a->extended_scan_angle == b->extended_scan_angle &&
           a->extended_point_type == b->extended_point_type && a->extended_scanner_channel == b->extended_scanner_channel &&
           a->extended_classification_flags == b->extended_classification_flags &&
           a->extended_classification == b->extended_classification && a->extended_return_number == b->extended_return_number &&
           a->extended_number_of_returns == b->extended_number_of_returns &&
           std::memcmp(&a->gps_time, &b->gps_time, sizeof(a->gps_time)) == 0 &&
           std::memcmp(a->rgb, b->rgb, sizeof(a->rgb)) == 0 && std::memcmp(a->wave_packet, b->wave_packet, sizeof(a->wave_packet)) == 0 &&
           a->num_extra_bytes == b->num_extra_bytes &&
           (a->num_extra_bytes == 0 || std::memcmp(a->extra_bytes, b->extra_bytes, a->num_extra_bytes) == 0);
}

struct Reader
{
    laszip_POINTER h = nullptr;
    laszip_header *header = nullptr;
    laszip_point *point = nullptr;
    laszip_I64 count = 0;
    explicit Reader(const fs::path &p)
    {
        if (laszip_create(&h))
            throw std::runtime_error("laszip_create failed");
        laszip_BOOL compressed = 0;
        check(h, laszip_open_reader(h, p.string().c_str(), &compressed), "open " + p.string());
        check(h, laszip_get_header_pointer(h, &header), "header");
        check(h, laszip_get_point_pointer(h, &point), "point");
        count = header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records;
    }
    void read() { check(h, laszip_read_point(h), "read point"); }
    ~Reader()
    {
        laszip_close_reader(h);
        laszip_destroy(h);
    }
};

// writes the points with t0 <= gps_time < t1; returns {kept, total}
std::pair<size_t, size_t> cut_laz(const fs::path &in_path, const fs::path &out_path, double t0, double t1)
{
    Reader in(in_path);
    laszip_POINTER w = nullptr;
    if (laszip_create(&w))
        throw std::runtime_error("laszip_create failed");
    check(w, laszip_set_header(w, in.header), "set header");
    const laszip_BOOL compress = lower_ext(out_path) == ".laz";
    check(w, laszip_open_writer(w, out_path.string().c_str(), compress), "open " + out_path.string());
    size_t kept = 0;
    for (laszip_I64 i = 0; i < in.count; i++)
    {
        in.read();
        if (in.point->gps_time >= t0 && in.point->gps_time < t1)
        {
            check(w, laszip_set_point(w, in.point), "set point");
            check(w, laszip_write_point(w), "write point");
            check(w, laszip_update_inventory(w), "inventory");
            kept++;
        }
    }
    check(w, laszip_close_writer(w), "close " + out_path.string());
    laszip_destroy(w);
    return {kept, static_cast<size_t>(in.count)};
}

// re-reads both files: the output must be exactly the in-range input points, in order
void verify_laz(const fs::path &in_path, const fs::path &out_path, double t0, double t1, size_t kept)
{
    Reader in(in_path), out(out_path);
    if (static_cast<size_t>(out.count) != kept)
        throw std::runtime_error("verify: " + out_path.string() + " holds " + std::to_string(out.count) + " points, expected " + std::to_string(kept));
    if (in.header->x_scale_factor != out.header->x_scale_factor || in.header->y_scale_factor != out.header->y_scale_factor ||
        in.header->z_scale_factor != out.header->z_scale_factor || in.header->x_offset != out.header->x_offset ||
        in.header->y_offset != out.header->y_offset || in.header->z_offset != out.header->z_offset ||
        in.header->point_data_format != out.header->point_data_format)
        throw std::runtime_error("verify: header scale/offset/format differs in " + out_path.string());
    laszip_I64 j = 0;
    for (laszip_I64 i = 0; i < in.count; i++)
    {
        in.read();
        if (!(in.point->gps_time >= t0 && in.point->gps_time < t1))
            continue;
        out.read();
        if (!same_point(in.point, out.point))
            throw std::runtime_error("verify: point " + std::to_string(j) + " differs in " + out_path.string());
        j++;
    }
}
} // namespace

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        std::cerr << "usage: cut_recording <in_folder> <out_folder> <start_s> <duration_s>\n";
        return 2;
    }
    try
    {
        const fs::path in_dir = argv[1], out_dir = argv[2];
        const double start_s = std::stod(argv[3]), duration_s = std::stod(argv[4]);
        if (!fs::is_directory(in_dir) || duration_s <= 0)
            throw std::runtime_error("bad input folder or duration");
        if (fs::exists(out_dir) && fs::equivalent(in_dir, out_dir))
            throw std::runtime_error("output folder must differ from the input folder");

        std::vector<fs::path> laz, csv;
        for (const auto &e : fs::directory_iterator(in_dir))
        {
            const auto ext = lower_ext(e.path());
            if (ext == ".laz" || ext == ".las")
                laz.push_back(e.path());
            else if (ext == ".csv")
                csv.push_back(e.path());
        }
        if (laz.empty() || csv.empty())
            throw std::runtime_error("input folder needs .laz/.las and .csv files");

        if (fs::exists(out_dir))
            for (const auto &e : fs::directory_iterator(out_dir))
            {
                const auto ext = lower_ext(e.path());
                if (ext == ".laz" || ext == ".las" || ext == ".csv")
                    throw std::runtime_error("output folder already holds " + e.path().filename().string());
            }
        fs::create_directories(out_dir);

        double t_first = std::numeric_limits<double>::infinity();
        for (const auto &p : csv)
        {
            const double t = first_imu_time_s(p);
            if (!std::isnan(t))
                t_first = std::min(t_first, t);
        }
        const double t0 = t_first + start_s, t1 = t0 + duration_s;
        std::printf("first IMU sample %.9f s; cut [%.9f, %.9f) s\n", t_first, t0, t1);

        for (const auto &p : csv)
        {
            const size_t kept = cut_imu(p, out_dir / p.filename(), t0, t1);
            std::printf("%s: %zu IMU lines\n", p.filename().string().c_str(), kept);
        }
        for (const auto &p : laz)
        {
            const auto [kept, total] = cut_laz(p, out_dir / p.filename(), t0, t1);
            std::printf("%s: %zu of %zu points\n", p.filename().string().c_str(), kept, total);
            verify_laz(p, out_dir / p.filename(), t0, t1, kept);
            std::printf("%s: verified\n", p.filename().string().c_str());
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}
