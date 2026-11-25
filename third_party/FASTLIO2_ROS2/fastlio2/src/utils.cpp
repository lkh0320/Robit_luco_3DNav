#include "utils.h"
#include <cstring>
#include <limits>
#include <cmath>
// VLP-16 PointCloud2 -> PCL. 점별 시간(time 필드, 헤더 기준 초)을 curvature(ms, 스캔 첫 점 기준)에 넣어
// IMUProcessor::undistort 가 회전/이동 왜곡을 보정할 수 있게 함.
//   start_offset: 헤더 시각 기준 스캔 첫 점의 시각(초). velodyne ROS2 드라이버는 헤더 = 스캔 끝이라 약 -0.1
//   deskew=false 또는 time 필드가 없으면 원래처럼 모든 점 curvature=0, start_offset=0 (보정 없음)
// (원본은 여기서 법선을 계산했지만 어디에서도 쓰지 않아 제거: 스캔당 kd-tree 비용 절약)
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::velo2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range,
                                                           bool deskew, double *start_offset)
{
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    int off_x = -1, off_y = -1, off_z = -1, off_i = -1, off_t = -1;
    uint8_t type_t = 0;
    for (const auto &f : msg->fields)
    {
        if (f.name == "x") off_x = f.offset;
        else if (f.name == "y") off_y = f.offset;
        else if (f.name == "z") off_z = f.offset;
        else if (f.name == "intensity") off_i = f.offset;
        else if (f.name == "time" || f.name == "t" || f.name == "timestamp")
        {
            off_t = f.offset;
            type_t = f.datatype;
        }
    }
    const bool use_time = deskew && off_t >= 0 &&
                          (type_t == sensor_msgs::msg::PointField::FLOAT32 || type_t == sensor_msgs::msg::PointField::FLOAT64);

    auto rd = [&](size_t i, int off) -> float
    {
        float v;
        std::memcpy(&v, &msg->data[i * msg->point_step + off], sizeof(float));
        return v;
    };
    auto rd_time = [&](size_t i) -> double
    {
        const uint8_t *ptr = &msg->data[i * msg->point_step + off_t];
        if (type_t == sensor_msgs::msg::PointField::FLOAT64)
        {
            double v;
            std::memcpy(&v, ptr, sizeof(double));
            return v;
        }
        float v;
        std::memcpy(&v, ptr, sizeof(float));
        return v;
    };

    double t_first = 0.0;
    if (use_time && n > 0)
    {
        t_first = std::numeric_limits<double>::max();
        for (size_t i = 0; i < n; i++)
            t_first = std::min(t_first, rd_time(i));
    }
    if (start_offset)
        *start_offset = t_first;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
    output->reserve(n / std::max(filter_num, 1) + 1);
    const double min_sq = min_range * min_range, max_sq = max_range * max_range;
    for (size_t i = 0; i < n; i += std::max(filter_num, 1))
    {
        pcl::PointXYZINormal p;
        p.x = rd(i, off_x);
        p.y = rd(i, off_y);
        p.z = rd(i, off_z);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        const double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
        if (dist_sq < min_sq || dist_sq > max_sq)
            continue;
        p.intensity = off_i >= 0 ? rd(i, off_i) : 0.0f;
        p.normal_x = p.normal_y = p.normal_z = 0.0f;
        p.curvature = use_time ? static_cast<float>((rd_time(i) - t_first) * 1000.0) : 0.0f;
        output->push_back(p);
    }
    output->width = output->size();
    output->height = 1;
    return output;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
