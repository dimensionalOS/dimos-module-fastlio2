#include "utils.h"

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(
    const CstMsgConstPtr &msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = static_cast<int>(msg->points.size());
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            float x = static_cast<float>(msg->points[i].x);
            float y = static_cast<float>(msg->points[i].y);
            float z = static_cast<float>(msg->points[i].z);
            float dist_sq = x * x + y * y + z * z;
            if (dist_sq < min_range * min_range || dist_sq > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = static_cast<float>(msg->points[i].reflectivity);
            p.curvature = static_cast<float>(msg->points[i].offset_time) / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

IMUData Utils::imu2Data(const ImuConstPtr &msg)
{
    double t = msg->header.stamp.toSec();
    // Livox Mid-360 reports acceleration in 0.1 m/s² (g-units).
    // Multiply by 10 to get m/s², matching the reference implementation.
    V3D acc(msg->linear_acceleration.x * 10.0,
            msg->linear_acceleration.y * 10.0,
            msg->linear_acceleration.z * 10.0);
    V3D gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    return IMUData(acc, gyro, t);
}
