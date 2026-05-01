#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "msgs.h"
#include "map_builder/commons.h"

using custom_messages::CstMsgConstPtr;
using custom_messages::ImuConstPtr;

namespace Utils
{
    // Convert our CustomMsg (Livox format) to PCL with per-point timestamps.
    // filter_num: keep every Nth point (1 = all, 3 = every 3rd)
    // Curvature field stores offset_time in milliseconds.
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(
        const CstMsgConstPtr &msg,
        int filter_num = 3,
        double min_range = 0.5,
        double max_range = 20.0);

    // Convert our Imu message to IMUData struct
    IMUData imu2Data(const ImuConstPtr &msg);
}
