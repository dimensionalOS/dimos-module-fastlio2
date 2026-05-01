#ifndef FAST_LIO_H_
#define FAST_LIO_H_

// FastLio — DimOS wrapper around liangheming/FASTLIO2_ROS2's MapBuilder.
//
// Provides the feed_imu() / feed_lidar() / process() interface expected by
// the DimOS NativeModule, with in-RAM raw data recording (RawDump).

#include <deque>
#include <mutex>
#include <condition_variable>

#include "map_builder/commons.h"
#include "map_builder/ieskf.h"
#include "map_builder/map_builder.h"
#include "msgs.h"
#include "raw_dump.hpp"
#include "utils.h"

using custom_messages::ImuConstPtr;
using custom_messages::CstMsgConstPtr;
using custom_messages::OdomMsgPtr;

class FastLio
{
public:
    FastLio(const std::string& config_path = "../config/horizon.json",
            double msr_freq = 50.0, double main_freq = 5000.0);
    ~FastLio() = default;

    void feed_imu(const ImuConstPtr &imu_data);
    void feed_lidar(const CstMsgConstPtr &lidar_data);
    void process();

    std::vector<double> get_pose();
    const custom_messages::Odometry& get_odometry() const { return *m_odom; }

    // Get the undistorted scan in world frame (after last process() call).
    CloudType::Ptr get_world_cloud() const { return m_world_cloud; }

    // Enable raw sensor data recording.
    void enable_raw_dump(const std::string& path, double duration_sec = 120.0) {
        m_raw_dump.enable(path, duration_sec);
    }

private:
    bool sync_packages(SyncPackage &package);

    // Config
    Config m_config;

    // EKF + MapBuilder
    std::shared_ptr<IESKF> m_kf;
    std::unique_ptr<MapBuilder> m_builder;

    // Buffers
    std::mutex m_mtx;
    std::condition_variable m_sig;
    std::deque<IMUData> m_imu_buffer;
    std::deque<std::pair<double, CloudType::Ptr>> m_lidar_buffer;
    double m_last_imu_time = -1.0;
    double m_last_lidar_time = -1.0;
    bool m_lidar_pushed = false;
    double m_lidar_end_time = 0.0;

    // Output
    OdomMsgPtr m_odom;
    CloudType::Ptr m_world_cloud;

    // Raw recording
    RawDump m_raw_dump;
};

// ────────────────────────────────────────────────────────────────────────────

FastLio::FastLio(const std::string& config_path, double /*msr_freq*/, double /*main_freq*/)
    : m_odom(new custom_messages::Odometry)
{
    // TODO: Load config from YAML file at config_path.
    // For now use defaults from Config struct.
    (void)config_path;

    m_kf = std::make_shared<IESKF>();
    m_kf->setMaxIter(m_config.ieskf_max_iter);
    m_builder = std::make_unique<MapBuilder>(m_config, m_kf);
}

void FastLio::feed_imu(const ImuConstPtr &imu_data)
{
    m_raw_dump.record_imu(imu_data);

    IMUData data = Utils::imu2Data(imu_data);
    std::lock_guard<std::mutex> lock(m_mtx);
    if (data.time < m_last_imu_time) {
        m_imu_buffer.clear();
    }
    m_last_imu_time = data.time;
    m_imu_buffer.push_back(data);
    m_sig.notify_all();
}

void FastLio::feed_lidar(const CstMsgConstPtr &lidar_data)
{
    m_raw_dump.record_lidar(lidar_data);

    double timestamp = lidar_data->header.stamp.toSec();
    CloudType::Ptr cloud = Utils::livox2PCL(
        lidar_data, m_config.lidar_filter_num,
        m_config.lidar_min_range, m_config.lidar_max_range);

    std::lock_guard<std::mutex> lock(m_mtx);
    if (timestamp < m_last_lidar_time) {
        m_lidar_buffer.clear();
    }
    m_last_lidar_time = timestamp;
    m_lidar_buffer.emplace_back(timestamp, cloud);
    m_sig.notify_all();
}

bool FastLio::sync_packages(SyncPackage &package)
{
    std::lock_guard<std::mutex> lock(m_mtx);

    if (m_lidar_buffer.empty() || m_imu_buffer.empty())
        return false;

    if (!m_lidar_pushed) {
        auto &[ts, cloud] = m_lidar_buffer.front();
        package.cloud = cloud;
        package.cloud_start_time = ts;

        if (cloud->empty()) {
            m_lidar_end_time = ts + 0.1;
        } else {
            m_lidar_end_time = ts + cloud->points.back().curvature / 1000.0;
        }
        package.cloud_end_time = m_lidar_end_time;
        m_lidar_pushed = true;
    }

    if (m_last_imu_time < m_lidar_end_time)
        return false;

    // Collect IMU samples covering this scan
    package.imus.clear();
    while (!m_imu_buffer.empty()) {
        if (m_imu_buffer.front().time > m_lidar_end_time)
            break;
        package.imus.push_back(m_imu_buffer.front());
        m_imu_buffer.pop_front();
    }

    m_lidar_buffer.pop_front();
    m_lidar_pushed = false;
    return true;
}

void FastLio::process()
{
    SyncPackage package;
    if (!sync_packages(package))
        return;

    m_builder->process(package);

    if (m_builder->status() < BuilderStatus::MAPPING)
        return;

    // Extract pose from EKF state
    const State &state = m_kf->x();
    Eigen::Quaterniond q(state.r_wi);

    m_odom->header.stamp = m_odom->header.stamp.fromSec(package.cloud_end_time);
    m_odom->header.frame_id = "map";
    m_odom->child_frame_id = "body";
    m_odom->pose.pose.position.x = state.t_wi.x();
    m_odom->pose.pose.position.y = state.t_wi.y();
    m_odom->pose.pose.position.z = state.t_wi.z();
    m_odom->pose.pose.orientation.x = q.x();
    m_odom->pose.pose.orientation.y = q.y();
    m_odom->pose.pose.orientation.z = q.z();
    m_odom->pose.pose.orientation.w = q.w();

    // Transform scan to world frame
    m_world_cloud = LidarProcessor::transformCloud(
        package.cloud,
        m_builder->lidar_processor()->r_wl(),
        m_builder->lidar_processor()->t_wl());
}

std::vector<double> FastLio::get_pose()
{
    return {
        m_odom->pose.pose.position.x,
        m_odom->pose.pose.position.y,
        m_odom->pose.pose.position.z,
        m_odom->pose.pose.orientation.x,
        m_odom->pose.pose.orientation.y,
        m_odom->pose.pose.orientation.z,
        m_odom->pose.pose.orientation.w,
    };
}

#endif
