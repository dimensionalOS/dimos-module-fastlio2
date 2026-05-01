#ifndef FAST_LIO_H_
#define FAST_LIO_H_

#include "laserMapping.hpp"
#include "raw_dump.hpp"

class FastLio
{
public:
    FastLio(const std::string& config_path = CONFIG_FILE_PATH,
            double msr_freq = 50.0, double main_freq = 5000.0);
    ~FastLio();

    void feed_imu(const ImuConstPtr &imu_data);
    void feed_lidar(const CstMsgConstPtr &lidar_data);
    void process();
    std::vector<double> get_pose();
    const custom_messages::Odometry& get_odometry() const { return *odom_result; }
    PointCloudXYZI::Ptr get_world_cloud() const { return laser_mapping->get_world_cloud(); }
    void write_to_file(const std::vector<double> &pose);
    void write_to_file(const double &time);

    // Enable raw sensor data recording to a binary file.
    // Data is buffered in RAM and serialized after duration_sec.
    void enable_raw_dump(const std::string& path, double duration_sec = 120.0) {
        raw_dump_.enable(path, duration_sec);
    }

private:
    OdomMsgPtr odom_result;
    ofstream output_file, exec_time_file;
    std::unique_ptr<LaserMapping> laser_mapping;
    RawDump raw_dump_;
};

FastLio::FastLio(const std::string& config_path, double msr_freq, double main_freq)
    : odom_result(new custom_messages::Odometry), output_file("../data/output.txt"), exec_time_file("../data/time.txt")
{
    laser_mapping = std::make_unique<LaserMapping>(config_path, msr_freq, main_freq);
}

FastLio::~FastLio()
{
    if (output_file.is_open())
        output_file.close();
    if (exec_time_file.is_open())
        exec_time_file.close();
}

void FastLio::feed_imu(const ImuConstPtr &imu_data)
{
    raw_dump_.record_imu(imu_data);
    laser_mapping->imu_cbk(imu_data);
}

void FastLio::feed_lidar(const CstMsgConstPtr &lidar_data)
{
    raw_dump_.record_lidar(lidar_data);
    laser_mapping->livox_pcl_cbk(lidar_data);
}

void FastLio::process()
{
    laser_mapping->run(odom_result);
}

std::vector<double> FastLio::get_pose()
{
    std::vector<double> odom;
    odom.push_back(odom_result->pose.pose.position.x);
    odom.push_back(odom_result->pose.pose.position.y);
    odom.push_back(odom_result->pose.pose.position.z);
    odom.push_back(odom_result->pose.pose.orientation.x);
    odom.push_back(odom_result->pose.pose.orientation.y);
    odom.push_back(odom_result->pose.pose.orientation.z);
    odom.push_back(odom_result->pose.pose.orientation.w);
    return odom;
}

void FastLio::write_to_file(const std::vector<double> &pose)
{
    output_file << pose[0] << "," << pose[1] << "," << pose[2] << "\n"; 
}

void FastLio::write_to_file(const double &time)
{
    exec_time_file << time << "\n"; 
}

#endif