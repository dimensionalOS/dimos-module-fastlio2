#ifndef FAST_LIO_H_
#define FAST_LIO_H_

#include "laserMapping.hpp"

class FastLio
{
public:
    FastLio(const std::string& config_path = CONFIG_FILE_PATH,
            double msr_freq = 50.0, double main_freq = 5000.0,
            double guardrail_max_pos_jump_m = 0.5,
            double guardrail_max_accel_norm_ms2 = 30.0);
    ~FastLio();

    void feed_imu(const ImuConstPtr &imu_data);
    void feed_lidar(const CstMsgConstPtr &lidar_data);
    void process();
    std::vector<double> get_pose();
    const custom_messages::Odometry& get_odometry() const { return *odom_result; }
    PointCloudXYZI::Ptr get_world_cloud() const { return laser_mapping->get_world_cloud(); }
    void write_to_file(const std::vector<double> &pose);
    void write_to_file(const double &time);

    // External correction hook — replace the IESKF state's world-frame
    // position and velocity (xyz). Orientation/biases/gravity untouched.
    // Used by main.cpp's "ICP cross-check rollback" path: when the IESKF
    // state's velocity disagrees badly with scan-to-scan ICP, integrate
    // ICP velocities forward from an older known-good pose and overwrite
    // the IESKF state to match.
    void set_world_pose_vel(double px, double py, double pz,
                            double vx, double vy, double vz);

    /// Read-only access to the current IESKF world-frame orientation as a
    /// quaternion (qx, qy, qz, qw). Used by callers integrating body-frame
    /// velocities (e.g. ICP) into world-frame displacement.
    std::vector<double> get_world_quat() const;
    /// World-frame velocity magnitude from the IESKF state.
    double get_world_vel_norm() const;

private:
    OdomMsgPtr odom_result;
    ofstream output_file, exec_time_file;
    std::unique_ptr<LaserMapping> laser_mapping;
};

FastLio::FastLio(const std::string& config_path, double msr_freq, double main_freq,
                 double guardrail_max_pos_jump_m, double guardrail_max_accel_norm_ms2)
    : odom_result(new custom_messages::Odometry), output_file("../data/output.txt"), exec_time_file("../data/time.txt")
{
    laser_mapping = std::make_unique<LaserMapping>(
        config_path, msr_freq, main_freq,
        guardrail_max_pos_jump_m, guardrail_max_accel_norm_ms2);
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
    laser_mapping->imu_cbk(imu_data);
}

void FastLio::feed_lidar(const CstMsgConstPtr &lidar_data)
{
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

void FastLio::set_world_pose_vel(double px, double py, double pz,
                                 double vx, double vy, double vz)
{
    laser_mapping->set_world_pose_vel(px, py, pz, vx, vy, vz);
}

std::vector<double> FastLio::get_world_quat() const
{
    return laser_mapping->get_world_quat();
}

double FastLio::get_world_vel_norm() const
{
    return laser_mapping->get_world_vel_norm();
}

#endif