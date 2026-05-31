#ifndef FAST_LIO_H_
#define FAST_LIO_H_

#include "laserMapping.hpp"

class FastLio
{
public:
    FastLio(const std::string& config_path = CONFIG_FILE_PATH,
            double msr_freq = 50.0, double main_freq = 5000.0,
            double rotation_gap_threshold_deg_s = 10.0);
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
    // position, orientation (quaternion), and velocity. Biases/gravity
    // untouched. Used by main.cpp's "ICP cross-check rollback" path.
    void set_world_pose_quat_vel(double px, double py, double pz,
                                 double qx, double qy, double qz, double qw,
                                 double vx, double vy, double vz);

    /// Read-only access to the current IESKF world-frame orientation as a
    /// quaternion (qx, qy, qz, qw). Used by callers integrating body-frame
    /// velocities (e.g. ICP) into world-frame displacement.
    std::vector<double> get_world_quat() const;
    /// World-frame velocity magnitude from the IESKF state.
    double get_world_vel_norm() const;

    /// Preventative map-skip: the next call to process() will skip
    /// map_incremental if |ω_ieskf − ω_icp_body| (in deg/s) exceeds the
    /// configured threshold. Caller passes ICP's body-frame angular
    /// velocity (rad/s); FastLio computes the IESKF's body-frame ω
    /// internally from its own state, takes the magnitude of the diff,
    /// and gates the kd-tree insert.
    void set_icp_omega_body(double wx_rad_s, double wy_rad_s, double wz_rad_s);
    void clear_icp_omega();
    void set_rotation_gap_threshold_deg_s(double threshold);
    void set_angular_accel_cap_deg_s2(double cap);
    void set_icp_velocity_body(double vx_ms, double vy_ms, double vz_ms);
    void clear_icp_velocity();
    void set_linear_velocity_gap_threshold_ms(double threshold);
    void set_linear_accel_cap_ms2(double cap);

private:
    OdomMsgPtr odom_result;
    ofstream output_file, exec_time_file;
    std::unique_ptr<LaserMapping> laser_mapping;
};

FastLio::FastLio(const std::string& config_path, double msr_freq, double main_freq,
                 double rotation_gap_threshold_deg_s)
    : odom_result(new custom_messages::Odometry), output_file("../data/output.txt"), exec_time_file("../data/time.txt")
{
    laser_mapping = std::make_unique<LaserMapping>(
        config_path, msr_freq, main_freq, rotation_gap_threshold_deg_s);
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

void FastLio::set_world_pose_quat_vel(double px, double py, double pz,
                                      double qx, double qy, double qz, double qw,
                                      double vx, double vy, double vz)
{
    laser_mapping->set_world_pose_quat_vel(px, py, pz, qx, qy, qz, qw, vx, vy, vz);
}

std::vector<double> FastLio::get_world_quat() const
{
    return laser_mapping->get_world_quat();
}

double FastLio::get_world_vel_norm() const
{
    return laser_mapping->get_world_vel_norm();
}

void FastLio::set_icp_omega_body(double wx_rad_s, double wy_rad_s, double wz_rad_s)
{
    laser_mapping->set_icp_omega_body(wx_rad_s, wy_rad_s, wz_rad_s);
}

void FastLio::clear_icp_omega()
{
    laser_mapping->clear_icp_omega();
}

void FastLio::set_rotation_gap_threshold_deg_s(double threshold)
{
    laser_mapping->set_rotation_gap_threshold_deg_s(threshold);
}

void FastLio::set_angular_accel_cap_deg_s2(double cap)
{
    laser_mapping->set_angular_accel_cap_deg_s2(cap);
}

void FastLio::set_icp_velocity_body(double vx, double vy, double vz)
{
    laser_mapping->set_icp_velocity_body(vx, vy, vz);
}

void FastLio::clear_icp_velocity()
{
    laser_mapping->clear_icp_velocity();
}

void FastLio::set_linear_velocity_gap_threshold_ms(double threshold)
{
    laser_mapping->set_linear_velocity_gap_threshold_ms(threshold);
}

void FastLio::set_linear_accel_cap_ms2(double cap)
{
    laser_mapping->set_linear_accel_cap_ms2(cap);
}

#endif