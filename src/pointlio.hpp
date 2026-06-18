#ifndef POINTLIO_H_
#define POINTLIO_H_

#include "laserMapping.hpp"

// Point-LIO (IMU-as-output) SLAM core with a thin LCM-friendly public API, so
// the dimos glue (main.cpp in the dimos pointlio native module) is drop-in.
class PointLio
{
public:
    PointLio(const PointLioParams& params = {},
             double msr_freq = 50.0, double main_freq = 5000.0,
             double rotation_gap_threshold_deg_s = 10.0)
        : odom_result(new custom_messages::Odometry)
    {
        laser_mapping = std::make_unique<LaserMapping>(
            params, msr_freq, main_freq, rotation_gap_threshold_deg_s);
    }
    ~PointLio() {}

    void feed_imu(const ImuConstPtr &imu_data)   { laser_mapping->imu_cbk(imu_data); }
    void feed_lidar(const CstMsgConstPtr &lidar) { laser_mapping->livox_pcl_cbk(lidar); }
    void process()                               { laser_mapping->run(odom_result); }

    std::vector<double> get_pose() {
        const auto &p = odom_result->pose.pose;
        return { p.position.x, p.position.y, p.position.z,
                 p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w };
    }
    const custom_messages::Odometry& get_odometry() const { return *odom_result; }
    PointCloudXYZI::Ptr get_world_cloud() const { return laser_mapping->get_world_cloud(); }
    PointCloudXYZI::Ptr get_body_cloud() const { return laser_mapping->get_body_cloud(); }

    // Read-only state accessors (used by the LCM glue / ICP cross-check path).
    std::vector<double> get_world_quat() const { return laser_mapping->get_world_quat(); }
    double get_world_vel_norm() const          { return laser_mapping->get_world_vel_norm(); }

    // FAST-LIO2 reactive-guardrail hooks — forwarded to LaserMapping no-ops
    // (Point-LIO handles aggressive motion natively). Kept for interface parity.
    void set_world_pose_quat_vel(double px,double py,double pz,double qx,double qy,double qz,double qw,
                                 double vx,double vy,double vz) {
        laser_mapping->set_world_pose_quat_vel(px,py,pz,qx,qy,qz,qw,vx,vy,vz);
    }
    void set_icp_omega_body(double wx,double wy,double wz) { laser_mapping->set_icp_omega_body(wx,wy,wz); }
    void clear_icp_omega()                                { laser_mapping->clear_icp_omega(); }
    void set_rotation_gap_threshold_deg_s(double t)       { laser_mapping->set_rotation_gap_threshold_deg_s(t); }
    void set_angular_accel_cap_deg_s2(double c)           { laser_mapping->set_angular_accel_cap_deg_s2(c); }
    void set_icp_velocity_body(double vx,double vy,double vz) { laser_mapping->set_icp_velocity_body(vx,vy,vz); }
    void clear_icp_velocity()                             { laser_mapping->clear_icp_velocity(); }
    void set_linear_velocity_gap_threshold_ms(double t)   { laser_mapping->set_linear_velocity_gap_threshold_ms(t); }
    void set_linear_accel_cap_ms2(double c)               { laser_mapping->set_linear_accel_cap_ms2(c); }

private:
    OdomMsgPtr odom_result;
    std::unique_ptr<LaserMapping> laser_mapping;
};

#endif
