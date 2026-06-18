// #ifndef PARAM_H
// #define PARAM_H
#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include <string>
#include "preprocess.h"
#include "common_lib.h"
#include <ivox/ivox3d.h>

// #define IVOX_NODE_TYPE_PHC
#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

// Plain tuning struct, populated by the caller (the dimos pointlio module passes
// each field as a CLI arg). Replaces the old YAML config — defaults below match
// the upstream Point-LIO mid360 config so an unset field behaves as before.
struct PointLioParams {
    // odometry
    bool odom_only = false;
    std::string odom_header_frame_id = "camera_init";
    std::string odom_child_frame_id = "aft_mapped";
    bool publish_odometry_without_downsample = false;
    // common
    std::string lid_topic = "/livox/lidar";
    std::string imu_topic = "/livox/imu";
    bool con_frame = false;
    int con_frame_num = 1;
    bool cut_frame = false;
    double cut_frame_time_interval = 0.1;
    double time_lag_imu_to_lidar = 0.0;
    // preprocess
    int point_filter_num = 3;
    double blind = 0.5;
    int lidar_type = 1;
    int scan_line = 4;
    int scan_rate = 10;
    int timestamp_unit = 3;
    // mapping
    bool prop_at_freq_of_imu = true;
    bool use_imu_as_input = false;
    bool check_satu = true;
    int init_map_size = 10;
    bool space_down_sample = true;
    double satu_acc = 3.0;
    double satu_gyro = 35.0;
    double max_velocity_norm_ms = 0.0;
    int recovery_cooldown_scans = 30;
    double acc_norm = 1.0;
    float plane_thr = 0.1f;
    double filter_size_surf = 0.5;
    double filter_size_map = 0.5;
    double cube_side_length = 1000.0;
    float det_range = 100.0f;
    double fov_degree = 360.0;
    bool imu_en = true;
    bool start_in_aggressive_motion = false;
    bool extrinsic_est_en = false;
    double imu_time_inte = 0.005;
    double lidar_meas_cov = 0.01;
    double acc_cov_input = 0.1;
    double vel_cov = 20.0;
    double gyr_cov_input = 0.01;
    double gyr_cov_output = 1000.0;
    double acc_cov_output = 500.0;
    double b_gyr_cov = 0.0001;
    double b_acc_cov = 0.0001;
    double imu_meas_acc_cov = 0.01;
    double imu_meas_omg_cov = 0.01;
    double match_s = 81.0;
    bool gravity_align = true;
    std::vector<double> gravity{0.0, 0.0, -9.810};
    std::vector<double> gravity_init{0.0, 0.0, -9.810};
    std::vector<double> extrinsic_T{0.0, 0.0, 0.0};
    std::vector<double> extrinsic_R{1, 0, 0, 0, 1, 0, 0, 0, 1};
    float ivox_grid_resolution = 0.2f;
    int ivox_nearby_type = 18;
    // publish
    bool path_en = true;
    bool scan_publish_en = true;
    bool scan_bodyframe_pub_en = false;
    // misc
    bool runtime_pos_log = false;
    bool pcd_save_en = false;
    int pcd_save_interval = -1;
};

extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern state_input state_in;
extern state_output state_out;

extern bool odom_only;
extern std::string odom_header_frame_id;
extern std::string odom_child_frame_id;

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern double max_velocity_norm_ms;
extern int recovery_cooldown_scans;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len;
extern float DET_RANGE;
extern bool imu_en, gravity_align, non_station_start;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern bool runtime_pos_log, pcd_save_en, path_en;
extern bool scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;

// Non-ROS: populate the file-scope parameter globals from a PointLioParams
// struct, replacing the upstream ROS declare/get_parameter machinery.
void readParameters(const PointLioParams &params);
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &orient);
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init);
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);
