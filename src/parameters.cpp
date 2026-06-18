#include "parameters.h"

bool odom_only;
std::string odom_header_frame_id, odom_child_frame_id;

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;
state_input state_in;
state_output state_out;

std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
int init_map_size, con_frame_num;
double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
double max_velocity_norm_ms;
int recovery_cooldown_scans;
float plane_thr;
double filter_size_surf_min, filter_size_map_min, fov_deg;
double cube_len;
float DET_RANGE;
bool imu_en, gravity_align, non_station_start;
double imu_time_inte;
double laser_point_cov, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov;
int lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT;
std::vector<double> extrinR;
bool runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

void readParameters(const PointLioParams &params) {
    p_pre.reset(new Preprocess());

    odom_only             = params.odom_only;
    odom_header_frame_id  = params.odom_header_frame_id;
    odom_child_frame_id   = params.odom_child_frame_id;

    prop_at_freq_of_imu   = params.prop_at_freq_of_imu;
    use_imu_as_input      = params.use_imu_as_input;
    check_satu            = params.check_satu;
    init_map_size         = params.init_map_size;
    space_down_sample     = params.space_down_sample;
    satu_acc              = params.satu_acc;
    satu_gyro             = params.satu_gyro;
    // Post-update velocity sanity cap (m/s). 0 disables (upstream behavior).
    max_velocity_norm_ms  = params.max_velocity_norm_ms;
    recovery_cooldown_scans = params.recovery_cooldown_scans;
    acc_norm              = params.acc_norm;
    plane_thr             = params.plane_thr;
    p_pre->point_filter_num = params.point_filter_num;
    lid_topic             = params.lid_topic;
    imu_topic             = params.imu_topic;
    con_frame             = params.con_frame;
    con_frame_num         = params.con_frame_num;
    cut_frame             = params.cut_frame;
    cut_frame_time_interval = params.cut_frame_time_interval;
    time_lag_imu_to_lidar = params.time_lag_imu_to_lidar;
    filter_size_surf_min  = params.filter_size_surf;
    filter_size_map_min   = params.filter_size_map;
    cube_len              = params.cube_side_length;
    DET_RANGE             = params.det_range;
    fov_deg               = params.fov_degree;
    imu_en                = params.imu_en;
    non_station_start     = params.start_in_aggressive_motion;
    extrinsic_est_en      = params.extrinsic_est_en;
    imu_time_inte         = params.imu_time_inte;
    laser_point_cov       = params.lidar_meas_cov;
    acc_cov_input         = params.acc_cov_input;
    vel_cov               = params.vel_cov;
    gyr_cov_input         = params.gyr_cov_input;
    gyr_cov_output        = params.gyr_cov_output;
    acc_cov_output        = params.acc_cov_output;
    b_gyr_cov             = params.b_gyr_cov;
    b_acc_cov             = params.b_acc_cov;
    imu_meas_acc_cov      = params.imu_meas_acc_cov;
    imu_meas_omg_cov      = params.imu_meas_omg_cov;
    p_pre->blind          = params.blind;
    lidar_type            = params.lidar_type;
    p_pre->N_SCANS        = params.scan_line;
    p_pre->SCAN_RATE      = params.scan_rate;
    p_pre->time_unit      = params.timestamp_unit;
    match_s               = params.match_s;
    gravity_align         = params.gravity_align;
    gravity               = params.gravity;
    gravity_init          = params.gravity_init;
    extrinT               = params.extrinsic_T;
    extrinR               = params.extrinsic_R;
    publish_odometry_without_downsample = params.publish_odometry_without_downsample;
    path_en               = params.path_en;
    scan_pub_en           = params.scan_publish_en;
    scan_body_pub_en      = params.scan_bodyframe_pub_en;
    runtime_pos_log       = params.runtime_pos_log;
    pcd_save_en           = params.pcd_save_en;
    pcd_save_interval     = params.pcd_save_interval;

    lidar_type = p_pre->lidar_type = lidar_type;

    ivox_options_.resolution_ = params.ivox_grid_resolution;
    ivox_nearby_type          = params.ivox_nearby_type;
    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot)
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}
