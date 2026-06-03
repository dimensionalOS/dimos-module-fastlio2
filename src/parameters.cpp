#include "parameters.h"
#include <yaml-cpp/yaml.h>
#include <sstream>

bool odom_only;
std::string odom_header_frame_id, odom_child_frame_id;

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;

std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
int init_map_size, con_frame_num;
double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
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

namespace {
// Descend a dotted path ("mapping.satu_acc") through nested YAML maps; return
// `def` if any level is missing or the leaf fails to convert.
template <typename T>
T cfg(const YAML::Node &root, const std::string &dotted, const T &def) {
    YAML::Node n = YAML::Clone(root);
    std::stringstream ss(dotted);
    std::string key;
    while (std::getline(ss, key, '.')) {
        if (!n || !n.IsMap() || !n[key]) return def;
        n = n[key];
    }
    if (!n) return def;
    try { return n.as<T>(); } catch (...) { return def; }
}
}  // namespace

void readParameters(const std::string &config_path) {
    p_pre.reset(new Preprocess());
    YAML::Node c = YAML::LoadFile(config_path);

    odom_only             = cfg<bool>(c, "odometry.odom_only", false);
    odom_header_frame_id  = cfg<std::string>(c, "odometry.odom_header_frame_id", "camera_init");
    odom_child_frame_id   = cfg<std::string>(c, "odometry.odom_child_frame_id", "aft_mapped");

    prop_at_freq_of_imu   = cfg<bool>(c, "mapping.prop_at_freq_of_imu", true);
    use_imu_as_input      = cfg<bool>(c, "mapping.use_imu_as_input", false);
    check_satu            = cfg<bool>(c, "mapping.check_satu", true);
    init_map_size         = cfg<int>(c, "mapping.init_map_size", 10);
    space_down_sample     = cfg<bool>(c, "mapping.space_down_sample", true);
    satu_acc              = cfg<double>(c, "mapping.satu_acc", 3.0);
    satu_gyro             = cfg<double>(c, "mapping.satu_gyro", 35.0);
    acc_norm              = cfg<double>(c, "mapping.acc_norm", 1.0);
    plane_thr             = cfg<float>(c, "mapping.plane_thr", 0.1f);
    p_pre->point_filter_num = cfg<int>(c, "preprocess.point_filter_num", 3);
    lid_topic             = cfg<std::string>(c, "common.lid_topic", "/livox/lidar");
    imu_topic             = cfg<std::string>(c, "common.imu_topic", "/livox/imu");
    con_frame             = cfg<bool>(c, "common.con_frame", false);
    con_frame_num         = cfg<int>(c, "common.con_frame_num", 1);
    cut_frame             = cfg<bool>(c, "common.cut_frame", false);
    cut_frame_time_interval = cfg<double>(c, "common.cut_frame_time_interval", 0.1);
    time_lag_imu_to_lidar = cfg<double>(c, "common.time_lag_imu_to_lidar", 0.0);
    filter_size_surf_min  = cfg<double>(c, "mapping.filter_size_surf", 0.5);
    filter_size_map_min   = cfg<double>(c, "mapping.filter_size_map", 0.5);
    cube_len              = cfg<double>(c, "mapping.cube_side_length", 1000.0);
    DET_RANGE             = cfg<float>(c, "mapping.det_range", 100.f);
    fov_deg               = cfg<double>(c, "mapping.fov_degree", 360.0);
    imu_en                = cfg<bool>(c, "mapping.imu_en", true);
    non_station_start     = cfg<bool>(c, "mapping.start_in_aggressive_motion", false);
    extrinsic_est_en      = cfg<bool>(c, "mapping.extrinsic_est_en", false);
    imu_time_inte         = cfg<double>(c, "mapping.imu_time_inte", 0.005);
    laser_point_cov       = cfg<double>(c, "mapping.lidar_meas_cov", 0.01);
    acc_cov_input         = cfg<double>(c, "mapping.acc_cov_input", 0.1);
    vel_cov               = cfg<double>(c, "mapping.vel_cov", 20.0);
    gyr_cov_input         = cfg<double>(c, "mapping.gyr_cov_input", 0.01);
    gyr_cov_output        = cfg<double>(c, "mapping.gyr_cov_output", 1000.0);
    acc_cov_output        = cfg<double>(c, "mapping.acc_cov_output", 500.0);
    b_gyr_cov             = cfg<double>(c, "mapping.b_gyr_cov", 0.0001);
    b_acc_cov             = cfg<double>(c, "mapping.b_acc_cov", 0.0001);
    imu_meas_acc_cov      = cfg<double>(c, "mapping.imu_meas_acc_cov", 0.01);
    imu_meas_omg_cov      = cfg<double>(c, "mapping.imu_meas_omg_cov", 0.01);
    p_pre->blind          = cfg<double>(c, "preprocess.blind", 0.5);
    lidar_type            = cfg<int>(c, "preprocess.lidar_type", 1);
    p_pre->N_SCANS        = cfg<int>(c, "preprocess.scan_line", 4);
    p_pre->SCAN_RATE      = cfg<int>(c, "preprocess.scan_rate", 10);
    p_pre->time_unit      = cfg<int>(c, "preprocess.timestamp_unit", 3);
    match_s               = cfg<double>(c, "mapping.match_s", 81.0);
    gravity_align         = cfg<bool>(c, "mapping.gravity_align", true);
    gravity               = cfg<std::vector<double>>(c, "mapping.gravity", {0.0, 0.0, -9.810});
    gravity_init          = cfg<std::vector<double>>(c, "mapping.gravity_init", {0.0, 0.0, -9.810});
    extrinT               = cfg<std::vector<double>>(c, "mapping.extrinsic_T", {0.0, 0.0, 0.0});
    extrinR               = cfg<std::vector<double>>(c, "mapping.extrinsic_R", {1,0,0, 0,1,0, 0,0,1});
    publish_odometry_without_downsample = cfg<bool>(c, "odometry.publish_odometry_without_downsample", false);
    path_en               = cfg<bool>(c, "publish.path_en", true);
    scan_pub_en           = cfg<bool>(c, "publish.scan_publish_en", true);
    scan_body_pub_en      = cfg<bool>(c, "publish.scan_bodyframe_pub_en", false);
    runtime_pos_log       = cfg<bool>(c, "runtime_pos_log_enable", false);
    pcd_save_en           = cfg<bool>(c, "pcd_save.pcd_save_en", false);
    pcd_save_interval     = cfg<int>(c, "pcd_save.interval", -1);

    lidar_type = p_pre->lidar_type = lidar_type;
}
