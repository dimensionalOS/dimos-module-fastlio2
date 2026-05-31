#ifndef LASER_MAPPING_H_
#define LASER_MAPPING_H_

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <sstream>
#include <so3_math.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ikd-Tree/ikd_Tree.h>
#include <yaml-cpp/yaml.h>
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "msgs.h"
#include "fast_lio_debug.hpp"

using custom_messages::ImuConstPtr;
using custom_messages::ImuPtr;
using custom_messages::OdomMsgPtr;

#define CONFIG_FILE_PATH    std::string("../config/horizon.json")
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)

// Reject a point's NN correspondence if the farthest of the
// NUM_MATCH_POINTS neighbours is more than this many SQUARED metres
// away. Default 5 m² → 2.236 m radius. Picks bad matches when the
// IESKF pose prior is already wildly off; tightening helps the loop
// fail loudly (zero effective points → HSHARE_INVALID) instead of
// running away into hundreds of metres of state corruption.
#define NN_CORRESPONDENCE_MAX_SQ_DIST_M2 (5.0)

PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

double solve_time = 0, match_time = 0;

double res_mean_last = 0.05, total_residual = 0.0;

bool extrinsic_est_en = true;

float res_last[100000] = {0.0};

int effct_feat_num = 0;
int feats_down_size = 0;
bool   point_selected_surf[100000] = {0};

vector<PointVector>  Nearest_Points;
KD_TREE<PointType> ikdtree;

class LaserMapping
{
public:
    LaserMapping(const std::string& config_path = CONFIG_FILE_PATH,
                 double msr_freq = 50.0, double main_freq = 5000.0,
                 double guardrail_max_pos_jump_m_ = 0.5,
                 double guardrail_max_accel_norm_ms2_ = 30.0);
    ~LaserMapping();

    void livox_pcl_cbk(const CstMsgConstPtr &msg);
    void imu_cbk(const ImuConstPtr &msg_in);
    void run(OdomMsgPtr &msg_in);

    /** Return the full undistorted scan transformed to world frame. */
    PointCloudXYZI::Ptr get_world_cloud() const {
        if (!feats_undistort || feats_undistort->empty()) { return nullptr; }
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr cloud(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            const PointType &pi = feats_undistort->points[i];
            PointType &po = cloud->points[i];
            V3D p_body(pi.x, pi.y, pi.z);
            V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
            po.x = p_global(0);
            po.y = p_global(1);
            po.z = p_global(2);
            po.intensity = pi.intensity;
        }
        return cloud;
    }

private:
    void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s);
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po);
    void points_cache_collect();
    void lasermap_fov_segment();
    bool sync_packages(MeasureGroup &meas);
    void map_incremental();
    void update_odometry(OdomMsgPtr &msg_in);
    static void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
    {
        // c661fe8 fix restored — UB-clear() experiment in attempt_003 did NOT reproduce.
        double match_start = omp_get_wtime();
        laserCloudOri->resize(feats_down_size);
        corr_normvect->resize(feats_down_size);
        total_residual = 0.0;

        /** closest surface search and residual computation **/
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType &point_body  = feats_down_body->points[i]; 
            PointType &point_world = feats_down_world->points[i]; 

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto &points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > NN_CORRESPONDENCE_MAX_SQ_DIST_M2 ? false : true;
            }

            if (!point_selected_surf[i]) { continue; }

            VF(4) pabcd;
            point_selected_surf[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i] = true;
                    normvec->points[i].x = pabcd(0);
                    normvec->points[i].y = pabcd(1);
                    normvec->points[i].z = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i] = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                total_residual += res_last[i];
                effct_feat_num ++;
            }
        }

        if (effct_feat_num < 1)
        {
            if (fastlio_debug) { fprintf(stderr, "[fastlio] HSHARE_INVALID  reason=no_effective_points\n"); }
            ekfom_data.valid = false;
            return;
        }

        res_mean_last = total_residual / effct_feat_num;
        match_time  += omp_get_wtime() - match_start;
        double solve_start_  = omp_get_wtime();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType &laser_p  = laserCloudOri->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() *norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
        solve_time += omp_get_wtime() - solve_start_;
    }

    template<typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);

    template<typename T>
    void set_posestamp(T & out);

private:
    static const int MAXN = 720000;
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    double solve_const_H_time = 0;
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, path_en = true;

    double msr_freq = 0.0, main_freq = 0.0;
    // Guardrail thresholds set from the FastLio constructor (which gets them
    // from the dimos NativeModule CLI args). Zero or negative disables that
    // particular check.
    //
    // `guardrail_max_accel_norm_ms2` caps the per-update velocity correction
    // magnitude in physical-acceleration units — the check is
    //   (vel_post - vel_pre).norm() / scan_dt > guardrail_max_accel_norm_ms2
    // which lets steady-state high velocities through (a robot on a train at
    // 200 mph: tiny per-scan corrections, never trips) but catches the bug
    // pattern where IESKF applies a 100+ m/s correction in a single update.
    double guardrail_max_pos_jump_m = 0.5;
    double guardrail_max_accel_norm_ms2 = 30.0;
    double timediff_lidar_wrt_imu = 0.0;

    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    double time_diff_lidar_to_imu = 0.0;

    mutex mtx_buffer;
    condition_variable sig_buffer;

    string root_dir = ROOT_DIR;
    string map_file_path, lid_topic, imu_topic;

    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    int    time_log_counter = 0, scan_count = 0, publish_count = 0;
    int    iterCount = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
    bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

    vector<vector<int>>  pointSearchInd_surf; 
    vector<BoxPointType> cub_needrm;
    vector<double>       extrinT;
    vector<double>       extrinR;
    deque<double>                     time_buffer;
    deque<PointCloudXYZI::Ptr>        lidar_buffer;
    deque<ImuConstPtr> imu_buffer;

    PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr _featsArray;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    V3F XAxisPoint_body;
    V3F XAxisPoint_world;
    V3D euler_cur;
    V3D position_last;
    V3D Lidar_T_wrt_IMU;
    M3D Lidar_R_wrt_IMU;

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    state_ikfom last_good_state;        // last accepted post-update state (for guardrail rollback)
    bool last_good_state_valid = false; // false until first non-rejected scan
    double last_scan_end_time = 0.0;    // for computing scan_dt in the accel guardrail check
    vect3 pos_lid;

    custom_messages::Odometry odomAftMapped;
    custom_messages::Quaternion geoQuat;
    custom_messages::PoseStamped msg_body_pose;

    shared_ptr<Preprocess> p_pre;
    shared_ptr<ImuProcess> p_imu;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
};

LaserMapping::LaserMapping(const std::string& config_path, double msr_freq_, double main_freq_,
                           double guardrail_max_pos_jump_m_, double guardrail_max_accel_norm_ms2_) : extrinT(3, 0.0), extrinR(9, 0.0), featsFromMap(new PointCloudXYZI()), feats_undistort(new PointCloudXYZI()),\
                            XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0), XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0),\
                            position_last(Zero3d), Lidar_T_wrt_IMU(Zero3d), Lidar_R_wrt_IMU(Eye3d),\
                            p_pre(new Preprocess()), p_imu(new ImuProcess())
{
    // res_mean_last = 0.05;
    // total_residual = 0.0;
    // feats_down_size = 0;
    // effct_feat_num = 0;
    // match_time = 0.0;
    // extrinsic_est_en = true;
    // solve_time = 0.0;

    // feats_down_body = boost::make_shared<PointCloudXYZI>();
    // feats_down_world = boost::make_shared<PointCloudXYZI>();
    // normvec = boost::make_shared<PointCloudXYZI>(100000, 1);
    // laserCloudOri = boost::make_shared<PointCloudXYZI>(100000, 1);
    // corr_normvect = boost::make_shared<PointCloudXYZI>(100000, 1);

    // read YAML config file
    YAML::Node config = YAML::LoadFile(config_path);
    p_pre->lidar_type             = config["preprocess"]["lidar_type"].as<int>();
    if (p_pre->lidar_type == 2)
    {
        p_pre->SCAN_RATE              = config["preprocess"]["scan_rate"].as<int>();
        p_pre->time_unit              = config["preprocess"]["timestamp_unit"].as<int>();
    }
    time_sync_en                  = config["common"]["time_sync_en"].as<bool>();
    time_diff_lidar_to_imu        = config["common"]["time_offset_lidar_to_imu"].as<double>();
    msr_freq                      = msr_freq_;
    main_freq                     = main_freq_;
    guardrail_max_pos_jump_m      = guardrail_max_pos_jump_m_;
    guardrail_max_accel_norm_ms2  = guardrail_max_accel_norm_ms2_;
    p_pre->N_SCANS                = config["preprocess"]["scan_line"].as<int>();
    p_pre->blind                  = config["preprocess"]["blind"].as<double>();
    acc_cov                       = config["mapping"]["acc_cov"].as<double>();
    gyr_cov                       = config["mapping"]["gyr_cov"].as<double>();
    b_acc_cov                     = config["mapping"]["b_acc_cov"].as<double>();
    b_gyr_cov                     = config["mapping"]["b_gyr_cov"].as<double>();
    fov_deg                       = config["mapping"]["fov_degree"].as<int>();
    DET_RANGE                     = config["mapping"]["det_range"].as<double>();
    extrinsic_est_en              = config["mapping"]["extrinsic_est_en"].as<bool>();
    extrinT                       = config["mapping"]["extrinsic_T"].as<std::vector<double>>();
    extrinR                       = config["mapping"]["extrinsic_R"].as<std::vector<double>>();
    // Optional: align the world frame to measured gravity at IMU init.
    // Defaults to true; set `mapping.gravity_align: false` for legacy behaviour.
    bool gravity_align_en = true;
    if (config["mapping"]["gravity_align"]) {
        gravity_align_en = config["mapping"]["gravity_align"].as<bool>();
    }
    NUM_MAX_ITERATIONS            = 4;
    filter_size_corner_min        = 0.5;
    filter_size_surf_min          = 0.5;
    filter_size_map_min           = 0.5;
    cube_len                      = 200;
    p_pre->point_filter_num       = 2;
    p_pre->feature_enabled        = false;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->gravity_align_en = gravity_align_en;

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
}

LaserMapping::~LaserMapping()
{
    
}

void LaserMapping::pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::update_odometry(OdomMsgPtr &msg_in)
{
    msg_in->header.frame_id = "camera_init";
    msg_in->child_frame_id = "body";
    msg_in->header.stamp = custom_messages::Time().fromSec(lidar_end_time);
    set_posestamp(msg_in->pose);
    // No per-publish print — caller already has pos in the SCAN summary.
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        msg_in->pose.covariance[i*6 + 0] = P(k, 3);
        msg_in->pose.covariance[i*6 + 1] = P(k, 4);
        msg_in->pose.covariance[i*6 + 2] = P(k, 5);
        msg_in->pose.covariance[i*6 + 3] = P(k, 0);
        msg_in->pose.covariance[i*6 + 4] = P(k, 1);
        msg_in->pose.covariance[i*6 + 5] = P(k, 2);
    }

    // msg_in = odomAftMapped;
}

template<typename T>
void LaserMapping::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

template<typename T>
void LaserMapping::set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void LaserMapping::RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

void LaserMapping::lasermap_fov_segment()
{
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;

    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    // No per-call prints — fov_segment is called per-scan but with no important branches
    // to trace. Map-shift events are captured indirectly via the SCAN summary's ikd size delta.
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) { need_move = true; }
    }
    if (!need_move) { return; }
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0) { kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); }
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void LaserMapping::livox_pcl_cbk(const CstMsgConstPtr &msg)
{
    bool   timediff_set_flg = false;

    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    // std::cout << msg->header.stamp.toSec() << std::endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        if (fastlio_debug) { std::cout << "lidar loop back, clear buffer" << std::endl; }
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        if (fastlio_debug) { printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar); }
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        if (fastlio_debug) { printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu); }
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
   //  std::cout << "msg size: " << msg->points.size();
    p_pre->process(msg, ptr);
   //  std::cout << "ptr size: " << ptr->points.size() << std::endl;
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LaserMapping::imu_cbk(const ImuConstPtr &msg_in)
{
    publish_count ++;
   // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
   ImuPtr msg(new custom_messages::Imu(*msg_in));

   if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
   {
      msg->header.stamp = \
      custom_messages::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
   }

   msg->header.stamp = custom_messages::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

   double timestamp = msg->header.stamp.toSec();

   mtx_buffer.lock();

   if (timestamp < last_timestamp_imu)
   {
   //   ROS_WARN("imu loop back, clear buffer");
      imu_buffer.clear();
   }

   last_timestamp_imu = timestamp;

   imu_buffer.push_back(msg);
   mtx_buffer.unlock();
   sig_buffer.notify_all();
}

bool LaserMapping::sync_packages(MeasureGroup &meas)
{
    double lidar_mean_scantime = 0.0;
    int    scan_num = 0;

    // No prints — high call rate (5kHz). Per-scan summary is in LaserMapping::run().
    if (lidar_buffer.empty() || imu_buffer.empty()) { return false; }
    if(!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }
    if (last_timestamp_imu < lidar_end_time) { return false; }
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) { break; }
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void LaserMapping::map_incremental()
{
    int process_increments = 0;

    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) { break; }
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) { PointToAdd.push_back(feats_down_world->points[i]); }
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// static void LaserMapping::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     double match_start = omp_get_wtime();
//     laserCloudOri->clear(); 
//     corr_normvect->clear(); 
//     total_residual = 0.0; 

//     /** closest surface search and residual computation **/
//     #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for
//     #endif
//     for (int i = 0; i < feats_down_size; i++)
//     {
//         PointType &point_body  = feats_down_body->points[i]; 
//         PointType &point_world = feats_down_world->points[i]; 

//         /* transform to world frame */
//         V3D p_body(point_body.x, point_body.y, point_body.z);
//         V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
//         point_world.x = p_global(0);
//         point_world.y = p_global(1);
//         point_world.z = p_global(2);
//         point_world.intensity = point_body.intensity;

//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

//         auto &points_near = Nearest_Points[i];

//         if (ekfom_data.converge)
//         {
//             /** Find the closest surfaces in the map **/
//             ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
//             point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
//         }

//         if (!point_selected_surf[i]) continue;

//         VF(4) pabcd;
//         point_selected_surf[i] = false;
//         if (esti_plane(pabcd, points_near, 0.1f))
//         {
//             float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

//             if (s > 0.9)
//             {
//                 point_selected_surf[i] = true;
//                 normvec->points[i].x = pabcd(0);
//                 normvec->points[i].y = pabcd(1);
//                 normvec->points[i].z = pabcd(2);
//                 normvec->points[i].intensity = pd2;
//                 res_last[i] = abs(pd2);
//             }
//         }
//     }

//     effct_feat_num = 0;

//     for (int i = 0; i < feats_down_size; i++)
//     {
//         if (point_selected_surf[i])
//         {
//             laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
//             corr_normvect->points[effct_feat_num] = normvec->points[i];
//             total_residual += res_last[i];
//             effct_feat_num ++;
//         }
//     }

//     if (effct_feat_num < 1)
//     {
//         ekfom_data.valid = false;
//         //   ROS_WARN("No Effective Points! \n");
//         return;
//     }

//     res_mean_last = total_residual / effct_feat_num;
//     match_time  += omp_get_wtime() - match_start;
//     double solve_start_  = omp_get_wtime();

//     /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
//     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
//     ekfom_data.h.resize(effct_feat_num);

//     for (int i = 0; i < effct_feat_num; i++)
//     {
//         const PointType &laser_p  = laserCloudOri->points[i];
//         V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
//         M3D point_be_crossmat;
//         point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
//         V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
//         M3D point_crossmat;
//         point_crossmat<<SKEW_SYM_MATRX(point_this);

//         /*** get the normal vector of closest surface/corner ***/
//         const PointType &norm_p = corr_normvect->points[i];
//         V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

//         /*** calculate the Measuremnt Jacobian matrix H ***/
//         V3D C(s.rot.conjugate() *norm_vec);
//         V3D A(point_crossmat * C);
//         if (extrinsic_est_en)
//         {
//             V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
//         }
//         else
//         {
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//         }

//         /*** Measuremnt: distance to the closest surface/corner ***/
//         ekfom_data.h(i) = -norm_p.intensity;
//     }
//     solve_time += omp_get_wtime() - solve_start_;
// }

void LaserMapping::run(OdomMsgPtr &msg_in)
{
    if(sync_packages(Measures))
    {
        if (flg_first_scan)
        {
            if (fastlio_debug) { fprintf(stderr, "[fastlio] FIRST_SCAN t=%.3f\n", Measures.lidar_beg_time); }
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return;
        }

        double t0 = omp_get_wtime();
        match_time = 0; kdtree_search_time = 0.0; solve_time = 0;
        solve_const_H_time = 0;

        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty() || (feats_undistort == NULL)) {
            if (fastlio_debug) { fprintf(stderr, "[fastlio] SKIP t=%.3f reason=feats_empty\n", Measures.lidar_beg_time); }
            return;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

        lasermap_fov_segment();

        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        feats_down_size = feats_down_body->points.size();

        if(ikdtree.Root_Node == nullptr) {
            if(feats_down_size > 5) {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                ikdtree.Build(feats_down_world->points);
                if (fastlio_debug) {
                    fprintf(stderr, "[fastlio] IKD_INIT t=%.3f n=%d\n",
                            Measures.lidar_beg_time, feats_down_size);
                }
            }
            return;
        }

        if (feats_down_size < 5) {
            if (fastlio_debug) { fprintf(stderr, "[fastlio] SKIP t=%.3f reason=too_few_down\n", Measures.lidar_beg_time); }
            return;
        }

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true;

        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        // Snapshot state before the IESKF update so we can roll back if it tries to apply
        // an unphysical pose or velocity jump. Caps are sized to the Go2 physical envelope —
        // raise them for faster platforms.
        state_ikfom state_pre_update = kf.get_x();
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();

        // Caps come from the constructor (FastLio2Config in dimos → CLI args
        // on the native binary → FastLio ctor → here). A zero or negative cap
        // disables that particular check.
        //
        // pos_jump check: per-update position correction magnitude (m).
        // accel check: per-update velocity correction magnitude divided by
        //   scan_dt, interpreted as an effective acceleration (m/s²). This
        //   form lets the IESKF track steady-state high velocities — a
        //   platform that physically accelerated to 200 mph would do it via
        //   thousands of clean updates with tiny per-update delta-v, and
        //   never trip the cap. The divergence pattern we're guarding
        //   against is a single-update velocity jump of 100+ m/s, ~1000 m/s²,
        //   which the cap catches regardless of the platform's velocity
        //   envelope.
        const double pos_jump = (state_point.pos - state_pre_update.pos).norm();
        const double dvel = (state_point.vel - state_pre_update.vel).norm();
        // First-scan fallback: no prior scan_end_time yet → assume nominal
        // 100 ms (10 Hz scan rate). Subsequent scans use the actual delta.
        const double scan_dt = (last_scan_end_time > 0.0)
            ? std::max(lidar_end_time - last_scan_end_time, 1e-3)
            : 0.1;
        const double accel = dvel / scan_dt;
        bool guardrail_rejected = false;
        const bool pos_jump_exceeds = guardrail_max_pos_jump_m > 0
            && pos_jump > guardrail_max_pos_jump_m;
        const bool accel_exceeds = guardrail_max_accel_norm_ms2 > 0
            && accel > guardrail_max_accel_norm_ms2;
        if (pos_jump_exceeds || accel_exceeds) {
            fprintf(stderr,
                "[fastlio] guardrail: rejecting scan update (pos_jump=%.2fm, dvel=%.2fm/s, "
                "scan_dt=%.3fs, accel=%.2fm/s^2) — rollback to last good + freeze\n",
                pos_jump, dvel, scan_dt, accel);
            state_ikfom s_clean = last_good_state_valid ? last_good_state : state_pre_update;
            s_clean.vel.setZero();
            kf.change_x(s_clean);
            state_point = s_clean;
            guardrail_rejected = true;
        } else {
            last_good_state = state_point;
            last_good_state_valid = true;
        }
        last_scan_end_time = lidar_end_time;

        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];
        double t_update_end = omp_get_wtime();

        update_odometry(msg_in);
        // Skip map insertion on rejected scans so the kdtree doesn't accumulate points at a
        // divergent pose — this is what breaks FAST-LIO's reinforcing-loop divergence (bad
        // scan inserted → next NN search confirms wrong pose → snowball).
        if (!guardrail_rejected) {
            map_incremental();
        }
        double t5 = omp_get_wtime();

        // ONE compact summary line per scan — everything we need to trace divergence
        if (fastlio_debug) {
            fprintf(stderr,
                "[fastlio] SCAN t=%.3f pos=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f) bg=(%.2e,%.2e,%.2e) ba=(%.2e,%.2e,%.2e) "
                "feats_d=%d effct=%d res=%.4f iter_ms=%.1f tot_ms=%.1f ikd=%d add=%d inited=%d\n",
                Measures.lidar_beg_time,
                state_point.pos[0], state_point.pos[1], state_point.pos[2],
                euler_cur[0]*57.2958, euler_cur[1]*57.2958, euler_cur[2]*57.2958,
                state_point.bg[0], state_point.bg[1], state_point.bg[2],
                state_point.ba[0], state_point.ba[1], state_point.ba[2],
                feats_down_size, effct_feat_num, res_mean_last,
                (t_update_end-t_update_start)*1000, (t5-t0)*1000,
                ikdtree.size(), add_point_size, (int)flg_EKF_inited);
        }
    }
}

#endif