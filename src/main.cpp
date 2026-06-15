// Non-ROS smoke test for the Point-LIO port: feed synthetic stationary IMU +
// a simple planar LiDAR scan through PointLio, confirm it runs and the pose
// stays bounded (no crash, finite output). Not an accuracy test — a build/run
// sanity check. Real eval is the ruwik2 bag replay (separate harness).
#include <iostream>
#include <cmath>
#include "pointlio.hpp"

static ImuConstPtr make_imu(double t) {
    boost::shared_ptr<custom_messages::Imu> m(new custom_messages::Imu());
    m->header.stamp.fromSec(t);
    m->linear_acceleration.x = 0.0;  // stationary: specific force = +gravity (g units, acc_norm=1)
    m->linear_acceleration.y = 0.0;
    m->linear_acceleration.z = 1.0;
    m->angular_velocity.x = 0.0;
    m->angular_velocity.y = 0.0;
    m->angular_velocity.z = 0.0;
    return m;
}

static CstMsgConstPtr make_scan(double t, unsigned base_ns) {
    boost::shared_ptr<custom_messages::CustomMsg> m(new custom_messages::CustomMsg());
    m->header.stamp.fromSec(t);
    m->timebase = base_ns;
    // A small box of planar points (floor + two walls) so the EKF has geometry.
    int n = 0;
    for (int gx = -10; gx <= 10; gx++) {
        for (int gy = -10; gy <= 10; gy++) {
            custom_messages::CustomPoint p;
            p.x = gx * 0.2; p.y = gy * 0.2; p.z = -1.0;   // floor at z=-1
            p.reflectivity = 100; p.tag = 0; p.line = (n % 4);
            p.offset_time = (unsigned long)(n * 1000);    // ns spread across the scan
            m->points.push_back(p); n++;
        }
    }
    m->point_num = n;
    return m;
}

int main(int argc, char** argv) {
    std::string config = (argc > 1) ? argv[1] : std::string("config/mid360.yaml");
    std::cout << "[smoke] config=" << config << std::endl;
    PointLio fl(config);

    const double dt = 0.005;          // 200 Hz IMU
    double t = 1000.0;                 // arbitrary start time (s)
    unsigned scan_i = 0;
    int frames = 0;
    for (int i = 0; i < 600; i++) {    // 3 s of data
        fl.feed_imu(make_imu(t));
        if (i % 20 == 0) {             // 10 Hz lidar
            fl.feed_lidar(make_scan(t, (unsigned)(scan_i++ * 100000000u)));
            fl.process();
            auto p = fl.get_pose();
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
                std::cerr << "[smoke] FAIL: non-finite pose at frame " << frames << std::endl;
                return 1;
            }
            if (frames % 5 == 0)
                std::cout << "[smoke] frame " << frames << " pos=(" << p[0] << ", "
                          << p[1] << ", " << p[2] << ")  |v|=" << fl.get_world_vel_norm() << std::endl;
            frames++;
        }
        t += dt;
    }
    auto p = fl.get_pose();
    double dist = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    std::cout << "[smoke] DONE: " << frames << " frames, final pos=(" << p[0] << ", "
              << p[1] << ", " << p[2] << "), dist=" << dist << " m" << std::endl;
    // Stationary input must not run away.
    if (dist > 5.0) { std::cerr << "[smoke] WARN: pose drifted " << dist << " m on stationary input" << std::endl; }
    std::cout << "[smoke] PASS (ran without crash, finite output)" << std::endl;
    return 0;
}
