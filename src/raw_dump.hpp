#ifndef RAW_DUMP_H_
#define RAW_DUMP_H_

// Raw sensor data recorder for FastLIO2.
//
// When enabled (via enable(path, duration_sec)), records all incoming IMU
// and lidar data into pre-allocated in-RAM buffers. After duration_sec
// elapses, serializes the buffer to a binary file in a background thread.
//
// Binary format:
//   Header: "FLIO_RAW\0" (8 bytes) + version(u32) + imu_count(u64) + lidar_scan_count(u64)
//   IMU records: [timestamp_sec(f64), acc_x(f64), acc_y(f64), acc_z(f64),
//                 gyr_x(f64), gyr_y(f64), gyr_z(f64)] = 56 bytes each
//   Lidar scans: [timestamp_sec(f64), point_count(u32), padding(u32),
//                  points: [x(f32), y(f32), z(f32), reflectivity(u16),
//                           offset_time_ns(u32), line(u16)] = 20 bytes each]

#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdio>
#include "msgs.h"

using custom_messages::ImuConstPtr;
using custom_messages::CstMsgConstPtr;

struct RawImuSample {
    double timestamp_sec;
    double acc_x, acc_y, acc_z;
    double gyr_x, gyr_y, gyr_z;
};

struct RawLidarPoint {
    float x, y, z;
    uint16_t reflectivity;
    uint32_t offset_time_ns;
    uint16_t line;
};

struct RawLidarScan {
    double timestamp_sec;
    std::vector<RawLidarPoint> points;
};

class RawDump {
public:
    RawDump() = default;
    ~RawDump() {
        if (dump_thread_.joinable()) dump_thread_.join();
    }

    void enable(const std::string& output_path, double duration_sec = 120.0) {
        output_path_ = output_path;
        duration_sec_ = duration_sec;
        enabled_ = true;
        start_time_ = std::chrono::steady_clock::now();

        // Pre-allocate for ~2 min at typical rates
        // IMU: 200 Hz × 120s = 24000 samples × 56 bytes = ~1.3 MB
        imu_samples_.reserve(30000);
        // Lidar: 10 Hz × 120s = 1200 scans (points allocated per-scan)
        lidar_scans_.reserve(1500);

        printf("[raw_dump] Recording enabled → %s (%.0fs)\n",
               output_path_.c_str(), duration_sec_);
        fflush(stdout);
    }

    void record_imu(const ImuConstPtr& msg) {
        if (!enabled_ || dumped_) return;
        check_duration();

        RawImuSample sample;
        sample.timestamp_sec = msg->header.stamp.toSec();
        sample.acc_x = msg->linear_acceleration.x;
        sample.acc_y = msg->linear_acceleration.y;
        sample.acc_z = msg->linear_acceleration.z;
        sample.gyr_x = msg->angular_velocity.x;
        sample.gyr_y = msg->angular_velocity.y;
        sample.gyr_z = msg->angular_velocity.z;
        imu_samples_.push_back(sample);
    }

    void record_lidar(const CstMsgConstPtr& msg) {
        if (!enabled_ || dumped_) return;
        check_duration();

        RawLidarScan scan;
        scan.timestamp_sec = msg->header.stamp.toSec();
        scan.points.resize(msg->points.size());
        for (size_t i = 0; i < msg->points.size(); i++) {
            auto& p = scan.points[i];
            const auto& src = msg->points[i];
            p.x = static_cast<float>(src.x);
            p.y = static_cast<float>(src.y);
            p.z = static_cast<float>(src.z);
            p.reflectivity = src.reflectivity;
            p.offset_time_ns = static_cast<uint32_t>(src.offset_time);
            p.line = src.line;
        }
        lidar_scans_.push_back(std::move(scan));
    }

private:
    void check_duration() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();
        if (elapsed >= duration_sec_ && !dumped_) {
            dumped_ = true;
            // Serialize in background thread to avoid stalling the main loop
            dump_thread_ = std::thread([this]() { serialize(); });
        }
    }

    void serialize() {
        printf("[raw_dump] Serializing %zu IMU samples + %zu lidar scans...\n",
               imu_samples_.size(), lidar_scans_.size());
        fflush(stdout);

        std::ofstream out(output_path_, std::ios::binary);
        if (!out.is_open()) {
            fprintf(stderr, "[raw_dump] ERROR: cannot open %s\n", output_path_.c_str());
            return;
        }

        // Header
        const char magic[8] = {'F','L','I','O','_','R','A','W'};
        uint32_t version = 1;
        uint64_t imu_count = imu_samples_.size();
        uint64_t lidar_count = lidar_scans_.size();
        out.write(magic, 8);
        out.write(reinterpret_cast<const char*>(&version), 4);
        out.write(reinterpret_cast<const char*>(&imu_count), 8);
        out.write(reinterpret_cast<const char*>(&lidar_count), 8);

        // IMU data (contiguous, fixed-size records)
        out.write(reinterpret_cast<const char*>(imu_samples_.data()),
                  imu_samples_.size() * sizeof(RawImuSample));

        // Lidar scans (variable-length: header + points)
        for (const auto& scan : lidar_scans_) {
            uint32_t point_count = static_cast<uint32_t>(scan.points.size());
            uint32_t padding = 0;
            out.write(reinterpret_cast<const char*>(&scan.timestamp_sec), 8);
            out.write(reinterpret_cast<const char*>(&point_count), 4);
            out.write(reinterpret_cast<const char*>(&padding), 4);
            out.write(reinterpret_cast<const char*>(scan.points.data()),
                      scan.points.size() * sizeof(RawLidarPoint));
        }

        out.close();

        // Compute file size
        std::ifstream check(output_path_, std::ios::ate | std::ios::binary);
        auto size_bytes = check.tellg();
        printf("[raw_dump] Wrote %s (%.1f MB, %zu IMU + %zu scans)\n",
               output_path_.c_str(),
               static_cast<double>(size_bytes) / (1024.0 * 1024.0),
               imu_samples_.size(), lidar_scans_.size());
        fflush(stdout);

        // Free memory
        imu_samples_.clear();
        imu_samples_.shrink_to_fit();
        lidar_scans_.clear();
        lidar_scans_.shrink_to_fit();
    }

    std::string output_path_;
    double duration_sec_ = 120.0;
    bool enabled_ = false;
    std::atomic<bool> dumped_ = {false};
    std::chrono::steady_clock::time_point start_time_;
    std::vector<RawImuSample> imu_samples_;
    std::vector<RawLidarScan> lidar_scans_;
    std::thread dump_thread_;
};

#endif
