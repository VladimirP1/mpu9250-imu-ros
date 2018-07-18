#pragma once
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <hid_mpu.h>
#include <vector>

class SyncIMU : public MPU {
    struct {
        uint64_t offset = 0;
        int64_t for_lpf = 0;
        //double for_lpf2 = 9000.0;
        int corr_timer = 0;
    } time_calibration;
    uint64_t calibrate_time();
    ros::Time sync_to_ros(uint64_t us);
    uint64_t average(const std::vector<uint64_t>& data);
public:
    sensor_msgs::Imu get_next_msg();
    friend int main(int argc, char**argv);
};