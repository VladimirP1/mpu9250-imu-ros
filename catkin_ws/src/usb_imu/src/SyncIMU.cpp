#include "SyncIMU.h"

uint64_t SyncIMU::calibrate_time() {
    std::vector<uint64_t> offsets;
    for(int i = 0; i < 1024; i++) {
        MPU_sample sample = get_next_sample();
        uint64_t time_ros = ros::Time::now().toNSec();
        uint64_t time_sample = sample.time_us * 1000ULL;
        offsets.push_back(time_ros - time_sample);
    }
    return time_calibration.offset = average(offsets);
}

uint64_t SyncIMU::average(const std::vector<uint64_t>& data) {
    vector<uint64_t> newdata;
    for(size_t i = 0; i < data.size(); i+=2) {
        newdata.push_back((data[i] + data[i + 1]) / 2LL);
    }
    if(newdata.size() > 1) {
        return average(newdata);
    } else if (newdata.size() == 1) {
        return newdata.back();
    }
    return 0;
}

ros::Time SyncIMU::sync_to_ros(uint64_t us) {
    uint64_t time_ros = ros::Time::now().toNSec();
    uint64_t time_sample = us * 1000ULL;

    int64_t offset = time_ros - time_sample;
    int64_t error_inst = offset - time_calibration.offset;
    

    if(abs(error_inst) > 4000000ULL) {
        std::cout << error_inst << " " << offset << " " << us << " " << time_calibration.for_lpf << std::endl;
        std::cout << "Warning: high jitter" << std::endl;
    }
    time_calibration.corr_timer++;
    time_calibration.for_lpf =time_calibration.for_lpf / 10000.0 * 9999.0
                             + error_inst / 10000.0;

    if(time_calibration.corr_timer % 100 == 0) {
        time_calibration.offset = time_calibration.offset + time_calibration.for_lpf * (1.0 / 100.0);
    }
    
    time_sample = time_sample + time_calibration.offset;

    ros::Time ret; ret.fromNSec(time_sample);
    return ret;
}
double lpf = 0;
sensor_msgs::Imu SyncIMU::get_next_msg() {
    static sensor_msgs::Imu msg;
    MPU_sample sample = get_next_sample_cal();
    //msg.header.seq++;
    msg.header.stamp = sync_to_ros(sample.time_us);
    msg.header.frame_id = "world";

    msg.linear_acceleration.x = sample.acc_g[0] * 9.81;
    msg.linear_acceleration.y = sample.acc_g[1] * 9.81;
    msg.linear_acceleration.z = sample.acc_g[2] * 9.81;
    //msg.linear_acceleration_covariance = {0.001, 0, 0, 0, 0.001, 0, 0 , 0, 0.001};

    msg.angular_velocity.x = sample.gyr_degs[0] / 180.0 * M_PI;
    msg.angular_velocity.y = sample.gyr_degs[1] / 180.0 * M_PI;
    msg.angular_velocity.z = sample.gyr_degs[2] / 180.0 * M_PI;
    //msg.angular_velocity_covariance = {0.001, 0, 0, 0, 0.001, 0, 0 , 0, 0.001};

    std::cout << lpf << std::endl;
    lpf = lpf*0.999 + arma::norm(sample.acc_g)*0.001;
    //std::cout << sample.gyr_degs[0] << " " << sample.gyr_degs[1] << " " << sample.gyr_degs[2] << std::endl;

    msg.orientation_covariance = {99999.9,0,0,0,99999.9,0,0,0,99999.9};
    return msg;
}