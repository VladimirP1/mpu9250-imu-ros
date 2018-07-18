#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <hid_mpu.h>
#include "SyncIMU.h"
#include "init_rt.h"


int main(int argc, char**argv) {
    realtime_priority::init();
    ros::init(argc, argv, "usb_imu");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    
    std::ios::sync_with_stdio(false);
    try {
        SyncIMU hid;
        hid.open(0x0483, 0x5710);
        hid.load_calibration("/home/ros/src/calib.bin");
        hid.calibrate_gyr();
        hid.calibrate_time();
        while(1) {
            pub.publish(hid.get_next_msg());
            //ros::spinOnce();
        }
    } catch(HID_exception &e) {
        std::cout << e.get_desc() << std::endl;
        return 1;
    }
    return 0;
}