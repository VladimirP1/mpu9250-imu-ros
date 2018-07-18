#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

class Pumper {
    public:    
    Pumper() {
        sub = nh.subscribe("/imu/data", 1000, &Pumper::msgCallBack, this);
        pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    }
    void msgCallBack(const sensor_msgs::Imu& msg) {
        visualization_msgs::Marker mrk;
        mrk.header.frame_id = "world";
        mrk.header.stamp = ros::Time::now();
        mrk.ns = "my_namespace";
        mrk.id = 0;
        mrk.type = visualization_msgs::Marker::CUBE;
        mrk.action = visualization_msgs::Marker::ADD;
        mrk.pose.position.x = 0;
        mrk.pose.position.y = 0;
        mrk.pose.position.z = 0;
        mrk.pose.orientation = msg.orientation;
        mrk.scale.x = 0.5;
        mrk.scale.y = 0.5;
        mrk.scale.z = 0.5;
        mrk.color.a = 1.0; // Don't forget to set the alpha!
        mrk.color.r = 0.0;
        mrk.color.g = 1.0;
        mrk.color.b = 0.0;
        pub.publish( mrk );
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};