#include "pumper.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "repub");
    Pumper p;
    ros::spin();
}