
#include "ros/ros.h"
#include "ina260ros.h"

int main(int argc, char*argv[])
{

    ros::init(argc, argv, "ina260_node");

    INA260Ros ina260Ros;
    ros::Rate rate(10);

    while(ros::ok()) {
        ina260Ros.measure();
        rate.sleep();
    }

    return 0;
}

