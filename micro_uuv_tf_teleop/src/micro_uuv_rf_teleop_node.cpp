#include "micro_uuv_rf_teleop/micro_uuv_rf_teleop_robot.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_node");

    RfTeleopRobot r;

    ros::spin();

    return 0;
}