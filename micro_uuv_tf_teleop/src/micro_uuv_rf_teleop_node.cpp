#include "micro_uuv_rf_teleop/micro_uuv_rf_teleop.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_node");

    RfTeleop r;

    ros::spin();

    return 0;
}