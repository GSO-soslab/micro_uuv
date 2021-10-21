#include "micro_uuv_rf_teleop/micro_uuv_rf_teleop_remote.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_remote_node");

    RfRemote r;

    ros::spin();

    return 0;
}