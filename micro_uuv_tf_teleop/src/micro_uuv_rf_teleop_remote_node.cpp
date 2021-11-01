#include "micro_uuv_rf_teleop/micro_uuv_rf_teleop_operator.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_remote_node");

    RfRemoteOperator r;

    ros::spin();

    return 0;
}