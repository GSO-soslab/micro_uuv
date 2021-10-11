#ifndef INA260_INA260ROS_H
#define INA260_INA260ROS_H

#include "ros/ros.h"
#include "micro_uuv_msgs/Power.h"
#include "i2c_driver.h"
#include "ina260.h"

class INA260Ros {
private:

    I2C_Driver m_i2c_driver;

    INA260 m_ina260;

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_publisher;


    void m_initialize();

public:

    INA260Ros();

    void measure();
};


#endif //INA260_INA260ROS_H
