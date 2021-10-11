
#include "ina260ros.h"
#include "i2c_driver.h"
#include "exception"

INA260Ros::INA260Ros() :
        m_nh(),
        m_pnh("~"),
        m_i2c_driver("/dev/i2c-1"),
        m_ina260(&m_i2c_driver, 0x40)
{

    m_publisher = m_nh.advertise<micro_uuv_msgs::Power>("power", 1000);

    m_initialize();
}

void INA260Ros::m_initialize() {

    bool openSuccess = m_i2c_driver.open_i2c_device();
    if(!openSuccess) {
        ROS_ERROR_STREAM("FAILED to open I2C device.");
        throw std::runtime_error("i2c error");
    }

    INA260::Operating_Mode op_mode = INA260::Operating_Mode::current_voltage_continuous;
    INA260::Conversion_Time conv_time_current = INA260::Conversion_Time::t_1100_us;
    INA260::Conversion_Time conv_time_voltage = INA260::Conversion_Time::t_2116_us;
    INA260::Averaging_Mode avg_mode = INA260::Averaging_Mode::samples_0004;
    bool result = m_ina260.set_configuration(op_mode, conv_time_current, conv_time_voltage, avg_mode);
    if (result)
    {
        ROS_INFO("INA260 - set configuration successfully, for I2C address %d\n", m_ina260.get_i2c_address() );
    }
    else
    {
        ROS_ERROR("FAILED - INA260 - set configuration NOT successful for I2C address %d\n", m_ina260.get_i2c_address() );
        throw std::runtime_error("config error");
    }

}

void INA260Ros::measure() {
    float I_measurement = 0.0f;
    float V_measurement = 0.0f;
    float P_measurement = 0.0f;
    bool result_I = m_ina260.get_current_measurement_in_amps(&I_measurement);
    bool result_V = m_ina260.get_voltage_measurement_in_volts(&V_measurement);
    bool result_P = m_ina260.get_power_measurement_in_watts(&P_measurement);

    uint16_t I_measurement_uint16 = 0;
    bool result_I_uint16 = m_ina260.get_current_measurement_as_uint16(&I_measurement_uint16);

    micro_uuv_msgs::Power msg;
    msg.current = I_measurement;
    msg.power = P_measurement;
    msg.voltage = V_measurement;
    msg.header.stamp = ros::Time::now();

    m_publisher.publish(msg);

}
