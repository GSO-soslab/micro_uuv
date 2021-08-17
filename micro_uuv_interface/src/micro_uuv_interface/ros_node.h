#ifndef ROSNODE_H_
#define ROSNODE_H_

#include <utility>
#include <thread>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

#include "pool.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/MagneticField.h"

// MSG
#include "micro_uuv_msgs/Nav.h"
#include "micro_uuv_msgs/IvpHelmState.h"
#include "micro_uuv_msgs/Gps.h"
#include "micro_uuv_msgs/Pressure.h"
#include "dvl_a50_ros/DVL.h"

// SRV
#include "micro_uuv_msgs/SetWayPoint.h"
#include "micro_uuv_msgs/SetDepth.h"
#include "micro_uuv_msgs/SetHelmCondition.h"


namespace soslab {

    class MOOSNode;

    class ROSNode {
    private:
        micro_uuv_msgs::Nav m_nav_msg;
        micro_uuv_msgs::Gps m_gps_msg;
        sensor_msgs::Imu m_imu_msg;
        sensor_msgs::Imu m_ms_imu_msg;

        sensor_msgs::MagneticField m_mag;

        micro_uuv_msgs::Pressure m_pressure;

        micro_uuv_msgs::IvpHelmState m_ivp_helm_state_msg;

    protected:

        ros::NodeHandle m_nh;
        ros::NodeHandle m_pnh;

        ros::Publisher m_nav_publisher;

        ros::Publisher m_imu_publisher;

        ros::Publisher m_ms_imu_publisher;

        ros::Publisher m_gps_publisher;

        ros::Publisher m_pressure_publisher;

        ros::Publisher m_ivp_helm_state_publisher;

        ros::Publisher m_mag_publisher;

        ros::Subscriber m_dvl_subscriber;

        /** @brief: Receive and transmit way point to moos waypoint behavior */
        ros::ServiceServer m_wpt_service;

        /** @brief: Receive and transmit depth to moos constant depth behavior */
        ros::ServiceServer m_dep_service;

        /** @brief: Set helm state */
        ros::ServiceServer m_helm_state_service;

        /** @brief: m_manual_overide_service */
        ros::ServiceServer m_manual_overide_service;

        ros::ServiceServer m_calibration_service;

        std::shared_ptr<MOOSNode> m_moosNode;

        std::shared_ptr<pool_t> m_pool;

        bool wayPointService(micro_uuv_msgs::SetWayPoint::Request& req,
                             micro_uuv_msgs::SetWayPoint::Response& res);

        bool depthService(micro_uuv_msgs::SetDepth::Request& req,
                          micro_uuv_msgs::SetDepth::Response& res);

        bool ivpHelmConditionService(micro_uuv_msgs::SetHelmCondition::Request& req,
                                     micro_uuv_msgs::SetHelmCondition::Response& res);

        bool startCalibration(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool manualOverideService(std_srvs::SetBool::Request& req,
                                  std_srvs::SetBool::Response& res);

        void dvlCallback(const dvl_a50_ros::DVL::ConstPtr& msg);

    public:

        ROSNode();

        void Initialize();

        void SetMoosNode(std::shared_ptr<MOOSNode> n) { m_moosNode = n; }

        void SetPool(std::shared_ptr<pool_t> p) { m_pool = p; }

        void PublishNav();

        void PublishIvpHelmState();

        void PublishImu();

        void PublishGps();

        void PublishMsImu();

        void PublishPressure();

        void PublishMag();

    };
    
}

#endif
