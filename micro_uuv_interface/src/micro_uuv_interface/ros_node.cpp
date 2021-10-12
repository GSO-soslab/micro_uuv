
#include "ros_node.h"
#include "moos_node.h"
#include "tf2/LinearMath/Quaternion.h"

namespace soslab {

    ROSNode::ROSNode() :
        m_nh(),
        m_pnh("~")
    {
        m_nav_msg.header.seq = 0;
        m_ivp_helm_state_msg.header.seq = 0;

        // Publishers
        m_nav_publisher = m_pnh.advertise<micro_uuv_msgs::Nav>("pnav_state", 1000);
        m_ivp_helm_state_publisher = m_pnh.advertise<micro_uuv_msgs::IvpHelmState>("helm_state", 1000);
        m_imu_publisher = m_pnh.advertise<sensor_msgs::Imu>("embedded_imu/data", 1000);
        m_ms_imu_publisher = m_pnh.advertise<sensor_msgs::Imu>("imu/data", 1000);
        m_gps_publisher = m_pnh.advertise<micro_uuv_msgs::Gps>("gps",1000);

        m_pressure_publisher = m_pnh.advertise<micro_uuv_msgs::Pressure>("ps",1000);
        m_mag_publisher = m_pnh.advertise<sensor_msgs::MagneticField>("imu/mag",1000);

        // Subscribers
        m_dvl_subscriber = m_pnh.subscribe("transducer_report",1000, &ROSNode::dvlCallback, this);

        // Services
        m_wpt_service = m_pnh.advertiseService("send_waypoint", &ROSNode::wayPointService, this);
        m_dep_service = m_pnh.advertiseService("send_depth", &ROSNode::depthService, this);
        m_helm_state_service = m_pnh.advertiseService("set_helm_state", &ROSNode::ivpHelmConditionService, this);
        m_manual_overide_service = m_pnh.advertiseService("set_manual_overide", &ROSNode::manualOverideService, this);
        m_calibration_service = m_pnh.advertiseService("start_calibration", &ROSNode::startCalibration, this);

    }

    void ROSNode::Initialize()
    {
        if(not m_moosNode)
        {
            throw std::runtime_error("MOOS Node should be set up before running ROS Node");
        }

        std::thread t1([&](){
            ros::Rate r(10);
            while(ros::ok()) {
                PublishIvpHelmState();
                r.sleep();
            }
        });
        t1.detach();
    }

    bool ROSNode::wayPointService(micro_uuv_msgs::SetWayPointRequest& req,
                                  micro_uuv_msgs::SetWayPointResponse& res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        if(req.list.empty()) {
            return false;
        } else {
            m_pool->waypoint.update.clear();
            m_pool->waypoint.update += "points = pts={";
            for(const auto& point : req.list) {
                m_pool->waypoint.update += std::to_string(point.x) + "," + std::to_string(point.y) + ":";
            }
            m_pool->waypoint.update += "}";
        }
        return m_moosNode->publishWayPointUpdate();
    }

    bool ROSNode::depthService(micro_uuv_msgs::SetDepth::Request &req,
                               micro_uuv_msgs::SetDepth::Response &res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        m_pool->depth.update = "depth = " + std::to_string(req.depth);

        return m_moosNode->publishDepthUpdate();
    }

    bool ROSNode::startCalibration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        return m_moosNode->triggerCalibration();
    }

    bool ROSNode::ivpHelmConditionService(micro_uuv_msgs::SetHelmCondition::Request &req,
                                          micro_uuv_msgs::SetHelmCondition::Response &res) {
        const std::lock_guard<std::mutex> lock(m_pool->lock);

        auto r = std::find(
                m_pool->helm_status.condition_vars.begin(),
                m_pool->helm_status.condition_vars.end(),
                req.name
        );

        if(r == m_pool->helm_status.condition_vars.end()) {
            return false;
        } else {
            return m_moosNode->publishIvpHelmUpdate(req.name, req.value);
        }
    }

    bool ROSNode::manualOverideService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        res.success = m_moosNode->publishManualOveride(req.data);

        return res.success;
    }


    void ROSNode::PublishNav() {

        m_nav_msg.header.seq += 1;
        m_nav_msg.header.stamp = ros::Time().fromSec(m_pool->nav.time);
        m_nav_msg.x = m_pool->nav.x;
        m_nav_msg.y = m_pool->nav.y;
        m_nav_msg.z = m_pool->nav.z;
        m_nav_msg.depth = m_pool->nav.depth;
        m_nav_msg.longitude = m_pool->nav.longitude;
        m_nav_msg.latitude = m_pool->nav.latitude;
        m_nav_msg.heading = m_pool->nav.heading;
        m_nav_msg.speed = m_pool->nav.speed;

        m_nav_publisher.publish(m_nav_msg);
    }

    void ROSNode::PublishImu() {
        m_imu_msg.header.seq += 1;

        m_imu_msg.header.stamp = ros::Time().fromSec(m_pool->imu.time);
        tf2::Quaternion q;
        q.setRPY(m_pool->imu.roll, m_pool->imu.pitch, m_pool->imu.yaw);
        m_imu_msg.orientation.w = q.getW();
        m_imu_msg.orientation.x = q.getX();
        m_imu_msg.orientation.y = q.getY();
        m_imu_msg.orientation.z = q.getZ();

        // TODO: This comes as velocity, maybe we should turn this into acceleration
        m_imu_msg.linear_acceleration.x = m_pool->imu.x_vel;
        m_imu_msg.linear_acceleration.y = m_pool->imu.z_vel;
        m_imu_msg.linear_acceleration.z = m_pool->imu.y_vel;

        m_imu_msg.angular_velocity.x = m_pool->imu.x_gyro;
        m_imu_msg.angular_velocity.y = m_pool->imu.y_gyro;
        m_imu_msg.angular_velocity.z = m_pool->imu.z_gyro;

        m_imu_publisher.publish(m_imu_msg);
    }


    void ROSNode::PublishMsImu() {
        m_ms_imu_msg.header.seq += 1;

        m_ms_imu_msg.header.stamp = ros::Time().fromSec(m_pool->ms_imu.time);
        m_ms_imu_msg.orientation.w = m_pool->ms_imu.w_quat;
        m_ms_imu_msg.orientation.x = m_pool->ms_imu.x_quat;
        m_ms_imu_msg.orientation.y = m_pool->ms_imu.y_quat;
        m_ms_imu_msg.orientation.z = m_pool->ms_imu.z_quat;

        // TODO: This comes as velocity, maybe we should turn this into acceleration
        m_ms_imu_msg.linear_acceleration.x = m_pool->ms_imu.x_accel;
        m_ms_imu_msg.linear_acceleration.y = m_pool->ms_imu.z_accel;
        m_ms_imu_msg.linear_acceleration.z = m_pool->ms_imu.y_accel;

        m_ms_imu_msg.angular_velocity.x = m_pool->ms_imu.x_gyro;
        m_ms_imu_msg.angular_velocity.y = m_pool->ms_imu.y_gyro;
        m_ms_imu_msg.angular_velocity.z = m_pool->ms_imu.z_gyro;

        m_ms_imu_publisher.publish(m_ms_imu_msg);
    }

    void ROSNode::PublishGps() {
        m_gps_msg.header.seq += 1;
        m_gps_msg.header.stamp = ros::Time().fromSec(m_pool->gps.time);

        m_gps_msg.latitude = m_pool->gps.latitude;
        m_gps_msg.longitude = m_pool->gps.longitude;
        m_gps_msg.quality = m_pool->gps.quality;
        m_gps_msg.sat = m_pool->gps.sat;
        m_gps_msg.hdop = m_pool->gps.hdop;
        m_gps_msg.fix = m_pool->gps.fix;
        m_gps_msg.heading = m_pool->gps.heading;
        m_gps_msg.origin_latitude = m_pool->gps.origin_latitude;
        m_gps_msg.origin_longitude = m_pool->gps.origin_longitude;
        m_gps_msg.x = m_pool->gps.x;
        m_gps_msg.y = m_pool->gps.y;

        m_gps_publisher.publish(m_gps_msg);
    }

    void ROSNode::PublishIvpHelmState() {
        m_ivp_helm_state_msg.header.seq += 1;
        m_ivp_helm_state_msg.header.stamp = ros::Time().fromSec(m_pool->helm_status.time);
        m_ivp_helm_state_msg.state = m_pool->helm_status.state;

        m_ivp_helm_state_msg.condition_vars.clear();
        for(const auto &e : m_pool->helm_status.conditions) {
            micro_uuv_msgs::BoolMap m;
            m.key = e.first;
            m.value = e.second;
            m_ivp_helm_state_msg.condition_vars.push_back(m);
        }

        m_ivp_helm_state_msg.update_vars.clear();
        for(const auto& e : m_pool->helm_status.update_vars) {
            micro_uuv_msgs::StringMap m;
            m.key = e.first;
            m.value = e.second;
            m_ivp_helm_state_msg.update_vars.push_back(m);
        }

        m_ivp_helm_state_msg.manual_overide = m_pool->helm_status.manual_overide;

        m_ivp_helm_state_msg.allstop_msg = m_pool->helm_status.allstop_msg;

        m_ivp_helm_state_publisher.publish(m_ivp_helm_state_msg);
    }

    void ROSNode::PublishMag() {
        m_mag.header.seq += 1;
        m_mag.header.stamp = ros::Time().fromSec(m_pool->mag.time);

        m_mag.magnetic_field.x = m_pool->mag.x;
        m_mag.magnetic_field.y = m_pool->mag.y;
        m_mag.magnetic_field.z = m_pool->mag.z;

        m_mag_publisher.publish(m_mag);
    }

    void ROSNode::PublishPressure() {
        m_pressure.header.seq += 1;
        m_pressure.header.stamp = ros::Time().fromSec(m_pool->ps.time);

        m_pressure.status = m_pool->ps.status;
        m_pressure.depth = m_pool->ps.depth;
        m_pressure.filtered_depth = m_pool->ps.filtered_depth;
        m_pressure.temp = m_pool->ps.temp;
        m_pressure.pressure = m_pool->ps.pressure;

        m_pressure_publisher.publish(m_pressure);
    }

    void ROSNode::dvlCallback(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        m_pool->dvl.time = msg->header.stamp.toSec();
        m_pool->dvl.x = msg->report.vx;
        m_pool->dvl.y = msg->report.vy;
        m_pool->dvl.z = msg->report.vz;
        m_moosNode->publishDvl();
    }
}
