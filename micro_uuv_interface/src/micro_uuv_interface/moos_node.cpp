#include "moos_node.h"
#include "pool.h"

#include <thread>
#include <functional>

#include "ros_node.h"
#include "utils.hpp"

using namespace soslab;

void MOOSNode::Init(const char *MissionFile)
{
    if(not m_rosNode)
    {
        throw std::runtime_error("ROS Node should be set up before running MOOS Node");
    }

    th = std::thread(
        [this, MissionFile] {
            Run("iRosBridge", MissionFile);
        }
    );
}

bool MOOSNode::toMOOS(const std::string& moosName, double data)
{
    m_Comms.Notify(moosName, data, MOOSTime());
    return true;
}

bool MOOSNode::toMOOS(const std::string& moosName, std::string myString)
{
    m_Comms.Notify(moosName, myString, MOOSTime());
    return true;
}

//Support for binary-string from ROS to MOOS
bool MOOSNode::toMOOSBinaryString(const std::string& moosName, std::string myString)
{
    m_Comms.Notify(moosName, (void *)myString.c_str(), myString.size(), MOOSTime());
    return true;
}

bool MOOSNode::OnNewMail (MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;
    for( p = NewMail.begin() ; p != NewMail.end() ; p++ )
    {

        // todo: parser for each sensor measurements
        // todo: parser for helm behaviours
        
        CMOOSMsg & rMsg = *p;

        Translate(rMsg);
        
        // SENSORY INFORMATION INPUT
        rMsg.GetTime();
        rMsg.GetType();
        rMsg.GetKey();
        rMsg.IsDouble();
        rMsg.IsString();

    }
    return true;
}

/*
   called by the base class when the application has made contact with
   the MOOSDB and a channel has been opened . Place code to specify what
   notifications you want to receive here .
*/
bool MOOSNode::OnConnectToServer()
{
    DoRegistrations();
    return true;
}

/*
  Called by the base class periodically. This is where you place code
  which does the work of the application
*/
bool MOOSNode::Iterate()
{
    return true;
}

/*
   called by the base class before the first :: Iterate is called . Place
   startup code here − especially code which reads configuration data from the
   mission file
*/
bool MOOSNode::OnStartUp()
{
    appTick = 1000;
    commsTick = 1000;

    if(!m_MissionReader.GetConfigurationParam("AppTick",appTick))
    {
        MOOSTrace("Warning, AppTick not set.\n");
    }

    if(!m_MissionReader.GetConfigurationParam("CommsTick",commsTick))
    {
        MOOSTrace("Warning, CommsTick not set.\n");
    }

    SetAppFreq(appTick);
    SetCommsFreq(commsTick);

    DoRegistrations();

    return true;
}

void MOOSNode::DoRegistrations()
{
    // take 'em all
    m_Comms.Register("*","*", 0);
}

void MOOSNode::Translate(CMOOSMsg &msg) {
    const auto &key = msg.GetKey();
    const std::lock_guard<std::mutex> lock(m_pool->lock);

    // NAV
    if (key == "NAV_X") {
        m_pool->nav.x = msg.GetDouble();
        m_pool->nav._fill.x |= 1;
    } else if (key == "NAV_Y") {
        m_pool->nav.y = msg.GetDouble();
        m_pool->nav._fill.y |= 1;
    } else if (key == "NAV_Z") {
        m_pool->nav.z = msg.GetDouble();
        m_pool->nav._fill.z |= 1;
    } else if (key == "NAV_ROLL") {
        m_pool->nav.roll = msg.GetDouble();
        m_pool->nav._fill.roll |= 1;
    } else if (key == "NAV_PITCH") {
        m_pool->nav.pitch = msg.GetDouble();
        m_pool->nav._fill.pitch |= 1;
    } else if (key == "NAV_YAW") {
        m_pool->nav.yaw = msg.GetDouble();
        m_pool->nav._fill.yaw |= 1;
    } else if (key == "NAV_DEPTH") {
        m_pool->nav.depth = msg.GetDouble();
        m_pool->nav._fill.depth |= 1;
    } else if (key == "NAV_HEADING") {
        m_pool->nav.heading = msg.GetDouble();
        m_pool->nav._fill.heading |= 1;
    } else if (key == "NAV_LAT") {
        m_pool->nav.latitude = msg.GetDouble();
        m_pool->nav._fill.latitude |= 1;
    } else if (key == "NAV_LONG") {
        m_pool->nav.longitude = msg.GetDouble();
        m_pool->nav._fill.longitude |= 1;
    } else if (key == "NAV_SPEED") {
        m_pool->nav.speed = msg.GetDouble();
    }

        // BHV WAYPOINT
    else if (key == "WPT_STAT") {
        m_pool->waypoint.stat = msg.GetAsString();
        m_pool->waypoint._fill.stat |= 1;
    } else if (key == "WPT_ODO") {
        m_pool->waypoint.odo = msg.GetDouble();
        m_pool->waypoint._fill.odo |= 1;
    } else if (key == "WPT_UPDATE") {
        m_pool->waypoint.update = msg.GetAsString();
        m_pool->waypoint._fill.update |= 1;
    } else if (key == "WPT_INDEX") {
        m_pool->waypoint.index = (int) msg.GetDouble();
        m_pool->waypoint._fill.index |= 1;
    }

        // IVPHELM
    else if (key == "IVPHELM_STATE") {
        m_pool->helm_status.state = msg.GetAsString();
    } else if (key == "IVPHELM_STATEVARS") {
        m_pool->helm_status.condition_vars = parseCommaList(msg.GetAsString());
        for (const auto i : m_pool->helm_status.condition_vars) {
            auto s = m_Comms.GetRegistered();
            auto r = s.find(i);
            if (r == s.end()) {
                m_Comms.Register(i, 0);
                char buf[512];
                sprintf(buf, "Subscribing to %s\n", i.c_str());
                MOOSTrace(buf);
            }
        }
    } else if (key == "IVPHELM_UPDATEVARS") {
        m_pool->helm_status.update_vars = parseCommonMoosMsg(msg.GetAsString());
    } else if (key == "IVPHELM_ALLSTOP") {
        m_pool->helm_status.allstop_msg = msg.GetAsString();
    } else if (key == "MOOS_MANUAL_OVERIDE") {
        m_pool->helm_status.manual_overide = msg.GetAsString() == "true";
    }

        // IMU
    else if (key == "IMU_ROLL") {
        m_pool->imu.roll = msg.GetDouble();
        m_pool->imu._fill.roll |= 1;
    } else if (key == "IMU_PITCH") {
        m_pool->imu.pitch = msg.GetDouble();
        m_pool->imu._fill.pitch |= 1;
    } else if (key == "IMU_YAW") {
        m_pool->imu.yaw = msg.GetDouble();
        m_pool->imu._fill.yaw |= 1;
    } else if (key == "IMU_HEADING") {
        m_pool->imu.heading = msg.GetDouble();
        m_pool->imu._fill.heading |= 1;
    } else if (key == "IMU_X_ACCEL") {
        m_pool->imu.x_accel = msg.GetDouble();
    } else if (key == "IMU_Y_ACCEL") {
        m_pool->imu.y_accel = msg.GetDouble();
    } else if (key == "IMU_Z_ACCEL") {
        m_pool->imu.z_accel = msg.GetDouble();
    } else if (key == "IMU_X_GYRO") {
        m_pool->imu.x_gyro = msg.GetDouble();
    } else if (key == "IMU_Y_GYRO") {
        m_pool->imu.y_gyro = msg.GetDouble();
    } else if (key == "IMU_Z_GYRO") {
        m_pool->imu.z_gyro = msg.GetDouble();
    }

    else if (key == "PS_TEMP") {
        m_pool->ps.temp = msg.GetDouble();
        m_pool->ps._fill.temp |= 1;
    } else if (key == "PS_PRESSURE") {
        m_pool->ps.pressure= msg.GetDouble();
        m_pool->ps._fill.pressure |= 1;
    } else if (key == "PS_FILTERED_DEPTH") {
        m_pool->ps.filtered_depth = msg.GetDouble();
        m_pool->ps._fill.filtered_depth |= 1;
    } else if (key == "PS_DEPTH") {
        m_pool->ps.depth = msg.GetDouble();
        m_pool->ps._fill.depth |= 1;
    } else if (key == "PS_BAD_TEMP") {
        m_pool->ps.bad_temp = msg.GetDouble();
        m_pool->ps._fill.bad_temp |= 1;
    } else if (key == "PS_BAD_PRESSURE") {
        m_pool->ps.bad_pressure = msg.GetDouble();
        m_pool->ps._fill.bad_pressure |= 1;
    } else if (key == "PS_STATUS") {
        m_pool->ps.status = msg.GetDouble();
        m_pool->ps._fill.status |= 1;
    }


    // microstrain IMU
    else if (key == "MS_ROLL") {
        m_pool->ms_imu.roll = msg.GetDouble();
    } else if (key == "MS_PITCH") {
        m_pool->ms_imu.pitch = msg.GetDouble();
    } else if (key == "MS_YAW") {
        m_pool->ms_imu.yaw = msg.GetDouble();
    } else if (key == "MS_HEADING") {
        m_pool->ms_imu.heading = msg.GetDouble();
    } else if (key == "MS_X_ACCEL") {
        m_pool->ms_imu.x_accel = msg.GetDouble();
        m_pool->ms_imu._fill_extended.x_accel |= 1;
    } else if (key == "MS_Y_ACCEL") {
        m_pool->ms_imu.y_accel = msg.GetDouble();
        m_pool->ms_imu._fill_extended.y_accel |= 1;
    } else if (key == "MS_Z_ACCEL") {
        m_pool->ms_imu.z_accel = msg.GetDouble();
        m_pool->ms_imu._fill_extended.z_accel |= 1;
    } else if (key == "MS_X_GYRO") {
        m_pool->ms_imu.x_gyro = msg.GetDouble();
        m_pool->ms_imu._fill_extended.x_gyro |= 1;
    } else if (key == "MS_Y_GYRO") {
        m_pool->ms_imu.y_gyro = msg.GetDouble();
        m_pool->ms_imu._fill_extended.y_gyro |= 1;
    } else if (key == "MS_Z_GYRO") {
        m_pool->ms_imu.z_gyro = msg.GetDouble();
        m_pool->ms_imu._fill_extended.z_gyro |= 1;
    } else if (key == "MS_W_QUAT") {
        m_pool->ms_imu.w_quat = msg.GetDouble();
        m_pool->ms_imu._fill_extended.w_quat |= 1;
    } else if (key == "MS_X_QUAT") {
        m_pool->ms_imu.x_quat = msg.GetDouble();
        m_pool->ms_imu._fill_extended.x_quat |= 1;
    } else if (key == "MS_Y_QUAT") {
        m_pool->ms_imu.y_quat = msg.GetDouble();
        m_pool->ms_imu._fill_extended.y_quat |= 1;
    } else if (key == "MS_Z_QUAT") {
        m_pool->ms_imu.z_quat = msg.GetDouble();
        m_pool->ms_imu._fill_extended.z_quat |= 1;
    }

    else if (key == "MSRAW_X_MAG") {
        m_pool->mag.x = msg.GetDouble();
        m_pool->mag._fill.x |= 1;
        m_pool->mag.time = msg.m_dfTime;
    } else if (key == "MSRAW_Y_MAG") {
        m_pool->mag.y = msg.GetDouble();
        m_pool->mag._fill.y |= 1;
        m_pool->mag.time = msg.m_dfTime;
    } else if (key == "MSRAW_Z_MAG") {
        m_pool->mag.z = msg.GetDouble();
        m_pool->mag._fill.z |= 1;
        m_pool->mag.time = msg.m_dfTime;
    }

        // GPS
    else if (key == "GPS_ANTENNA_OKAY") {
        m_pool->gps.antenna_okay = (int)msg.GetDouble();
    } else if (key == "GPS_PARSE_ERRORS") {
        m_pool->gps.parse_errors = (int)msg.GetDouble();
    } else if (key == "GPS_FIX") {
        m_pool->gps.fix = (int)msg.GetDouble();
        m_pool->gps._fill.fix |= 1;
    } else if (key == "GPS_HDOP") {
        m_pool->gps.hdop = msg.GetDouble();
    } else if (key == "GPS_LAST_COMMS") {
        m_pool->gps.last_comms = msg.GetDouble();
    } else if (key == "GPS_QUALITY") {
        m_pool->gps.quality = (int)msg.GetDouble();
    } else if (key == "GPS_X") {
        m_pool->gps.x = msg.GetDouble();
    } else if (key == "GPS_Y") {
        m_pool->gps.y = msg.GetDouble();
    } else if (key == "GPS_LONGITUDE") {
        m_pool->gps.longitude = msg.GetDouble();
    } else if (key == "GPS_LATITUDE") {
        m_pool->gps.latitude = msg.GetDouble();
    } else if (key == "GPS_SPEED") {
        m_pool->gps.speed = msg.GetDouble();
    } else if (key == "GPS_SAT") {
        m_pool->gps.sat = (int)msg.GetDouble();
        m_pool->gps._fill.sat |= 1;
    } else if (key == "ORIGIN_LONGITUDE") {
        m_pool->gps.origin_longitude = msg.GetDouble();
    } else if (key == "ORIGIN_LATITUDE") {
        m_pool->gps.origin_latitude = msg.GetDouble();
    } else if (key == "GPS_QUALITY") {
        m_pool->gps.quality = (int)msg.GetDouble();
    }


    if(key.rfind("GPS_", 0) == 0) {
        m_pool->gps.time = msg.m_dfTime;
    }

    if(key.rfind("IMU_", 0) == 0) {
        m_pool->imu.time = msg.m_dfTime;
    }

    if(key.rfind("PS_", 0) == 0) {
        m_pool->ps.time = msg.m_dfTime;
    }

    if(key.rfind("MS_", 0) == 0) {
        m_pool->ms_imu.time = msg.m_dfTime;
    }

    if(key.rfind("IVPHELM_", 0) == 0) {
        m_pool->helm_status.time = msg.m_dfTime;
    }

    else {
        auto state_key_find = std::find(
                m_pool->helm_status.condition_vars.begin(),
                m_pool->helm_status.condition_vars.end(),
                key
                );
        if(state_key_find != m_pool->helm_status.condition_vars.end()) {
            m_pool->helm_status.conditions[key] = msg.GetAsString() == "true";

        }
    }

    if(TEST_NAV_FILL(m_pool->nav)) {
        m_rosNode->PublishNav();
        FLUSH_FILL(m_pool->nav);
    }

    if(TEST_WAYPOINT_FILL(m_pool->waypoint)) {
        FLUSH_FILL(m_pool->waypoint);

    }

    if(TEST_IMU_FILL(m_pool->imu)) {
        m_rosNode->PublishImu();
        FLUSH_FILL(m_pool->imu);
    }

    if(TEST_IMU_FILL_EXTENDED(m_pool->ms_imu)) {
        m_rosNode->PublishMsImu();
        FLUSH_FILL_EXTENDED(m_pool->ms_imu);
    }

    if(TEST_GPS_FILL(m_pool->gps)) {
        m_rosNode->PublishGps();
        FLUSH_FILL(m_pool->gps);
    }

    if(TEST_PS_FILL(m_pool->ps)) {
        m_rosNode->PublishPressure();
        FLUSH_FILL(m_pool->ps);
    }

    if(TEST_MAG_FILL(m_pool->mag)) {
        m_rosNode->PublishMag();
        FLUSH_FILL(m_pool->mag);
    }


}

bool MOOSNode::publishDvl() {
    return
        Notify("DVL_VEL_X", m_pool->dvl.x, m_pool->dvl.time) &&
        Notify("DVL_VEL_Y", m_pool->dvl.z, m_pool->dvl.time) &&
        Notify("DVL_VEL_Z", m_pool->dvl.y, m_pool->dvl.time);
}

bool MOOSNode::publishWayPointUpdate()
{
    return toMOOS("WPT_UPDATE", m_pool->waypoint.update);
}

bool MOOSNode::triggerCalibration()
{
    return toMOOS("RT_CALIBRATE_IMU", "1");
}

bool MOOSNode::publishDepthUpdate()
{
    return toMOOS("DEP_UPDATE", m_pool->depth.update);
}

bool MOOSNode::publishIvpHelmUpdate(const std::string& name, bool state)
{
    return toMOOS(name, state ? "true" : "false");
}

// TODO: investigate the reason behind this typo
bool MOOSNode::publishManualOveride(bool state) {
    return toMOOS("MOOS_MANUAL_OVERIDE", state ? "true" : "false");
}
