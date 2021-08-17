#pragma once

#if !defined(MOOS_NODE_H_)
#define MOOS_NODE_H_

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <vector>
#include <thread>
#include "pool.h"

namespace soslab {

    class ROSNode;

    class MOOSNode : public CMOOSApp {
    public:
        ~MOOSNode() override = default;

        bool toMOOS(const std::string& moosName, double data);
        bool toMOOS(const std::string& moosName, std::string myString);
        bool toMOOSBinaryString(const std::string& moosName, std::string myString);

        void Init(const char * MissionFile);

        bool publishWayPointUpdate();

        bool publishDepthUpdate();

        bool publishDvl();

        bool publishIvpHelmUpdate(const std::string& name, bool state);

        bool publishManualOveride(bool state);

        bool triggerCalibration();

        void SetPool(std::shared_ptr<pool_t> p) { m_pool = p; }

        void SetRosNode(std::shared_ptr<ROSNode> n) { m_rosNode = n; }

    protected:
        
        double appTick;
        double commsTick;

        // where we handle new mail
        bool OnNewMail(MOOSMSG_LIST &NewMail) override;

        // where we do the work
        bool Iterate() override;

        //called when we connect to the server
        bool OnConnectToServer() override;

        // called when we are starting up..
        bool OnStartUp() override;

        void DoRegistrations();
        
        void Translate(CMOOSMsg & msg);

        std::thread th;

    private:
        std::shared_ptr<pool_t> m_pool;

        std::shared_ptr<ROSNode> m_rosNode;
    };
}

#endif
