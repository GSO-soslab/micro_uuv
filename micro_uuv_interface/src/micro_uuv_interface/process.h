#ifndef PROCESS_H_
#define PROCESS_H_

#include "pool.h"
#include "moos_node.h"
#include "ros_node.h"

namespace soslab {

    class Process {
    private:
        std::shared_ptr<MOOSNode> m_moosNode;
        std::shared_ptr<ROSNode> m_rosNode;
        std::shared_ptr<pool_t> m_pool;
    public:
        Process();
        void Run(int argc, char* argv[]);
    };
}


#endif
