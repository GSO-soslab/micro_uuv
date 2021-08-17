#include "micro_uuv_interface/ros_node.h"
#include "micro_uuv_interface/process.h"


int main(int argc, char* argv[]) {

    soslab::Process p;
    
    p.Run(argc, argv);
    
    return 0; 
}
