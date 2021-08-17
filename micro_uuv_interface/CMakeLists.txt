cmake_minimum_required(VERSION 3.0.2)
project(micro_uuv_interface VERSION 0.0.1 LANGUAGES CXX)

add_compile_options(-std=c++11)

set(MOOS_DIR "/opt/moos-ivp/build/MOOS/MOOSCore")
find_package(MOOS REQUIRED)
include_directories(${MOOS_INCLUDE_DIRS})
link_directories(${MOOS_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  auv_msgs
  message_generation
  tf2
  dvl_a50_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES micro_uuv_interface 
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    auv_msgs
    message_runtime
    tf2
    dvl_a50_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(interface_node
  src/interface_node.cpp
  src/micro_uuv_interface/moos_node.cpp
  src/micro_uuv_interface/ros_node.cpp
  src/micro_uuv_interface/process.cpp
)

target_link_libraries(interface_node
  ${catkin_LIBRARIES}
  ${MOOS_LIBRARIES}
)

add_dependencies(interface_node ${interface_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
