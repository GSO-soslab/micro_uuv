cmake_minimum_required(VERSION 3.0.2)
project(micro_uuv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
    BoolMap.msg
    Gps.msg
    IvpHelmState.msg
    MoosDebug.msg
    Nav.msg
    NMEA.msg
    Pressure.msg
    StringMap.msg
    WayPoint.msg
)

add_service_files(
  FILES
    SetDepth.srv
    SetHelmCondition.srv
    SetWayPoint.srv
)

# add_action_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)
