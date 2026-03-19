cmake_minimum_required(VERSION 2.8.3)

project(fsd_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  geometry_msgs
  sensor_msgs
  )

add_message_files(
  FILES
  Cone.msg
  ControlCommand.msg
  CarState.msg
  CarStateDt.msg
  Map.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)
