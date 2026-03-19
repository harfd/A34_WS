cmake_minimum_required(VERSION 2.8.3)
project(imu_load)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  sensor_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  serial
  sensor_msgs
  )

add_message_files(FILES
  imu_state.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
) 

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/imu_load.cpp
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )


# add_message_files(FILES
#   imu_state.msg
# )

# generate_messages(DEPENDENCIES
#   std_msgs
# )
