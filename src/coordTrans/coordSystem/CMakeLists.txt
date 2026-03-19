cmake_minimum_required(VERSION 2.8.3)
project(coordSystem)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  )



find_package(Eigen3 QUIET)
find_package(OpenCV REQUIRED)

add_message_files(FILES
  imu_state.msg
)

generate_messages(DEPENDENCIES
  std_msgs
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
  ${EIGEN3_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/coordSystem_handle.cpp
  src/coordSystem.cpp
  src/main.cpp
  src/solver/line_fitting.cpp
  src/visual.cpp
  )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  /usr/local/lib/libproj.so
  )
