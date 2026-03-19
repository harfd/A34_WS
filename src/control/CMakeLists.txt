cmake_minimum_required(VERSION 2.8.3)
project(control)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O2")
set(PROJECT_DEPS
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  fsd_common_msgs
  fsd_tools
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  /usr/include/eigen3
  /usr/include/cppad 
)

add_executable(${PROJECT_NAME}
  src/control.cpp
  src/control_handle.cpp
  src/main.cpp
  include/Basic/types.h
  src/Track/track_base.cpp
  src/Track/track_acceleration.cpp
  src/Track/track_skidpad.cpp
  src/Track/track_trackdrive.cpp
  src/Solver/solver_base.cpp
  src/Solver/solver_purePursuit.cpp
  src/Solver/solver_mpc.cpp
  src/Visualization/visualization.cpp
  )
  
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  /usr/local/lib/libipopt.so
  )