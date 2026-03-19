cmake_minimum_required(VERSION 2.8.3)
project(fusion)

# add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  cv_bridge
  image_transport
  message_filters
  fsd_common_msgs
  )

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(FILES
        ObjectCount.msg
        BoundingBoxes.msg
        BoundingBox.msg
        Cone.msg
        Map.msg
        ClusterCone.msg
        ClusterCones.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime  pcl_conversions pcl_msgs pcl_ros cv_bridge image_transport roscpp rospy
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )


# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/fusion_handle.cpp
  src/fusion.cpp
  src/main.cpp
  src/Hungarian.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
