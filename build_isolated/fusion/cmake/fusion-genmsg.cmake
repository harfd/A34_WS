# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fusion: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ifusion:/home/RZS/A06_ws/src/perception/fusion/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fusion_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" "fusion/BoundingBox:std_msgs/Header"
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" "std_msgs/String:geometry_msgs/Point"
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:fusion/ClusterCone:std_msgs/Header"
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" "std_msgs/String:geometry_msgs/Point:fusion/Cone:std_msgs/Header"
)

get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_custom_target(_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fusion" "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)
_generate_msg_cpp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
)

### Generating Services

### Generating Module File
_generate_module_cpp(fusion
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fusion_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fusion_generate_messages fusion_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_cpp _fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fusion_gencpp)
add_dependencies(fusion_gencpp fusion_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fusion_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)
_generate_msg_eus(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
)

### Generating Services

### Generating Module File
_generate_module_eus(fusion
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fusion_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fusion_generate_messages fusion_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_eus _fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fusion_geneus)
add_dependencies(fusion_geneus fusion_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fusion_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)
_generate_msg_lisp(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
)

### Generating Services

### Generating Module File
_generate_module_lisp(fusion
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fusion_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fusion_generate_messages fusion_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_lisp _fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fusion_genlisp)
add_dependencies(fusion_genlisp fusion_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fusion_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)
_generate_msg_nodejs(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fusion
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fusion_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fusion_generate_messages fusion_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_nodejs _fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fusion_gennodejs)
add_dependencies(fusion_gennodejs fusion_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fusion_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)
_generate_msg_py(fusion
  "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
)

### Generating Services

### Generating Module File
_generate_module_py(fusion
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fusion_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fusion_generate_messages fusion_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Cone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCones.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ObjectCount.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/BoundingBox.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/Map.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/RZS/A06_ws/src/perception/fusion/msg/ClusterCone.msg" NAME_WE)
add_dependencies(fusion_generate_messages_py _fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fusion_genpy)
add_dependencies(fusion_genpy fusion_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fusion_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fusion
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fusion_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fusion_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(fusion_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fusion
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fusion_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fusion_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(fusion_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fusion
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fusion_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fusion_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(fusion_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fusion
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fusion_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fusion_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(fusion_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fusion
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fusion_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fusion_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(fusion_generate_messages_py sensor_msgs_generate_messages_py)
endif()
