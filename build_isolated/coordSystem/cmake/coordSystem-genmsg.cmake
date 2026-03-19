# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "coordSystem: 1 messages, 0 services")

set(MSG_I_FLAGS "-IcoordSystem:/home/RZS/A06_ws/src/coordTrans/coordSystem/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(coordSystem_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_custom_target(_coordSystem_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "coordSystem" "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(coordSystem
  "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/coordSystem
)

### Generating Services

### Generating Module File
_generate_module_cpp(coordSystem
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/coordSystem
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(coordSystem_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(coordSystem_generate_messages coordSystem_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_dependencies(coordSystem_generate_messages_cpp _coordSystem_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(coordSystem_gencpp)
add_dependencies(coordSystem_gencpp coordSystem_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS coordSystem_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(coordSystem
  "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/coordSystem
)

### Generating Services

### Generating Module File
_generate_module_eus(coordSystem
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/coordSystem
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(coordSystem_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(coordSystem_generate_messages coordSystem_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_dependencies(coordSystem_generate_messages_eus _coordSystem_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(coordSystem_geneus)
add_dependencies(coordSystem_geneus coordSystem_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS coordSystem_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(coordSystem
  "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/coordSystem
)

### Generating Services

### Generating Module File
_generate_module_lisp(coordSystem
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/coordSystem
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(coordSystem_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(coordSystem_generate_messages coordSystem_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_dependencies(coordSystem_generate_messages_lisp _coordSystem_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(coordSystem_genlisp)
add_dependencies(coordSystem_genlisp coordSystem_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS coordSystem_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(coordSystem
  "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/coordSystem
)

### Generating Services

### Generating Module File
_generate_module_nodejs(coordSystem
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/coordSystem
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(coordSystem_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(coordSystem_generate_messages coordSystem_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_dependencies(coordSystem_generate_messages_nodejs _coordSystem_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(coordSystem_gennodejs)
add_dependencies(coordSystem_gennodejs coordSystem_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS coordSystem_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(coordSystem
  "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/coordSystem
)

### Generating Services

### Generating Module File
_generate_module_py(coordSystem
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/coordSystem
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(coordSystem_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(coordSystem_generate_messages coordSystem_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/RZS/A06_ws/src/coordTrans/coordSystem/msg/imu_state.msg" NAME_WE)
add_dependencies(coordSystem_generate_messages_py _coordSystem_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(coordSystem_genpy)
add_dependencies(coordSystem_genpy coordSystem_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS coordSystem_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/coordSystem)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/coordSystem
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(coordSystem_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/coordSystem)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/coordSystem
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(coordSystem_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/coordSystem)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/coordSystem
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(coordSystem_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/coordSystem)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/coordSystem
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(coordSystem_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/coordSystem)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/coordSystem\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/coordSystem
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(coordSystem_generate_messages_py std_msgs_generate_messages_py)
endif()
