# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "odom_aux: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iodom_aux:/code/ex7/ex_workspace/src/odom_aux/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(odom_aux_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg" NAME_WE)
add_custom_target(_odom_aux_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odom_aux" "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg" ""
)

get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg" NAME_WE)
add_custom_target(_odom_aux_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odom_aux" "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(odom_aux
  "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_aux
)
_generate_msg_cpp(odom_aux
  "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_aux
)

### Generating Services

### Generating Module File
_generate_module_cpp(odom_aux
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_aux
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(odom_aux_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(odom_aux_generate_messages odom_aux_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg" NAME_WE)
add_dependencies(odom_aux_generate_messages_cpp _odom_aux_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg" NAME_WE)
add_dependencies(odom_aux_generate_messages_cpp _odom_aux_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_aux_gencpp)
add_dependencies(odom_aux_gencpp odom_aux_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_aux_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(odom_aux
  "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux
)
_generate_msg_py(odom_aux
  "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux
)

### Generating Services

### Generating Module File
_generate_module_py(odom_aux
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(odom_aux_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(odom_aux_generate_messages odom_aux_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/DistWheel.msg" NAME_WE)
add_dependencies(odom_aux_generate_messages_py _odom_aux_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/code/ex7/ex_workspace/src/odom_aux/msg/Pose2D.msg" NAME_WE)
add_dependencies(odom_aux_generate_messages_py _odom_aux_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_aux_genpy)
add_dependencies(odom_aux_genpy odom_aux_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_aux_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_aux)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_aux
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(odom_aux_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_aux
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(odom_aux_generate_messages_py std_msgs_generate_messages_py)
endif()
