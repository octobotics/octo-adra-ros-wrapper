# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "octo_adra_ros_wrapper: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iocto_adra_ros_wrapper:/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(octo_adra_ros_wrapper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_custom_target(_octo_adra_ros_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "octo_adra_ros_wrapper" "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" ""
)

get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_custom_target(_octo_adra_ros_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "octo_adra_ros_wrapper" "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Services
_generate_srv_cpp(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Module File
_generate_module_cpp(octo_adra_ros_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/octo_adra_ros_wrapper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(octo_adra_ros_wrapper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(octo_adra_ros_wrapper_generate_messages octo_adra_ros_wrapper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_cpp _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_cpp _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(octo_adra_ros_wrapper_gencpp)
add_dependencies(octo_adra_ros_wrapper_gencpp octo_adra_ros_wrapper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS octo_adra_ros_wrapper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Services
_generate_srv_eus(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Module File
_generate_module_eus(octo_adra_ros_wrapper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/octo_adra_ros_wrapper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(octo_adra_ros_wrapper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(octo_adra_ros_wrapper_generate_messages octo_adra_ros_wrapper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_eus _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_eus _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(octo_adra_ros_wrapper_geneus)
add_dependencies(octo_adra_ros_wrapper_geneus octo_adra_ros_wrapper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS octo_adra_ros_wrapper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Services
_generate_srv_lisp(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Module File
_generate_module_lisp(octo_adra_ros_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/octo_adra_ros_wrapper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(octo_adra_ros_wrapper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(octo_adra_ros_wrapper_generate_messages octo_adra_ros_wrapper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_lisp _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_lisp _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(octo_adra_ros_wrapper_genlisp)
add_dependencies(octo_adra_ros_wrapper_genlisp octo_adra_ros_wrapper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS octo_adra_ros_wrapper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Services
_generate_srv_nodejs(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Module File
_generate_module_nodejs(octo_adra_ros_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/octo_adra_ros_wrapper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(octo_adra_ros_wrapper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(octo_adra_ros_wrapper_generate_messages octo_adra_ros_wrapper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_nodejs _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_nodejs _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(octo_adra_ros_wrapper_gennodejs)
add_dependencies(octo_adra_ros_wrapper_gennodejs octo_adra_ros_wrapper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS octo_adra_ros_wrapper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Services
_generate_srv_py(octo_adra_ros_wrapper
  "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper
)

### Generating Module File
_generate_module_py(octo_adra_ros_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(octo_adra_ros_wrapper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(octo_adra_ros_wrapper_generate_messages octo_adra_ros_wrapper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/srv/SetMode.srv" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_py _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/octobotics/octo-adra-ros-wrapper/src/octo_adra_ros_wrapper/msg/TargetValue.msg" NAME_WE)
add_dependencies(octo_adra_ros_wrapper_generate_messages_py _octo_adra_ros_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(octo_adra_ros_wrapper_genpy)
add_dependencies(octo_adra_ros_wrapper_genpy octo_adra_ros_wrapper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS octo_adra_ros_wrapper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/octo_adra_ros_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/octo_adra_ros_wrapper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET std_srvs_generate_messages_cpp)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_cpp std_srvs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/octo_adra_ros_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/octo_adra_ros_wrapper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET std_srvs_generate_messages_eus)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_eus std_srvs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/octo_adra_ros_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/octo_adra_ros_wrapper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET std_srvs_generate_messages_lisp)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_lisp std_srvs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/octo_adra_ros_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/octo_adra_ros_wrapper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET std_srvs_generate_messages_nodejs)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_nodejs std_srvs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/octo_adra_ros_wrapper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET std_srvs_generate_messages_py)
  add_dependencies(octo_adra_ros_wrapper_generate_messages_py std_srvs_generate_messages_py)
endif()
