cmake_minimum_required(VERSION 3.0.2)
project(m4_base)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_subdirectory(third-party/yaml-cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	m4_msgs
	mavros_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	message_generation
	)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Set project root directory
add_definitions("-DPROJECT_ROOT_DIR=\"${PROJECT_SOURCE_DIR}\"")

# Modify the latency timer in dynamixel_sdk. (Value used in src/dynamixel_sdk/port_handler_linux.cpp)
add_definitions("-DLATENCY_TIMER=1")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WheelVelocityCmd.msg
  DerivedVelocityCmd.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include include/dynamixel_sdk third-party/yaml-cpp/include
  LIBRARIES DynamixelInterface
  CATKIN DEPENDS m4_msgs roscpp std_msgs mavros_msgs geometry_msgs message_runtime
#   DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
include/dynamixel_sdk
third-party/yaml-cpp/include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# DynamixelInterface library
add_library(DynamixelInterface STATIC
    src/DynamixelInterface.cpp
    include/DynamixelInterface.h
)

# Utils library
add_library(utils STATIC
    src/utils.cpp
    include/utils.h
)

# Logger library
add_library(Logger STATIC
    src/Logger.cpp
    include/Logger.h
)

# M4_base library
add_library(m4_base STATIC
    src/m4_base.cpp
    include/m4_base.h
)

# Dynamixel SDK
add_library(dynamixel_sdk STATIC
	src/dynamixel_sdk/group_bulk_read.cpp
	src/dynamixel_sdk/group_bulk_write.cpp
	src/dynamixel_sdk/group_fast_bulk_read.cpp
	src/dynamixel_sdk/group_fast_sync_read.cpp
	src/dynamixel_sdk/group_handler.cpp
	src/dynamixel_sdk/group_sync_read.cpp
	src/dynamixel_sdk/group_sync_write.cpp
	src/dynamixel_sdk/packet_handler.cpp
	src/dynamixel_sdk/port_handler.cpp
	src/dynamixel_sdk/port_handler_arduino.cpp
	src/dynamixel_sdk/port_handler_linux.cpp
	src/dynamixel_sdk/port_handler_mac.cpp
	src/dynamixel_sdk/port_handler_windows.cpp
	src/dynamixel_sdk/protocol1_packet_handler.cpp
	src/dynamixel_sdk/protocol2_packet_handler.cpp
	include/dynamixel_sdk/dynamixel_sdk.h
	include/dynamixel_sdk/group_bulk_read.h
	include/dynamixel_sdk/group_bulk_write.h
	include/dynamixel_sdk/group_fast_bulk_read.h
	include/dynamixel_sdk/group_fast_sync_read.h
	include/dynamixel_sdk/group_handler.h
	include/dynamixel_sdk/group_sync_read.h
	include/dynamixel_sdk/group_sync_write.h
	include/dynamixel_sdk/packet_handler.h
	include/dynamixel_sdk/port_handler.h
	include/dynamixel_sdk/port_handler_arduino.h
	include/dynamixel_sdk/port_handler_linux.h
	include/dynamixel_sdk/port_handler_mac.h
	include/dynamixel_sdk/port_handler_windows.h
	include/dynamixel_sdk/protocol1_packet_handler.h
	include/dynamixel_sdk/protocol2_packet_handler.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# Add the executables
 add_executable(m4_base_test
 	unit-tests/m4_base_test.cpp
 )

add_executable(keyboard_interface
	src/keyboard_interface.cpp
)
 
add_executable(m4_controller_node
 	 nodes/m4_controller_node.cpp
 )
# 

add_dependencies(m4_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(m4_profile_node nodes/m4_profile_node.cpp src/m4_base.cpp)
target_link_libraries(m4_profile_node 
m4_base DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
${catkin_LIBRARIES}
)
add_dependencies(m4_profile_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link common libraries
 target_link_libraries(m4_base_test 
 m4_base DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
 ${catkin_LIBRARIES}
 )

target_link_libraries(m4_controller_node 
m4_base DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
${catkin_LIBRARIES}
)

target_link_libraries(keyboard_interface
m4_base DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
${catkin_LIBRARIES}
)

set(UNIT_TEST_SCRIPTS
	# Stored in unit-tests
	dynamixel_interface_test
)

set(COMMON_LIBRARIES
	DynamixelInterface dynamixel_sdk yaml-cpp utils Logger
)

# Set up the unit tests
foreach(SCRIPT IN LISTS UNIT_TEST_SCRIPTS)
	# List of source files
	set(SOURCES unit-tests/${SCRIPT}.cpp)

	# Add the executables
	add_executable(${SCRIPT} ${SOURCES})

	# Link common libraries
	target_link_libraries(${SCRIPT} ${COMMON_LIBRARIES})
endforeach()

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamixel_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
