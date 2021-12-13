cmake_minimum_required(VERSION 3.0.2)
project(world_step_control_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  world_step_control
  roscpp
  rospy
  std_msgs
)

## Find Gazebo
find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  world_step_control
  std_msgs
)

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
  INCLUDE_DIRS include
  LIBRARIES world_step_control
  CATKIN_DEPENDS world_step_control std_msgs roscpp rospy
  DEPENDS GAZEBO
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo configuration
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


###
# World Control Plugin
###
add_library(WorldStepControlPlugin
  src/world_step_control_plugin.cpp
  src/world_step_control.cpp
)

add_dependencies(WorldStepControlPlugin 
	${WorldStepControlPlugin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${world_step_control_EXPORTED_TARGETS})

set_target_properties(WorldStepControlPlugin PROPERTIES PREFIX "")
target_link_libraries(WorldStepControlPlugin 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  world_step_control)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
set(NODE_NAME ${PROJECT_NAME}_node)

## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${NODE_NAME} src/world_step_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${NODE_NAME} PROPERTIES OUTPUT_NAME world_step_control PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${world_step_control_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${NODE_NAME}
  WorldStepControlPlugin
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${NODE_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS WorldStepControlPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#  # myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# Set GAZEBO_PLUGIN_PATH to be changed on source
configure_file(cmake/env_setup.sh.in "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/env_hooks/10_gz_step_setup.sh" @ONLY)
configure_file(cmake/env_setup.sh.in "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/env_hooks/10_gz_step_setup.bash" @ONLY)
configure_file(cmake/env_setup.sh.in "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/env_hooks/10_gz_step_setup.zsh" @ONLY)
configure_file(cmake/env_setup.sh.in "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/env_hooks/10_gz_step_setup.tcsh" @ONLY)

catkin_add_env_hooks(10_gz_step_setup SHELLS sh bash zsh tcsh DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/env_hooks/")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_world_step_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
