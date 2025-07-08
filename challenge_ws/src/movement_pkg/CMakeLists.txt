cmake_minimum_required(VERSION 3.0.2)
project(movement_pkg)

## Compile as C++11, pulled from the advertised default C++ standards of
## declared catkin packages.
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## If catkin is not installed or catkin_make is not working
## enable this line (and comment of the catkin_package args)
# find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # Agrega aquí otras dependencias de ROS si las usas, por ejemplo:
  # actionlib
  # geometry_msgs
  # sensor_msgs
)

###################################
## Declare ROS messages and services
###################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing messages this package uses
# )

###################################
## Declare package dependencies
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES movement_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your own headers are in include/movement_pkg
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are compiled into their own subdirectory
## so these paths are relative to the package root.

# Nodo SimpleWalk (C++)
add_executable(simple_walk_action src/nodes/simple_walk_action.cpp)
add_dependencies(simple_walk_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(simple_walk_action
  ${catkin_LIBRARIES}
)

# Nodo WalkToTarget (C++)
add_executable(walk_to_target_action src/nodes/walk_to_target_action.cpp)
add_dependencies(walk_to_target_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(walk_to_target_action
  ${catkin_LIBRARIES}
)

## Mark executable scripts (Python)
## Catkin will install these in the devel space and add them to the PATH
catkin_install_python(PROGRAMS scripts/state_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# No necesitas `add_executable` ni `target_link_libraries` para los scripts Python.
# `catkin_install_python` los maneja.

## Add extra cmake modules to be searched
# find_package(Boost REQUIRED COMPONENTS system thread)
# find_package(Eigen REQUIRED)

## Uncomment to add other libraries to be compiled and linked
# add_library(movement_pkg_lib src/movement_pkg/movement_pkg_lib.cpp)
# target_link_libraries(movement_pkg_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# Instalar los ejecutables C++
install(TARGETS
  simple_walk_action
  walk_to_target_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instalar los archivos de cabecera
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Puedes instalar otros archivos si es necesario, como launch files, rviz configs, etc.
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )