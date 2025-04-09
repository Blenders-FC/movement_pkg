cmake_minimum_required(VERSION 2.8.3)
project(movement_pkg)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    robotis_controller_msgs
    sensor_msgs
    std_msgs
    rospy
    cv_bridge
    image_transport
    geometry_msgs
    op3_action_module_msgs
    robotis_math
    op3_walking_module_msgs
#   message_generation
    robotis_framework_common
)

# Add Behavior Tree Core source files
set(BT_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/action_node.cpp
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/condition_node.cpp
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/control_node.cpp
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/decorator_node.cpp
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/leaf_node.cpp
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/src/tree_node.cpp
)

set(SOURCE_FILES
  src/motion_controller_node.cpp
  src/nodes/ball_detected_condition.cpp
  src/nodes/ball_direction_condition.cpp
  src/nodes/ball_in_center_condition.cpp
  src/nodes/center_ball_action.cpp
  src/nodes/choose_kick_foot_condition.cpp
  src/nodes/fov_walking_action.cpp
  src/nodes/get_up_backwards_action.cpp
  src/nodes/get_up_forward_action.cpp
  src/nodes/left_kick_action.cpp
  src/nodes/manager_running_condition.cpp
  src/nodes/manager_done_condition.cpp
  src/nodes/right_kick_action.cpp
  src/nodes/robot_fallen_condition.cpp
  src/nodes/search_ball_action.cpp
  src/nodes/simple_walk_action.cpp
  src/nodes/stand_up_action.cpp
  src/nodes/start_button_condition.cpp
  src/nodes/turn_left_action.cpp
  src/nodes/turn_right_action.cpp
  src/nodes/walk_to_target_action.cpp
  src/nodes/online_walk_to_target_action.cpp
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
#   referee.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


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
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    robotis_controller_msgs
    sensor_msgs
    std_msgs
    rospy
    cv_bridge
    image_transport
    geometry_msgs
    op3_action_module_msgs
    robotis_math
    op3_walking_module_msgs
#    message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_core/include
  ${PROJECT_SOURCE_DIR}/../ROS-Behavior-Tree/behavior_tree_leaves/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/soccer_pkg.cpp
# )

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_node src/soccer_pkg_node.cpp)
add_executable(motion_controller_node
  src/motion_controller_node.cpp
  src/nodes/ball_detected_condition.cpp
  src/nodes/ball_direction_condition.cpp
  src/nodes/ball_in_center_condition.cpp
  src/nodes/center_ball_action.cpp
  src/nodes/choose_kick_foot_condition.cpp
  src/nodes/fov_walking_action.cpp
  src/nodes/get_up_backwards_action.cpp
  src/nodes/get_up_forward_action.cpp
  src/nodes/left_kick_action.cpp
  src/nodes/manager_running_condition.cpp
  src/nodes/manager_done_condition.cpp
  src/nodes/right_kick_action.cpp
  src/nodes/robot_fallen_condition.cpp
  src/nodes/search_ball_action.cpp
  src/nodes/simple_walk_action.cpp
  src/nodes/stand_up_action.cpp
  src/nodes/start_button_condition.cpp
  src/nodes/turn_left_action.cpp
  src/nodes/turn_right_action.cpp
  src/nodes/walk_to_target_action.cpp
  src/nodes/online_walk_to_target_action.cpp
)

## Add cmake target dependencies of the library
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(motion_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#)

add_library(cb_data_manager src/cb_data_manager.cpp)
add_library(walking_controller src/walking_controller.cpp)
add_library(utils src/utils.cpp)
add_library(fov_calculation src/fov_calculation.cpp)
add_library(blackboard src/blackboard.cpp)

target_link_libraries(motion_controller_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  behavior_tree_core
  cb_data_manager
  walking_controller
  utils
  fov_calculation
  blackboard
)

target_link_libraries(cb_data_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  robotis_framework_common
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS motion_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_soccer_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
