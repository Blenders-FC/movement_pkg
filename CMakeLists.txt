cmake_minimum_required(VERSION 3.8)
project(movement_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_online_walking_module_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(humanoid_nav_msgs REQUIRED)



# ------------------------------
# Include directories
# ------------------------------
include_directories(
  include
  ${robotis_math_INCLUDE_DIRS}
)

# BT plugin
add_library(manager_bt_plugin SHARED
  src/nodes/manager_running_condition.cpp
  src/nodes/left_kick_action.cpp
  src/nodes/simple_walk_action.cpp
  src/nodes/right_kick_action.cpp
  src/nodes/ball_detected_condition.cpp
  src/nodes/search_sinusoidal_action.cpp
  src/nodes/center_ball_YOLO_PID_action.cpp
  src/nodes/choose_kick_foot_condition.cpp
  src/bt_plugin_register.cpp
  src/walking_controller.cpp
  src/utils.cpp
  src/cb_data_manager.cpp
)

# utils
#add_library(utils src/utils.cpp)


set(SHARED_LIBRARIES
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  #behavior_tree_core
  #cb_data_manager
  #walking_controller
  #utils
  #fov_calculation
  #blackboard
  #tree_builder
  #repeat_node
)


ament_target_dependencies(manager_bt_plugin
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  robotis_math
  behaviortree_cpp
  robotis_controller_msgs
  op3_online_walking_module_msgs
  op3_action_module_msgs
  op3_walking_module_msgs
  robotis_framework_common
  robotis_device
  humanoid_nav_msgs
  
)

target_include_directories(manager_bt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS manager_bt_plugin
  LIBRARY DESTINATION lib
)

# BT executor
add_executable(bt_executor
  src/tree_builder.cpp
)

ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(tree_builder)
ament_export_dependencies(rclcpp behaviortree_cpp)

ament_package()
