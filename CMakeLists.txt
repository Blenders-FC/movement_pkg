cmake_minimum_required(VERSION 3.8)
project(movement_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# ------------------------------
# Include directories
# ------------------------------
include_directories(
  include
)
# BT plugin
add_library(manager_bt_plugin SHARED
  src/nodes/manager_running_condition.cpp
  src/nodes/left_kick_action.cpp
  src/bt_plugin_register.cpp
)

ament_target_dependencies(manager_bt_plugin
  rclcpp
  behaviortree_cpp
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