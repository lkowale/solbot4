cmake_minimum_required(VERSION 3.8)
project(move_seq_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(solbot4_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_behavior_tree
  pluginlib
  solbot4_msgs
  behaviortree_cpp
)

# Navigator plugin library
add_library(move_seq_navigator SHARED
  src/move_seq_navigator.cpp
)
ament_target_dependencies(move_seq_navigator ${dependencies})

# BT action node plugin library
add_library(move_sequence_action_bt_node SHARED
  src/move_sequence_action.cpp
)
ament_target_dependencies(move_sequence_action_bt_node ${dependencies})
# Required for BT_REGISTER_NODES macro to export the symbol
target_compile_definitions(move_sequence_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

# BT condition node plugin library for pause functionality
add_library(pause_condition_bt_node SHARED
  src/pause_condition.cpp
)
ament_target_dependencies(pause_condition_bt_node ${dependencies})
target_compile_definitions(pause_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  move_seq_navigator
  move_sequence_action_bt_node
  pause_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install behavior trees
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install plugin description files
pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(move_seq_navigator move_sequence_action_bt_node pause_condition_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
