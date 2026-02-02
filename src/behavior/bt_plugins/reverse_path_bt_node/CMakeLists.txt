cmake_minimum_required(VERSION 3.8)
project(reverse_path_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# BT node for reversing a path
add_library(reverse_path_bt_node SHARED
  src/reverse_path.cpp
)
ament_target_dependencies(reverse_path_bt_node ${dependencies})
target_compile_definitions(reverse_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  reverse_path_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(reverse_path_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
