cmake_minimum_required(VERSION 3.8)
project(get_pose_from_poses_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  geometry_msgs
)

# BT node for getting a pose from a vector of poses
add_library(get_pose_from_poses_bt_node SHARED
  src/get_pose_from_poses.cpp
)
ament_target_dependencies(get_pose_from_poses_bt_node ${dependencies})
target_compile_definitions(get_pose_from_poses_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  get_pose_from_poses_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(get_pose_from_poses_bt_node)
ament_export_dependencies(${dependencies})

ament_package()
